from __future__ import annotations
import rby1_sdk

from typing import Optional, List
import sys
import time
import logging

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose

logging.basicConfig(level=logging.INFO)

DELTA_TIMEOUT = 0.5
BODY_LINK_NAME = {"A": "link_torso_5", "M": "link_torso_5"}
GRIPPER_MASS = 1.9
GRAVITY = 9.81

LINEAR_VELOCITY_LIMIT = 0.3
ANGULAR_VELOCITY_LIMIT = 1.0
ACCELERATION_LIMIT_SCALING = 1.0

# Vive Tracker Safety Parameters
MAX_LINEAR_DELTA = 0.05     # 5cm
MAX_ANGULAR_DELTA = 0.8      # 45도


class TargetStore:
    right: Optional[Pose] = None
    left: Optional[Pose] = None
    right_time: float = 0.0
    left_time: float = 0.0
    right_timed_out: bool = True
    left_timed_out: bool = True

# -------------------------
# Transform utilities
# -------------------------


def quaternion_to_rotation_matrix(q) -> np.ndarray:
    x, y, z, w = q.x, q.y, q.z, q.w
    n = x * x + y * y + z * z + w * w
    if n == 0.0:
        return np.eye(3, dtype=float)
    s = 2.0 / n
    xx, yy, zz = x * x * s, y * y * s, z * z * s
    xy, xz, yz = x * y * s, x * z * s, y * z * s
    wx, wy, wz = w * x * s, w * y * s, w * z * s
    return np.array(
        [
            [1.0 - (yy + zz), xy - wz, xz + wy],
            [xy + wz, 1.0 - (xx + zz), yz - wx],
            [xz - wy, yz + wx, 1.0 - (xx + yy)],
        ],
        dtype=float,
    )


def pose_to_transform(pose: Pose) -> np.ndarray:
    R = quaternion_to_rotation_matrix(pose.orientation)
    t = np.array([pose.position.x, pose.position.y,
                 pose.position.z], dtype=float)
    T = np.eye(4, dtype=float)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


def rot_y(angle: float) -> np.ndarray:
    c, s = float(np.cos(angle)), float(np.sin(angle))
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]], dtype=float)


def make_transform(R: np.ndarray, t_xyz: List[float]) -> np.ndarray:
    T = np.eye(4, dtype=float)
    T[:3, :3] = R
    T[:3, 3] = np.array(t_xyz, dtype=float)
    return T


def limit_delta(T_delta: np.ndarray, max_pos: float, max_rot: float) -> np.ndarray:
    """간단하게 delta를 제한."""
    T_limited = T_delta.copy()

    # 위치 제한
    pos = T_delta[:3, 3]
    pos_norm = np.linalg.norm(pos)
    if pos_norm > max_pos:
        T_limited[:3, 3] = pos * (max_pos / pos_norm)

    # 회전 제한 (스케일 축소하기)
    R = T_delta[:3, :3]
    angle = np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1))
    if angle > max_rot:
        # 회전을 완전히 무시하지 말고, 스케일만 줄임
        scale = max_rot / angle

        # 회전축 추출
        axis = np.array([
            R[2, 1] - R[1, 2],
            R[0, 2] - R[2, 0],
            R[1, 0] - R[0, 1]
        ])
        axis_norm = np.linalg.norm(axis)

        if axis_norm > 1e-6:
            axis = axis / axis_norm
            # 스케일된 각도로 회전 재구성
            scaled_angle = angle * scale
            K = np.array([
                [0, -axis[2], axis[1]],
                [axis[2], 0, -axis[0]],
                [-axis[1], axis[0], 0]
            ])
            R_scaled = np.eye(3) + np.sin(scaled_angle) * K + \
                (1 - np.cos(scaled_angle)) * (K @ K)
            T_limited[:3, :3] = R_scaled

    return T_limited

# -------------------------
# ROS Node
# -------------------------


class TeleopCartesianNode(Node):
    def __init__(self):
        super().__init__("teleop_cartesian_from_cumulative_ee_delta")

        # Subscriptions: cumulative delta in EE(start) frame
        self.create_subscription(
            Pose, "/cartesian_target/ee_right", self._cb_right, 10)
        self.create_subscription(
            Pose, "/cartesian_target/ee_left", self._cb_left, 10)

        # Robot parameter
        self.declare_parameter("robot_address", "192.168.0.101:50051")
        self.declare_parameter("control_rate", 200)
        self.declare_parameter("reference_frame", "link_torso_5")

        # Cartesian command limits (07-style)
        self.declare_parameter("linear_velocity_limit", LINEAR_VELOCITY_LIMIT)
        self.declare_parameter("angular_velocity_limit",
                               ANGULAR_VELOCITY_LIMIT)
        self.declare_parameter(
            "acceleration_limit_scaling", ACCELERATION_LIMIT_SCALING)

        # Read parameters
        self.robot_address = str(self.get_parameter("robot_address").value)
        self.control_rate = float(self.get_parameter("control_rate").value)
        self.reference_frame = str(self.get_parameter("reference_frame").value)

        self.linear_velocity_limit = float(
            self.get_parameter("linear_velocity_limit").value)
        self.angular_velocity_limit = float(
            self.get_parameter("angular_velocity_limit").value)
        self.acceleration_limit_scaling = float(
            self.get_parameter("acceleration_limit_scaling").value)

        self.get_logger().info(
            "Initialized. Set1 is right arm."
            f"addr={self.robot_address}, rate={self.control_rate} Hz, ref={self.reference_frame}, "
            f"linear_vel_limit={self.linear_velocity_limit}, angular_vel_limit={self.angular_velocity_limit}, "
            f"accel_scaling={self.acceleration_limit_scaling}"
        )

    def _cb_right(self, msg: Pose) -> None:
        TargetStore.right = msg
        TargetStore.right_time = time.time()
        TargetStore.right_timed_out = False

    def _cb_left(self, msg: Pose) -> None:
        TargetStore.left = msg
        TargetStore.left_time = time.time()
        TargetStore.left_timed_out = False


# -------------------------
# Main loop
# -------------------------
def main(args=None):
    rclpy.init(args=args)
    node = TeleopCartesianNode()

    from threading import Thread
    ros_thread = Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    # Connect to robot
    robot = rby1_sdk.create_robot_a(node.robot_address)
    robot.connect()

    # Apply robot-side parameters (optional)
    try:
        robot.set_parameter("default.acceleration_limit_scaling",
                            str(node.acceleration_limit_scaling))
        robot.set_parameter("default.linear_velocity_limit",
                            str(node.linear_velocity_limit))
        robot.set_parameter("default.angular_velocity_limit",
                            str(node.angular_velocity_limit))
        node.get_logger().info("Robot parameters set successfully")
    except Exception as e:
        node.get_logger().warning(f"Could not set robot parameters: {e}")

    robot.power_on(".*")
    robot.servo_on("right_arm_.*|left_arm_.*|torso_.*")
    robot.reset_fault_control_manager()
    robot.enable_control_manager()

    stream = robot.create_command_stream()

    period = 1.0 / max(node.control_rate, 1.0)
    node.get_logger().info("Starting teleop loop (CartesianCommand)")

    try:
        # initialize teleop start pose ONCE
        state = robot.get_state()

        model_name = robot.model().model_name
        body_link = BODY_LINK_NAME.get(model_name, node.reference_frame)

        dyn_robot = robot.get_dynamics()
        link_names = dyn_robot.get_link_names()
        dyn_joint_names = dyn_robot.get_joint_names()

        model_joint_names = robot.model().robot_joint_names

        name_to_pos = dict(zip(model_joint_names, state.position))

        q_dyn = np.array([name_to_pos[name]
                         for name in dyn_joint_names], dtype=float)

        dyn_state = dyn_robot.make_state(link_names, dyn_joint_names)
        dyn_state.set_q(q_dyn)
        dyn_state.set_qdot(np.zeros_like(q_dyn))
        dyn_state.set_qddot(np.zeros_like(q_dyn))

        dyn_robot.compute_forward_kinematics(dyn_state)

        base_idx = link_names.index(body_link)
        ee_r_idx = link_names.index("ee_right")
        ee_l_idx = link_names.index("ee_left")

        node.T_start_right = dyn_robot.compute_transformation(
            dyn_state, base_idx, ee_r_idx
        )
        node.T_start_left = dyn_robot.compute_transformation(
            dyn_state, base_idx, ee_l_idx
        )

        node.get_logger().info("Teleop start pose initialized.")

        while rclpy.ok():
            now = time.time()
            right_alive = (now - TargetStore.right_time) <= DELTA_TIMEOUT
            left_alive = (now - TargetStore.left_time) <= DELTA_TIMEOUT

            # 타임아웃 감지 로직
            if not right_alive and not TargetStore.right_timed_out:
                TargetStore.right_timed_out = True  # ← 타임아웃 처리
                node.get_logger().warn("RIGHT arm timed out!")
                TargetStore.right = None

            if not left_alive and not TargetStore.left_timed_out:
                TargetStore.left_timed_out = True  # ← 타임아웃 처리
                node.get_logger().warn("LEFT arm timed out!")
                TargetStore.left = None

            # 상태 조회를 한 번만 수행
            state = robot.get_state()
            model_joint_names = robot.model().robot_joint_names
            dyn_joint_names = dyn_robot.get_joint_names()
            name_to_pos = dict(zip(model_joint_names, state.position))
            q_dyn = np.array([name_to_pos[n]
                             for n in dyn_joint_names], dtype=float)

            # RIGHT arm 재초기화
            if TargetStore.right_timed_out:
                dyn_state.set_q(q_dyn)
                dyn_robot.compute_forward_kinematics(dyn_state)
                node.T_start_right = dyn_robot.compute_transformation(
                    dyn_state, base_idx, ee_r_idx
                )

            # LEFT arm 재초기화
            if TargetStore.left_timed_out:
                dyn_state.set_q(q_dyn)
                dyn_robot.compute_forward_kinematics(dyn_state)
                node.T_start_left = dyn_robot.compute_transformation(
                    dyn_state, base_idx, ee_l_idx
                )

            t0 = time.time()

            targets = []

            # RIGHT arm
            if TargetStore.right is not None:
                try:
                    delta = pose_to_transform(TargetStore.right)

                    # Safety: delta 제한
                    delta = limit_delta(
                        delta, MAX_LINEAR_DELTA, MAX_ANGULAR_DELTA)

                    T_right = node.T_start_right @ delta
                    targets.append(("ee_right", T_right))
                    TargetStore.right_timed_out = False

                except Exception as e:
                    node.get_logger().error(f"Right target error: {e}")

            # LEFT arm (동일)
            if TargetStore.left is not None:
                try:
                    delta = pose_to_transform(TargetStore.left)

                    # Safety: delta 제한
                    delta = limit_delta(
                        delta, MAX_LINEAR_DELTA, MAX_ANGULAR_DELTA)

                    T_left = node.T_start_left @ delta
                    targets.append(("ee_left", T_left))
                    TargetStore.left_timed_out = False

                except Exception as e:
                    node.get_logger().error(f"Left target error: {e}")

            # Send one Cartesian command containing available targets
            if targets:
                right_imp_cmd = None
                left_imp_cmd = None

                for link_name, T in targets:
                    # Cartesian Impedance control을 하는데, joint space에서 PID 설정
                    # >>> cmd = (
                    # ...   rby.CartesianImpedanceControlCommandBuilder()
                    # ...   .add_target("base_link", "right_wrist", T,
                    # ...               linear_velocity_limit=0.3,    # solver internal bounds
                    # ...               angular_velocity_limit=0.8)
                    # ...   .add_joint_position_target("right_arm_3", 0.5)  # joint hint in the OC stage
                    # ...   .set_joint_stiffness(np.full(7, 40.0))          # K diag for joint impedance
                    # ...   .set_joint_damping_ratio(0.7)                   # ζ → D
                    # ...   .set_joint_torque_limit(np.full(7, 35.0))       # τ saturation
                    # ...   .set_minimum_time(2.0)
                    # ... )

                    # Cartesian Impedance control을 하는데, Cartesian space에서 PID 설정
                    # >>> cmd = (
                    # ...   rby.ImpedanceControlCommandBuilder()
                    # ...   .set_reference_link_name("base_link")
                    # ...   .set_link_name("right_wrist")
                    # ...   .set_transformation(T)
                    # ...   .set_translation_weight(np.array([1200.0, 1200.0, 1600.0]))  # N/m
                    # ...   .set_rotation_weight(np.array([60.0, 60.0, 60.0]))           # Nm/rad
                    # ...   .set_damping_ratio(0.7)
                    # ... )
                    # """

                    # 중력 보상: Z축 위치 보정
                    translation_weights = [1200, 1200, 1200]  # N/m
                    gravity_force = GRIPPER_MASS * GRAVITY    # N (약 24.5 N)

                    # Z축 강성으로 중력을 보상하는데 필요한 위치 오프셋
                    # F = K * delta_z  =>  delta_z = F / K
                    z_offset_compensation = gravity_force / \
                        translation_weights[2]

                    # Target 위치를 위로 보정 (중력만큼 올려줌)
                    T_compensated = T.copy()
                    T_compensated[2, 3] += z_offset_compensation

                    imp_cmd = (
                        rby1_sdk.ImpedanceControlCommandBuilder()
                        .set_command_header(
                            rby1_sdk.CommandHeaderBuilder()
                            .set_control_hold_time(0.1)
                        )
                        .set_reference_link_name(body_link)
                        .set_link_name(link_name)
                        .set_translation_weight([1200, 1200, 1200])
                        .set_rotation_weight([60, 60, 60])
                        # .set_translation_weight([600, 600, 1200])
                        # .set_rotation_weight([30, 30, 30])
                        .set_transformation(T_compensated)
                        .set_damping_ratio(0.7)
                    )

                    if link_name == "ee_right":
                        right_imp_cmd = imp_cmd
                    elif link_name == "ee_left":
                        left_imp_cmd = imp_cmd

                body_cmd = rby1_sdk.BodyComponentBasedCommandBuilder()

                if right_imp_cmd is not None:
                    body_cmd = body_cmd.set_right_arm_command(right_imp_cmd)

                if left_imp_cmd is not None:
                    body_cmd = body_cmd.set_left_arm_command(left_imp_cmd)

                rc = rby1_sdk.RobotCommandBuilder().set_command(
                    rby1_sdk.ComponentBasedCommandBuilder()
                    .set_body_command(body_cmd)
                )

                try:
                    stream.send_command(rc)
                except RuntimeError as e:
                    if "expired" in str(e):
                        node.get_logger().info("Command stream expired. Recreating stream...")
                        stream = robot.create_command_stream()
                        stream.send_command(rc)
                    else:
                        raise
                # Logging joint pos
                state = robot.get_state()

            # timing
            elapsed = time.time() - t0
            to_sleep = period - elapsed
            if to_sleep > 0:
                time.sleep(to_sleep)

    except KeyboardInterrupt:
        node.get_logger().info("Shutting down")

    finally:
        node.destroy_node()
        rclpy.shutdown()
        ros_thread.join(timeout=1.0)


if __name__ == "__main__":
    main()

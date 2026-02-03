from __future__ import annotations

from typing import Optional, List
import sys
import time
import logging
import math

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose

logging.basicConfig(level=logging.INFO)

# NOTE: Adjust this path to your rby1_sdk location or install the SDK
sys.path.append('/home/ian/rby1_ws/rby1/rby1-sdk')
import rby1_sdk


BODY_LINK_NAME = {"A": "link_torso_5", "M": "link_torso_5"}


# -------------------------
# Shared store for ROS callbacks
# -------------------------
class TargetStore:
    right: Optional[Pose] = None
    left: Optional[Pose] = None


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
    t = np.array([pose.position.x, pose.position.y, pose.position.z], dtype=float)
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

def make_start_transform(yaw: float, offset_xyz: List[float]) -> np.ndarray:
    """
    body_link -> ee_start transform (fixed start pose in body frame)
    """
    T = np.eye(4, dtype=float)
    T[:3, :3] = rot_y(yaw)
    T[:3, 3] = np.array(offset_xyz, dtype=float)
    return T


# -------------------------
# ROS Node
# -------------------------
class TeleopCartesianNode(Node):
    def __init__(self):
        super().__init__("teleop_cartesian_from_cumulative_ee_delta")

        # Subscriptions: cumulative delta in EE(start) frame
        self.create_subscription(Pose, "/cartesian_target/ee_right", self._cb_right, 10)
        self.create_subscription(Pose, "/cartesian_target/ee_left", self._cb_left, 10)

        # Robot connection
        self.declare_parameter("robot_address", "192.168.0.100:50051")
        self.declare_parameter("control_rate", 500)

        # Reference frame
        self.declare_parameter("reference_frame", "link_torso_5")

        # Fixed teleop start pose (body frame)
        self.declare_parameter("right_start_offset", [0.4, -0.4, 0.0])
        self.declare_parameter("left_start_offset", [0.4, 0.4, 0.0])
        # self.declare_parameter("right_start_yaw", 0.0)
        # self.declare_parameter("left_start_yaw", 0.0)
        # 07 example uses rot_y(angle): angle = -pi/4 for both arms
        self.declare_parameter("right_start_rot_y", float(-np.pi / 4))
        self.declare_parameter("left_start_rot_y", float(-np.pi / 4))

        # Cartesian command limits (07-style)
        self.declare_parameter("minimum_time", 0.15)               # seconds (>= loop period)
        self.declare_parameter("linear_velocity_limit", 0.3)      # m/s (slow)
        self.declare_parameter("angular_velocity_limit", 1.0)      # rad/s (slow)
        self.declare_parameter("acceleration_limit_scaling", 1.0)  # dimensionless

        # Robot parameters (optional, but mirrors 07 demo intent)
        self.declare_parameter("robot_default_accel_scaling", "0.8")
        self.declare_parameter("robot_cartesian_cutoff_hz", "5")
        self.declare_parameter("robot_default_linear_acc_limit", "5")

        # Read parameters
        self.robot_address = str(self.get_parameter("robot_address").value)
        self.control_rate = float(self.get_parameter("control_rate").value)

        self.reference_frame = str(self.get_parameter("reference_frame").value)

        self.right_start_offset = list(self.get_parameter("right_start_offset").value)
        self.left_start_offset = list(self.get_parameter("left_start_offset").value)
        # self.right_start_yaw = float(self.get_parameter("right_start_yaw").value)
        # self.left_start_yaw = float(self.get_parameter("left_start_yaw").value)
        self.right_start_rot_y = float(self.get_parameter("right_start_rot_y").value)
        self.left_start_rot_y = float(self.get_parameter("left_start_rot_y").value)

        self.minimum_time = float(self.get_parameter("minimum_time").value)
        self.linear_velocity_limit = float(self.get_parameter("linear_velocity_limit").value)
        self.angular_velocity_limit = float(self.get_parameter("angular_velocity_limit").value)
        self.acceleration_limit_scaling = float(self.get_parameter("acceleration_limit_scaling").value)

        self.robot_default_accel_scaling = str(self.get_parameter("robot_default_accel_scaling").value)
        self.robot_cartesian_cutoff_hz = str(self.get_parameter("robot_cartesian_cutoff_hz").value)
        self.robot_default_linear_acc_limit = str(self.get_parameter("robot_default_linear_acc_limit").value)

        # Fixed start transforms (body->ee_start)
        # self.T_start_right = make_start_transform(self.right_start_yaw, self.right_start_offset)
        # self.T_start_left = make_start_transform(self.left_start_yaw, self.left_start_offset)
        self.T_start_right = make_transform(rot_y(self.right_start_rot_y), self.right_start_offset)
        self.T_start_left  = make_transform(rot_y(self.left_start_rot_y),  self.left_start_offset)

        self.get_logger().info(
            "Initialized. Set1 is right arm."
            f"addr={self.robot_address}, rate={self.control_rate} Hz, ref={self.reference_frame}, "
            f"v_lim={self.linear_velocity_limit} m/s, w_lim={self.angular_velocity_limit} rad/s, "
            f"min_time={self.minimum_time}s"
        )

    def _cb_right(self, msg: Pose) -> None:
        TargetStore.right = msg

    def _cb_left(self, msg: Pose) -> None:
        TargetStore.left = msg


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
        robot.set_parameter("default.acceleration_limit_scaling", node.robot_default_accel_scaling)
        robot.set_parameter("cartesian_command.cutoff_frequency", node.robot_cartesian_cutoff_hz)
        robot.set_parameter("default.linear_acceleration_limit", node.robot_default_linear_acc_limit)
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
        while rclpy.ok():
            t0 = time.time()

            model_name = robot.model().model_name
            body_link = BODY_LINK_NAME.get(model_name, node.reference_frame)

            targets = []

            # RIGHT: T_target = T_start_right @ Delta_cumulative
            if TargetStore.right is not None:
                try:
                    delta = pose_to_transform(TargetStore.right)
                    T_right = node.T_start_right @ delta
                    targets.append(("ee_right", T_right))
                    print("T_right:", T_right, flush=True)
                except Exception as e:
                    node.get_logger().error(f"Right target error: {e}")

            # LEFT: T_target = T_start_left @ Delta_cumulative
            if TargetStore.left is not None:
                try:
                    delta = pose_to_transform(TargetStore.left)
                    T_left = node.T_start_left @ delta
                    targets.append(("ee_left", T_left))
                    print("T_left:", T_left, flush=True)
                except Exception as e:
                    node.get_logger().error(f"Left target error: {e}")

            # Send one Cartesian command containing available targets
            if targets:
                cb = rby1_sdk.CartesianCommandBuilder()
                for link_name, T in targets:
                    cb = cb.add_target(
                        body_link,
                        link_name,
                        T,
                        node.linear_velocity_limit,
                        node.angular_velocity_limit,
                        node.acceleration_limit_scaling,
                    )
                cb = cb.set_minimum_time(max(node.minimum_time, period))

                rc = rby1_sdk.RobotCommandBuilder().set_command(
                    rby1_sdk.ComponentBasedCommandBuilder().set_body_command(cb)
                )
                stream.send_command(rc)

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

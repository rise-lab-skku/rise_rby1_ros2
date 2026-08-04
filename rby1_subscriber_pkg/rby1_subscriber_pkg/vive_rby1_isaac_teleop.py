#!/usr/bin/env python3

import os
import sys

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from scipy.spatial.transform import Rotation as R
import numpy as np

from ament_index_python.packages import get_package_share_directory
import xacro
from urdf_parser_py.urdf import URDF

from kdl_parser_py.urdf import treeFromUrdfModel
import PyKDL


class TeleopPosePublisher(Node):
    def __init__(self):
        super().__init__("vive_teleop_pose_publisher")

        self.test_mode = False
        self.test_pose = Pose()
        self.test_pose.position.x = 0.0
        self.test_pose.position.y = 0.0
        self.test_pose.position.z = 0.0
        self.test_pose.orientation.x = 0.0
        self.test_pose.orientation.y = 0.0
        self.test_pose.orientation.z = 0.0
        self.test_pose.orientation.w = 1.0

        self.enable_teleop = True
        self.curr_pose_set1 = None
        self.curr_pose_set2 = None
        self.curr_gripper_width_set1 = None
        self.curr_gripper_width_set2 = None
        self.latest_joint_state = None
        self.base_ee_left = None
        self.base_ee_right = None
        self.latest_left_positions = None
        self.latest_right_positions = None

        self.left_ee_link = "vive_tracker_left"
        self.right_ee_link = "vive_tracker_right"
        self.base_link = None
        self.left_chain = None
        self.right_chain = None
        self.left_joint_names = []
        self.right_joint_names = []
        self.left_fk = None
        self.right_fk = None
        self.left_ik_vel = None
        self.right_ik_vel = None
        self.left_ik = None
        self.right_ik = None
        self.joint_position_limits = {}
        self.model_loaded = False
        self.torso_joint_names = [
            "torso_0",
            "torso_1",
            "torso_2",
            "torso_3",
            "torso_4",
            "torso_5",
        ]
        self.torso_hold_positions = None

        self._load_rby1_model()

        self.create_subscription(Bool, "/enable_teleop", self.cb_enable_teleop, 10)
        self.create_subscription(Pose, "/vive_tracker_1/pose", self.cb_set1_pose, 10)
        self.create_subscription(Pose, "/vive_tracker_2/pose", self.cb_set2_pose, 10)
        self.create_subscription(
            JointState, "/isaacsim/joint_states", self.cb_joint_states, 10
        )

        # self.create_subscription(JointState, '/piper_set1/joint_states', self.cb_set1_gripper, 10)
        # self.create_subscription(JointState, '/piper_set2/joint_states', self.cb_set2_gripper, 10)

        # Publish joint targets for RBY1
        # self.pub_joint_targets = self.create_publisher(JointState, "/joint_states", 10)
        self.pub_joint_targets = self.create_publisher(JointState, "/joint_states_arm", 3)
        self.publish_timer = self.create_timer(0.05, self.cb_publish_timer)

        # Gripper publishers are disabled for now (will add later)
        # self.pub_set1_gripper = self.create_publisher(Float32, "/fr3_set1_gripper/gripper_width", 10)
        # self.pub_set2_gripper = self.create_publisher(Float32, "/fr3_set2_gripper/gripper_width", 10)

    def cb_enable_teleop(self, msg: Bool):
        self.get_logger().info(f"enable_teleop: {msg.data}")
        if msg.data and not self.enable_teleop:
            self.curr_pose_set1 = None
            self.curr_pose_set2 = None
            self.curr_gripper_width_set1 = None
            self.curr_gripper_width_set2 = None
            self.base_ee_left = None
            self.base_ee_right = None
            self.enable_teleop = msg.data

        elif not msg.data and self.enable_teleop:
            self.enable_teleop = msg.data

    def cb_set1_pose(self, msg: Pose):
        if not self.enable_teleop:
            return
        if not self.model_loaded:
            return

        if self.test_mode:
            if self.curr_pose_set1 is None:
                self.curr_pose_set1 = msg
            return

        if self.left_chain is None:
            return

        if self.latest_joint_state is None:
            return

        if self.curr_pose_set1 is None:
            self.curr_pose_set1 = msg
            if self.base_ee_left is None:
                self.base_ee_left = self._get_current_ee_frame(
                    self.left_chain, self.left_fk, self.left_joint_names
                )
            return

        rel_pose = self.compute_relative_pose(self.curr_pose_set1, msg)
        target_frame = self._compose_target_frame(self.base_ee_left, rel_pose)
        left_positions = self._solve_ik(
            self.left_chain, self.left_ik, self.left_joint_names, target_frame
        )
        if left_positions is not None:
            self.latest_left_positions = left_positions

    def cb_set2_pose(self, msg: Pose):
        if not self.enable_teleop:
            return
        if not self.model_loaded:
            return

        if self.test_mode:
            if self.curr_pose_set2 is None:
                self.curr_pose_set2 = msg
            return

        if self.right_chain is None:
            return

        if self.latest_joint_state is None:
            return

        if self.curr_pose_set2 is None:
            self.curr_pose_set2 = msg
            if self.base_ee_right is None:
                self.base_ee_right = self._get_current_ee_frame(
                    self.right_chain, self.right_fk, self.right_joint_names
                )
            return

        rel_pose = self.compute_relative_pose(self.curr_pose_set2, msg)
        target_frame = self._compose_target_frame(self.base_ee_right, rel_pose)
        right_positions = self._solve_ik(
            self.right_chain, self.right_ik, self.right_joint_names, target_frame
        )
        if right_positions is not None:
            self.latest_right_positions = right_positions

    def cb_joint_states(self, msg: JointState):
        self.latest_joint_state = msg

    def cb_publish_timer(self):
        if not self.enable_teleop:
            return
        if not self.model_loaded:
            return
        if self.test_mode:
            if self.latest_joint_state is None:
                return

            if self.base_ee_left is None:
                self.base_ee_left = self._get_current_ee_frame(
                    self.left_chain, self.left_fk, self.left_joint_names
                )
            if self.base_ee_right is None:
                self.base_ee_right = self._get_current_ee_frame(
                    self.right_chain, self.right_fk, self.right_joint_names
                )

            current_left_frame = self._get_current_ee_frame(
                self.left_chain, self.left_fk, self.left_joint_names
            )
            current_right_frame = self._get_current_ee_frame(
                self.right_chain, self.right_fk, self.right_joint_names
            )

            if current_left_frame is not None:
                left_pos = current_left_frame.p
                self.get_logger().info(
                    f"Current left EE position (torso_5): {left_pos.x():.4f}, {left_pos.y():.4f}, {left_pos.z():.4f}"
                )
            if current_right_frame is not None:
                right_pos = current_right_frame.p
                self.get_logger().info(
                    f"Current right EE position (torso_5): {right_pos.x():.4f}, {right_pos.y():.4f}, {right_pos.z():.4f}"
                )

            left_target = self._compose_target_frame(self.base_ee_left, self.test_pose)
            right_target = self._compose_target_frame(
                self.base_ee_right, self.test_pose
            )

            left_positions = self._solve_ik(
                self.left_chain, self.left_ik, self.left_joint_names, left_target
            )
            right_positions = self._solve_ik(
                self.right_chain, self.right_ik, self.right_joint_names, right_target
            )

            if left_positions is not None:
                left_str = ", ".join(f"{x:.4f}" for x in left_positions)
                self.get_logger().info(f"Test pose IK results - left: {left_str}")
            if right_positions is not None:
                right_str = ", ".join(f"{x:.4f}" for x in right_positions)
                self.get_logger().info(f"Test pose IK results - right: {right_str}")

            if left_positions is not None:
                self.latest_left_positions = left_positions
            if right_positions is not None:
                self.latest_right_positions = right_positions

        self._publish_joint_targets()

    def compute_relative_pose(self, ref: Pose, target: Pose) -> Pose:
        dx = target.position.x - ref.position.x
        dy = target.position.y - ref.position.y
        dz = target.position.z - ref.position.z
        o_R_ee = R.from_quat(
            [ref.orientation.x, ref.orientation.y, ref.orientation.z, ref.orientation.w]
        ).as_matrix()
        trans_vec = np.array([dx, dy, dz])
        trans_vec_ee = np.dot(np.transpose(o_R_ee), trans_vec)

        q1 = np.array(
            [ref.orientation.x, ref.orientation.y, ref.orientation.z, ref.orientation.w]
        )
        q2 = np.array(
            [
                target.orientation.x,
                target.orientation.y,
                target.orientation.z,
                target.orientation.w,
            ]
        )

        o_R_q1 = R.from_quat(q1)
        o_R_q2 = R.from_quat(q2)
        rot_rel = o_R_q1.inv() * o_R_q2
        q_rel = rot_rel.as_quat()

        rel_pose = Pose()
        rel_pose.position.x = trans_vec_ee[0]
        rel_pose.position.y = trans_vec_ee[1]
        rel_pose.position.z = trans_vec_ee[2]
        rel_pose.orientation.x = q_rel[0]
        rel_pose.orientation.y = q_rel[1]
        rel_pose.orientation.z = q_rel[2]
        rel_pose.orientation.w = q_rel[3]
        return rel_pose

    def _load_rby1_model(self) -> None:
        self.model_loaded = False
        try:
            share_dir = get_package_share_directory("rby1_description")
        except Exception as exc:
            self.get_logger().error(f"Failed to find rby1_description: {exc}")
            return

        urdf_path = os.path.join(share_dir, "urdf", "rby1.urdf.xacro")
        if not os.path.exists(urdf_path):
            self.get_logger().error(f"rby1.urdf.xacro not found at {urdf_path}")
            return

        try:
            robot_xml = xacro.process_file(urdf_path).toxml()
            robot = URDF.from_xml_string(robot_xml)
        except Exception as exc:
            self.get_logger().error(f"Failed to parse URDF: {exc}")
            return

        if self.left_ee_link not in robot.link_map:
            fallback_left = "link_left_arm_6"
            if fallback_left in robot.link_map:
                self.get_logger().warn(
                    f"End-effector link '{self.left_ee_link}' not found in URDF. "
                    f"Falling back to '{fallback_left}'."
                )
                self.left_ee_link = fallback_left
            else:
                self.get_logger().error(
                    f"End-effector link '{self.left_ee_link}' not found and fallback '{fallback_left}' is missing."
                )
                return

        if self.right_ee_link not in robot.link_map:
            fallback_right = "link_right_arm_6"
            if fallback_right in robot.link_map:
                self.get_logger().warn(
                    f"End-effector link '{self.right_ee_link}' not found in URDF. "
                    f"Falling back to '{fallback_right}'."
                )
                self.right_ee_link = fallback_right
            else:
                self.get_logger().error(
                    f"End-effector link '{self.right_ee_link}' not found and fallback '{fallback_right}' is missing."
                )
                return

        base_link_name = "link_torso_5"
        if base_link_name not in robot.link_map:
            self.get_logger().error(
                f"Base link '{base_link_name}' not found in URDF; available links: {len(robot.link_map)}"
            )
            return
        self.base_link = base_link_name
        ok, tree = treeFromUrdfModel(robot)
        if not ok:
            self.get_logger().error("Failed to build KDL tree from URDF")
            return

        try:
            self.left_chain = tree.getChain(self.base_link, self.left_ee_link)
            self.right_chain = tree.getChain(self.base_link, self.right_ee_link)
        except Exception as exc:
            self.get_logger().error(f"Failed to get KDL chains: {exc}")
            return

        self.left_joint_names = self._get_chain_joint_names(self.left_chain)
        self.right_joint_names = self._get_chain_joint_names(self.right_chain)
        if len(self.left_joint_names) == 0 or len(self.right_joint_names) == 0:
            self.get_logger().error(
                "KDL chain has zero movable joints. Check base_link/ee_link names and fixed joints. "
                f"left_joints={len(self.left_joint_names)}, right_joints={len(self.right_joint_names)}"
            )
            return
        self.left_fk = PyKDL.ChainFkSolverPos_recursive(self.left_chain)
        self.right_fk = PyKDL.ChainFkSolverPos_recursive(self.right_chain)

        try:
            left_q_min, left_q_max = self._get_chain_joint_limits(
                robot, self.left_joint_names
            )
            right_q_min, right_q_max = self._get_chain_joint_limits(
                robot, self.right_joint_names
            )
        except ValueError as exc:
            self.get_logger().error(f"Failed to load IK joint limits: {exc}")
            return

        self.left_ik_vel = PyKDL.ChainIkSolverVel_pinv(self.left_chain)
        self.right_ik_vel = PyKDL.ChainIkSolverVel_pinv(self.right_chain)
        self.left_ik = PyKDL.ChainIkSolverPos_NR_JL(
            self.left_chain,
            left_q_min,
            left_q_max,
            self.left_fk,
            self.left_ik_vel,
            200,
            1e-6,
        )
        self.right_ik = PyKDL.ChainIkSolverPos_NR_JL(
            self.right_chain,
            right_q_min,
            right_q_max,
            self.right_fk,
            self.right_ik_vel,
            200,
            1e-6,
        )
        self.model_loaded = True

        self.get_logger().info(
            f"Loaded RBY1 model. base_link={self.base_link}, "
            f"left_joints={len(self.left_joint_names)}, right_joints={len(self.right_joint_names)}"
        )

    def _get_chain_joint_names(self, chain: PyKDL.Chain) -> list:
        names = []
        for i in range(chain.getNrOfSegments()):
            joint = chain.getSegment(i).getJoint()
            if joint.getType() != PyKDL.Joint.Fixed:
                names.append(joint.getName())
        return names

    def _get_chain_joint_limits(self, robot: URDF, joint_names: list):
        q_min = PyKDL.JntArray(len(joint_names))
        q_max = PyKDL.JntArray(len(joint_names))

        for i, name in enumerate(joint_names):
            urdf_joint = robot.joint_map.get(name)
            if urdf_joint is None:
                raise ValueError(f"joint '{name}' is missing from the URDF")
            if (
                urdf_joint.limit is None
                or urdf_joint.limit.lower is None
                or urdf_joint.limit.upper is None
            ):
                raise ValueError(
                    f"joint '{name}' has no finite position limit"
                )

            lower = float(urdf_joint.limit.lower)
            upper = float(urdf_joint.limit.upper)
            valid_limits = (
                np.isfinite(lower) and np.isfinite(upper) and lower <= upper
            )
            if not valid_limits:
                raise ValueError(
                    f"joint '{name}' has invalid limits [{lower}, {upper}]"
                )

            q_min[i] = lower
            q_max[i] = upper
            self.joint_position_limits[name] = (lower, upper)

        return q_min, q_max

    def _get_current_joint_positions(self, joint_names: list) -> np.ndarray:
        if self.latest_joint_state is None:
            return None

        name_to_index = {name: i for i, name in enumerate(self.latest_joint_state.name)}
        positions = []
        for name in joint_names:
            idx = name_to_index.get(name)
            if idx is None or idx >= len(self.latest_joint_state.position):
                return None
            positions.append(self.latest_joint_state.position[idx])
        return np.array(positions, dtype=float)

    def _get_current_ee_frame(
        self,
        chain: PyKDL.Chain,
        fk_solver: PyKDL.ChainFkSolverPos_recursive,
        joint_names: list,
    ) -> PyKDL.Frame:
        if chain is None or fk_solver is None or len(joint_names) == 0:
            return None
        current_positions = self._get_current_joint_positions(joint_names)
        if current_positions is None:
            return None

        q = PyKDL.JntArray(len(joint_names))
        for i, pos in enumerate(current_positions):
            q[i] = pos

        frame = PyKDL.Frame()
        fk_solver.JntToCart(q, frame)
        return frame

    def _compose_target_frame(
        self, base_frame: PyKDL.Frame, rel_pose: Pose
    ) -> PyKDL.Frame:
        if base_frame is None:
            return None

        rot = R.from_quat(
            [
                rel_pose.orientation.x,
                rel_pose.orientation.y,
                rel_pose.orientation.z,
                rel_pose.orientation.w,
            ]
        )
        rot_m = rot.as_matrix()
        kdl_rot = PyKDL.Rotation(
            rot_m[0, 0],
            rot_m[0, 1],
            rot_m[0, 2],
            rot_m[1, 0],
            rot_m[1, 1],
            rot_m[1, 2],
            rot_m[2, 0],
            rot_m[2, 1],
            rot_m[2, 2],
        )
        rel_frame = PyKDL.Frame(
            kdl_rot,
            PyKDL.Vector(rel_pose.position.x, rel_pose.position.y, rel_pose.position.z),
        )
        return base_frame * rel_frame

    def _solve_ik(
        self,
        chain: PyKDL.Chain,
        ik_solver: PyKDL.ChainIkSolverPos_NR_JL,
        joint_names: list,
        target_frame: PyKDL.Frame,
    ):
        if len(joint_names) == 0:
            return None
        if target_frame is None:
            return None

        current_positions = self._get_current_joint_positions(joint_names)
        if current_positions is None:
            self.get_logger().warn("Joint states missing for IK seed")
            return None

        q_seed = PyKDL.JntArray(len(joint_names))
        for i, pos in enumerate(current_positions):
            lower, upper = self.joint_position_limits[joint_names[i]]
            q_seed[i] = min(max(float(pos), lower), upper)

        q_out = PyKDL.JntArray(len(joint_names))
        ret = ik_solver.CartToJnt(q_seed, target_frame, q_out)
        if ret < 0:
            self.get_logger().warn(f"IK failed with code {ret}")
            return None

        positions = []
        for i, name in enumerate(joint_names):
            lower, upper = self.joint_position_limits[name]
            positions.append(min(max(float(q_out[i]), lower), upper))
        return positions

    def _publish_joint_targets(self) -> None:
        if self.latest_joint_state is None:
            return

        left_positions = self.latest_left_positions
        right_positions = self.latest_right_positions

        if left_positions is None:
            left_positions = self._get_current_joint_positions(self.left_joint_names)
        if right_positions is None:
            right_positions = self._get_current_joint_positions(self.right_joint_names)

        if left_positions is None or right_positions is None:
            return

        if self.torso_hold_positions is None:
            torso_positions = self._get_current_joint_positions(self.torso_joint_names)
            if torso_positions is not None:
                self.torso_hold_positions = torso_positions
                torso_map = dict(
                    zip(self.torso_joint_names, [float(v) for v in torso_positions])
                )
                self.get_logger().info(f"Torso joints (hold): {torso_map}")
        else:
            torso_positions = self.torso_hold_positions

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.left_joint_names) + list(self.right_joint_names)
        msg.position = list(left_positions) + list(right_positions)

        if torso_positions is not None:
            msg.name += list(self.torso_joint_names)
            msg.position = list(msg.position) + list(torso_positions)
        self.pub_joint_targets.publish(msg)


def main():
    rclpy.init()
    node = TeleopPosePublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

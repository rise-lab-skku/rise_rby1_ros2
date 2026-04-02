#!/usr/bin/env python3

import os

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool
from scipy.spatial.transform import Rotation as R
import numpy as np

from ament_index_python.packages import get_package_share_directory
import xacro
from urdf_parser_py.urdf import URDF

from kdl_parser_py.urdf import treeFromUrdfModel
import PyKDL

# RBY1 SDK Import
import rby1_sdk

# RBY1 joint names from URDF (24 joints total)
# Order: wheels, torso, right_arm, left_arm, head
DEFAULT_JOINT_NAMES = [
    "right_wheel",
    "left_wheel",
    "torso_0",
    "torso_1",
    "torso_2",
    "torso_3",
    "torso_4",
    "torso_5",
    "right_arm_0",
    "right_arm_1",
    "right_arm_2",
    "right_arm_3",
    "right_arm_4",
    "right_arm_5",
    "right_arm_6",
    "left_arm_0",
    "left_arm_1",
    "left_arm_2",
    "left_arm_3",
    "left_arm_4",
    "left_arm_5",
    "left_arm_6",
    "head_0",
    "head_1",
]


class TeleopPosePublisher(Node):
    def __init__(self):
        super().__init__("vive_teleop_pose_publisher")

        self.declare_parameter("robot_address", "192.168.0.101:50051")
        self.robot_address = (
            self.get_parameter(
                "robot_address").get_parameter_value().string_value
        )

        self.test_mode = False
        self.test_pose = Pose()
        self.test_pose.orientation.w = 1.0

        self.enable_teleop = False
        self.curr_pose_set1 = None
        self.curr_pose_set2 = None

        self.current_joint_dict = {}  # 로봇에서 읽어온 현재 관절 상태 캐시
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
        self.left_ik = None
        self.right_ik = None
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

        # 1. RBY1 SDK 로봇 연결 및 초기화
        self._init_robot()

        # 2. RBY1 URDF/KDL 모델 로드
        self._load_rby1_model()

        # 3. ROS 구독자 설정
        self.create_subscription(
            Bool, "/enable_teleop", self.cb_enable_teleop, 10)
        self.create_subscription(
            Pose, "/vive_tracker_1/pose", self.cb_set1_pose, 10)
        self.create_subscription(
            Pose, "/vive_tracker_2/pose", self.cb_set2_pose, 10)

        # 4. 제어 루프 타이머 (100Hz)
        self.publish_timer = self.create_timer(0.01, self.control_loop)

    def _init_robot(self):
        self.get_logger().info(
            f"Connecting to RBY1 at {self.robot_address}...")
        self.robot = rby1_sdk.create_robot_a(self.robot_address)
        self.robot.connect()

        self.robot.set_parameter(
            "joint_position_command.cutoff_frequency", "5")
        self.robot.set_parameter("default.acceleration_limit_scaling", "0.8")

        self.robot.power_on(".*")
        self.robot.servo_on("right_arm_.*|left_arm_.*|torso_.*")
        self.robot.reset_fault_control_manager()
        self.robot.enable_control_manager()

        self.stream = self.robot.create_command_stream()
        self.get_logger().info("Robot connected and control manager enabled.")

    def _extract_first_existing_attr(self, obj, names, default=None):
        """SDK State 객체에서 속성을 안전하게 추출하기 위한 유틸리티"""
        for name in names:
            if hasattr(obj, name):
                return getattr(obj, name)
        return default

    def _update_current_state(self) -> bool:
        """로봇에서 관절 상태를 읽어 내부 딕셔너리에 저장합니다."""
        try:
            state = self.robot.get_state()
            position = self._extract_first_existing_attr(
                state, ["position", "joint_position", "current_position"], default=[]
            )
            position = list(position)

            if not position or len(position) != len(DEFAULT_JOINT_NAMES):
                return False

            self.current_joint_dict = dict(zip(DEFAULT_JOINT_NAMES, position))
            return True
        except Exception as error:
            self.get_logger().warning(
                f"Failed to read joint states from robot: {error}"
            )
            return False

    def cb_enable_teleop(self, msg: Bool):
        self.get_logger().info(f"enable_teleop: {msg.data}")
        if msg.data and not self.enable_teleop:
            self.curr_pose_set1 = None
            self.curr_pose_set2 = None
            self.base_ee_left = None
            self.base_ee_right = None
            self.enable_teleop = msg.data
        elif not msg.data and self.enable_teleop:
            self.enable_teleop = msg.data

    def cb_set1_pose(self, msg: Pose):
        if (
            not self.enable_teleop
            or not self.model_loaded
            or not self.current_joint_dict
        ):
            return

        if self.test_mode:
            if self.curr_pose_set1 is None:
                self.curr_pose_set1 = msg
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
        if (
            not self.enable_teleop
            or not self.model_loaded
            or not self.current_joint_dict
        ):
            return

        if self.test_mode:
            if self.curr_pose_set2 is None:
                self.curr_pose_set2 = msg
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

    def control_loop(self):
        # 1. 최신 관절 상태 업데이트
        if not self._update_current_state():
            return

        if not self.enable_teleop or not self.model_loaded:
            return

        if self.test_mode:
            # (테스트 모드 로직은 원본 유지. 직접 IK 구해서 latest_xxx_positions 갱신)
            if self.base_ee_left is None:
                self.base_ee_left = self._get_current_ee_frame(
                    self.left_chain, self.left_fk, self.left_joint_names
                )
            if self.base_ee_right is None:
                self.base_ee_right = self._get_current_ee_frame(
                    self.right_chain, self.right_fk, self.right_joint_names
                )
            left_target = self._compose_target_frame(
                self.base_ee_left, self.test_pose)
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
                self.latest_left_positions = left_positions
            if right_positions is not None:
                self.latest_right_positions = right_positions

        # 2. 로봇으로 명령 전송
        self._send_robot_command()

    def compute_relative_pose(self, ref: Pose, target: Pose) -> Pose:
        dx = target.position.x - ref.position.x
        dy = target.position.y - ref.position.y
        dz = target.position.z - ref.position.z
        o_R_ee = R.from_quat(
            [ref.orientation.x, ref.orientation.y,
                ref.orientation.z, ref.orientation.w]
        ).as_matrix()
        trans_vec = np.array([dx, dy, dz])
        trans_vec_ee = np.dot(np.transpose(o_R_ee), trans_vec)

        q1 = np.array(
            [ref.orientation.x, ref.orientation.y,
                ref.orientation.z, ref.orientation.w]
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
            self.get_logger().error(
                f"rby1.urdf.xacro not found at {urdf_path}")
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
                self.left_ee_link = fallback_left
            else:
                return

        if self.right_ee_link not in robot.link_map:
            fallback_right = "link_right_arm_6"
            if fallback_right in robot.link_map:
                self.right_ee_link = fallback_right
            else:
                return

        base_link_name = "link_torso_5"
        if base_link_name not in robot.link_map:
            return

        self.base_link = base_link_name
        ok, tree = treeFromUrdfModel(robot)
        if not ok:
            return

        try:
            self.left_chain = tree.getChain(self.base_link, self.left_ee_link)
            self.right_chain = tree.getChain(
                self.base_link, self.right_ee_link)
        except Exception:
            return

        self.left_joint_names = self._get_chain_joint_names(self.left_chain)
        self.right_joint_names = self._get_chain_joint_names(self.right_chain)
        if len(self.left_joint_names) == 0 or len(self.right_joint_names) == 0:
            return

        self.left_fk = PyKDL.ChainFkSolverPos_recursive(self.left_chain)
        self.right_fk = PyKDL.ChainFkSolverPos_recursive(self.right_chain)
        self.left_ik = PyKDL.ChainIkSolverPos_LMA(self.left_chain)
        self.right_ik = PyKDL.ChainIkSolverPos_LMA(self.right_chain)
        self.model_loaded = True

        self.get_logger().info("Loaded RBY1 model for KDL.")

    def _get_chain_joint_names(self, chain: PyKDL.Chain) -> list:
        names = []
        for i in range(chain.getNrOfSegments()):
            joint = chain.getSegment(i).getJoint()
            if joint.getType() != PyKDL.Joint.Fixed:
                names.append(joint.getName())
        return names

    def _get_current_joint_positions(self, joint_names: list) -> np.ndarray:
        if not self.current_joint_dict:
            return None

        positions = []
        for name in joint_names:
            if name not in self.current_joint_dict:
                return None
            positions.append(self.current_joint_dict[name])
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
            PyKDL.Vector(rel_pose.position.x, rel_pose.position.y,
                         rel_pose.position.z),
        )
        return base_frame * rel_frame

    def _solve_ik(
        self,
        chain: PyKDL.Chain,
        ik_solver: PyKDL.ChainIkSolverPos_LMA,
        joint_names: list,
        target_frame: PyKDL.Frame,
    ):
        if len(joint_names) == 0 or target_frame is None:
            return None

        current_positions = self._get_current_joint_positions(joint_names)
        if current_positions is None:
            return None

        q_seed = PyKDL.JntArray(len(joint_names))
        for i, pos in enumerate(current_positions):
            q_seed[i] = pos

        q_out = PyKDL.JntArray(len(joint_names))
        ret = ik_solver.CartToJnt(q_seed, target_frame, q_out)
        if ret < 0:
            return None

        return [q_out[i] for i in range(q_out.rows())]

    def _send_robot_command(self) -> None:
        """IK로 계산된 각도를 로봇에 전송합니다."""
        left_positions = self.latest_left_positions
        right_positions = self.latest_right_positions

        # 계산된 타겟이 없으면 현재 상태 유지
        if left_positions is None:
            left_positions = self._get_current_joint_positions(
                self.left_joint_names)
        if right_positions is None:
            right_positions = self._get_current_joint_positions(
                self.right_joint_names)

        if left_positions is None or right_positions is None:
            return

        if self.torso_hold_positions is None:
            torso_positions = self._get_current_joint_positions(
                self.torso_joint_names)
            if torso_positions is not None:
                self.torso_hold_positions = torso_positions
        else:
            torso_positions = self.torso_hold_positions

        if torso_positions is None:
            return

        try:
            rc = rby1_sdk.RobotCommandBuilder().set_command(
                rby1_sdk.ComponentBasedCommandBuilder().set_body_command(
                    rby1_sdk.BodyComponentBasedCommandBuilder()
                    .set_torso_command(
                        rby1_sdk.JointPositionCommandBuilder()
                        .set_command_header(
                            rby1_sdk.CommandHeaderBuilder().set_control_hold_time(0.5)
                        )
                        .set_minimum_time(0.05)
                        .set_position(torso_positions)
                    )
                    .set_right_arm_command(
                        rby1_sdk.JointPositionCommandBuilder()
                        .set_command_header(
                            rby1_sdk.CommandHeaderBuilder().set_control_hold_time(0.5)
                        )
                        .set_minimum_time(0.05)
                        .set_position(right_positions)
                    )
                    .set_left_arm_command(
                        rby1_sdk.JointPositionCommandBuilder()
                        .set_command_header(
                            rby1_sdk.CommandHeaderBuilder().set_control_hold_time(0.5)
                        )
                        .set_minimum_time(0.05)
                        .set_position(left_positions)
                    )
                )
            )
            self.stream.send_command(rc)
        except Exception as error:
            self.get_logger().warning(
                f"Failed to send command to robot: {error}")


def main():
    rclpy.init()
    node = TeleopPosePublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if hasattr(node, "robot"):
                node.get_logger().info("Disconnecting from RBY1 robot...")
                node.robot.disconnect()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

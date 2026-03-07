#!/usr/bin/env python3

import os
import time  # Homing 시 time.sleep 사용

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool, Float32
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

        self.declare_parameter("robot_address", "192.168.30.1:50051")
        self.robot_address = (
            self.get_parameter("robot_address").get_parameter_value().string_value
        )

        self.test_mode = False
        self.test_pose = Pose()
        self.test_pose.orientation.w = 1.0

        self.enable_teleop = False
        self.curr_pose_set1 = None
        self.curr_pose_set2 = None

        self.current_joint_dict = {}
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

        # 그리퍼 Homing 값 저장을 위한 변수 초기화
        self.gripper_min_q = np.array([np.inf, np.inf])
        self.gripper_max_q = np.array([-np.inf, -np.inf])

        # 1. RBY1 SDK 로봇 연결 및 초기화
        self._init_robot()

        # 1-2. 그리퍼 초기화 및 Homing
        self.gripper_bus = None
        self.gripper_action_ready = False
        self.latest_left_gripper_action = 0.0
        self.latest_right_gripper_action = 0.0
        self._init_gripper()

        # 왼손/오른손 각각 Float32 퍼블리셔 생성
        self.left_gripper_pub = self.create_publisher(Float32, "/rby1/left_gripper_state", 10)
        self.right_gripper_pub = self.create_publisher(Float32, "/rby1/right_gripper_state", 10)

        # 2. RBY1 URDF/KDL 모델 로드
        self._load_rby1_model()

        # 3. ROS 구독자 설정
        self.create_subscription(Bool, "/enable_teleop", self.cb_enable_teleop, 10)
        self.create_subscription(Pose, "/vive_tracker_1/pose", self.cb_set1_pose, 10)
        self.create_subscription(Pose, "/vive_tracker_2/pose", self.cb_set2_pose, 10)
        self.create_subscription(
            Float32, "/rby1/left_gripper_action", self.cb_left_gripper_action, 10
        )
        self.create_subscription(
            Float32, "/rby1/right_gripper_action", self.cb_right_gripper_action, 10
        )

        # 4. 제어 루프 타이머 (100Hz)
        self.publish_timer = self.create_timer(0.01, self.control_loop)

    def _init_robot(self):
        self.get_logger().info(f"Connecting to RBY1 at {self.robot_address}...")
        self.robot = rby1_sdk.create_robot_a(self.robot_address)
        self.robot.connect()

        self.robot.set_parameter("joint_position_command.cutoff_frequency", "5")
        self.robot.set_parameter("default.acceleration_limit_scaling", "0.8")

        self.robot.power_on(".*")
        self.robot.servo_on("right_arm_.*|left_arm_.*|torso_.*")
        for arm in ["right", "left"]:
            if not self.robot.set_tool_flange_output_voltage(arm, 12):
                self.get_logger().error(f"Failed to set tool flange output voltage ({arm}) as 12v")
        self.robot.reset_fault_control_manager()
        self.robot.enable_control_manager()

        self.stream = self.robot.create_command_stream()
        self.get_logger().info("Robot connected and control manager enabled.")

    def _init_gripper(self):
        self.get_logger().info("Init gripper...")
        try:
            self.get_logger().info("Initializing gripper...")
            self.gripper_bus = rby1_sdk.DynamixelBus(rby1_sdk.upc.GripperDeviceName)
            
            if not self.gripper_bus.open_port():
                self.get_logger().error("Failed to open port for Gripper DynamixelBus.")
                self.gripper_bus = None
                return
            
            self.gripper_bus.set_baud_rate(2_000_000)
            self.gripper_bus.set_torque_constant([1, 1])
            
            gripper_active = True
            for dev_id in [0, 1]:
                if not self.gripper_bus.ping(dev_id):
                    self.get_logger().error(f"Gripper Dynamixel ID {dev_id} is not active")
                    gripper_active = False
            
            if gripper_active:
                self.get_logger().info("Gripper initialized successfully. Starting homing process...")
                self.gripper_bus.group_sync_write_torque_enable([(dev_id, 1) for dev_id in [0, 1]])
                self._homing_gripper()
                self._enable_gripper_action_control()
            else:
                self.gripper_bus = None
                
        except Exception as e:
            self.get_logger().warning(f"Failed to setup gripper: {e}")
            self.gripper_bus = None

    def _enable_gripper_action_control(self):
        if self.gripper_bus is None:
            self.gripper_action_ready = False
            return

        try:
            self._set_gripper_operating_mode(
                rby1_sdk.DynamixelBus.CurrentBasedPositionControlMode
            )
            self.gripper_bus.group_sync_write_send_torque(
                [(dev_id, 5) for dev_id in [0, 1]]
            )
            self.gripper_action_ready = True
            self.get_logger().info(
                "Gripper action control enabled: /rby1/left_gripper_action, /rby1/right_gripper_action"
            )
        except Exception as error:
            self.gripper_action_ready = False
            self.get_logger().warning(
                f"Failed to enable gripper action control: {error}"
            )

    # 그리퍼 Operating Mode 설정 
    def _set_gripper_operating_mode(self, mode):
        self.gripper_bus.group_sync_write_torque_enable([(dev_id, 0) for dev_id in [0, 1]])
        self.gripper_bus.group_sync_write_operating_mode([(dev_id, mode) for dev_id in [0, 1]])
        self.gripper_bus.group_sync_write_torque_enable([(dev_id, 1) for dev_id in [0, 1]])

    # 그리퍼 Homing 로직 (가동 범위 확인)
    def _homing_gripper(self):
        self._set_gripper_operating_mode(rby1_sdk.DynamixelBus.CurrentControlMode)
        direction = 0
        q = np.array([0, 0], dtype=np.float64)
        prev_q = np.array([0, 0], dtype=np.float64)
        counter = 0
        
        while direction < 2:
            self.gripper_bus.group_sync_write_send_torque(
                [(dev_id, 0.5 * (1 if direction == 0 else -1)) for dev_id in [0, 1]]
            )
            rv = self.gripper_bus.group_fast_sync_read_encoder([0, 1])
            if rv is not None:
                for dev_id, enc in rv:
                    q[dev_id] = enc
            
            self.gripper_min_q = np.minimum(self.gripper_min_q, q)
            self.gripper_max_q = np.maximum(self.gripper_max_q, q)
            
            if np.array_equal(prev_q, q):
                counter += 1
            
            prev_q = q.copy()  # 원본 버그 수정: 참조 복사가 아닌 값 복사로 변경
            
            if counter >= 30:
                direction += 1
                counter = 0
            time.sleep(0.1)

        self.get_logger().info(f"Gripper homing completed. Min: {self.gripper_min_q}, Max: {self.gripper_max_q}")
        
        # Homing 후 그리퍼를 자유롭게 읽을 수 있도록 토크를 끕니다
        # (만약 로봇 제어쪽에서 위치를 계속 유지해야 한다면 이 부분을 주석 처리해야 할 수 있습니다.)
        self.gripper_bus.group_sync_write_torque_enable([(dev_id, 0) for dev_id in [0, 1]])

    def _extract_first_existing_attr(self, obj, names, default=None):
        for name in names:
            if hasattr(obj, name):
                return getattr(obj, name)
        return default

    def _update_current_state(self) -> bool:
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

    def _set_gripper_action(self, dev_id: int, action_value: float):
        if self.gripper_bus is None or not self.gripper_action_ready:
            return

        if not np.isfinite(self.gripper_min_q[dev_id]) or not np.isfinite(
            self.gripper_max_q[dev_id]
        ):
            return

        action_clipped = float(np.clip(action_value, 0.0, 1.0))
        target_enc = action_clipped * (
            self.gripper_max_q[dev_id] - self.gripper_min_q[dev_id]
        ) + self.gripper_min_q[dev_id]

        try:
            self.gripper_bus.group_sync_write_send_position([(dev_id, float(target_enc))])
        except Exception as error:
            self.get_logger().warning(
                f"Failed to send gripper action (id={dev_id}, action={action_clipped}): {error}"
            )

    def cb_left_gripper_action(self, msg: Float32):
        self.latest_left_gripper_action = float(np.clip(msg.data, 0.0, 1.0))
        self._set_gripper_action(dev_id=1, action_value=self.latest_left_gripper_action)

    def cb_right_gripper_action(self, msg: Float32):
        self.latest_right_gripper_action = float(np.clip(msg.data, 0.0, 1.0))
        self._set_gripper_action(dev_id=0, action_value=self.latest_right_gripper_action)

    # [수정된 부분] Homing 값으로 Normalize 처리 추가
    def _publish_gripper_state(self):
        if self.gripper_bus is None:
            return

        try:
            rv = self.gripper_bus.group_fast_sync_read_encoder([0, 1])
            if rv is not None:
                for dev_id, enc in rv:
                    
                    # 0으로 나누어지는 오류(Division by Zero) 방지
                    range_val = self.gripper_max_q[dev_id] - self.gripper_min_q[dev_id]
                    if range_val <= 0:
                        continue

                    # 0.0 ~ 1.0 범위로 정규화 및 클리핑
                    normalized_val = (enc - self.gripper_min_q[dev_id]) / range_val
                    normalized_val = float(np.clip(normalized_val, 0.0, 1.0))

                    msg = Float32()
                    msg.data = normalized_val
                    
                    # ID 0 = Right, ID 1 = Left 매핑
                    if dev_id == 0:
                        self.right_gripper_pub.publish(msg)
                    elif dev_id == 1:
                        self.left_gripper_pub.publish(msg)
                        
        except Exception as e:
            self.get_logger().error(f"Error reading gripper encoder: {e}", throttle_duration_sec=2.0)

    def control_loop(self):
        # 1. 최신 관절 상태 업데이트
        if not self._update_current_state():
            return

        # 1.5. 그리퍼 상태 읽기 및 정규화하여 퍼블리시
        self._publish_gripper_state()

        if not self.enable_teleop or not self.model_loaded:
            return

        if self.test_mode:
            if self.base_ee_left is None:
                self.base_ee_left = self._get_current_ee_frame(
                    self.left_chain, self.left_fk, self.left_joint_names
                )
            if self.base_ee_right is None:
                self.base_ee_right = self._get_current_ee_frame(
                    self.right_chain, self.right_fk, self.right_joint_names
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
            self.right_chain = tree.getChain(self.base_link, self.right_ee_link)
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

    def _positions_to_list(self, positions):
        if positions is None:
            return None
        if isinstance(positions, np.ndarray):
            return positions.tolist()
        return list(positions)

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
        else:
            torso_positions = self.torso_hold_positions

        if torso_positions is None:
            return

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

        try:
            self.stream.send_command(rc)
        except RuntimeError as error:
            if "expired" in str(error).lower():
                self.get_logger().warning(
                    "Command stream expired. Recreating stream and retrying once."
                )
                try:
                    self.stream = self.robot.create_command_stream()
                    self.stream.send_command(rc)
                except Exception as retry_error:
                    self.get_logger().warning(
                        f"Failed after recreating command stream: {retry_error}"
                    )
            else:
                self.get_logger().warning(f"Failed to send command to robot: {error}")
        except Exception as error:
            self.get_logger().warning(f"Failed to send command to robot: {error}")


def main():
    rclpy.init()
    node = TeleopPosePublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if hasattr(node, "gripper_bus") and node.gripper_bus is not None:
                node.gripper_bus.close_port()
                
            if hasattr(node, "robot"):
                node.get_logger().info("Disconnecting from RBY1 robot...")
                node.robot.disconnect()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
from typing import List

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import rby1_sdk

# RBY1 joint names from URDF (24 joints total)
# Order: wheels, torso, right_arm, left_arm, head
DEFAULT_JOINT_NAMES = [
    # Wheels (2)
    'right_wheel', 'left_wheel',
    # Torso (6)
    'torso_0', 'torso_1', 'torso_2', 'torso_3', 'torso_4', 'torso_5',
    # Right Arm (7)
    'right_arm_0', 'right_arm_1', 'right_arm_2', 'right_arm_3',
    'right_arm_4', 'right_arm_5', 'right_arm_6',
    # Left Arm (7)
    'left_arm_0', 'left_arm_1', 'left_arm_2', 'left_arm_3',
    'left_arm_4', 'left_arm_5', 'left_arm_6',
    # Head (2)
    'head_0', 'head_1',
]


class Rby1JointStatePublisher(Node):
    def __init__(self) -> None:
        super().__init__('rby1_joint_state_publisher')

        self.declare_parameter('robot_address', '192.168.0.101:50051')
        self.declare_parameter('output_topic', '/joint_states')
        self.declare_parameter('frame_id', 'vive_teleop')
        self.declare_parameter('publish_rate', 100.0)
        self.declare_parameter('joint_names', [])

        self.robot_address = self.get_parameter(
            'robot_address').get_parameter_value().string_value
        output_topic = self.get_parameter(
            'output_topic').get_parameter_value().string_value
        self.frame_id = self.get_parameter(
            'frame_id').get_parameter_value().string_value
        self.publish_rate = self.get_parameter(
            'publish_rate').get_parameter_value().double_value
        self.configured_joint_names = list(
            self.get_parameter(
                'joint_names').get_parameter_value().string_array_value
        )

        self.publisher = self.create_publisher(JointState, output_topic, 10)
        self.robot = rby1_sdk.create_robot_a(self.robot_address)
        self.robot.connect()

        self.joint_names: List[str] = []

        period = 1.0 / self.publish_rate if self.publish_rate > 0.0 else 0.01
        self.timer = self.create_timer(period, self.publish_joint_state)

        self.get_logger().info(
            f'Connected to {self.robot_address}, publishing JointState on {output_topic} '
            f'at {self.publish_rate:.1f} Hz'
        )

    def _extract_first_existing_attr(self, obj, names, default=None):
        for name in names:
            if hasattr(obj, name):
                return getattr(obj, name)
        return default

    def _resolve_joint_names(self, state, num_positions: int) -> List[str]:
        # Already cached
        if self.joint_names and len(self.joint_names) == num_positions:
            return self.joint_names

        # Priority 1: Use default RBY1 joint names if count matches
        if len(DEFAULT_JOINT_NAMES) == num_positions:
            self.joint_names = list(DEFAULT_JOINT_NAMES)
            return self.joint_names

        # Priority 2: Use configured parameter
        if self.configured_joint_names and len(self.configured_joint_names) == num_positions:
            self.joint_names = list(self.configured_joint_names)
            return self.joint_names

        # Priority 3: Try to extract from state object
        state_joint_names = self._extract_first_existing_attr(
            state,
            ['joint_names', 'joint_name', 'name'],
            default=None,
        )
        if state_joint_names is not None and len(state_joint_names) == num_positions:
            self.joint_names = list(state_joint_names)
            return self.joint_names

        # Fallback: Generate generic names
        self.joint_names = [f'joint_{idx}' for idx in range(num_positions)]
        return self.joint_names

    def publish_joint_state(self) -> None:
        try:
            state = self.robot.get_state()
            position = self._extract_first_existing_attr(
                state,
                ['position', 'joint_position', 'current_position'],
                default=[],
            )
            position = list(position)

            if not position:
                return

            names = self._resolve_joint_names(state, len(position))

            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.frame_id
            msg.name = names
            msg.position = position

            velocity = self._extract_first_existing_attr(
                state,
                ['velocity', 'joint_velocity', 'current_velocity'],
                default=None,
            )
            if velocity is not None:
                velocity = list(velocity)
                if len(velocity) == len(position):
                    msg.velocity = velocity

            effort = self._extract_first_existing_attr(
                state,
                ['torque', 'effort', 'joint_torque', 'current_torque'],
                default=None,
            )
            if effort is not None:
                effort = list(effort)
                if len(effort) == len(position):
                    msg.effort = effort

            self.publisher.publish(msg)
        except Exception as error:
            self.get_logger().warning(
                f'Failed to publish joint states: {error}')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Rby1JointStatePublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if hasattr(node.robot, 'disconnect'):
                node.robot.disconnect()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

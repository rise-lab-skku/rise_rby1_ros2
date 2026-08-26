import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # declare launch arguments
    arg_model_name = DeclareLaunchArgument(
        "model",
        default_value="a",
        description="Model variant appended to 'rby1' (e.g., a or m)",
    )
    arg_model_version = DeclareLaunchArgument(
        "version",
        default_value="1_2",
        description="Model version used in the URDF filename (e.g., 1_0)",
    )
    # set urdf path
    robot_description = Command(
        [
            "cat ",
            FindPackageShare("rby1_description"),
            "/urdf/rby1",
            LaunchConfiguration("model"),
            "/model_v",
            LaunchConfiguration("version"),
            ".urdf",
        ]
    )
    # set parameters
    params = {
        "robot_description": robot_description,
        "use_sim_time": LaunchConfiguration("use_sim_time"),
    }

    return LaunchDescription(
        [
            arg_model_name,
            arg_model_version,
            DeclareLaunchArgument(
                "use_sim_time", default_value="false", description="Use simulated time"
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="screen",
                parameters=[params],
            ),
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                name="joint_state_publisher",
                output="screen",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=[
                    "-d",
                    os.path.join(
                        get_package_share_directory("rby1_description"),
                        "rviz",
                        "robot.rviz",
                    ),
                ],
            ),
        ]
    )

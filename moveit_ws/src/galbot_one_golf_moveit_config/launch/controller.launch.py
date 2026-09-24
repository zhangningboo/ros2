from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder(
        "galbot_one_golf", package_name="galbot_one_golf_moveit_config"
    ).to_moveit_configs()

    return LaunchDescription([
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[str(moveit_config.package_path / "config/ros2_controllers.yaml")],
            # Jazzy subscribes to robot_description, not ~/robot_description.
            # The publisher must supply the MoveIt xacro including ros2_control.
            remappings=[("robot_description", "/robot_description")],
            output="screen",
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(moveit_config.package_path / "launch/spawn_controllers.launch.py")
            ),
        ),
    ])

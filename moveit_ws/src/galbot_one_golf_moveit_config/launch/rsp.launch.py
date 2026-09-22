from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_rsp_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("galbot_one_golf", package_name="galbot_one_golf_moveit_config").to_moveit_configs()
    # rsp: robot_state_publisher
    return generate_rsp_launch(moveit_config)

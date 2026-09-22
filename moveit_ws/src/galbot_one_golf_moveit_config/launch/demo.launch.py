# /opt/ros/jazzy/lib/python3.12/site-packages/moveit_configs_utils/__init__.py
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("galbot_one_golf", package_name="galbot_one_golf_moveit_config").to_moveit_configs()
    # 启动 galbot_one_golf_moveit_config/launch/static_virtual_joint_tfs.launch.py
    # 启动 galbot_one_golf_moveit_config/launch/rsp.launch.py
    # 启动 galbot_one_golf_moveit_config/launch/move_group.launch.py
    # 启动 galbot_one_golf_moveit_config/launch/spawn_controllers.launch.py
    # 启动 galbot_one_golf_moveit_config/launch/moveit_rviz.launch.py
    # 启动 galbot_one_golf_moveit_config/launch/warehouse_db.launch.py

    # 使用 galbot_one_golf_moveit_config/config/ros2_controllers.yaml
    
    # debug:=true
    return generate_demo_launch(moveit_config)

# 每个终端先执行
  source install/setup.zsh
  export ROS_DOMAIN_ID=17
  export LD_LIBRARY_PATH=/opt/ros/jazzy/lib:$LD_LIBRARY_PATH

# 终端 1：RViz，加载包含控制配置的描述
  ros2 launch galbot_one_golf_description display.launch.py \
    publish_joint_states:=false \
    urdf_path:="$(ros2 pkg prefix --share galbot_one_golf_moveit_config)/config/galbot_one_golf.urdf.xacro"

# 终端 2
  ros2 launch galbot_one_golf_moveit_config controller.launch.py

# 终端 3
  ros2 launch galbot_one_golf_moveit_config move_group.launch.py

# 控制器激活后，终端 4
  python3 src/rust_moveit_demo/tests/plan_exec.py
sudo apt install python3-colcon-cargo
python3 -m pip install --break-system-packages -U colcon-ros-cargo
rustup toolchain install stable
sudo apt install ros-jazzy-test-msgs

cd moveit_ws/src
ros2 pkg create \
  --build-type ament_cargo \
  rust_moveit_demo

cd moveit_ws
colcon build --packages-select rust_moveit_demo
source ./install/local_setup.zsh
ros2 pkg list | grep rust_moveit_demo
ros2 run rust_moveit_demo rust_moveit_demo


ros2 launch galbot_one_golf_moveit_config move_group.launch.py
ros2 node list
ros2 action list
ros2 action info /move_action


查询关节坐标
ros2 run tf2_ros tf2_echo base_link left_gripper_tcp_link
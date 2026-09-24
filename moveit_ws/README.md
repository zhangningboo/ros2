https://moveit.picknik.ai/main/doc/examples/urdf_srdf/urdf_srdf_tutorial.html

### URDF (Unified Robot Description Format)
- XML定义文件中，节点的 `safety_controller` 属性 `soft_lower_limit` 和 `soft_upper_limit` 设置的是关节的开合最值
- 测试URDF：`check_urdf moveit_ws/src/galbot_one_golf_description/urdf/galbot_one_golf.urdf`
- 查看URDF连接信息: `urdf_to_graphiz moveit_ws/src/galbot_one_golf_description/urdf/galbot_one_golf.urdf`


### SRDF (Semantic Robot Description Format)
- 可以使用 `MoveIt Setup Assistant` 生成
- 包含 `Move Group` 设置
- 包含更丰富的碰撞检测信息
- 包含一些机器人配置
- 包含一些特定位置的变换关系

### Virtual Joints
- 将机器人的 `base_link` 与 `world` 关联起来
- 移动机器人可以设置一个 `planner` 虚拟关节，将 `base_link` 与 `world` 关联，机器人就可以在 `world` 的平面中移动

### Passive Joints
- 被动关节，无法直接被驱动的关节

### Groups
- 又可称为 `JointGroup` 和 `PlanningGroup`
- 包含不同的 `link` 和 `joint`
- MoveIt 运动规划的主体，不在这个组内的零件，不会被MoveIt计算


### Moveit Setup Assistant

- [使用教程](https://moveit.picknik.ai/main/doc/examples/setup_assistant/setup_assistant_tutorial.html)
- 开始配置
    -  启动配置助手： `ros2 launch moveit_setup_assistant setup_assistant.launch.py`
    - 使用的urdf文件是：`src/galbot_one_golf_description/urdf/galbot_one_golf.urdf`
- 验证配置结果：`ros2 launch galbot_one_golf_moveit_config demo.launch.py`



### 分终端启动模拟规划与执行（ROS 2 Jazzy）

先停止之前的 display、move_group、controller 及 `fake_joint_state.py` 进程。
在工作空间构建配置包：

```bash
colcon build --packages-select galbot_one_golf_description galbot_one_golf_moveit_config
```

每个终端使用相同的 ROS Domain，并加载工作空间（zsh 使用 `setup.zsh`）：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=17
# 若 Galbot SDK 的 libfastcdr 与 ROS 冲突，确保 ROS 库优先：
export LD_LIBRARY_PATH=/opt/ros/jazzy/lib:$LD_LIBRARY_PATH
```

依次在三个终端启动：

```bash
# 1. RViz + robot_state_publisher：必须发布含 ros2_control 的完整描述
ros2 launch galbot_one_golf_description display.launch.py \
  publish_joint_states:=false \
  urdf_path:="$(ros2 pkg prefix --share galbot_one_golf_moveit_config)/config/galbot_one_golf.urdf.xacro"

# 2. 模拟硬件、双臂/夹爪控制器以及 joint_state_broadcaster
ros2 launch galbot_one_golf_moveit_config controller.launch.py

# 3. MoveIt
ros2 launch galbot_one_golf_moveit_config move_group.launch.py
```

确认控制器状态均为 `active` 后请求规划执行：

```bash
ros2 service call /controller_manager/list_controllers controller_manager_msgs/srv/ListControllers '{}'
python3 src/rust_moveit_demo/tests/plan_exec.py
```

此流程使用 `mock_components/GenericSystem` 模拟硬件。初始姿态由
`src/galbot_one_golf_moveit_config/config/initial_positions.yaml` 提供；修改后重新构建配置包并重启上述节点。
不要同时运行 `fake_joint_state.py`：它以 100 Hz 发布固定位置，不能设置控制器内部状态，
还会与 `joint_state_broadcaster` 的执行反馈冲突。当前 YAML 与该脚本的初始值不同（包括腿部），
需要复现脚本姿态时应将相应值写入 YAML。
`gui:=false` 只切换到无界面的 joint_state_publisher；`publish_joint_states:=false` 才会关闭两种发布器。

若 controller_manager 一直提示等待 `robot_description`，先查看该日志之前的错误。
普通 description URDF 没有 `<ros2_control>`，即使话题已收到也会初始化失败。
Jazzy 使用 `robot_description` 订阅名，而非旧版的 `~/robot_description`。

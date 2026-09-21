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
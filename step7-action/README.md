# ros2

## Action通信模型
[![Understanding-ROS2-Actions](./Client-Server-Model.gif)](http://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Actions/Understanding-ROS2-Actions.html)

### 基础环境
```shell
$ pip install rosdepc
$ rosdepc init
$ rosdepc update
```

### 创建工作空间

```shell
$ mkdir -p dev_ws/src
$ cd dev_ws && colcon build && source install/local_setup.bash
```

#### 自定义服务
```shell
$ cd dev_ws/src
$ ros2 pkg create --build-type ament_cmake interface_define_pkg
$ mkdir interface_define_pkg/action
$ touch interface_define_pkg/action/CustomAction.action
# 添加编译配置 CMakeLists.txt
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/AddTowInts.srv"
)
# 添加包配置
  <build_depend>rosidl_default_generators</build_depend>
  <exec_depend>rosidl_default_runtime</exec_depend>
  <member_of_group>rosidl_interface_packages</member_of_group>
$ cd dev_src && colcon build --packages-select interface_define_pkg
$ source install/local_setup.bash
$ ros2 interface package interface_define_pkg
interface_define_pkg/action/CustomAction
$ ros2 interface show interface_define_pkg/action/CustomAction
bool enable  # 目标
---
bool finish  # 结果
---
int32 state  # 反馈
```

#### 测试
```shell
$ cd dev_ws/src
$ ros2 pkg create --build-type ament_python test_action_pkg
$ cd dev_ws
$ # 修改 setup.py 文件，添加以下内容
'console_scripts': [
  'action_server   = test_action_pkg.action_server:main',
  'action_client   = test_action_pkg.action_client:main',
],
$ colcon build
$ source install/local_setup.bash
$ ros2 run test_action_pkg action_server
$ ros2 run test_action_pkg action_client
```
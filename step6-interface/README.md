# ros2

## Interface
- msg
- srv
- action

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
$ mkdir interface_define_pkg/srv
$ touch interface_define_pkg/srv/AddTwoInts.srv
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
interface_define_pkg/srv/CustomService
interface_define_pkg/msg/CustomMsg
$ ros2 interface show interface_define_pkg/srv/CustomService
int64 a
int64 b
string name
---
bool result
```

#### 测试
```shell
$ cd dev_ws/src
$ ros2 pkg create --build-type ament_python test_interface_pkg
$ cd dev_ws
$ # 修改 setup.py 文件，添加以下内容
'console_scripts': [
  'pub_custom_msg   = test_interface_pkg.pub_custom_msg:main',
  'sub_custom_msg   = test_interface_pkg.sub_custom_msg:main',
  'service_server   = test_interface_pkg.service_server:main',
  'service_client   = test_interface_pkg.service_client:main',
],
$ colcon build
$ source install/local_setup.bash
$ ros2 run test_interface_pkg pub_custom_msg
$ ros2 run test_interface_pkg sub_custom_msg

$ ros2 run test_interface_pkg service_server
$ ros2 run test_interface_pkg service_client
```
# ros2

## Service
- 客户端/服务器模型
- 服务端唯一，客户端可以不唯一
- 同步通信
- .srv定义请求和响应格式

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
$ ros2 pkg create --build-type ament_cmake service_define_pkg
$ mkdir service_define_pkg/srv
$ touch service_define_pkg/srv/AddTwoInts.srv
# 添加编译配置 CMakeLists.txt
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/AddTowInts.srv"
)
# 添加包配置
  <build_depend>rosidl_default_generators</build_depend>
  <exec_depend>rosidl_default_runtime</exec_depend>
  <member_of_group>rosidl_interface_packages</member_of_group>
$ cd dev_src && colcon build --packages-select service_define_pkg
$ source install/local_setup.bash
$ ros2 interface show service_define_pkg/srv/AddTwoInts
int64 a
int64 b
---
int64 sum
```

#### 服务端/客户端
```shell
$ cd dev_ws/src
$ ros2 pkg create --build-type ament_python service_pkg
$ cd dev_ws
$ # 修改 setup.py 文件，添加以下内容
'console_scripts': [
    'server   = service_pkg.server:main',
    'client   = service_pkg.client:main',
],
$ colcon build
$ source install/local_setup.bash
$ ros2 run service_pkg server
$ ros2 run service_pkg client
```

#### service查询
```shell
$ ros2 service --help
$ ros2 service list
```
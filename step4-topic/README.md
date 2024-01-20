# ros2

## Topic
- 发布订阅模式
- 订阅者、发布者不一定唯一
- 异步通信
- .msg定义消息格式

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

#### 创建包
```shell
$ cd dev_ws/src
$ ros2 pkg create --build-type ament_python topic_pkg
$ cd dev_ws
$ # 修改 setup.py 文件，添加以下内容
'console_scripts': [
    'node_helloworld    = hello_pkg.node_helloworld:main',
    'node_objectdetect  = hello_pkg.node_objectdetect:main',
],
$ colcon build
$ source install/local_setup.bash
$ ros2 run topic_pkg helloworld_pub
$ ros2 run topic_pkg helloworld_sub

$ ros2 run topic_pkg image_source_pub
$ ros2 run topic_pkg image_source_sub
```

#### ROS相机驱动
```shell
$ sudo apt install ros-humble-usb-cam
$ ros2 run usb_cam usb_cam_node_exe
```
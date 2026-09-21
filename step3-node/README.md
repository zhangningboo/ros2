# ros2

### 基础环境
```shell
$ sudo apt install python3-pip
$ sudo apt install python3-rosdep2
$ pip3 install colcon-cargo colcon-ros-cargo
$ sudo apt install ros-humble-example-interfaces ros-humble-test-msgs
```

### 创建工作空间

```shell
$ mkdir -p dev_ws/src
$ cd dev_ws
$ colcon build
$ source install/local_setup.bash
```

### 创建python包
```shell
$ cd dev_ws/src
$ ros2 pkg create --build-type ament_python hello_pkg
$ cd dev_ws
$ # 修改 setup.py 文件，添加以下内容
'console_scripts': [
    'node_helloworld    = hello_pkg.node_helloworld:main',
    'node_objectdetect  = hello_pkg.node_objectdetect:main',
],
$ colcon build
$ source install/local_setup.bash
$ ros2 run hello_pkg node_helloworld
$ ros2 run hello_pkg node_objectdetect
```
### 创建rust包
```shell
$ ros2 pkg create --build-type ament_cargo hello_pkg_rs
$ colcon build
$ source install/local_setup.bash
$ ros2 run hello_pkg_rs hello_pkg_rs
```

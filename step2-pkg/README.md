# ros2


### 基础环境
step3-node```shell
$ pip3 install rosdepc
$ pip3 install colcon-cargo
$ sudo rosdepc init
$ rosdepc update
```
### 创建工作空间

```shell
$ mkdir -p dev_ws/src
$ cd dev_ws
$ colcon build
$ source install/local_setup.bash
# or
$ source install/local_setup.zsh
```

#### 创建包
```shell
$ cd dev_ws/src
$ ros2 pkg create --build-type ament_cmake cpp_pkg
$ ros2 pkg create --build-type ament_python py_pkg
$ ros2 pkg create --build-type ament_cargo rs_pkg
$ cd ..
$ colcon build
$ source install/local_setup.bash
# or
$ source install/local_setup.zsh
```
# ros2


### 基础环境
```shell
$ pip install rosdepc
$ rosdepc init
$ rosdepc update
```
### 创建工作空间

```shell
$ mkdir -p dev_ws/src
$ cd dev_ws
$ colcon build
$ source install/local_setup.bash
```

#### 创建包
```shell
$ cd dev_ws/src
$ ros2 pkg create --build-type ament_cmake cpp_pkg
$ ros2 pkg create --build-type ament_python py_pkg
```
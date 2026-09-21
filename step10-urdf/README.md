# ros2


### 基础环境
```shell
$ sudo apt install python3-pip
$ sudo apt install python3-rosdep2
$ pip install rosdepc
$ rosdepc init
$ rosdepc update
$ rosdepc install urdf_launch
```

#### 创建包
```shell
$ sudo apt install ros-humble-urdf-launch
$ cd dev_ws/src
$ git submodule update --remote
# or
$ git clone https://github.com/ros/urdf_tutorial.git -b ros2 
$ cd ..
$ colcon build
$ source install/local_setup.bash
# or
$ source install/local_setup.zsh
$ ros2 launch urdf_tutorial display.launch.py model:=urdf/01-myfirst.urdf
```
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
$ cd dev_ws && colcon build && source install/local_setup.bash
```

#### 自定义服务
```shell
$ cd dev_ws/src
$ ros2 pkg create --build-type ament_python test_param_pkg
$ cd dev_ws
$ # 修改 setup.py 文件，添加以下内容
'console_scripts': [
  'param_operator = test_param_pkg.param_operator:main',
],
$ colcon build
$ source install/local_setup.bash
$ ros2 run test_param_pkg param_operator
```

### 命令使用 `ros2 param <option>`
```shell
$ ros2 run turtlesim turtlesim_node
$ ros2 param list
/turtlesim:
  background_b
  background_g
  background_r
  qos_overrides./parameter_events.publisher.depth
  qos_overrides./parameter_events.publisher.durability
  qos_overrides./parameter_events.publisher.history
  qos_overrides./parameter_events.publisher.reliability
  use_sim_time
$ ros2 param describe /turtlesim back
ground_b
Parameter name: background_b
  Type: integer
  Description: Blue channel of the background color
  Constraints:
    Min value: 0
    Max value: 255
    Step: 1
$ ros2 param get /turtlesim background_b
Integer value is: 255
$ ros2 param set /turtlesim background_b 100
Set parameter successful
$ ros2 param dump /turtlesim
$ ros2 param dump /turtlesim > turtlesim_param.yaml
$ ros2 param load /turtlesim turtlesim_param.yaml
```
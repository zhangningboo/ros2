## ROS2 基础命令

#### 启动 turtlesim 节点
```shell
$ ros2 run turtlesim turtlesim_node
```

#### 启动 turtle_teleop_key 节点
```shell
$ ros2 run turtlesim turtle_teleop_key
```

#### 查看当前发布的节点
```shell
ubuntu@ningboo:~$ ros2 node list
/teleop_turtle
/turtlesim
ubuntu@ningboo:~$ ros2 node info /teleop_turtle
/teleop_turtle
  Subscribers:
    /parameter_events: rcl_interfaces/msg/ParameterEvent
  Publishers:
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /rosout: rcl_interfaces/msg/Log
    /turtle1/cmd_vel: geometry_msgs/msg/Twist
  Service Servers:
    /teleop_turtle/describe_parameters: rcl_interfaces/srv/DescribeParameters
    /teleop_turtle/get_parameter_types: rcl_interfaces/srv/GetParameterTypes
    /teleop_turtle/get_parameters: rcl_interfaces/srv/GetParameters
    /teleop_turtle/list_parameters: rcl_interfaces/srv/ListParameters
    /teleop_turtle/set_parameters: rcl_interfaces/srv/SetParameters
    /teleop_turtle/set_parameters_atomically: rcl_interfaces/srv/SetParametersAtomically
  Service Clients:

  Action Servers:

  Action Clients:
    /turtle1/rotate_absolute: turtlesim/action/RotateAbsolute
ubuntu@ningboo:~$
```

#### 查询话题
```shell
ubuntu@ningboo:~$ ros2 topic list
/parameter_events
/rosout
/turtle1/cmd_vel
/turtle1/color_sensor
/turtle1/pose
ubuntu@ningboo:~$ ros2 topic echo /turtle1/pose
```

#### 转圈圈
```shell
ubuntu@ningboo:~$ ros2 topic pub --rate 1 /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear:{x: 2.0, y: 0.0, z: 0.0}, angular:{x: 0.0, y: 0.0, z: 1.8}}"
```
![Alt text](image.png)
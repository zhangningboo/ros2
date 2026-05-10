# ros2

### 常用命令
| 命令| 作用 |
| --- | --- |
| ros2 run turtlesim turtlesim_node   | 启动节点    |
| ros2 node list | 查询当前发布的节点 |
| ros2 node info /teleop_turtle | 查询节点详情 |
| ros2 topic list | 查询话题 |
| ros2 topic echo /turtle1/pose | 查看话题数据(不停刷新) |
| ros2 topic pub --rate 1 /turtle1/cmd_vel geometry_msgs/msg/Twist | 发布数据 |

### 创建工作空间

```shell
$ mkdir step2/src
```
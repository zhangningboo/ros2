# ros2


### 基础环境
```shell
$ pip install rosdepc
$ rosdepc init
$ rosdepc update
```

## ROS DDS

```shell
$ mkdir -p dev_ws/src
$ cd dev_ws && colcon build && source install/local_setup.bash
```
### [ROS_DOMAIN_ID](https://docs.ros.org/en/rolling/Concepts/Intermediate/About-Domain-ID.html)
- DDS基于Domain ID在一个物理网络内切分为若干逻辑网络。
- 在同一域（domain）中的ROS 2节点可以被自由发现并通信，在不同域中则不能互通。
- 所有的ROS 2节点默认使用domain ID 0。
- 不同的组使用不同的domain ID。
- ROS_DOMAIN_ID有两种（short version / long version
  - short version: [0, 101]。
  - long version: [0, 232]。
  - 默认情况下，ROS 2节点使用short version。
- 设置ROS_DOMAIN_ID: `export ROS_DOMAIN_ID=1`。
- 环境变量`RMW_IMPLEMENTATION`: 设置在运行ROS 2应用程序时选择要使用的RMW实现。

```shell
$ apt search rti-connext-dds
Sorting... Done
Full Text Search... Done
ros-humble-rti-connext-dds-cmake-module/jammy 0.11.2-1jammy.20231117174715 amd64
  Helper module to provide access to RTI products like Connext DDS Professional
ros-iron-rti-connext-dds-cmake-module/jammy 0.14.1-1jammy.20230908.160947amd64
  Helper module to provide access to RTI products like Connext DDS Professional
ros-rolling-rti-connext-dds-cmake-module/jammy 0.18.0-1jammy.20231004144426 amd64
  Helper module to provide access to RTI products like Connext DDS Professional
rti-connext-dds-6.0.1/jammy 6.0.1.25-1 amd64
  real-time messaging and integration platform.
ubuntu@ningboo:$ sudo apt install rti-connext-dds-6.0.1
...
export RTI_LICENSE_FILE=/opt/rti.com/rti_connext_dds-6.0.1/rti_license.dat
export RMW_IMPLEMENTATION=rmw_connext_cpp
$ colcon build
```

### Qos/服务质量
- 对数据如何进行底层的传输进行配置
- QoS策略
  - History：
    - 保留近期记录(Keep last)：缓存最多N条记录，可通过队列长度选项来配置。
    - 保留所有记录(Keep all)：缓存所有记录，但受限于底层中间件可配置的最大资源。
  - Depth：队列深度(Size of the queue)，只能与Keep last配合使用。
  - Reliability：
    - BestEffort：尽力而为，不保证消息的传递。
    - Reliable：可靠，保证消息的传递。反复重传以保证数据成功传输。
  - Durability：
    - TransientLocal：临时，仅在当前节点有效。发布器为晚连接(late-joining)的订阅器保留数据。
    - Volatile：易失，在当前节点有效，但重启后丢失。不保留任何数据。


与ROS1的比较，Qos有以下好处：
- ROS2的 History和Depth结合起来类似于ROS的队列大小功能
- ROS2的Reliability取Best-effort类似于Ros1的UDPROS(仅 roscpp包含此功能),取 Reliable类似于ROS1的TCPROS
- ROS2的Durability和队列深度为1的 Depth结合起来类似于ROS 1中的latching订阅器

#### 使用
```shell  
$ ros2 topic pub /chatter std_msgs/msg/Int32 "data: 11" --qos-
--qos-depth        --qos-durability   --qos-history      --qos-profile      --qos-reliability
$ ros2 topic pub /chatter std_msgs/msg/Int32 "data: 11" --qos-reliability 
best_effort     reliable        system_default  unknown 
$ ros2 topic pub /chatter std_msgs/msg/Int32 "data: 11" --qos-reliability best_effort
# 订阅端
$ ros2 topic echo /chatter  --qos-reliability best_effort
$ ros2 topic info /chatter --verbose
Type: std_msgs/msg/Int32

Publisher count: 1

Node name: _ros2cli_13273
Node namespace: /
Topic type: std_msgs/msg/Int32
Endpoint type: PUBLISHER
GID: 01.0f.39.fe.d9.33.ff.bf.01.00.00.00.00.00.05.03.00.00.00.00.00.00.00.00
QoS profile:
  Reliability: BEST_EFFORT
  History (Depth): UNKNOWN
  Durability: TRANSIENT_LOCAL
  Lifespan: Infinite
  Deadline: Infinite
  Liveliness: AUTOMATIC
  Liveliness lease duration: Infinite

Subscription count: 0
```

```shell
$ cd dev_ws/src
$ ros2 pkg create --build-type ament_python test_qos_pkg
$ cd dev_ws
$ # 修改 setup.py 文件，添加以下内容
'console_scripts': [
    'helloworld_pub   = test_qos_pkg.helloworld_pub:main',
    'helloworld_sub   = test_qos_pkg.helloworld_sub:main',
],
$ colcon build
$ source install/local_setup.bash
$ ros2 run test_qos_pkg helloworld_pub
$ ros2 run test_qos_pkg helloworld_sub
```
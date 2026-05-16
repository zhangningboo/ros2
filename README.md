# ros2

### ros2包安装
```shell
$ sudo apt install ros-humble-ros2run
$ sudo apt install ros-humble-turtlesim
$ sudo apt install python3-pip
$ sudo apt install python3-rosdep2
$ pip3 install colcon-cargo colcon-ros-cargo
$ sudo apt install ros-humble-example-interfaces ros-humble-test-msgs
```

### python uv
```shell
$ export UV_DOWNLOAD_PROXY="https://gh-proxy.com"
$ export UV_PYPI_MIRROR="https://mirrors.ustc.edu.cn/pypi/simple"
$ curl -LsSf https://astral.sh/uv/install.sh | sh

$ uv venv --python 3.10 --seed --managed-python
$ source .venv/bin/activate
```

### rust
```shell
# 用于更新 toolchain
$ export RUSTUP_DIST_SERVER=https://mirrors.ustc.edu.cn/rust-static
# 用于更新 rustup
$ export RUSTUP_UPDATE_ROOT=https://mirrors.ustc.edu.cn/rust-static/rustup
$ curl --proto '=https' --tlsv1.2 -sSf https://mirrors.ustc.edu.cn/misc/rustup-install.sh | sh
```

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
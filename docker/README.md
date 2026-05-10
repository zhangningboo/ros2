### 编译镜像
```shell
$ podman build -f DockerfileMac -t ros2-humble-amd64:ubuntu22.04
$ podman image ls
REPOSITORY                                                         TAG                     IMAGE ID      CREATED         SIZE
localhost/ros2-humble-amd64                                        ubuntu22.04             b2c38c909c84  39 minutes ago  5.82 GB
```

### 启动容器
- mac宿主机
    ```shell
    # mac上
    $ export DISPLAY=:0
    # 1. 确保 XQuartz 正在运行
    $ open -a XQuartz
    
    # 2. 在 XQuartz 偏好设置中，确保允许网络连接
    # XQuartz -> Preferences -> Security -> 勾选 "Allow connections from network clients"

    # 3. 重启 XQuartz 使设置生效
    # 右键点击 XQuartz 图标 -> Quit，然后重新打开
    # 4. 添加授权
    $ xhost +
    ```
- 启动容器
    - podman
    ```shell
    $ MAC_IP=$(ifconfig en0 | grep inet | awk '$1=="inet" {print $2}')
    $ podman run -it --rm \
        -e DISPLAY=host.containers.internal:0 \
        -v $HOME/.Xauthority:/root/.Xauthority \
        -p 5900:5900 \
        --name ros2-humble-amd64-ubuntu22.04 \
        registry.cn-hangzhou.aliyuncs.com/zhangningboo/linux_amd64_ros2_humble:ubuntu22.04 \
        zsh
    # 容器内启动小乌龟
    $ ros2 run turtlesim turtlesim_node
    ```
    - podman vnc
    ```shell
    $ podman run -it --rm \
        -e DISPLAY=host.containers.internal:0 \
        -v $HOME/.Xauthority:/root/.Xauthority \
        -p 5900:5900 \
        --name ros2-humble-amd64-ubuntu22.04-vnc \
        registry.cn-hangzhou.aliyuncs.com/zhangningboo/linux_amd64_ros2_humble:ubuntu22.04 \
        zsh
    $ sudo apt-get update
	$ sudo apt-get install -y xvfb x11vnc openbox
    # 将 DISPLAY 环境变量指向虚拟显示器
	$ export DISPLAY=:1
    # 声明软件渲染和运行时目录
	$ export LIBGL_ALWAYS_SOFTWARE=1
	$ export XDG_RUNTIME_DIR=/tmp/runtime-root
    $ x11vnc -display :1 -forever -rfbport 5900 -passwd 123456 &
    # 在mac上，打开 Finder 中前往 -> 连接服务器，地址：vnc://localhost:5900
    # 回到容器
    $ rviz2
    ```
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
        --name ros2-humble-amd64-ubuntu22.04 \
        localhost/ros2-humble-amd64:ubuntu22.04 zsh
    # 容器内启动小乌龟
    $ ros2 run turtlesim turtlesim_node
    ```
    - docker
        ```shell
        $ docker run -itd --network=host --privileged --group-add video --gpus=all --isolation=process --name ros2-humble-v1 ros2-humble-v1 /bin/zsh
        $ docker run -itd --privileged -e DISPLAY=${REPLACE_YOUR_IP}:0.0 --shm-size 16G --name ros2-humble-v1 ros2-humble-v1 /bin/zsh
        $ docker run -itd --privileged -e DISPLAY=192.168.3.2:0.0 --shm-size 16G --name ros2-humble-v1 ros2-humble-v1 /bin/zsh
        ```
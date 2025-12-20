#!/bin/bash
# 🚀 启动 ROS2 Foxy 开发容器（改进版）
# 说明：
# 1. 保留原始逻辑和结构
# 2. 新增 --network host 和 --privileged 以支持 Jetson 硬件访问
# 3. 可选添加 --device 参数映射 USB-CAN

# 自动获取宿主机的 docker0 网卡 IP 作为代理主机
PROXY_HOST=$(ip addr show docker0 | grep "inet " | awk '{print $2}' | cut -d/ -f1)
PROXY_PORT=7897

# 容器名称（你可以改）
CONTAINER_NAME=ros2_foxy_container

# 镜像名称（与你 build 时一致）
# ⚠️ 请确保和 Dockerfile 构建时的镜像名一致
IMAGE_NAME=ros2_foxy_dev:v2

# 挂载宿主机 ROS2 工作空间路径
WORKSPACE=~/docker_ros2_foxy/workspace

# 检查容器是否已存在
if [ "$(docker ps -aq -f name=$CONTAINER_NAME)" ]; then
    echo "🔁 容器已存在，正在启动..."
    docker start -ai $CONTAINER_NAME
else
    echo "🚀 容器不存在，正在创建新容器..."

    # ---------------------------
    # 参数说明：
    # --privileged    🔑 允许访问 Jetson 硬件（CAN、USB、GPIO 等）
    # --network host  🌐 让 ROS2 DDS 通信不被隔离
    # --device        ⚙️ （可选）映射 USB-CAN 转接器
    # -v              💾 挂载宿主机工作空间到容器内
    # -e              🌍 设置 http/https 代理环境变量（如使用 Clash）
    # ---------------------------

    docker run -it \
        --name $CONTAINER_NAME \
        --runtime nvidia \
        --privileged \
        --network host \
	--device /dev/ttyUSB0 \
        -v $WORKSPACE:/home/rosdev/ros2_ws \
	-e DISPLAY=$DISPLAY \
       	-e QT_X11_NO_MITSHM=1 \
	-v /tmp/.X11-unix:/tmp/.X11-unix \
	-e http_proxy="http://$PROXY_HOST:$PROXY_PORT" \
        -e https_proxy="http://$PROXY_HOST:$PROXY_PORT" \
        $IMAGE_NAME \
        bash
fi

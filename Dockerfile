# 使用官方的 ROS2 Foxy 基础镜像（基于 Ubuntu 20.04）
FROM ros:foxy-ros-base

# 安装常用工具与依赖
RUN apt-get update && apt-get install -y \
    sudo \
    git \
    curl \
    wget \
    vim \
    nano \
    build-essential \
    cmake \
    # Python 相关
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-serial \
    python3-dev \
    # CAN 总线工具（Hunter AGV 需要）
    can-utils \
    # C++ 库依赖
    libasio-dev \
    libyaml-cpp-dev \
    libboost-all-dev \
    libpcl-dev \
    libpcap-dev \
    libeigen3-dev \
    # OpenMP 支持（FAST-LIO 加速）
    libomp-dev \
    # SSL 证书
    ca-certificates \
    # ROS2 Foxy 包
    ros-foxy-teleop-twist-keyboard \
    ros-foxy-rviz2 \
    ros-foxy-pcl-conversions \
    ros-foxy-builtin-interfaces \
    ros-foxy-rosidl-default-generators \
    ros-foxy-tf2-ros \
    ros-foxy-tf2-geometry-msgs \
    && rm -rf /var/lib/apt/lists/*

# 初始化 rosdep（无论执行多少次都不会出错）
RUN rosdep init || true && rosdep update || true

# 创建一个普通用户（避免直接使用 root）
ARG USERNAME=rosdev
ARG USER_UID=1000
ARG USER_GID=1000

RUN groupadd --gid $USER_GID $USERNAME \
 && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME -s /bin/bash \
 && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
 && chmod 0440 /etc/sudoers.d/$USERNAME

USER $USERNAME
WORKDIR /home/$USERNAME

# 设置 ROS2 工作区
RUN mkdir -p /home/$USERNAME/ros2_ws/src
ENV ROS2_WS=/home/$USERNAME/ros2_ws

# 自动加载 ROS2 环境
RUN echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc \
 && echo "export PS1='(ros2_foxy) \\u@\\h:\\w\\$ '" >> ~/.bashrc \
 && echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc

# ⚙️ 你可以在容器启动后自己执行：
#cd ~/ros2_ws/src
#git clone https://github.com/westonrobot/ugv_sdk.git
#cd ~/ros2_ws && colcon build

# 默认启动 bash
CMD ["/bin/bash"]

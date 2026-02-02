# FAST-LIO2 ROS2 工作空间# FAST_LIO_ROS2

FAST-LIO2 workspace and drivers

基于 ROS2 Foxy 的 FAST-LIO2 SLAM 系统完整工作空间，集成激光雷达驱动、IMU 驱动和建图算法。

说明（简体中文）

## 📋 项目简介

以下包含两种在本仓库环境下运行 FAST-LIO2 的方法：

本项目是一个完整的 ROS2 工作空间，用于运行 FAST-LIO2（Fast LiDAR-Inertial Odometry）实时 SLAM 系统。FAST-LIO2 是一种高效的激光雷达-惯性里程计算法，能够在资源受限的平台上实现实时定位与建图。1) 手动按步骤启动（分开启动激光雷达驱动、IMU 驱动与 FAST-LIO 节点）

2) 使用仓库内的一键脚本 `start_all.sh` 启动（更方便）

### 主要特性

重要前置项

- ✅ 实时 3D 点云建图- 本仓库基于 ROS2（你的环境是 Foxy），请先确保已安装并配置好 ROS2 环境。

- ✅ 激光雷达-IMU 紧耦合融合- 若你在当前机器上尚未构建过工作区，先在工作区根目录运行一次构建：

- ✅ 支持多种激光雷达型号（Livox、Velodyne、Lslidar等）

- ✅ 低计算资源消耗，适合嵌入式平台```bash

- ✅ 一键启动脚本，简化操作流程# 进入工作区

- ✅ Docker 容器化支持（如果在容器环境中运行）cd /home/rosdev/ros2_ws



---# （可选）将源码更新或检查修改

# git status

## 🛠 硬件要求

# 构建（只需在第一次或修改源码后运行）

### 必需硬件colcon build --symlink-install

```

| 硬件类型 | 型号/规格 | 说明 |

|---------|----------|------|- 每次启动前请 source ROS2 与本工作区的 setup 文件：

| **激光雷达** | Lslidar C16 | 16线机械式激光雷达，网络连接（192.168.1.200） |

| **IMU** | HandsFree IMU | 9轴 IMU，串口连接（/dev/ttyUSB0，921600波特率） |```bash

| **计算平台** | Jetson/PC | 推荐 Jetson Xavier/Orin 或 x86 PC（4核+，8GB+ RAM） |source /opt/ros/foxy/setup.bash

source /home/rosdev/ros2_ws/install/local_setup.bash

### 可选硬件```



- 移动底盘：Hunter AGV（已集成驱动包）方法一 — 手动逐个启动（推荐用于调试）

- GPS/RTK：可扩展全局定位- 适用于需要观察单个节点日志或分步调试的场景。



---典型启动顺序：

1. 启动激光雷达驱动（根据型号选择对应 launch）：

## 📦 软件架构```bash

# 例如 Lslidar C16 驱动

### 功能包说明ros2 launch lslidar_driver lslidar_c16_launch.py

```

```

ros2_ws/2. 启动 IMU 驱动：

├── src/```bash

│   ├── FAST_LIO_ROS2/          # 核心SLAM算法包# 示例：handsfree_imu_ros2 的 launch

│   │   ├── src/                # C++源码（laserMapping.cpp等）ros2 launch handsfree_imu_ros2 imu.launch.py

│   │   ├── config/             # 配置文件（c16.yaml等）```

│   │   ├── launch/             # 启动文件

│   │   └── rviz/               # RViz可视化配置3. 启动 FAST-LIO mapping 节点（含参数与 RViz，可按需开启）：

│   │```bash

│   ├── Lslidar_ROS2_driver/    # 镭神激光雷达驱动ros2 launch FAST_LIO_ROS2 mapping.launch.py use_rviz:=true

│   │   └── lslidar_driver/     # C16等型号驱动```

│   │

│   ├── handsfree_imu_ros2/     # HandsFree IMU ROS2驱动运行提示与常见问题：

│   │   ├── imu_node.py         # IMU数据解析与发布节点- 如果控制台输出 "No Effective Points!"，可能是机器人静止或点云被过度滤波，请移动平台并检查 `config/*.yaml` 中的滤波尺寸参数（例如 `filter_size_surf`、`point_filter_num` 等）。

│   │   └── launch/             # IMU启动文件- 如果看到 PCL 提示 `Leaf size is too small`，请适当调大体素网格 (voxel) 的 leaf size。

│   │- 确认话题 `/cx/lslidar_point_cloud`（点云）与 `/imu/data`（IMU）有数据：

│   ├── hunter_ros2/            # Hunter移动底盘驱动（可选）```bash

│   │   └── hunter_base/        # 底盘控制接口ros2 topic hz /cx/lslidar_point_cloud

│   │ros2 topic hz /imu/data

│   └── ugv_sdk/                # 通用无人车SDK（Hunter依赖）```

│

├── start_all.sh                # 一键启动脚本（推荐）方法二 — 使用一键脚本 `start_all.sh`（推荐快速启动）

├── start_fastlio_full.sh       # 完整启动脚本（含RViz）- 仓库根目录已有 `start_all.sh`，脚本会按顺序启动雷达驱动、IMU 驱动和 FAST-LIO（可选 RViz）。

└── README.md                   # 本文档- 使用前同样需要 source 环境：

``````bash

source /opt/ros/foxy/setup.bash

### 各功能包详细说明source /home/rosdev/ros2_ws/install/local_setup.bash

chmod +x /home/rosdev/ros2_ws/start_all.sh

#### 1. FAST_LIO_ROS2（核心建图包）/home/rosdev/ros2_ws/start_all.sh

```

**作用**：实现激光雷达-IMU融合的实时SLAM，输出高精度位姿和点云地图。

- 脚本会在当前终端启动多个子进程；要停止可用 `pkill -f <node_name>` 或在另一个终端执行：

**关键文件**：```bash

- `src/laserMapping.cpp`：主算法实现（点云处理、IMU预积分、KD-Tree地图更新）pkill -f fast_lio || true

- `config/c16.yaml`：Lslidar C16配置（话题映射、滤波参数、外参等）pkill -f lslidar || true

- `launch/mapping.launch.py`：启动文件（可选是否启动RViz）pkill -f imu_node || true

```

**发布话题**：

- `/Odometry`：实时里程计（nav_msgs/Odometry）小结与建议

- `/cloud_registered`：配准后的点云地图- 开发/调试时优先用 方法一（分步启动、逐个检查话题与日志）；演示/部署时可用 方法二（脚本自动化）。

- `/path`：运动轨迹- 启动后若无法建图，请确保平台运动且传感器数据频率稳定。

- 我可以把更多运行示例（launch 参数说明、config 模板、回放 bag）加入本 README，告诉我你需要哪些示例。

**订阅话题**：

- `/cx/lslidar_point_cloud`：激光雷达点云（sensor_msgs/PointCloud2）
- `/imu/data`：IMU数据（sensor_msgs/Imu）

#### 2. Lslidar_ROS2_driver（雷达驱动包）

**作用**：连接镭神C16激光雷达硬件，解析UDP数据包并发布ROS2点云消息。

**关键参数**（在launch文件中）：
- `device_ip`：雷达IP地址（默认192.168.1.200）
- `frame_id`：坐标系名称（默认`laser_link`）
- `scan_topic`：发布话题名（`/cx/lslidar_point_cloud`）

**使用方法**：
```bash
ros2 launch lslidar_driver lslidar_c16_launch.py
```

#### 3. handsfree_imu_ros2（IMU驱动包）

**作用**：读取HandsFree IMU串口数据，解析姿态/角速度/加速度并发布。

**已修复问题**：
- 修正了数据包解析逻辑（0x2c和0x14包类型）
- 解决了IMU不发布数据的bug

**关键参数**：
- `port`：串口设备（默认`/dev/ttyUSB0`）
- `baud_rate`：波特率（921600）

**发布话题**：
- `/imu/data`：IMU数据（sensor_msgs/Imu）
- `/imu/mag`：磁力计数据

#### 4. hunter_ros2 & ugv_sdk（底盘驱动，可选）

**作用**：控制Hunter AGV移动底盘（如果你的系统使用该底盘）。

**说明**：如果你只做建图不需要控制底盘，可以忽略此包。

---

## 🚀 快速开始

### 环境要求

- **操作系统**：Ubuntu 20.04（推荐）或兼容Docker容器
- **ROS版本**：ROS2 Foxy
- **依赖库**：PCL、Eigen、livox_ros_driver2（如果使用Livox雷达）

### 1. 构建工作空间

首次使用或修改代码后需要编译：

```bash
cd /home/rosdev/ros2_ws

# 安装依赖（首次）
rosdep install --from-paths src --ignore-src -r -y

# 编译（使用symlink可避免重复安装Python包）
colcon build --symlink-install

# Source环境
source install/setup.bash
```

### 2. 配置硬件连接

#### 激光雷达网络配置

确保你的电脑与雷达在同一网段：

```bash
# 检查网络接口
ip addr show

# 配置静态IP（示例，根据实际网卡调整）
sudo ip addr add 192.168.1.102/24 dev eth0

# 测试连通性
ping 192.168.1.200
```

#### IMU串口权限

```bash
# 查看串口设备
ls -l /dev/ttyUSB*

# 添加当前用户到dialout组（避免sudo）
sudo usermod -aG dialout $USER
# 注销并重新登录生效

# 或临时授权
sudo chmod 666 /dev/ttyUSB0
```

### 3. 运行系统

提供两种启动方式：

#### 方法一：使用一键脚本（推荐）

最简单的启动方式，自动按顺序启动所有节点：

```bash
# Source环境
source /opt/ros/foxy/setup.bash
source ~/ros2_ws/install/setup.bash

# 运行启动脚本
./start_all.sh
```

**脚本功能**：
- ✅ 自动检查雷达和IMU设备连接
- ✅ 按顺序启动雷达驱动 → IMU驱动 → FAST-LIO
- ✅ 捕获Ctrl+C自动清理所有进程
- ✅ 彩色输出，状态一目了然

停止系统：按 `Ctrl+C`

#### 方法二：手动分步启动（调试用）

适合需要观察每个节点日志的场景，推荐开三个终端：

**终端1 - 启动激光雷达驱动**：
```bash
source /opt/ros/foxy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch lslidar_driver lslidar_c16_launch.py
```

**终端2 - 启动IMU驱动**：
```bash
source /opt/ros/foxy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch handsfree_imu_ros2 imu.launch.py
```

**终端3 - 启动FAST-LIO**：
```bash
source /opt/ros/foxy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch fast_lio mapping.launch.py config_file:=c16.yaml rviz:=true
```

### 4. 验证系统运行

在新终端检查话题数据：

```bash
# 检查所有话题
ros2 topic list

# 检查雷达数据频率（应约10Hz）
ros2 topic hz /cx/lslidar_point_cloud

# 检查IMU数据频率（应约100-200Hz）
ros2 topic hz /imu/data

# 查看里程计输出
ros2 topic echo /Odometry
```

---

## 🔧 常见问题排查

### 问题1：控制台输出 "No Effective Points!"

**原因**：
- 机器人静止不动，导致特征点不足
- 点云被过度滤波，有效点太少

**解决方法**：
1. **移动平台**：FAST-LIO需要运动才能初始化和建图
2. **调整滤波参数**：编辑 `src/FAST_LIO_ROS2/config/c16.yaml`
   ```yaml
   filter_size_surf: 0.5          # 增大值减少滤波强度
   filter_size_map: 0.5           # 增大值减少滤波强度
   point_filter_num: 1            # 减小值保留更多点
   ```
3. **重新编译并启动**：
   ```bash
   colcon build --packages-select fast_lio
   source install/setup.bash
   ```

### 问题2：PCL报错 "Leaf size is too small"

**原因**：体素网格滤波器的叶子尺寸对于当前点云密度过小。

**解决方法**：
在 `c16.yaml` 中增大 `filter_size_surf` 和 `filter_size_map`（参考问题1）。

### 问题3：IMU无数据发布

**状态**：已修复（修改了 `imu_node.py` 的数据包解析逻辑）。

**验证修复**：
```bash
ros2 topic hz /imu/data
# 应显示频率约100-200Hz
```

如果仍无数据，检查：
```bash
# 串口是否可访问
ls -l /dev/ttyUSB0

# 查看原始数据
sudo cat /dev/ttyUSB0 | xxd | head -20
# 应看到 aa55 开头的数据包
```

### 问题4：雷达连接失败

**检查步骤**：
```bash
# 1. 网络连通性
ping 192.168.1.200

# 2. 网络接口配置
ip addr show

# 3. 防火墙（如果有）
sudo ufw status
sudo ufw allow from 192.168.1.200
```

### 问题5：RViz无法启动或崩溃

**临时方案**：不启动RViz（后台建图仍正常）
```bash
ros2 launch fast_lio mapping.launch.py rviz:=false
```

**查看地图**：在另一台机器运行RViz并订阅话题（需配置ROS_DOMAIN_ID）。

---

## 🐳 Docker 使用说明

### 检查是否在Docker容器中运行

```bash
# 方法1：检查 /.dockerenv 文件
ls -la /.dockerenv

# 方法2：查看cgroup
cat /proc/1/cgroup | grep docker

# 方法3：检查主机名
hostname
# Docker容器通常有随机生成的主机名
```

### Docker容器管理建议

如果你在Docker容器中运行本工作空间，以下是推荐的管理方式：

#### 1. 持久化工作空间（重要）

确保工作空间目录映射到宿主机：

```bash
# 在宿主机上创建持久化目录
mkdir -p ~/ros2_workspace_persistent

# 启动容器时挂载
docker run -it --rm \
  --name ros2_fastlio \
  -v ~/ros2_workspace_persistent:/home/rosdev/ros2_ws \
  -v /dev:/dev \
  --privileged \
  --network host \
  your_ros2_image:latest
```

#### 2. 设备访问

激光雷达和IMU需要特殊权限：

```bash
docker run -it --rm \
  --device=/dev/ttyUSB0 \        # IMU串口
  --network host \               # 网络雷达需要
  --privileged \                 # 或使用--cap-add=SYS_ADMIN
  your_image
```

#### 3. 图形界面（RViz）

如果需要在Docker中运行RViz：

```bash
# 允许X11转发
xhost +local:docker

# 启动容器
docker run -it --rm \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  --network host \
  your_image

# 使用完毕后恢复安全设置
xhost -local:docker
```

#### 4. 创建专用镜像（推荐）

在工作空间根目录创建 `Dockerfile`：

```dockerfile
FROM ros:foxy

# 安装依赖
RUN apt-get update && apt-get install -y \
    ros-foxy-pcl-ros \
    ros-foxy-rviz2 \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# 复制工作空间
COPY . /home/rosdev/ros2_ws
WORKDIR /home/rosdev/ros2_ws

# 编译
RUN . /opt/ros/foxy/setup.sh && \
    colcon build --symlink-install

# 默认启动命令
CMD ["bash", "-c", "source /opt/ros/foxy/setup.bash && source install/setup.bash && ./start_all.sh"]
```

构建和运行：

```bash
# 构建镜像
docker build -t fastlio2:latest .

# 运行
docker run -it --rm \
  --device=/dev/ttyUSB0 \
  --network host \
  --privileged \
  fastlio2:latest
```

#### 5. Docker Compose（推荐用于生产）

创建 `docker-compose.yml`：

```yaml
version: '3.8'

services:
  fastlio2:
    image: fastlio2:latest
    container_name: fastlio_slam
    privileged: true
    network_mode: host
    devices:
      - /dev/ttyUSB0:/dev/ttyUSB0
    volumes:
      - ./ros2_ws:/home/rosdev/ros2_ws
      - /tmp/.X11-unix:/tmp/.X11-unix
    environment:
      - DISPLAY=${DISPLAY}
      - ROS_DOMAIN_ID=0
    command: bash -c "source /opt/ros/foxy/setup.bash && source install/setup.bash && ./start_all.sh"
```

使用：
```bash
docker-compose up      # 启动
docker-compose down    # 停止
docker-compose logs -f # 查看日志
```

---

## 📊 性能优化建议

### 针对嵌入式平台（Jetson等）

1. **降低点云密度**：
   ```yaml
   # c16.yaml
   point_filter_num: 3    # 增大此值
   ```

2. **关闭RViz**：
   ```bash
   ros2 launch fast_lio mapping.launch.py rviz:=false
   ```

3. **设置CPU频率**：
   ```bash
   # Jetson
   sudo nvpmodel -m 0          # 最高性能模式
   sudo jetson_clocks          # 锁定最高频率
   ```

### 针对PC平台

1. **启用多线程**：检查 `c16.yaml` 中的线程数配置

2. **使用GPU加速**（如果支持）：部分PCL操作可用CUDA加速

---

## 📝 开发与贡献

### 修改配置文件

主要配置文件：`src/FAST_LIO_ROS2/config/c16.yaml`

关键参数说明：
```yaml
common:
    lid_topic:  "/cx/lslidar_point_cloud"   # 雷达话题
    imu_topic:  "/imu/data"                 # IMU话题
    
preprocess:
    lidar_type: 2                           # 2=Velodyne/Lslidar
    scan_line: 16                           # 线数
    blind: 1.0                              # 盲区距离(m)
    
mapping:
    filter_size_surf: 0.5                   # 点云滤波尺寸
    filter_size_map: 0.5                    # 地图滤波尺寸
    cube_side_length: 200                   # 地图立方体边长(m)
    
publish:
    path_publish_en: true                   # 发布轨迹
    scan_publish_en: true                   # 发布点云
```

修改后需重新编译：
```bash
colcon build --packages-select fast_lio
source install/setup.bash
```

### 查看源码

- FAST-LIO核心算法：`src/FAST_LIO_ROS2/src/laserMapping.cpp`
- IMU驱动节点：`src/handsfree_imu_ros2/handsfree_imu_ros2/imu_node.py`
- 雷达驱动：`src/Lslidar_ROS2_driver/lslidar_driver/src/`

---

## 📄 许可证

各子包遵循各自的开源许可证：
- FAST_LIO_ROS2: GPLv2
- Lslidar_ROS2_driver: BSD
- handsfree_imu_ros2: MIT

---

## 🙏 致谢

- [FAST-LIO2](https://github.com/hku-mars/FAST_LIO) 原始算法实现
- Lslidar ROS2驱动维护者
- HandsFree Robotics 开源社区

---

## 📞 支持与反馈

如遇到问题或有改进建议，请通过以下方式反馈：

1. 提交 GitHub Issue
2. 查看 [FAST-LIO2 官方文档](https://github.com/hku-mars/FAST_LIO)
3. 检查本文档"常见问题排查"章节

---

**最后更新**：2025-12-20

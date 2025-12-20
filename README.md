# FAST-LIO2 ROS2 完整开发环境# FAST-LIO2 ROS2 工作空间# FAST_LIO_ROS2



> **基于 Docker 的 FAST-LIO2 实时 SLAM 系统容器化方案**  FAST-LIO2 workspace and drivers

> 本项目将 FAST-LIO2（Fast LiDAR-Inertial Odometry）及其依赖的雷达/IMU/底盘驱动完整打包，提供 Docker 镜像方便快速部署。

基于 ROS2 Foxy 的 FAST-LIO2 SLAM 系统完整工作空间，集成激光雷达驱动、IMU 驱动和建图算法。

---

说明（简体中文）

## 📋 项目概述

## 📋 项目简介

本仓库是一个**完整的 ROS2 工作空间**，包含：

以下包含两种在本仓库环境下运行 FAST-LIO2 的方法：

- ✅ **FAST-LIO2 算法**：实时激光雷达-IMU 紧耦合 SLAM

- ✅ **激光雷达驱动**：Lslidar C16（支持其他机械式/混合固态雷达）本项目是一个完整的 ROS2 工作空间，用于运行 FAST-LIO2（Fast LiDAR-Inertial Odometry）实时 SLAM 系统。FAST-LIO2 是一种高效的激光雷达-惯性里程计算法，能够在资源受限的平台上实现实时定位与建图。1) 手动按步骤启动（分开启动激光雷达驱动、IMU 驱动与 FAST-LIO 节点）

- ✅ **IMU 驱动**：HandsFree A9 九轴 IMU

- ✅ **底盘驱动**：Hunter AGV（可选）2) 使用仓库内的一键脚本 `start_all.sh` 启动（更方便）

- ✅ **Docker 容器化**：基于 ROS2 Foxy，支持 x86 PC 和 Jetson 平台

- ✅ **一键启动脚本**：自动启动所有节点### 主要特性



### 支持的硬件平台重要前置项



| 组件类型 | 型号 | 接口 | 说明 |- ✅ 实时 3D 点云建图- 本仓库基于 ROS2（你的环境是 Foxy），请先确保已安装并配置好 ROS2 环境。

|---------|------|------|------|

| **激光雷达** | Lslidar C16/C32 | 网络（UDP） | 16/32 线机械式雷达，默认 IP 192.168.1.200 |- ✅ 激光雷达-IMU 紧耦合融合- 若你在当前机器上尚未构建过工作区，先在工作区根目录运行一次构建：

| **IMU** | HandsFree A9/TBA9 | 串口 | 9 轴 IMU，默认 /dev/ttyUSB0，波特率 921600 |

| **底盘**（可选） | Hunter 2.0 | CAN | AgileX Hunter AGV，需 CAN-USB 适配器 |- ✅ 支持多种激光雷达型号（Livox、Velodyne、Lslidar等）

| **计算平台** | x86 PC / Jetson | - | 推荐 4 核+ CPU，8GB+ RAM |

- ✅ 低计算资源消耗，适合嵌入式平台```bash

---

- ✅ 一键启动脚本，简化操作流程# 进入工作区

## 🚀 快速开始（推荐使用 Docker）

- ✅ Docker 容器化支持（如果在容器环境中运行）cd /home/rosdev/ros2_ws

### 方法一：使用 Docker Compose（最简单）



#### 前置条件

---# （可选）将源码更新或检查修改

1. **安装 Docker** ≥ 20.10

   ```bash# git status

   sudo apt-get update

   sudo apt-get install docker.io docker-compose## 🛠 硬件要求

   sudo usermod -aG docker $USER  # 添加当前用户到 docker 组

   # 注销并重新登录以生效# 构建（只需在第一次或修改源码后运行）

   ```

### 必需硬件colcon build --symlink-install

2. **（可选）安装 NVIDIA Container Toolkit**（GPU 加速）

   ```bash```

   distribution=$(. /etc/os-release;echo $ID$VERSION_ID)

   curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -| 硬件类型 | 型号/规格 | 说明 |

   curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list \

     | sudo tee /etc/apt/sources.list.d/nvidia-docker.list|---------|----------|------|- 每次启动前请 source ROS2 与本工作区的 setup 文件：

   sudo apt-get update && sudo apt-get install -y nvidia-container-toolkit

   sudo systemctl restart docker| **激光雷达** | Lslidar C16 | 16线机械式激光雷达，网络连接（192.168.1.200） |

   ```

| **IMU** | HandsFree IMU | 9轴 IMU，串口连接（/dev/ttyUSB0，921600波特率） |```bash

3. **允许 X11 转发**（用于 RViz2 可视化）

   ```bash| **计算平台** | Jetson/PC | 推荐 Jetson Xavier/Orin 或 x86 PC（4核+，8GB+ RAM） |source /opt/ros/foxy/setup.bash

   xhost +local:docker

   ```source /home/rosdev/ros2_ws/install/local_setup.bash



#### 步骤### 可选硬件```



```bash

# 1. 克隆仓库

git clone https://github.com/superclash1/FAST_LIO_ROS2.git- 移动底盘：Hunter AGV（已集成驱动包）方法一 — 手动逐个启动（推荐用于调试）

cd FAST_LIO_ROS2

- GPS/RTK：可扩展全局定位- 适用于需要观察单个节点日志或分步调试的场景。

# 2. 构建 Docker 镜像

docker compose build



# 3. 启动容器---典型启动顺序：

docker compose up -d

1. 启动激光雷达驱动（根据型号选择对应 launch）：

# 4. 进入容器

docker exec -it fastlio_ros2 bash## 📦 软件架构```bash



# 5. 容器内首次构建工作区# 例如 Lslidar C16 驱动

source /opt/ros/foxy/setup.bash

cd ~/ros2_ws### 功能包说明ros2 launch lslidar_driver lslidar_c16_launch.py

rosdep install --from-paths src --ignore-src -r -y

colcon build --symlink-install```

source install/setup.bash

```

# 6. 一键启动 FAST-LIO2 系统

./start_all.shros2_ws/2. 启动 IMU 驱动：

```

├── src/```bash

---

│   ├── FAST_LIO_ROS2/          # 核心SLAM算法包# 示例：handsfree_imu_ros2 的 launch

### 方法二：使用便捷脚本 `scripts/run.sh`

│   │   ├── src/                # C++源码（laserMapping.cpp等）ros2 launch handsfree_imu_ros2 imu.launch.py

脚本会自动检测并映射串口设备、配置 GPU、设置 X11 转发。

│   │   ├── config/             # 配置文件（c16.yaml等）```

```bash

# 1. 克隆仓库│   │   ├── launch/             # 启动文件

git clone https://github.com/superclash1/FAST_LIO_ROS2.git

cd FAST_LIO_ROS2│   │   └── rviz/               # RViz可视化配置3. 启动 FAST-LIO mapping 节点（含参数与 RViz，可按需开启）：



# 2. 构建镜像│   │```bash

docker build -t fastlio_ros2:foxy .

│   ├── Lslidar_ROS2_driver/    # 镭神激光雷达驱动ros2 launch FAST_LIO_ROS2 mapping.launch.py use_rviz:=true

# 3. 运行脚本（自动创建或复用容器）

./scripts/run.sh│   │   └── lslidar_driver/     # C16等型号驱动```



# 4. 容器内构建工作区（首次运行）│   │

source /opt/ros/foxy/setup.bash

cd ~/ros2_ws│   ├── handsfree_imu_ros2/     # HandsFree IMU ROS2驱动运行提示与常见问题：

colcon build --symlink-install

source install/setup.bash│   │   ├── imu_node.py         # IMU数据解析与发布节点- 如果控制台输出 "No Effective Points!"，可能是机器人静止或点云被过度滤波，请移动平台并检查 `config/*.yaml` 中的滤波尺寸参数（例如 `filter_size_surf`、`point_filter_num` 等）。



# 5. 一键启动│   │   └── launch/             # IMU启动文件- 如果看到 PCL 提示 `Leaf size is too small`，请适当调大体素网格 (voxel) 的 leaf size。

./start_all.sh

```│   │- 确认话题 `/cx/lslidar_point_cloud`（点云）与 `/imu/data`（IMU）有数据：



**脚本特性**：│   ├── hunter_ros2/            # Hunter移动底盘驱动（可选）```bash

- 自动挂载当前目录到容器 `/home/rosdev/ros2_ws`

- 自动检测 `/dev/ttyUSB0` 并映射（可通过 `IMU_DEVICE` 环境变量自定义）│   │   └── hunter_base/        # 底盘控制接口ros2 topic hz /cx/lslidar_point_cloud

- 支持代理设置（检测 `http_proxy` 环境变量）

│   │ros2 topic hz /imu/data

**自定义示例**：

```bash│   └── ugv_sdk/                # 通用无人车SDK（Hunter依赖）```

# 使用不同的串口设备

IMU_DEVICE=/dev/ttyUSB1 ./scripts/run.sh│



# 设置代理├── start_all.sh                # 一键启动脚本（推荐）方法二 — 使用一键脚本 `start_all.sh`（推荐快速启动）

export http_proxy=http://192.168.1.100:7890

./scripts/run.sh├── start_fastlio_full.sh       # 完整启动脚本（含RViz）- 仓库根目录已有 `start_all.sh`，脚本会按顺序启动雷达驱动、IMU 驱动和 FAST-LIO（可选 RViz）。

```

└── README.md                   # 本文档- 使用前同样需要 source 环境：

---

``````bash

## 📦 功能包说明

source /opt/ros/foxy/setup.bash

### 1. FAST_LIO_ROS2（核心 SLAM 算法）

### 各功能包详细说明source /home/rosdev/ros2_ws/install/local_setup.bash

**功能**：实时激光雷达-IMU 紧耦合里程计与建图。

chmod +x /home/rosdev/ros2_ws/start_all.sh

**关键文件**：

- `config/c16.yaml`：Lslidar C16 配置文件#### 1. FAST_LIO_ROS2（核心建图包）/home/rosdev/ros2_ws/start_all.sh

- `config/avia.yaml`：Livox Avia 配置文件

- `launch/mapping.launch.py`：启动文件```



**发布话题**：**作用**：实现激光雷达-IMU融合的实时SLAM，输出高精度位姿和点云地图。

- `/Odometry`：实时里程计（nav_msgs/Odometry）

- `/cloud_registered`：配准后的点云地图（sensor_msgs/PointCloud2）- 脚本会在当前终端启动多个子进程；要停止可用 `pkill -f <node_name>` 或在另一个终端执行：

- `/path`：运动轨迹（nav_msgs/Path）

**关键文件**：```bash

**订阅话题**：

- `/cx/lslidar_point_cloud`：激光雷达点云- `src/laserMapping.cpp`：主算法实现（点云处理、IMU预积分、KD-Tree地图更新）pkill -f fast_lio || true

- `/imu/data`：IMU 数据

- `config/c16.yaml`：Lslidar C16配置（话题映射、滤波参数、外参等）pkill -f lslidar || true

---

- `launch/mapping.launch.py`：启动文件（可选是否启动RViz）pkill -f imu_node || true

### 2. Lslidar_ROS2_driver（镭神激光雷达驱动）

```

**功能**：支持镭神 C16/C32 等机械式雷达和 CX 系列混合固态雷达。

**发布话题**：

**关键参数**（在 launch 文件中配置）：

- `device_ip`：雷达 IP 地址（默认 `192.168.1.200`）- `/Odometry`：实时里程计（nav_msgs/Odometry）小结与建议

- `frame_id`：坐标系名称（默认 `laser_link`）

- `scan_topic`：发布话题名（默认 `/cx/lslidar_point_cloud`）- `/cloud_registered`：配准后的点云地图- 开发/调试时优先用 方法一（分步启动、逐个检查话题与日志）；演示/部署时可用 方法二（脚本自动化）。



**使用方法**：- `/path`：运动轨迹- 启动后若无法建图，请确保平台运动且传感器数据频率稳定。

```bash

# Lslidar C16- 我可以把更多运行示例（launch 参数说明、config 模板、回放 bag）加入本 README，告诉我你需要哪些示例。

ros2 launch lslidar_driver lslidar_cx_launch.py

**订阅话题**：

# 查看点云数据

ros2 topic echo /cx/lslidar_point_cloud- `/cx/lslidar_point_cloud`：激光雷达点云（sensor_msgs/PointCloud2）

```- `/imu/data`：IMU数据（sensor_msgs/Imu）



---#### 2. Lslidar_ROS2_driver（雷达驱动包）



### 3. handsfree_imu_ros2（HandsFree IMU 驱动）**作用**：连接镭神C16激光雷达硬件，解析UDP数据包并发布ROS2点云消息。



**功能**：读取 HandsFree A9/TBA9 IMU 串口数据，发布 IMU 消息。**关键参数**（在launch文件中）：

- `device_ip`：雷达IP地址（默认192.168.1.200）

**关键参数**：- `frame_id`：坐标系名称（默认`laser_link`）

- `port`：串口设备（默认 `/dev/ttyUSB0`）- `scan_topic`：发布话题名（`/cx/lslidar_point_cloud`）

- `baud_rate`：波特率（默认 `921600`）

**使用方法**：

**发布话题**：```bash

- `/imu/data`：IMU 数据（sensor_msgs/Imu）ros2 launch lslidar_driver lslidar_c16_launch.py

- `/imu/mag`：磁力计数据（sensor_msgs/MagneticField）```



**使用方法**：#### 3. handsfree_imu_ros2（IMU驱动包）

```bash

# 启动 IMU 驱动**作用**：读取HandsFree IMU串口数据，解析姿态/角速度/加速度并发布。

ros2 launch handsfree_imu_ros2 imu.launch.py port:=/dev/ttyUSB0

**已修复问题**：

# 查看 IMU 数据- 修正了数据包解析逻辑（0x2c和0x14包类型）

ros2 topic echo /imu/data- 解决了IMU不发布数据的bug

```

**关键参数**：

**常见问题**：- `port`：串口设备（默认`/dev/ttyUSB0`）

- 如果看不到数据，检查串口权限：`sudo chmod 666 /dev/ttyUSB0`- `baud_rate`：波特率（921600）

- 确认波特率正确：`921600`

**发布话题**：

---- `/imu/data`：IMU数据（sensor_msgs/Imu）

- `/imu/mag`：磁力计数据

### 4. hunter_ros2（Hunter AGV 底盘驱动，可选）

#### 4. hunter_ros2 & ugv_sdk（底盘驱动，可选）

**功能**：控制 AgileX Hunter 2.0 移动底盘。

**作用**：控制Hunter AGV移动底盘（如果你的系统使用该底盘）。

**依赖**：

- CAN-USB 适配器（推荐 PEAK PCAN-USB）**说明**：如果你只做建图不需要控制底盘，可以忽略此包。

- `ugv_sdk`：通用无人车 SDK

---

**使用方法**：

```bash## 🚀 快速开始

# 1. 设置 CAN 接口

cd ~/ros2_ws/src/ugv_sdk/scripts### 环境要求

sudo bash setup_can2usb.bash

- **操作系统**：Ubuntu 20.04（推荐）或兼容Docker容器

# 每次重启后需要运行- **ROS版本**：ROS2 Foxy

sudo bash bringup_can2usb_500k.bash- **依赖库**：PCL、Eigen、livox_ros_driver2（如果使用Livox雷达）



# 2. 启动底盘驱动### 1. 构建工作空间

ros2 launch hunter_base hunter_base.launch.py

首次使用或修改代码后需要编译：

# 3. 发送速度命令

ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \```bash

  "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"cd /home/rosdev/ros2_ws

```

# 安装依赖（首次）

**⚠️ 安全提示**：测试时务必准备好遥控器随时接管控制！rosdep install --from-paths src --ignore-src -r -y



---# 编译（使用symlink可避免重复安装Python包）

colcon build --symlink-install

## 📊 运行说明

# Source环境

### 一键启动（推荐）source install/setup.bash

```

```bash

# 容器内执行### 2. 配置硬件连接

cd ~/ros2_ws

source install/setup.bash#### 激光雷达网络配置

./start_all.sh

```确保你的电脑与雷达在同一网段：



脚本会按顺序启动：```bash

1. 激光雷达驱动# 检查网络接口

2. IMU 驱动ip addr show

3. FAST-LIO2 建图节点（含 RViz2 可视化）

# 配置静态IP（示例，根据实际网卡调整）

按 `Ctrl+C` 停止所有节点。sudo ip addr add 192.168.1.102/24 dev eth0



---# 测试连通性

ping 192.168.1.200

### 手动分步启动（调试用）```



```bash#### IMU串口权限

# 终端 1：启动激光雷达

ros2 launch lslidar_driver lslidar_cx_launch.py```bash

# 查看串口设备

# 终端 2：启动 IMUls -l /dev/ttyUSB*

ros2 launch handsfree_imu_ros2 imu.launch.py port:=/dev/ttyUSB0

# 添加当前用户到dialout组（避免sudo）

# 终端 3：启动 FAST-LIO2sudo usermod -aG dialout $USER

ros2 launch fast_lio mapping.launch.py config_file:=c16.yaml rviz:=true# 注销并重新登录生效

```

# 或临时授权

---sudo chmod 666 /dev/ttyUSB0

```

### 检查传感器数据

### 3. 运行系统

```bash

# 查看所有话题提供两种启动方式：

ros2 topic list

#### 方法一：使用一键脚本（推荐）

# 检查激光雷达频率（应该约 10 Hz）

ros2 topic hz /cx/lslidar_point_cloud最简单的启动方式，自动按顺序启动所有节点：



# 检查 IMU 频率（应该约 100-200 Hz）```bash

ros2 topic hz /imu/data# Source环境

source /opt/ros/foxy/setup.bash

# 查看里程计source ~/ros2_ws/install/setup.bash

ros2 topic echo /Odometry

```# 运行启动脚本

./start_all.sh

---```



## ⚙️ 配置说明**脚本功能**：

- ✅ 自动检查雷达和IMU设备连接

### 1. 修改激光雷达参数- ✅ 按顺序启动雷达驱动 → IMU驱动 → FAST-LIO

- ✅ 捕获Ctrl+C自动清理所有进程

编辑 `src/FAST_LIO_ROS2/config/c16.yaml`：- ✅ 彩色输出，状态一目了然



```yaml停止系统：按 `Ctrl+C`

common:

    lid_topic: "/cx/lslidar_point_cloud"  # 雷达话题#### 方法二：手动分步启动（调试用）

    imu_topic: "/imu/data"                # IMU 话题

    time_sync_en: false                   # 时间同步（通常为 false）适合需要观察每个节点日志的场景，推荐开三个终端：



preprocess:**终端1 - 启动激光雷达驱动**：

    lidar_type: 2          # 0-Avia, 1-Livox, 2-Velodyne/机械式```bash

    scan_line: 16          # C16 是 16 线source /opt/ros/foxy/setup.bash

    blind: 0.5             # 盲区距离（米）source ~/ros2_ws/install/setup.bash

ros2 launch lslidar_driver lslidar_c16_launch.py

mapping:```

    filter_size_surf: 0.5  # 体素滤波尺寸（米）

    point_filter_num: 3    # 点跳数（每 N 个点取 1 个）**终端2 - 启动IMU驱动**：

    extrinsic_est_en: true # 启用外参自动估计```bash

```source /opt/ros/foxy/setup.bash

source ~/ros2_ws/install/setup.bash

---ros2 launch handsfree_imu_ros2 imu.launch.py

```

### 2. 修改设备映射

**终端3 - 启动FAST-LIO**：

**方法 1：编辑 `docker-compose.yml`**```bash

source /opt/ros/foxy/setup.bash

取消注释并修改设备路径：source ~/ros2_ws/install/setup.bash

ros2 launch fast_lio mapping.launch.py config_file:=c16.yaml rviz:=true

```yaml```

devices:

  - /dev/ttyUSB0:/dev/ttyUSB0:rwm  # IMU 串口### 4. 验证系统运行

  - /dev/ttyUSB1:/dev/ttyUSB1:rwm  # 额外设备

```在新终端检查话题数据：



**方法 2：使用脚本环境变量**```bash

# 检查所有话题

```bashros2 topic list

IMU_DEVICE=/dev/ttyUSB1 ./scripts/run.sh

```# 检查雷达数据频率（应约10Hz）

ros2 topic hz /cx/lslidar_point_cloud

---

# 检查IMU数据频率（应约100-200Hz）

### 3. 保存建图结果ros2 topic hz /imu/data



编辑 `src/FAST_LIO_ROS2/config/c16.yaml`：# 查看里程计输出

ros2 topic echo /Odometry

```yaml```

pcd_save:

    pcd_save_en: true  # 启用 PCD 保存---

    interval: 1        # 保存间隔（秒，-1 为最后一次保存）

```## 🔧 常见问题排查



地图保存路径：`~/ros2_ws/test.pcd`### 问题1：控制台输出 "No Effective Points!"



---**原因**：

- 机器人静止不动，导致特征点不足

## 🐛 常见问题- 点云被过度滤波，有效点太少



### 1. 启动后看到 "No Effective Points!"**解决方法**：

1. **移动平台**：FAST-LIO需要运动才能初始化和建图

**原因**：机器人静止或点云被过滤掉。2. **调整滤波参数**：编辑 `src/FAST_LIO_ROS2/config/c16.yaml`

   ```yaml

**解决**：   filter_size_surf: 0.5          # 增大值减少滤波强度

- 移动平台，让激光雷达扫到更多特征   filter_size_map: 0.5           # 增大值减少滤波强度

- 调整 `c16.yaml` 滤波参数：   point_filter_num: 1            # 减小值保留更多点

  ```yaml   ```

  filter_size_surf: 0.3  # 减小滤波尺寸3. **重新编译并启动**：

  point_filter_num: 2    # 减小跳点数   ```bash

  blind: 0.3             # 减小盲区   colcon build --packages-select fast_lio

  ```   source install/setup.bash

   ```

---

### 问题2：PCL报错 "Leaf size is too small"

### 2. RViz2 无法显示或报错

**原因**：体素网格滤波器的叶子尺寸对于当前点云密度过小。

**原因**：X11 转发未配置。

**解决方法**：

**解决**：在 `c16.yaml` 中增大 `filter_size_surf` 和 `filter_size_map`（参考问题1）。

```bash

# 宿主机执行### 问题3：IMU无数据发布

xhost +local:docker

docker compose restart**状态**：已修复（修改了 `imu_node.py` 的数据包解析逻辑）。

```

**验证修复**：

---```bash

ros2 topic hz /imu/data

### 3. 找不到 IMU 设备 `/dev/ttyUSB0`# 应显示频率约100-200Hz

```

**原因**：串口设备名称不同或权限不足。

如果仍无数据，检查：

**解决**：```bash

```bash# 串口是否可访问

# 查看所有串口ls -l /dev/ttyUSB0

ls /dev/ttyUSB*

# 查看原始数据

# 修改权限sudo cat /dev/ttyUSB0 | xxd | head -20

sudo chmod 666 /dev/ttyUSB0# 应看到 aa55 开头的数据包

```

# 或永久配置 udev 规则（推荐）

sudo usermod -aG dialout $USER  # 添加当前用户到 dialout 组### 问题4：雷达连接失败

# 注销重新登录

```**检查步骤**：

```bash

---# 1. 网络连通性

ping 192.168.1.200

### 4. 无法连接激光雷达（192.168.1.200）

# 2. 网络接口配置

**原因**：网络配置或雷达 IP 不匹配。ip addr show



**解决**：# 3. 防火墙（如果有）

```bashsudo ufw status

# 1. 检查宿主机网络sudo ufw allow from 192.168.1.200

ping 192.168.1.200```



# 2. 配置静态 IP（与雷达同网段）### 问题5：RViz无法启动或崩溃

sudo ifconfig eth0 192.168.1.100 netmask 255.255.255.0

**临时方案**：不启动RViz（后台建图仍正常）

# 3. 修改雷达 IP（参考 Lslidar 官方工具）```bash

```ros2 launch fast_lio mapping.launch.py rviz:=false

```

---

**查看地图**：在另一台机器运行RViz并订阅话题（需配置ROS_DOMAIN_ID）。

### 5. 容器内构建失败

---

**可能原因**：依赖缺失、网络问题。

## 🐳 Docker 使用说明

**解决**：

```bash### 检查是否在Docker容器中运行

# 1. 更新 rosdep

sudo rosdep init || true```bash

rosdep update# 方法1：检查 /.dockerenv 文件

ls -la /.dockerenv

# 2. 安装依赖

cd ~/ros2_ws# 方法2：查看cgroup

rosdep install --from-paths src --ignore-src -r -ycat /proc/1/cgroup | grep docker



# 3. 清理后重新构建# 方法3：检查主机名

rm -rf build install loghostname

colcon build --symlink-install# Docker容器通常有随机生成的主机名

``````



---### Docker容器管理建议



### 6. Hunter 底盘 CAN 通信失败如果你在Docker容器中运行本工作空间，以下是推荐的管理方式：



**解决**：#### 1. 持久化工作空间（重要）

```bash

# 1. 检查 CAN 模块确保工作空间目录映射到宿主机：

lsmod | grep gs_usb

```bash

# 2. 加载模块# 在宿主机上创建持久化目录

sudo modprobe gs_usbmkdir -p ~/ros2_workspace_persistent



# 3. 配置 CAN 接口# 启动容器时挂载

cd ~/ros2_ws/src/ugv_sdk/scriptsdocker run -it --rm \

sudo bash setup_can2usb.bash  --name ros2_fastlio \

sudo bash bringup_can2usb_500k.bash  -v ~/ros2_workspace_persistent:/home/rosdev/ros2_ws \

  -v /dev:/dev \

# 4. 测试接收数据  --privileged \

candump can0  --network host \

```  your_ros2_image:latest

```

---

#### 2. 设备访问

## 🔧 进阶使用

激光雷达和IMU需要特殊权限：

### 1. 录制 ROS2 bag

```bash

```bashdocker run -it --rm \

# 录制所有话题  --device=/dev/ttyUSB0 \        # IMU串口

ros2 bag record -a  --network host \               # 网络雷达需要

  --privileged \                 # 或使用--cap-add=SYS_ADMIN

# 录制特定话题  your_image

ros2 bag record /cx/lslidar_point_cloud /imu/data /Odometry```

```

#### 3. 图形界面（RViz）

### 2. 回放 bag 文件进行离线建图

如果需要在Docker中运行RViz：

```bash

# 启动 FAST-LIO（不启动传感器驱动）```bash

ros2 launch fast_lio mapping.launch.py config_file:=c16.yaml rviz:=true# 允许X11转发

xhost +local:docker

# 另一个终端回放 bag

ros2 bag play <bag文件路径># 启动容器

```docker run -it --rm \

  -e DISPLAY=$DISPLAY \

### 3. 添加本地配置覆盖  -v /tmp/.X11-unix:/tmp/.X11-unix \

  --network host \

创建 `docker-compose.override.yml`（不会被 git 提交）：  your_image



```yaml# 使用完毕后恢复安全设置

version: "3.8"xhost -local:docker

services:```

  fastlio:

    devices:#### 4. 创建专用镜像（推荐）

      - /dev/ttyUSB1:/dev/ttyUSB0:rwm  # 自定义串口

    environment:在工作空间根目录创建 `Dockerfile`：

      - http_proxy=http://192.168.1.100:7890  # 代理

``````dockerfile

FROM ros:foxy

---

# 安装依赖

## 📚 参考资料RUN apt-get update && apt-get install -y \

    ros-foxy-pcl-ros \

- [FAST-LIO2 原始论文](https://github.com/hku-mars/FAST_LIO)    ros-foxy-rviz2 \

- [FAST-LIO2 ROS2 移植](https://github.com/Ericsii/FAST_LIO_ROS2)    python3-pip \

- [Lslidar ROS2 驱动](https://github.com/Lslidar/lslidar_ros2)    && rm -rf /var/lib/apt/lists/*

- [UGV SDK（Hunter）](https://github.com/westonrobot/ugv_sdk)

- [ikd-Tree](https://github.com/hku-mars/ikd-Tree)：动态 KD 树加速# 复制工作空间

COPY . /home/rosdev/ros2_ws

---WORKDIR /home/rosdev/ros2_ws



## 🤝 贡献# 编译

RUN . /opt/ros/foxy/setup.sh && \

欢迎提交 Issue 和 Pull Request！    colcon build --symlink-install



如果本项目对你有帮助，请给个 ⭐ Star 支持一下！# 默认启动命令

CMD ["bash", "-c", "source /opt/ros/foxy/setup.bash && source install/setup.bash && ./start_all.sh"]

---```



## 📄 许可证构建和运行：



本项目遵循各子模块的原始许可证：```bash

- FAST-LIO2：MIT License# 构建镜像

- Lslidar驱动：Apache-2.0 Licensedocker build -t fastlio2:latest .

- UGV SDK：Apache-2.0 License

# 运行

详见各目录的 LICENSE 文件。docker run -it --rm \

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

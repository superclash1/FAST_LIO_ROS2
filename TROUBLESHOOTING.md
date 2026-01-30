# FAST-LIO2 故障排除指南

本文档记录了在 Jetson Xavier 上部署 FAST-LIO2 系统（Lslidar C16 + HandsFree IMU）时遇到的所有问题及解决方案。

## 目录

- [系统配置](#系统配置)
- [问题1: 激光雷达无数据](#问题1-激光雷达无数据)
- [问题2: IMU 串口设备不存在](#问题2-imu-串口设备不存在)
- [问题3: FAST-LIO 警告信息](#问题3-fast-lio-警告信息)
- [问题4: 建图退化与位姿发散](#问题4-建图退化与位姿发散)
- [问题5: USB 串口名称变化](#问题5-usb-串口名称变化)
- [完整解决方案](#完整解决方案)
- [验证测试](#验证测试)

---

## 系统配置

### 硬件环境
- **平台**: Jetson Xavier NX
- **操作系统**: Ubuntu 20.04.6 LTS (Docker 容器)
- **ROS版本**: ROS2 Foxy
- **激光雷达**: Lslidar C16 v3.0 (16线机械式激光雷达)
  - IP: 192.168.1.200
  - MSOP 端口: 2368
  - DIFOP 端口: 2369
  - 网络接口: 100Mbps 全双工
- **IMU**: HandsFree A9/TBA9
  - 接口: USB 转串口 (CP2102 芯片)
  - 波特率: 921600
  - 输出频率: ~200Hz

### 软件环境
- **FAST-LIO2**: ROS2 版本
- **激光雷达驱动**: Lslidar_ROS2_driver
- **IMU驱动**: handsfree_imu_ros2 (从 ROS1 移植)

---

## 问题1: 激光雷达无数据

### 症状
```bash
[lslidar_driver_node-1] Lidar model error, please check Lidar model. Retrying...
[lslidar_driver_node-1] poll() timeout, port:2369
```

查看网络接口统计：
```bash
$ ifconfig eth0
RX packets:0  errors:0  dropped:0  overruns:0  frame:0
```
**关键：`rx_packets=0` 表示网卡完全没有收到任何数据包！**

### 根本原因

**网络物理层 (PHY Layer) 速度/双工模式不匹配**

1. **Jetson 默认配置**：
   - 速度：自动协商 (Auto-negotiation)
   - 通常协商到 1000Mbps (1Gbps)
   - 双工模式：Full Duplex

2. **Lslidar C16 硬件限制**：
   - 仅支持 **100Mbps** 以太网
   - 仅支持 **Full Duplex**
   - **不支持自动协商**（或协商失败）

3. **协商失败后果**：
   - 双方速度不匹配，物理层链路无法建立
   - 即使 `ifconfig` 显示 UP，也无法传输数据
   - 所有 UDP 数据包在物理层被丢弃

### 解决方案

强制设置网络接口参数：

```bash
sudo ethtool -s eth0 speed 100 duplex full autoneg off
```

**参数说明**：
- `speed 100`: 强制 100Mbps
- `duplex full`: 全双工模式
- `autoneg off`: 禁用自动协商

### 验证

```bash
# 1. 检查网络配置
$ sudo ethtool eth0
Settings for eth0:
        Speed: 100Mb/s
        Duplex: Full
        Auto-negotiation: off

# 2. 验证数据包接收
$ ifconfig eth0
RX packets:15234  errors:0  dropped:0  overruns:0  frame:0
# rx_packets 应该在持续增加

# 3. 启动激光雷达驱动
$ ros2 launch lslidar_driver lslidar_cx_launch.py
[lslidar_driver_node-1] [INFO] lslidar init success! vertical angle: 30
[lslidar_driver_node-1] [INFO] C16 lidar start
```

### 持久化配置 (可选)

如果需要开机自动配置，创建 systemd 服务：

```bash
sudo tee /etc/systemd/system/lidar-network-setup.service > /dev/null <<EOF
[Unit]
Description=Setup network for Lslidar C16
After=network.target

[Service]
Type=oneshot
ExecStart=/sbin/ethtool -s eth0 speed 100 duplex full autoneg off
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
EOF

sudo systemctl enable lidar-network-setup.service
sudo systemctl start lidar-network-setup.service
```

---

## 问题2: IMU 串口设备不存在

### 症状

```bash
$ ls -l /dev/ttyUSB0
ls: cannot access '/dev/ttyUSB0': No such file or directory
```

启动 IMU 驱动报错：
```bash
[imu_node-1] [ERROR] Cannot open /dev/ttyUSB0: No such file or directory
```

### 根本原因

**Docker 容器设备隔离问题**

1. **USB 转串口芯片 CP2102** 在宿主机上正常工作，设备为 `/dev/ttyUSB0`
2. **Docker 容器内** 没有自动映射该设备节点
3. 即使使用 `--device` 或 `--privileged` 启动容器，设备节点可能也未创建

### 检查设备

在**宿主机**上：
```bash
# 查看 USB 设备
$ lsusb | grep CP210
Bus 001 Device 005: ID 10c4:ea60 Silicon Labs CP210x UART Bridge

# 查看串口设备
$ ls -l /dev/ttyUSB*
crw-rw---- 1 root dialout 188, 0 Jan 30 08:00 /dev/ttyUSB0

# 查看设备信息
$ dmesg | grep tty | tail -5
[12345.678] usb 1-2: cp210x converter now attached to ttyUSB0
```

在**容器内**：
```bash
$ ls -l /dev/ttyUSB0
ls: cannot access '/dev/ttyUSB0': No such file or directory
```

### 解决方案

#### 方案 A: 手动创建设备节点（临时，推荐用于容器）

```bash
# 创建字符设备节点
sudo mknod /dev/ttyUSB0 c 188 0

# 设置权限
sudo chmod 666 /dev/ttyUSB0

# 验证
$ ls -l /dev/ttyUSB0
crw-rw-rw- 1 root root 188, 0 Jan 30 09:00 /dev/ttyUSB0
```

**参数说明**：
- `mknod`: 创建设备文件
- `/dev/ttyUSB0`: 设备路径
- `c`: 字符设备 (Character device)
- `188`: 主设备号 (USB 串口)
- `0`: 从设备号 (第一个 USB 串口)

#### 方案 B: Docker 启动参数（永久）

修改容器启动命令：
```bash
docker run -it --rm \
  --device=/dev/ttyUSB0:/dev/ttyUSB0 \
  --privileged \
  -v /dev:/dev \
  your_image
```

#### 方案 C: udev 规则（宿主机，自动创建）

在宿主机创建 udev 规则：
```bash
# 创建规则文件
sudo tee /etc/udev/rules.d/99-handsfree-imu.rules > /dev/null <<EOF
# HandsFree IMU (CP2102 USB-UART)
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK+="imu", MODE="0666"
EOF

# 重新加载 udev 规则
sudo udevadm control --reload-rules
sudo udevadm trigger

# 拔插 USB 后会自动创建 /dev/imu
```

### 验证

```bash
# 1. 测试串口
$ sudo chmod 666 /dev/ttyUSB0
$ cat /dev/ttyUSB0
# 应该能看到二进制数据流

# 2. 启动 IMU 驱动
$ ros2 launch handsfree_imu_ros2 imu.launch.py port:=/dev/ttyUSB0
[imu_node-1] [INFO] Successfully opened serial port: /dev/ttyUSB0
[imu_node-1] [INFO] 📊 IMU数据 #200: 姿态=[-1.12°, -0.37°, 195.55°]
```

---

## 问题3: FAST-LIO 警告信息

### 症状

运行 FAST-LIO 时不断输出警告：
```
[fastlio_mapping-1] Failed to find match for field 't'.
[fastlio_mapping-1] Failed to find match for field 'reflectivity'.
[fastlio_mapping-1] Failed to find match for field 'ring'.
[fastlio_mapping-1] Failed to find match for field 'ambient'.
[fastlio_mapping-1] Failed to find match for field 'range'.
```

### 根本原因

**激光雷达类型配置错误**

1. **配置文件 `c16.yaml` 设置**：
   ```yaml
   preprocess:
       lidar_type: 3  # ❌ 错误！
   ```

2. **lidar_type 对应关系**（定义在 `preprocess.h`）：
   ```cpp
   enum LID_TYPE {
       AVIA = 1,      // Livox Avia
       VELO16 = 2,    // Velodyne VLP-16（及兼容格式，如 Lslidar C16）
       OUST64 = 3,    // Ouster OS1-64/OS2-64
       MID360 = 4     // Livox MID-360
   };
   ```

3. **点云格式对比**：

   | 雷达 | 点云字段 |
   |------|---------|
   | **Lslidar C16** | `x, y, z, intensity, ring, time` |
   | **Ouster (type=3)** | `x, y, z, intensity, t, reflectivity, ring, ambient, range` |
   | **Velodyne (type=2)** | `x, y, z, intensity, ring, time` ✅ |

4. **错误后果**：
   - PCL 库尝试将 Lslidar 点云转换为 Ouster 格式
   - 找不到 `t`, `reflectivity`, `ambient`, `range` 字段
   - 每帧产生 5 条警告（虽然不影响功能，但误导性强）

### 解决方案

修改 `src/FAST_LIO_ROS2/config/c16.yaml`：

```yaml
preprocess:
    lidar_type: 2                # ✅ 2 = VELO16 (Velodyne格式，适用于Lslidar C16)
    scan_line: 16                # C16 是 16 线雷达
    scan_rate: 10
    timestamp_unit: 0            # ✅ 0 = 秒 (Lslidar使用秒为单位的float时间戳)
    blind: 0.3                   # 盲区 0.3米
```

**timestamp_unit 说明**：
```cpp
enum TIME_UNIT {
    SEC = 0,   // 秒 (Lslidar 使用)
    MS = 1,    // 毫秒
    US = 2,    // 微秒
    NS = 3     // 纳秒
};
```

### 重新编译

```bash
cd /home/rosdev/ros2_ws
source /opt/ros/foxy/setup.bash
colcon build --packages-select fast_lio --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### 验证

重新运行后，警告消失：
```bash
$ ros2 launch fast_lio mapping.launch.py config_file:=c16.yaml

# ✅ 正常输出，无警告
[fastlio_mapping-1] [VoxelGrid] Points: 14026, Range: [-2.49,9.23] x [-0.48,8.91] x [-0.90,3.00], Leaf: 0.25
[imu_node-1] [INFO] 📊 IMU数据 #3600: 姿态=[-1.10°, -0.37°, 195.55°]
```

---

## 问题4: 建图退化与位姿发散

### 症状

运行一段时间后出现：
```
[fastlio_mapping-1] No Effective Points! total=611 no_near=0 dist_fail=611 
                    pos=[-258.784,441.958,186.748] pos_change=21.8809
[fastlio_mapping-1] No Effective Points! total=611 no_near=0 dist_fail=611 
                    pos=[-277.507,492.198,294.452] pos_change=142.139
[fastlio_mapping-1] No Effective Points! total=611 no_near=0 dist_fail=611 
                    pos=[-300.792,548.445,438.947] pos_change=298.719
[fastlio_mapping-1] [RECOVERY] Position diverged! Resetting to last valid position.
```

**关键现象**：
- 位置疯狂跳动（从几米到几百米）
- `pos_change` 持续增大
- 所有点都被 `dist_fail` 拒绝（距离检查失败）
- 建图消失或严重变形

### 根本原因

**时间戳解析错误导致 IMU-LiDAR 时间同步失败**

1. **问题链**：
   ```
   lidar_type: 3 (Ouster)
        ↓
   使用 oust64_handler() 解析点云
        ↓
   期望 uint32_t 类型的 't' 字段（纳秒级时间戳）
        ↓
   但 Lslidar 实际是 float 'time' 字段（秒级时间戳）
        ↓
   时间戳解析失败 → 时间同步错误
        ↓
   IMU 预积分时间错误 → 状态估计发散
        ↓
   位姿跳动 → 点云配准失败 → 建图退化
   ```

2. **时间同步的重要性**：
   - FAST-LIO 使用 **紧耦合** IMU-LiDAR 融合
   - 每个激光点需要精确的时间戳（毫秒级精度）
   - IMU 预积分需要准确的时间间隔
   - 时间错误 → 运动补偿错误 → 点云畸变 → 配准失败

### 解决方案

**同时修复 lidar_type 和 timestamp_unit**：

```yaml
preprocess:
    lidar_type: 2                # ✅ 使用 Velodyne 处理器
    timestamp_unit: 0            # ✅ 秒 (Lslidar 时间戳单位)
```

**时间戳转换逻辑** (`preprocess.cpp`):
```cpp
switch (time_unit) {
    case SEC:  time_unit_scale = 1.e3f;   break;  // 秒 → 毫秒
    case MS:   time_unit_scale = 1.f;     break;  // 毫秒 → 毫秒
    case US:   time_unit_scale = 1.e-3f;  break;  // 微秒 → 毫秒
    case NS:   time_unit_scale = 1.e-6f;  break;  // 纳秒 → 毫秒
}
// Lslidar: float time (秒) × 1000 = 毫秒 ✅
```

### 验证稳定性

运行后观察：
```bash
# ✅ 正常运行，位置稳定
[fastlio_mapping-1] [VoxelGrid] Points: 14026, Range: [-2.49,9.23] x [-0.48,8.91] x [-0.90,3.00]
[fastlio_mapping-1] [VoxelGrid] Points: 14048, Range: [-2.42,9.22] x [-0.48,8.91] x [-0.88,3.00]
# 位置在正常范围内（几米），不会突变

# ❌ 如果仍然发散
[fastlio_mapping-1] No Effective Points! pos_change=500+
# 可能需要标定 IMU 外参（extrinsic_T, extrinsic_R）
```

---

## 问题5: USB 串口名称变化

### 问题描述

**USB 串口设备名可能变化的场景**：
1. 拔插 USB 线
2. 重启 Jetson 或容器
3. 多个 USB 串口设备同时连接
4. 内核 USB 驱动重新加载

**后果**：
- `/dev/ttyUSB0` 变成 `/dev/ttyUSB1`
- 原脚本写死的 `/dev/ttyUSB0` 失效
- IMU 驱动无法启动

### 查找当前串口名称

```bash
# 方法1: 查看所有 USB 串口
ls -l /dev/ttyUSB* /dev/ttyACM*

# 方法2: 查看内核日志（最近插入的设备）
dmesg | grep -i "tty\|usb.*serial" | tail -20
# 输出示例：
# [12345.678] usb 1-2: cp210x converter now attached to ttyUSB0

# 方法3: 通过厂商/产品 ID 查找（最可靠）
ls -l /dev/serial/by-id/
# 输出示例：
# usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0 -> ../../ttyUSB0

# 方法4: 查看所有串口详细信息
for port in /dev/ttyUSB* /dev/ttyACM*; do
    udevadm info --name=$port | grep -E "DEVNAME|ID_VENDOR_ID|ID_MODEL_ID"
done
```

### 解决方案

#### 方案 A: 使用 udev 符号链接（推荐）

创建固定的符号链接 `/dev/imu`：

```bash
# 1. 获取 USB 设备 VID/PID
$ lsusb | grep CP210
Bus 001 Device 005: ID 10c4:ea60 Silicon Labs CP210x UART Bridge
#                      ^^^^:^^^^ 
#                      VID   PID

# 2. 创建 udev 规则
sudo tee /etc/udev/rules.d/99-handsfree-imu.rules > /dev/null <<'EOF'
# HandsFree IMU - CP2102 USB-UART
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", \
    SYMLINK+="imu", MODE="0666", GROUP="dialout"
EOF

# 3. 重新加载规则
sudo udevadm control --reload-rules
sudo udevadm trigger

# 4. 拔插 USB 后验证
$ ls -l /dev/imu
lrwxrwxrwx 1 root root 7 Jan 30 10:00 /dev/imu -> ttyUSB0
# 即使 ttyUSB0 变成 ttyUSB1，/dev/imu 始终指向正确设备
```

**修改启动脚本**：
```bash
IMU_LAUNCH="handsfree_imu_ros2 imu.launch.py port:=/dev/imu"
```

#### 方案 B: 启动时动态检测

修改 `start_all.sh`：
```bash
# 自动检测 CP2102 设备
IMU_PORT=$(ls -l /dev/serial/by-id/*CP2102* 2>/dev/null | awk '{print $NF}' | head -1)
if [ -z "$IMU_PORT" ]; then
    # 回退方案：尝试常见端口
    for port in /dev/ttyUSB0 /dev/ttyUSB1 /dev/ttyACM0; do
        if [ -e "$port" ]; then
            IMU_PORT=$port
            break
        fi
    done
fi

if [ -z "$IMU_PORT" ]; then
    echo "❌ 错误: 未找到 IMU 串口设备"
    exit 1
fi

echo "✓ IMU 串口: $IMU_PORT"
IMU_LAUNCH="handsfree_imu_ros2 imu.launch.py port:=$IMU_PORT"
```

#### 方案 C: 按序号绑定（高级）

如果有多个相同芯片的 USB 设备，按物理端口绑定：
```bash
# 查看 USB 拓扑
$ lsusb -t
/:  Bus 01.Port 1: Dev 1, Class=root_hub
    |__ Port 2: Dev 5, If 0, Class=Vendor Specific, CP210x

# 绑定到特定物理端口
SUBSYSTEM=="tty", KERNELS=="1-2:1.0", SYMLINK+="imu", MODE="0666"
```

---

## 完整解决方案

### 最终配置文件

#### 1. `start_all.sh`
```bash
#!/bin/bash
set -e

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${BLUE}========================================"
echo -e "  FAST-LIO2 系统启动脚本"
echo -e "========================================${NC}"

WORKSPACE_DIR="/home/rosdev/ros2_ws"

# Source ROS2环境
source /opt/ros/foxy/setup.bash
source ${WORKSPACE_DIR}/install/setup.bash

# 配置参数
LIDAR_LAUNCH="lslidar_driver lslidar_cx_launch.py"
IMU_LAUNCH="handsfree_imu_ros2 imu.launch.py port:=/dev/ttyUSB0"
FASTLIO_CONFIG="c16.yaml"

# 清理函数
cleanup() {
    echo -e "\n${YELLOW}正在清理进程...${NC}"
    pkill -f "lslidar_driver_node" 2>/dev/null || true
    pkill -f "imu_node" 2>/dev/null || true
    pkill -f "fastlio_mapping" 2>/dev/null || true
    pkill -f "rviz2" 2>/dev/null || true
    echo -e "${GREEN}清理完成${NC}"
    exit 0
}

trap cleanup SIGINT SIGTERM

# ========== 硬件初始化 ==========
echo -e "${YELLOW}正在初始化硬件...${NC}"

# 1. 配置网络 (激光雷达需要 100Mbps 全双工)
echo -e "${BLUE}配置网络接口...${NC}"
sudo ethtool -s eth0 speed 100 duplex full autoneg off 2>/dev/null && \
    echo -e "${GREEN}✓ 网络配置完成 (100Mbps Full Duplex)${NC}" || \
    echo -e "${YELLOW}⚠ 网络配置跳过 (可能已配置或无权限)${NC}"

# 2. 创建 USB 串口设备节点 (容器环境需要)
if [ ! -e /dev/ttyUSB0 ]; then
    echo -e "${BLUE}创建 USB 串口设备节点...${NC}"
    sudo mknod /dev/ttyUSB0 c 188 0 2>/dev/null && \
    sudo chmod 666 /dev/ttyUSB0 2>/dev/null && \
    echo -e "${GREEN}✓ /dev/ttyUSB0 已创建${NC}" || \
    echo -e "${RED}✗ 无法创建 /dev/ttyUSB0${NC}"
fi

# 检查设备
echo -e "${YELLOW}正在检查设备...${NC}"

if [ -e /dev/ttyUSB0 ]; then
    echo -e "${GREEN}✓ IMU 串口设备就绪 (/dev/ttyUSB0)${NC}"
else
    echo -e "${RED}✗ 错误: IMU 串口设备不存在 (/dev/ttyUSB0)${NC}"
    echo -e "${RED}  请检查 IMU 连接${NC}"
    echo -e "${YELLOW}  提示: 在宿主机运行 'ls /dev/ttyUSB*' 查看实际设备名${NC}"
fi

echo ""

# 先清理可能存在的旧进程
echo -e "${YELLOW}清理旧进程...${NC}"
pkill -f "lslidar_driver_node" 2>/dev/null || true
pkill -f "imu_node" 2>/dev/null || true
pkill -f "fastlio_mapping" 2>/dev/null || true
pkill -f "rviz2" 2>/dev/null || true
sleep 2

# 启动激光雷达驱动
echo -e "${BLUE}[1/3] 启动激光雷达驱动...${NC}"
ros2 launch ${LIDAR_LAUNCH} &
LIDAR_PID=$!
sleep 3

# 启动 IMU 驱动
echo -e "${BLUE}[2/3] 启动 IMU 驱动...${NC}"
ros2 launch ${IMU_LAUNCH} &
IMU_PID=$!
sleep 2

# 启动 FAST-LIO2
echo -e "${BLUE}[3/3] 启动 FAST-LIO2...${NC}"
ros2 launch fast_lio mapping.launch.py config_file:=${FASTLIO_CONFIG} rviz:=true &
FASTLIO_PID=$!
sleep 5

echo ""
echo -e "${GREEN}========================================"
echo -e "  所有节点已启动!"
echo -e "========================================${NC}"
echo -e "  激光雷达驱动 PID: ${LIDAR_PID}"
echo -e "  IMU驱动 PID: ${IMU_PID}"
echo -e "  FAST-LIO2 PID: ${FASTLIO_PID}"
echo ""

# 验证点云话题
echo -e "${YELLOW}正在验证系统状态...${NC}"
sleep 2

if ros2 topic list 2>/dev/null | grep -q "/cx/lslidar_point_cloud"; then
    echo -e "${GREEN}✓ 激光雷达工作正常${NC}"
    echo -e "${GREEN}✓ 系统已就绪${NC}"
    echo -e "${GREEN}  提示: RViz窗口应能看到点云数据${NC}"
else
    echo -e "${RED}✗ 未检测到激光雷达点云数据${NC}"
    echo -e "${YELLOW}  请检查: 雷达电源、网络连接 (192.168.1.200)${NC}"
fi
echo ""

echo -e "${YELLOW}按 Ctrl+C 停止所有节点${NC}"
echo ""

wait
```

#### 2. `config/c16.yaml`
```yaml
/**:
    ros__parameters:
        feature_extract_enable: false
        point_filter_num: 2
        max_iteration: 3
        filter_size_surf: 0.25
        filter_size_map: 0.25
        cube_side_length: 100.0
        runtime_pos_log_enable: false
        map_file_path: "./indoor_map.pcd"

        common:
            lid_topic: "/cx/lslidar_point_cloud"
            imu_topic: "/imu/data"
            time_sync_en: true
            time_offset_lidar_to_imu: 0.0

        preprocess:
            lidar_type: 2                # 2 = VELO16 (Velodyne格式，适用于Lslidar C16)
            scan_line: 16                # C16 是 16 线雷达
            scan_rate: 10
            timestamp_unit: 0            # 0 = 秒 (Lslidar使用秒为单位的float时间戳)
            blind: 0.3
            
        filter:
            voxel_size: 0.25

        mapping:
            acc_cov: 0.08
            gyr_cov: 0.004
            b_acc_cov: 0.0008
            b_gyr_cov: 0.00004
            fov_degree: 360.0
            det_range: 25.0
            minimum_pts: 50
            extrinsic_est_en: false
            extrinsic_T: [0.0, 0.0, 0.0]
            extrinsic_R: [1.0, 0.0, 0.0,
                          0.0, 1.0, 0.0,
                          0.0, 0.0, 1.0]
        
        imu:
            imu_en: true
            imu_rate: 200
            acc_n: 0.08
            gyr_n: 0.004
            acc_w: 0.0008
            gyr_w: 0.00004

        publish:
            path_en: true
            scan_publish_en: true
            dense_publish_en: true
            scan_bodyframe_pub_en: true
            effect_map_en: false
            map_en: true

        pcd_save:
            pcd_save_en: false
            interval: -1
```

### 一键启动流程

```bash
# 1. 确保在 Docker 容器内或已配置好环境
cd /home/rosdev/ros2_ws

# 2. 编译工作空间（首次或修改后）
source /opt/ros/foxy/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# 3. 运行启动脚本
bash start_all.sh
```

---

## 验证测试

### 1. 网络验证
```bash
# 检查网络配置
$ sudo ethtool eth0 | grep -E "Speed|Duplex|Auto"
        Speed: 100Mb/s
        Duplex: Full
        Auto-negotiation: off

# 监控数据包接收
$ watch -n 1 'ifconfig eth0 | grep "RX packets"'
RX packets:15234  # 应该持续增加
```

### 2. 激光雷达验证
```bash
# 检查话题
$ ros2 topic list | grep lidar
/cx/lslidar_point_cloud
/cx/lslidar_packet_difop
/cx/lslidar_packet_msop

# 查看点云频率
$ ros2 topic hz /cx/lslidar_point_cloud
average rate: 10.023
        min: 0.095s max: 0.102s std dev: 0.00234s window: 10

# 查看点云数据
$ ros2 topic echo /cx/lslidar_point_cloud --field width --once
28000  # C16 约 28000 点/帧
```

### 3. IMU 验证
```bash
# 检查话题
$ ros2 topic list | grep imu
/imu/data
/imu/mag

# 查看 IMU 频率
$ ros2 topic hz /imu/data
average rate: 200.145
        min: 0.004s max: 0.006s std dev: 0.00012s window: 200

# 查看 IMU 数据
$ ros2 topic echo /imu/data --field orientation --once
x: 0.0123
y: -0.0045
z: 0.9876
w: 0.1234
```

### 4. FAST-LIO 验证
```bash
# 检查建图话题
$ ros2 topic list | grep fast
/Odometry
/path
/cloud_registered
/cloud_effected

# 监控日志（无错误/警告）
$ ros2 launch fast_lio mapping.launch.py config_file:=c16.yaml

# ✅ 正常输出
[fastlio_mapping-1] [VoxelGrid] Points: 14026, Range: [-2.49,9.23] x [-0.48,8.91]
[imu_node-1] [INFO] 📊 IMU数据 #3600: 姿态=[-1.10°, -0.37°, 195.55°]

# ❌ 异常输出（需检查配置）
[fastlio_mapping-1] Failed to find match for field...
[fastlio_mapping-1] No Effective Points! pos_change=500+
```

### 5. 完整系统测试

在 RViz 中验证：
1. **点云显示**: 应看到连续更新的 3D 点云
2. **轨迹显示**: `/path` 话题显示机器人运动轨迹
3. **里程计**: `/Odometry` 输出稳定，无突变
4. **建图效果**: 移动机器人，地图应平滑累积

---

## 常见问题排查

### Q1: 激光雷达重启后又没数据了
**原因**: 网络配置被重置（某些系统重启后会恢复默认）

**解决**: 
```bash
# 临时
sudo ethtool -s eth0 speed 100 duplex full autoneg off

# 永久（参考问题1的 systemd 服务）
```

### Q2: 容器重启后 IMU 不工作
**原因**: `/dev/ttyUSB0` 设备节点未重新创建

**解决**: 运行 `start_all.sh` 会自动创建，或手动：
```bash
sudo mknod /dev/ttyUSB0 c 188 0 && sudo chmod 666 /dev/ttyUSB0
```

### Q3: 建图一段时间后突然消失
**可能原因**:
1. 激光雷达网络中断 → 检查网络配置
2. 机器人快速运动导致退化 → 降低速度
3. 环境特征不足（空旷走廊） → 增加 `minimum_pts`
4. IMU 外参不准确 → 需要标定

**调试**:
```bash
# 查看是否有点云输入
ros2 topic hz /cx/lslidar_point_cloud

# 查看 FAST-LIO 日志
# 正常: VoxelGrid Points: 14000+
# 异常: No Effective Points
```

### Q4: 编译 FAST-LIO 报错
**常见错误**:
```bash
# 缺少依赖
sudo apt install ros-foxy-pcl-ros ros-foxy-eigen3-cmake-module

# PCL 版本不兼容（警告可忽略）
# 只要最后显示 "Finished <<< fast_lio" 即可
```

---

## 参考资料

### 硬件文档
- [Lslidar C16 用户手册](http://www.lslidar.com/)
- [HandsFree IMU 规格说明](https://github.com/HANDS-FREE)
- [CP2102 数据手册](https://www.silabs.com/documents/public/data-sheets/CP2102-9.pdf)

### 软件文档
- [FAST-LIO2 论文](https://github.com/hku-mars/FAST_LIO)
- [ROS2 Foxy 文档](https://docs.ros.org/en/foxy/)
- [PCL 点云库](https://pointclouds.org/)

### 工具命令速查

```bash
# 网络诊断
sudo ethtool eth0                    # 查看网络配置
ifconfig eth0                        # 查看网络统计
sudo tcpdump -i eth0 port 2368       # 抓包验证雷达数据

# 串口诊断
ls -l /dev/ttyUSB*                   # 列出串口
dmesg | grep tty                     # 查看串口内核日志
sudo cat /dev/ttyUSB0                # 测试串口读取

# ROS2 诊断
ros2 topic list                      # 列出所有话题
ros2 topic hz /topic_name            # 查看话题频率
ros2 topic echo /topic_name          # 查看话题数据
ros2 node list                       # 列出所有节点

# 系统诊断
lsusb                                # 列出 USB 设备
dmesg -T                             # 查看系统日志（带时间戳）
htop                                 # 查看系统资源
```

---

## 更新记录

| 日期 | 版本 | 更新内容 | 作者 |
|------|------|---------|------|
| 2026-01-30 | 1.0 | 初始版本，记录完整故障排除流程 | - |

---

## 贡献

如果你遇到其他问题并找到解决方案，欢迎提交 PR 更新本文档！

## 许可证

本文档采用 [CC BY-SA 4.0](https://creativecommons.org/licenses/by-sa/4.0/) 许可证。

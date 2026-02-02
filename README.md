# FAST-LIO2 ROS2 工作区

基于 Lslidar C16 雷达和 HandsFree IMU 的 FAST-LIO2 建图系统。

## 📁 目录结构

```
ros2_ws/
├── README.md              # 本文件
├── start_all.sh           # 🚀 主启动脚本（一键启动所有节点）
│
├── src/                   # ROS2 源代码包
│   ├── FAST_LIO_ROS2/     # FAST-LIO2 主程序
│   ├── Lslidar_ROS2_driver/  # 镭神雷达驱动
│   ├── handsfree_ros_imu/    # HandsFree IMU 驱动
│   └── ...
│
├── config/                # 配置文件
│   └── c16.yaml           # Lslidar C16 雷达参数
│
├── scripts/               # 启动脚本
│   └── lslidar_no_rviz.launch.py  # 雷达驱动启动（无RViz）
│
├── utils/                 # 工具脚本
│   ├── monitor_resources.sh      # 📊 系统资源监控
│   └── start_fastlio_full.sh     # 完整启动脚本（备用）
│
├── docs/                  # 文档
│   ├── QUICK_START.md            # 快速开始指南
│   ├── TROUBLESHOOTING.md        # 故障排查
│   ├── DEPLOYMENT_SUMMARY.md     # 部署总结
│   └── ...
│
├── build/                 # 编译生成文件
├── install/               # 安装文件
└── log/                   # 日志文件
```

## 🚀 快速开始

### 1. 启动建图系统

```bash
cd /home/rosdev/ros2_ws
bash start_all.sh
```

这会自动启动：
- ✅ 激光雷达驱动 (Lslidar C16)
- ✅ IMU 驱动 (HandsFree A9)
- ✅ FAST-LIO2 建图节点
- ✅ RViz2 可视化

### 2. 监控系统资源（可选）

打开新终端：
```bash
cd /home/rosdev/ros2_ws
./utils/monitor_resources.sh
```

实时显示：CPU、GPU、内存、温度、进程状态

### 3. 查看建图日志

```bash
tail -f /tmp/fastlio.log | grep VoxelGrid
```

### 4. 停止系统

按 `Ctrl+C` 或：
```bash
pkill -9 -f lslidar; pkill -9 -f imu_node; pkill -9 -f fastlio; pkill -9 -f rviz2
```

## ⚙️ 配置文件

### FAST-LIO2 配置
- 主配置：`src/FAST_LIO_ROS2/config/c16.yaml`
- RViz配置：`src/FAST_LIO_ROS2/rviz/fastlio.rviz`

### 当前参数设置
- 雷达扫描频率：10 Hz (实测 ~9.1 Hz)
- IMU 频率：200 Hz (实测 ~152 Hz)
- 建图范围：50 米
- 时间同步：**禁用** (time_sync_en=false) ⚠️ 启用会导致发散
- 外参估计：启用 (extrinsic_est_en=true)
- 最小有效点数：50 (minimum_pts=50)

## 🔧 硬件配置

- **平台**: Jetson Xavier NX (Ubuntu 20.04.6 LTS)
- **激光雷达**: Lslidar C16 v3.0 (16线, 10Hz)
  - IP: 192.168.1.200
  - 本机: 192.168.1.102
  - 端口: 2368 (数据), 2369 (设备信息)
  
- **IMU**: HandsFree A9
  - 接口: USB-UART (CP2102)
  - 设备: /dev/ttyUSB0
  - 波特率: 921600
  - 实际频率: ~152 Hz

## 📚 文档

- [快速开始指南](docs/QUICK_START.md)
- [故障排查](docs/TROUBLESHOOTING.md)
- [部署总结](docs/DEPLOYMENT_SUMMARY.md)

## 🛠️ 常用命令

```bash
# 编译工作区
colcon build --packages-select fast_lio

# 查看话题
ros2 topic list

# 测量话题频率
ros2 topic hz /cx/lslidar_point_cloud
ros2 topic hz /imu/data

# 保存地图
ros2 service call /save_map std_srvs/srv/Empty
# 地图保存位置: src/FAST_LIO_ROS2/PCD/indoor_map.pcd
```

## 📝 重要注意事项

### ⚠️ 关键配置说明

1. **时间同步必须禁用**: `time_sync_en: false`
   - USB IMU 与雷达时间戳不同步
   - 启用会导致系统立即发散（位置值达到数百万）

2. **网络配置**: 启动前自动配置 eth0
   - 必须强制 100Mbps 全双工模式
   - 自动协商会导致丢包

3. **USB设备**: 确保 /dev/ttyUSB0 存在且权限正确

4. **窗户反射问题**: 通过 det_range=50 和 minimum_pts=50 缓解

### 📊 正常工作指标

查看 `/tmp/fastlio.log`:
```
VoxelGrid filter: Points 7900~8100, Range [7m × 12m × 3.5m]
```

### ❌ 异常情况处理

- 如看到 `OutOfRange`, `dist_fail` 增加 → 检查配置
- 如看到 `No Effective Points` → 检查雷达连接
- 如位置值异常大 → 确认 `time_sync_en: false`

## 🔗 相关链接

- GitHub: https://github.com/superclash1/FAST_LIO_ROS2
- FAST-LIO2 原始仓库: https://github.com/hku-mars/FAST_LIO

---

**最后更新**: 2026-02-01  
**配置状态**: 室内走廊建图稳定版本

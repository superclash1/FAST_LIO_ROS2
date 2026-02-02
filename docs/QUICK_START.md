# 快速开始指南

## 📚 重要文档链接

| 文档 | 说明 |
|------|------|
| [TROUBLESHOOTING.md](./TROUBLESHOOTING.md) | 🔧 **完整故障排除文档** - 详细的问题诊断和解决方案 |
| [start_all.sh](./start_all.sh) | 🚀 **一键启动脚本** - 自动配置并启动系统 |
| [README.md](./README.md) | 📖 项目说明文档 |

## 🎯 一键启动

```bash
# 1. 进入工作空间
cd /home/rosdev/ros2_ws

# 2. 直接运行（脚本会自动完成所有配置）
bash start_all.sh
```

脚本会自动：
- ✅ 配置网络（100Mbps 全双工）
- ✅ 创建 USB 串口设备节点
- ✅ 启动激光雷达驱动
- ✅ 启动 IMU 驱动
- ✅ 启动 FAST-LIO2 建图

## ⚠️ 遇到问题？

**立即查看**: [TROUBLESHOOTING.md](./TROUBLESHOOTING.md)

常见问题快速索引：
- [激光雷达无数据](./TROUBLESHOOTING.md#问题1-激光雷达无数据) - 网络配置问题
- [IMU 设备不存在](./TROUBLESHOOTING.md#问题2-imu-串口设备不存在) - 串口设备节点
- [FAST-LIO 警告](./TROUBLESHOOTING.md#问题3-fast-lio-警告信息) - 配置错误
- [建图消失/退化](./TROUBLESHOOTING.md#问题4-建图退化与位姿发散) - 时间同步问题
- [串口名称变化](./TROUBLESHOOTING.md#问题5-usb-串口名称变化) - USB 设备管理

## 🔍 快速诊断命令

```bash
# 检查网络配置
sudo ethtool eth0 | grep -E "Speed|Duplex|Auto"

# 检查串口设备
ls -l /dev/ttyUSB*

# 检查 ROS2 话题
ros2 topic list | grep -E "lidar|imu"

# 查看激光雷达数据频率
ros2 topic hz /cx/lslidar_point_cloud

# 查看 IMU 数据频率
ros2 topic hz /imu/data
```

## 📦 系统配置

- **激光雷达**: Lslidar C16 v3.0 (192.168.1.200)
- **IMU**: HandsFree A9 (CP2102 USB, 921600 baud)
- **ROS2**: Foxy (Ubuntu 20.04)
- **平台**: Jetson Xavier NX

## 🌐 在线资源

- **GitHub 仓库**: https://github.com/superclash1/FAST_LIO_ROS2
- **问题反馈**: 提交 Issue 到 GitHub

---

**提示**: 首次运行请完整阅读 [TROUBLESHOOTING.md](./TROUBLESHOOTING.md)，了解系统架构和可能遇到的问题。

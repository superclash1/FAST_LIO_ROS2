#!/bin/bash
# 一键启动 FAST-LIO2 系统
# 包括: 激光雷达驱动 + IMU驱动 + FAST-LIO2

set -e

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}  FAST-LIO2 系统启动脚本${NC}"
echo -e "${BLUE}========================================${NC}"

# 设置工作区路径
WORKSPACE_DIR="/home/rosdev/ros2_ws"

# Source ROS2环境
source /opt/ros/foxy/setup.bash
source ${WORKSPACE_DIR}/install/setup.bash

# 配置参数
LIDAR_LAUNCH="lslidar_driver lslidar_cx_launch.py"
IMU_LAUNCH="handsfree_imu_ros2 imu.launch.py port:=/dev/ttyUSB0"
FASTLIO_CONFIG="c16.yaml"

# 函数：清理后台进程
cleanup() {
    echo -e "\n${YELLOW}正在清理进程...${NC}"
    pkill -f "lslidar_driver_node" 2>/dev/null || true
    pkill -f "imu_node" 2>/dev/null || true
    pkill -f "fastlio_mapping" 2>/dev/null || true
    pkill -f "rviz2" 2>/dev/null || true
    echo -e "${GREEN}清理完成${NC}"
    exit 0
}

# 捕获 Ctrl+C
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

# 检查 IMU 串口
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
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}  所有节点已启动!${NC}"
echo -e "${GREEN}========================================${NC}"
echo -e "  激光雷达驱动 PID: ${LIDAR_PID}"
echo -e "  IMU驱动 PID: ${IMU_PID}"
echo -e "  FAST-LIO2 PID: ${FASTLIO_PID}"
echo ""

# 验证点云话题
echo -e "${YELLOW}正在验证系统状态...${NC}"
sleep 2  # 等待点云驱动完全初始化

# 检测点云话题是否发布
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

# 等待进程
wait

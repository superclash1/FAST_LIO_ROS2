#!/bin/bash

# 颜色定义
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${BLUE}=============================================${NC}"
echo -e "${BLUE}🚀 FAST-LIO2 完整系统自动启动${NC}"
echo -e "${BLUE}=============================================${NC}"
echo ""

cd /home/rosdev/ros2_ws
source install/setup.bash

# 设置串口权限
echo -e "${YELLOW}步骤 1/4: 设置IMU串口权限...${NC}"
sudo chmod 666 /dev/ttyUSB0 2>/dev/null && echo -e "   ${GREEN}✅ 串口权限已设置${NC}" || echo -e "   ${RED}⚠️  串口权限设置失败${NC}"
sleep 1

# 启动雷达
echo ""
echo -e "${YELLOW}步骤 2/4: 启动LSLidar C16雷达驱动...${NC}"
ros2 launch lslidar_driver lslidar_cx_launch.py > /tmp/lidar.log 2>&1 &
LIDAR_PID=$!
echo -e "   ${GREEN}✅ 雷达驱动已启动 (PID: $LIDAR_PID)${NC}"
sleep 3

# 启动IMU
echo ""
echo -e "${YELLOW}步骤 3/4: 启动HandsFree A9 IMU驱动...${NC}"
ros2 launch handsfree_imu_ros2 imu.launch.py port:=/dev/ttyUSB0 > /tmp/imu.log 2>&1 &
IMU_PID=$!
echo -e "   ${GREEN}✅ IMU驱动已启动 (PID: $IMU_PID)${NC}"
sleep 2

# 检查传感器状态
echo ""
echo -e "${YELLOW}步骤 4/4: 检查传感器状态...${NC}"
LIDAR_TOPICS=$(ros2 topic list 2>/dev/null | grep lslidar | wc -l)
IMU_TOPICS=$(ros2 topic list 2>/dev/null | grep imu | wc -l)

if [ "$LIDAR_TOPICS" -gt 0 ]; then
    echo -e "   ${GREEN}✅ 雷达: $LIDAR_TOPICS 个话题${NC}"
    ros2 topic list 2>/dev/null | grep lslidar | sed 's/^/      /'
else
    echo -e "   ${RED}❌ 雷达: 未检测到话题${NC}"
fi

if [ "$IMU_TOPICS" -gt 0 ]; then
    echo -e "   ${GREEN}✅ IMU: $IMU_TOPICS 个话题${NC}"
    ros2 topic list 2>/dev/null | grep imu | sed 's/^/      /'
else
    echo -e "   ${RED}❌ IMU: 未检测到话题${NC}"
fi

echo ""
echo -e "${GREEN}=============================================${NC}"
echo -e "${GREEN}✅ 传感器系统已就绪！${NC}"
echo -e "${GREEN}=============================================${NC}"
echo ""
echo -e "${BLUE}📊 现在启动FAST-LIO建图...${NC}"
echo ""
sleep 2

# 启动FAST-LIO
ros2 launch fast_lio mapping.launch.py config_file:=c16.yaml rviz:=true

# 捕获退出信号，清理进程
trap "echo ''; echo -e '${YELLOW}正在关闭所有进程...${NC}'; kill $LIDAR_PID $IMU_PID 2>/dev/null; echo -e '${GREEN}✅ 已关闭${NC}'; exit 0" SIGINT SIGTERM

wait

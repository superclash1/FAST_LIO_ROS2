#!/bin/bash

echo "========================================="
echo "🔍 FAST-LIO 诊断脚本"
echo "========================================="
echo ""

cd /home/rosdev/ros2_ws
source install/setup.bash

echo "1️⃣  检查雷达数据频率..."
timeout 5 ros2 topic hz /cx/lslidar_point_cloud 2>&1 | grep "average rate" | head -n 2 || echo "  ❌ 雷达无数据"

echo ""
echo "2️⃣  检查IMU数据频率..."
timeout 5 ros2 topic hz /imu/data 2>&1 | grep "average rate" | head -n 2 || echo "  ❌ IMU无数据"

echo ""
echo "3️⃣  检查雷达数据内容..."
timeout 2 ros2 topic echo /cx/lslidar_point_cloud --once 2>&1 | head -n 20 || echo "  ❌ 无法读取雷达数据"

echo ""
echo "4️⃣  检查IMU数据内容..."
timeout 2 ros2 topic echo /imu/data 2>&1 | head -n 20 || echo "  ❌ 无法读取IMU数据"

echo ""
echo "========================================="

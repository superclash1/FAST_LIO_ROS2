#!/bin/bash

echo "========================================="
echo "完整系统测试 - 雷达 + IMU + FAST-LIO"
echo "========================================="
echo ""

cd /home/rosdev/ros2_ws
source install/setup.bash

# 检查雷达话题
echo "1️⃣  检查雷达话题..."
LIDAR_TOPICS=$(ros2 topic list | grep -E "(lslidar|scan|points)")
if [ -z "$LIDAR_TOPICS" ]; then
    echo "   ❌ 未找到雷达话题"
else
    echo "   ✅ 雷达话题:"
    echo "$LIDAR_TOPICS" | sed 's/^/      /'
    echo ""
    LIDAR_TOPIC=$(echo "$LIDAR_TOPICS" | head -n 1)
    echo "   📊 雷达频率测试 ($LIDAR_TOPIC):"
    timeout 5 ros2 topic hz "$LIDAR_TOPIC" 2>&1 | grep "average rate" | head -n 2
fi

echo ""
# 检查IMU话题
echo "2️⃣  检查IMU话题..."
IMU_TOPICS=$(ros2 topic list | grep imu)
if [ -z "$IMU_TOPICS" ]; then
    echo "   ❌ 未找到IMU话题"
else
    echo "   ✅ IMU话题:"
    echo "$IMU_TOPICS" | sed 's/^/      /'
    echo ""
    echo "   📊 IMU频率测试 (/imu/data):"
    timeout 5 ros2 topic hz /imu/data 2>&1 | grep "average rate" | head -n 2
fi

echo ""
echo "3️⃣  检查运行的节点..."
ros2 node list

echo ""
echo "========================================="
echo "测试完成！"
echo "========================================="
echo ""
echo "📝 下一步操作:"
echo ""
echo "如果雷达和IMU数据都正常，启动FAST-LIO:"
echo "  ros2 launch fast_lio mapping.launch.py config_file:=c16.yaml rviz:=true"
echo ""

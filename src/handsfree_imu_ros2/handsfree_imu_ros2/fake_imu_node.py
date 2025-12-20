#!/usr/bin/env python3
# -*- coding:utf-8 -*-
"""
模拟 IMU 数据发布器
用于测试 FAST-LIO 在没有真实 IMU 硬件时的工作

假设静止状态，加速度仅有重力分量
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import math


class FakeIMUPublisher(Node):
    def __init__(self):
        super().__init__('fake_imu_publisher')
        
        # 参数
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('rate', 200.0)  # 200 Hz
        
        imu_topic = self.get_parameter('imu_topic').value
        self.frame_id = self.get_parameter('frame_id').value
        rate = self.get_parameter('rate').value
        
        # 发布器
        self.imu_pub = self.create_publisher(Imu, imu_topic, 10)
        
        # 定时器
        self.timer = self.create_timer(1.0 / rate, self.publish_imu)
        
        self.get_logger().info(f'🎭 模拟 IMU 发布器已启动')
        self.get_logger().info(f'   话题: {imu_topic}')
        self.get_logger().info(f'   频率: {rate} Hz')
        self.get_logger().info(f'   坐标系: {self.frame_id}')
        
        self.count = 0
        
    def publish_imu(self):
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        
        # 姿态四元数 (静止，无旋转)
        msg.orientation.w = 1.0
        msg.orientation.x = 0.0
        msg.orientation.y = 0.0
        msg.orientation.z = 0.0
        
        # 角速度 (静止)
        msg.angular_velocity.x = 0.0
        msg.angular_velocity.y = 0.0
        msg.angular_velocity.z = 0.0
        
        # 线加速度 (仅重力)
        msg.linear_acceleration.x = 0.0
        msg.linear_acceleration.y = 0.0
        msg.linear_acceleration.z = 9.81  # 重力加速度
        
        # 协方差
        msg.orientation_covariance = [
            0.01, 0.0, 0.0,
            0.0, 0.01, 0.0,
            0.0, 0.0, 0.01
        ]
        msg.angular_velocity_covariance = [
            0.001, 0.0, 0.0,
            0.0, 0.001, 0.0,
            0.0, 0.0, 0.001
        ]
        msg.linear_acceleration_covariance = [
            0.001, 0.0, 0.0,
            0.0, 0.001, 0.0,
            0.0, 0.0, 0.001
        ]
        
        self.imu_pub.publish(msg)
        
        self.count += 1
        if self.count % 200 == 0:
            self.get_logger().info(f'📊 已发布 {self.count} 条 IMU 数据')


def main(args=None):
    rclpy.init(args=args)
    node = FakeIMUPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

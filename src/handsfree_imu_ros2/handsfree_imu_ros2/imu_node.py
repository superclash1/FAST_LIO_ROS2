#!/usr/bin/env python3
# -*- coding:utf-8 -*-

"""
HandsFree A9/TBA9 IMU Driver for ROS2
Ported from handsfree_ros_imu (ROS1) to ROS2
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
import serial
import struct
import math


class HandsfreeIMUNode(Node):
    def __init__(self):
        super().__init__('handsfree_imu_node')
        
        # 声明参数
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 921600)
        self.declare_parameter('imu_frame_id', 'imu_link')
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('mag_topic', '/imu/mag')
        self.declare_parameter('gra_normalization', True)  # 重力归一化
        
        # 获取参数
        self.port = self.get_parameter('port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.imu_frame_id = self.get_parameter('imu_frame_id').value
        imu_topic = self.get_parameter('imu_topic').value
        mag_topic = self.get_parameter('mag_topic').value
        self.gra_normalization = self.get_parameter('gra_normalization').value
        
        # 创建发布器
        self.imu_pub = self.create_publisher(Imu, imu_topic, 10)
        self.mag_pub = self.create_publisher(MagneticField, mag_topic, 10)
        
        # 初始化数据缓存
        self.buff = {}
        self.key = 0
        self.angle_degree = [0.0, 0.0, 0.0]
        self.magnetometer = [0.0, 0.0, 0.0]
        self.acceleration = [0.0, 0.0, 0.0]
        self.angular_velocity = [0.0, 0.0, 0.0]
        self.pub_flag = [True, True]
        self.timestamp = 0.0
        self.data_right_count = 0  # 数据错误计数器
        self.serial_port = None  # 串口对象初始化为None，支持重连
        
        # 串口打开将在定时器中处理，支持自动重连
        self.get_logger().info(f'📡 IMU驱动初始化完成')
        self.get_logger().info(f'   串口: {self.port}, 波特率: {self.baudrate}')
        self.get_logger().info(f'   IMU话题: {imu_topic}')
        self.get_logger().info(f'   磁力计话题: {mag_topic}')
        self.get_logger().info(f'   重力归一化: {self.gra_normalization}')
        
        # 创建定时器读取数据
        self.timer = self.create_timer(0.001, self.read_serial_data)  # 1ms = 1000Hz
        
        self.msg_count = 0
        
    def checksum(self, list_data, check_data):
        """CRC校验"""
        data = bytearray(list_data)
        crc = 0xFFFF
        for pos in data:
            crc ^= pos
            for i in range(8):
                if (crc & 1) != 0:
                    crc >>= 1
                    crc ^= 0xA001
                else:
                    crc >>= 1
        return hex(((crc & 0xff) << 8) + (crc >> 8)) == hex(check_data[0] << 8 | check_data[1])
    
    def hex_to_ieee(self, raw_data):
        """16进制转IEEE浮点数"""
        ieee_data = []
        raw_data.reverse()
        for i in range(0, len(raw_data), 4):
            data2str = (hex(raw_data[i] | 0xff00)[4:6] +
                       hex(raw_data[i + 1] | 0xff00)[4:6] +
                       hex(raw_data[i + 2] | 0xff00)[4:6] +
                       hex(raw_data[i + 3] | 0xff00)[4:6])
            ieee_data.append(struct.unpack('>f', bytes.fromhex(data2str))[0])
        ieee_data.reverse()
        return ieee_data
    
    def handle_serial_data(self, raw_data):
        """处理串口数据"""
        # 数据错误检测 - 保留ROS1原版功能
        if self.data_right_count > 200000:
            self.get_logger().error('❌ 设备传输数据错误次数过多，退出节点')
            raise RuntimeError('IMU数据传输错误')
        
        self.buff[self.key] = raw_data
        self.key += 1
        
        if self.buff[0] != 0xaa:
            self.data_right_count += 1
            self.key = 0
            return
        if self.key < 3:
            return
        if self.buff[1] != 0x55:
            self.key = 0
            return
        if self.key < self.buff[2] + 5:
            return
        
        # 数据正确，重置错误计数
        self.data_right_count = 0
        data_buff = list(self.buff.values())
        
        # 处理0x2c数据包 (角速度、加速度、磁力计)
        if self.buff[2] == 0x2c:
            if self.checksum(data_buff[2:47], data_buff[47:49]):
                ts_us = ((data_buff[10] & 0xFF) << 24) | ((data_buff[9] & 0xFF) << 16) | \
                       ((data_buff[8] & 0xFF) << 8) | (data_buff[7] & 0xFF)
                self.timestamp = ts_us / 1e6
                
                data = self.hex_to_ieee(data_buff[7:47])
                self.angular_velocity = data[1:4]
                self.acceleration = data[4:7]
                self.magnetometer = data[7:10]
                
                # 发布磁力计数据
                self.publish_mag_data()
                self.pub_flag[0] = True  # 收到0x2c数据
            else:
                self.get_logger().warn('0x2c 数据校验失败')
            
        # 处理0x14数据包 (姿态角)
        elif self.buff[2] == 0x14:
            if self.checksum(data_buff[2:23], data_buff[23:25]):
                data = self.hex_to_ieee(data_buff[7:23])
                self.angle_degree = data[1:4]
                
                # 发布IMU数据 (无论是否有0x2c数据都发布)
                self.publish_imu_data()
            else:
                self.get_logger().warn('0x14 数据校验失败')
        
        self.key = 0
        self.buff = {}
    
    def publish_imu_data(self):
        """发布IMU数据"""
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.imu_frame_id
        
        # 将欧拉角转换为四元数
        roll = math.radians(self.angle_degree[0])
        pitch = math.radians(self.angle_degree[1])
        yaw = math.radians(self.angle_degree[2])
        
        # 欧拉角 -> 四元数
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        
        msg.orientation.w = cy * cp * cr + sy * sp * sr
        msg.orientation.x = cy * cp * sr - sy * sp * cr
        msg.orientation.y = sy * cp * sr + cy * sp * cr
        msg.orientation.z = sy * cp * cr - cy * sp * sr
        
        # 角速度 (rad/s)
        msg.angular_velocity.x = self.angular_velocity[0]
        msg.angular_velocity.y = self.angular_velocity[1]
        msg.angular_velocity.z = self.angular_velocity[2]
        
        # 线加速度 (m/s^2) - 保留ROS1原版的重力归一化功能
        acc_k = math.sqrt(self.acceleration[0]**2 + self.acceleration[1]**2 + self.acceleration[2]**2)
        if acc_k == 0:
            acc_k = 1
        
        if self.gra_normalization:
            # 重力归一化：归一化到9.8 m/s^2
            msg.linear_acceleration.x = self.acceleration[0] * 9.8 / acc_k
            msg.linear_acceleration.y = self.acceleration[1] * 9.8 / acc_k
            msg.linear_acceleration.z = self.acceleration[2] * 9.8 / acc_k
        else:
            # 直接乘以重力加速度
            msg.linear_acceleration.x = self.acceleration[0] * 9.8
            msg.linear_acceleration.y = self.acceleration[1] * 9.8
            msg.linear_acceleration.z = self.acceleration[2] * 9.8
        
        # 设置协方差
        msg.orientation_covariance = [0.01, 0.0, 0.0,
                                      0.0, 0.01, 0.0,
                                      0.0, 0.0, 0.01]
        msg.angular_velocity_covariance = [0.001, 0.0, 0.0,
                                           0.0, 0.001, 0.0,
                                           0.0, 0.0, 0.001]
        msg.linear_acceleration_covariance = [0.001, 0.0, 0.0,
                                              0.0, 0.001, 0.0,
                                              0.0, 0.0, 0.001]
        
        self.imu_pub.publish(msg)
        
        self.msg_count += 1
        if self.msg_count % 200 == 0:
            self.get_logger().info(
                f'📊 IMU数据 #{self.msg_count}: '
                f'姿态=[{self.angle_degree[0]:.2f}°, {self.angle_degree[1]:.2f}°, {self.angle_degree[2]:.2f}°] '
                f'角速度=[{self.angular_velocity[0]:.3f}, {self.angular_velocity[1]:.3f}, {self.angular_velocity[2]:.3f}]'
            )
    
    def publish_mag_data(self):
        """发布磁力计数据"""
        msg = MagneticField()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.imu_frame_id
        
        msg.magnetic_field.x = self.magnetometer[0]
        msg.magnetic_field.y = self.magnetometer[1]
        msg.magnetic_field.z = self.magnetometer[2]
        
        msg.magnetic_field_covariance = [0.0, 0.0, 0.0,
                                         0.0, 0.0, 0.0,
                                         0.0, 0.0, 0.0]
        
        self.mag_pub.publish(msg)
    
    def read_serial_data(self):
        """读取串口数据 - 包含自动重连机制(保留ROS1原版功能)"""
        # 串口未打开，尝试打开
        if self.serial_port is None:
            try:
                self.serial_port = serial.Serial(
                    port=self.port,
                    baudrate=self.baudrate,
                    timeout=0.5
                )
                if self.serial_port.is_open:
                    self.get_logger().info(f'✅ 成功打开串口: {self.port}')
            except Exception as e:
                self.get_logger().warn(f'⚠️  串口打开失败: {e}，1秒后重试...')
                self.serial_port = None
                return
        
        # 读取串口数据
        try:
            if self.serial_port.in_waiting > 0:
                buff_count = self.serial_port.in_waiting
                buff_data = self.serial_port.read(buff_count)
                for i in range(buff_count):
                    self.handle_serial_data(buff_data[i])
        except (serial.SerialException, IOError) as e:
            self.get_logger().error(f'❌ 串口错误: {e}，准备重连...')
            if self.serial_port is not None:
                self.serial_port.close()
            self.serial_port = None
        except Exception as e:
            self.get_logger().error(f'❌ 数据处理错误: {e}')
    
    def destroy_node(self):
        """节点销毁时关闭串口"""
        if hasattr(self, 'serial_port') and self.serial_port.is_open:
            self.serial_port.close()
            self.get_logger().info('串口已关闭')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = HandsfreeIMUNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

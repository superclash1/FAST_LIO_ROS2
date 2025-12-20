#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # 声明launch参数
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='/dev/ttyUSB0',
        description='IMU串口设备路径'
    )
    
    baudrate_arg = DeclareLaunchArgument(
        'baudrate',
        default_value='921600',
        description='串口波特率'
    )
    
    imu_frame_id_arg = DeclareLaunchArgument(
        'imu_frame_id',
        default_value='imu_link',
        description='IMU坐标系名称'
    )
    
    imu_topic_arg = DeclareLaunchArgument(
        'imu_topic',
        default_value='/imu/data',
        description='IMU数据话题名称'
    )
    
    mag_topic_arg = DeclareLaunchArgument(
        'mag_topic',
        default_value='/imu/mag',
        description='磁力计数据话题名称'
    )
    
    # IMU节点
    imu_node = Node(
        package='handsfree_imu_ros2',
        executable='imu_node',
        name='handsfree_imu_node',
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('port'),
            'baudrate': LaunchConfiguration('baudrate'),
            'imu_frame_id': LaunchConfiguration('imu_frame_id'),
            'imu_topic': LaunchConfiguration('imu_topic'),
            'mag_topic': LaunchConfiguration('mag_topic'),
        }]
    )
    
    return LaunchDescription([
        port_arg,
        baudrate_arg,
        imu_frame_id_arg,
        imu_topic_arg,
        mag_topic_arg,
        imu_node,
    ])

# FAST-LIO2 配置文件

此目录包含针对不同硬件的 FAST-LIO2 配置文件。

## c16.yaml - Lslidar C16 配置

**适用硬件**:
- 激光雷达: Lslidar C16 v3.0
- IMU: HandsFree A9/TBA9

**关键配置**:
```yaml
preprocess:
    lidar_type: 2                # VELO16 (Velodyne格式，兼容 Lslidar C16)
    timestamp_unit: 0            # 秒 (Lslidar 使用秒为单位的时间戳)
```

**使用方法**:

1. **复制到 FAST_LIO_ROS2**:
   ```bash
   cp config/c16.yaml src/FAST_LIO_ROS2/config/
   ```

2. **启动 FAST-LIO**:
   ```bash
   ros2 launch fast_lio mapping.launch.py config_file:=c16.yaml
   ```

   或使用一键启动脚本:
   ```bash
   bash start_all.sh
   ```

## 配置说明

### lidar_type 参数
- `1` = AVIA (Livox Avia)
- `2` = VELO16 (Velodyne VLP-16 及兼容格式)
- `3` = OUST64 (Ouster OS1-64/OS2-64)
- `4` = MID360 (Livox MID-360)

**注意**: Lslidar C16 使用 Velodyne 兼容格式，应设置为 `2`

### timestamp_unit 参数
- `0` = 秒 (SEC)
- `1` = 毫秒 (MS)
- `2` = 微秒 (US)
- `3` = 纳秒 (NS)

**注意**: Lslidar C16 的时间戳是 float 类型，单位为秒，应设置为 `0`

## 故障排除

如果遇到以下问题，请检查配置：

1. **警告: Failed to find match for field 't', 'reflectivity'...**
   - 原因: `lidar_type` 设置错误
   - 解决: 确保设置为 `2` (VELO16)

2. **建图退化/位姿发散**
   - 原因: `timestamp_unit` 设置错误导致时间同步失败
   - 解决: 确保设置为 `0` (秒)

详细故障排除请参考: [TROUBLESHOOTING.md](../TROUBLESHOOTING.md)

## 参考资料

- [FAST-LIO2 官方文档](https://github.com/hku-mars/FAST_LIO)
- [Lslidar ROS2 驱动](https://github.com/Lslidar/lslidar_ros2)
- [故障排除完整指南](../TROUBLESHOOTING.md)

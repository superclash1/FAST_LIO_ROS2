# FAST_LIO_ROS2
FAST-LIO2 workspace and drivers

说明（简体中文）

以下包含两种在本仓库环境下运行 FAST-LIO2 的方法：
1) 手动按步骤启动（分开启动激光雷达驱动、IMU 驱动与 FAST-LIO 节点）
2) 使用仓库内的一键脚本 `start_all.sh` 启动（更方便）

重要前置项
- 本仓库基于 ROS2（你的环境是 Foxy），请先确保已安装并配置好 ROS2 环境。
- 若你在当前机器上尚未构建过工作区，先在工作区根目录运行一次构建：

```bash
# 进入工作区
cd /home/rosdev/ros2_ws

# （可选）将源码更新或检查修改
# git status

# 构建（只需在第一次或修改源码后运行）
colcon build --symlink-install
```

- 每次启动前请 source ROS2 与本工作区的 setup 文件：

```bash
source /opt/ros/foxy/setup.bash
source /home/rosdev/ros2_ws/install/local_setup.bash
```

方法一 — 手动逐个启动（推荐用于调试）
- 适用于需要观察单个节点日志或分步调试的场景。

典型启动顺序：
1. 启动激光雷达驱动（根据型号选择对应 launch）：
```bash
# 例如 Lslidar C16 驱动
ros2 launch lslidar_driver lslidar_c16_launch.py
```

2. 启动 IMU 驱动：
```bash
# 示例：handsfree_imu_ros2 的 launch
ros2 launch handsfree_imu_ros2 imu.launch.py
```

3. 启动 FAST-LIO mapping 节点（含参数与 RViz，可按需开启）：
```bash
ros2 launch FAST_LIO_ROS2 mapping.launch.py use_rviz:=true
```

运行提示与常见问题：
- 如果控制台输出 "No Effective Points!"，可能是机器人静止或点云被过度滤波，请移动平台并检查 `config/*.yaml` 中的滤波尺寸参数（例如 `filter_size_surf`、`point_filter_num` 等）。
- 如果看到 PCL 提示 `Leaf size is too small`，请适当调大体素网格 (voxel) 的 leaf size。
- 确认话题 `/cx/lslidar_point_cloud`（点云）与 `/imu/data`（IMU）有数据：
```bash
ros2 topic hz /cx/lslidar_point_cloud
ros2 topic hz /imu/data
```

方法二 — 使用一键脚本 `start_all.sh`（推荐快速启动）
- 仓库根目录已有 `start_all.sh`，脚本会按顺序启动雷达驱动、IMU 驱动和 FAST-LIO（可选 RViz）。
- 使用前同样需要 source 环境：
```bash
source /opt/ros/foxy/setup.bash
source /home/rosdev/ros2_ws/install/local_setup.bash
chmod +x /home/rosdev/ros2_ws/start_all.sh
/home/rosdev/ros2_ws/start_all.sh
```

- 脚本会在当前终端启动多个子进程；要停止可用 `pkill -f <node_name>` 或在另一个终端执行：
```bash
pkill -f fast_lio || true
pkill -f lslidar || true
pkill -f imu_node || true
```

小结与建议
- 开发/调试时优先用 方法一（分步启动、逐个检查话题与日志）；演示/部署时可用 方法二（脚本自动化）。
- 启动后若无法建图，请确保平台运动且传感器数据频率稳定。
- 我可以把更多运行示例（launch 参数说明、config 模板、回放 bag）加入本 README，告诉我你需要哪些示例。


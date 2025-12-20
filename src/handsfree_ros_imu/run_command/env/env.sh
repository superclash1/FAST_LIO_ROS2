#!/bin/bash

# 运行脚本兼容 Ubuntu和ROS 的不同版本，支持在任意目录启动
###########################################
# 获取当前脚本所在的目录，切换到脚本所在目录
current_dir="$(dirname "$(readlink -f "$0")")"
cd "$current_dir"
package_dir="$current_dir/../../../.."

###########################################
# 检测 Ubuntu 版本和 ROS 版本
version=$(lsb_release -sc) 
case $version in
    "xenial") ROS_DISTRO="kinetic" ;;
    "bionic") ROS_DISTRO="melodic" ;;
    "focal")  ROS_DISTRO="noetic" ;;
    *)
        echo "不支持的 Ubuntu 版本用于 ROS 安装. 退出."
        exit 1
        ;;
esac
echo "检测 Ubuntu 版本: $version"
echo "适配到的 ROS 发行版: $ROS_DISTRO"

###########################################
# source ROS 环境变量
source /opt/ros/$ROS_DISTRO/setup.bash

###########################################
# 设置运行环境变量

###########################################
# 设置 ROS 功能包环境变量

# 用于设置 ROS 环境变量的函数
set_ros_env() {
    local workspace_path=$1
    
    # 检查工作空间路径是否存在
    if [ ! -d "$workspace_path" ]; then
        echo "Error: Workspace path '$workspace_path' does not exist."
        return 1
    fi

    # 设置 ROS_PACKAGE_PATH
    export ROS_PACKAGE_PATH=$workspace_path/src:$ROS_PACKAGE_PATH
    # 设置 CMAKE_PREFIX_PATH
    export CMAKE_PREFIX_PATH=$workspace_path/devel:$CMAKE_PREFIX_PATH
    # 设置 LD_LIBRARY_PATH
    export LD_LIBRARY_PATH=$workspace_path/devel/lib:$LD_LIBRARY_PATH
    # 设置 PYTHONPATH
    export PYTHONPATH=$workspace_path/devel/lib/python2.7/dist-packages:$PYTHONPATH
    # 设置 PATH
    export PATH=$workspace_path/devel/bin:$PATH
    return 0
}

#设置ROS变量，传入的参数为 ROS 功能包的路径
set_ros_env $package_dir

#!/bin/bash
# 实时监控系统资源使用情况
# Jetson Xavier NX - CPU/GPU/内存/温度监控

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

clear

echo -e "${CYAN}========================================${NC}"
echo -e "${CYAN}  FAST-LIO 系统资源监控${NC}"
echo -e "${CYAN}  Jetson Xavier NX${NC}"
echo -e "${CYAN}========================================${NC}"
echo ""

# 检查是否在Jetson上
if [ -f /sys/devices/virtual/thermal/thermal_zone0/temp ]; then
    JETSON=true
else
    JETSON=false
fi

while true; do
    clear
    echo -e "${CYAN}========================================${NC}"
    echo -e "${CYAN}  FAST-LIO 系统资源监控${NC}"
    echo -e "${CYAN}  $(date '+%Y-%m-%d %H:%M:%S')${NC}"
    echo -e "${CYAN}========================================${NC}"
    echo ""
    
    # CPU使用率
    echo -e "${BLUE}📊 CPU 使用率:${NC}"
    if command -v mpstat &> /dev/null; then
        mpstat 1 1 | grep -A 5 "all" | tail -1 | awk '{print "  总体: " 100-$NF "%"}'
    else
        top -bn1 | grep "Cpu(s)" | sed "s/.*, *\([0-9.]*\)%* id.*/\1/" | awk '{print "  总体: " 100-$1 "%"}'
    fi
    
    # 各核心使用率
    if command -v mpstat &> /dev/null; then
        echo -e "  ${GREEN}各核心:${NC}"
        mpstat -P ALL 1 1 | grep -E "^[0-9]" | awk '{printf "    Core %s: %.1f%%\n", $2, 100-$NF}'
    fi
    echo ""
    
    # GPU使用率 (Jetson专用)
    if [ "$JETSON" = true ]; then
        echo -e "${BLUE}🎮 GPU 使用率:${NC}"
        if command -v tegrastats &> /dev/null; then
            # Jetson使用tegrastats
            timeout 1 tegrastats --interval 1000 2>/dev/null | head -1 | grep -oP 'GR3D_FREQ \K[0-9]+' | awk '{printf "  GPU频率: %d MHz\n", $1}'
            timeout 1 tegrastats --interval 1000 2>/dev/null | head -1 | grep -oP 'GPU@\K[0-9.]+' | awk '{printf "  GPU温度: %.1f°C\n", $1}'
        elif [ -f /sys/devices/gpu.0/load ]; then
            GPU_LOAD=$(cat /sys/devices/gpu.0/load 2>/dev/null || echo "0")
            echo -e "  负载: ${GPU_LOAD}/1000 ($(echo "scale=1; $GPU_LOAD/10" | bc)%)"
        else
            echo -e "  ${YELLOW}(tegrastats 不可用)${NC}"
        fi
    else
        echo -e "${BLUE}🎮 GPU 使用率:${NC}"
        if command -v nvidia-smi &> /dev/null; then
            nvidia-smi --query-gpu=utilization.gpu,memory.used,memory.total,temperature.gpu --format=csv,noheader,nounits | \
            awk -F', ' '{printf "  GPU: %s%%, 显存: %s/%s MB, 温度: %s°C\n", $1, $2, $3, $4}'
        else
            echo -e "  ${YELLOW}(nvidia-smi 不可用)${NC}"
        fi
    fi
    echo ""
    
    # 内存使用
    echo -e "${BLUE}💾 内存使用:${NC}"
    free -h | awk 'NR==2{printf "  已用: %s / %s (%.1f%%)\n", $3, $2, $3/$2*100}'
    echo ""
    
    # 温度
    echo -e "${BLUE}🌡️  温度:${NC}"
    if [ "$JETSON" = true ]; then
        # Jetson温度区域
        for zone in /sys/devices/virtual/thermal/thermal_zone*/temp; do
            if [ -f "$zone" ]; then
                zone_type=$(cat ${zone/temp/type} 2>/dev/null || echo "unknown")
                temp=$(cat $zone)
                temp_c=$(echo "scale=1; $temp/1000" | bc)
                
                # 温度颜色标记
                if (( $(echo "$temp_c > 80" | bc -l) )); then
                    color=$RED
                elif (( $(echo "$temp_c > 60" | bc -l) )); then
                    color=$YELLOW
                else
                    color=$GREEN
                fi
                
                printf "  ${color}%-15s: %.1f°C${NC}\n" "$zone_type" "$temp_c"
            fi
        done
    else
        # 通用Linux温度
        if command -v sensors &> /dev/null; then
            sensors | grep -E "^Core|^Package|temp" | head -5
        else
            echo -e "  ${YELLOW}(sensors 不可用)${NC}"
        fi
    fi
    echo ""
    
    # 进程监控
    echo -e "${BLUE}🔍 FAST-LIO 相关进程:${NC}"
    ps aux | grep -E "fastlio|lslidar|imu_node|rviz2" | grep -v grep | \
    awk '{printf "  %-20s CPU: %5s%%  MEM: %5s%%  PID: %s\n", substr($11,1,20), $3, $4, $2}'
    
    if [ $(ps aux | grep -E "fastlio|lslidar|imu_node" | grep -v grep | wc -l) -eq 0 ]; then
        echo -e "  ${YELLOW}(无运行进程)${NC}"
    fi
    echo ""
    
    # 磁盘使用
    echo -e "${BLUE}💿 磁盘使用:${NC}"
    df -h / | awk 'NR==2{printf "  根分区: %s / %s (%s)\n", $3, $2, $5}'
    echo ""
    
    # 网络流量 (eth0 - 雷达数据)
    if [ -d /sys/class/net/eth0 ]; then
        echo -e "${BLUE}🌐 网络流量 (eth0 - 雷达):${NC}"
        RX_BYTES=$(cat /sys/class/net/eth0/statistics/rx_bytes)
        TX_BYTES=$(cat /sys/class/net/eth0/statistics/tx_bytes)
        sleep 1
        RX_BYTES_NEW=$(cat /sys/class/net/eth0/statistics/rx_bytes)
        TX_BYTES_NEW=$(cat /sys/class/net/eth0/statistics/tx_bytes)
        
        RX_RATE=$(( ($RX_BYTES_NEW - $RX_BYTES) / 1024 ))
        TX_RATE=$(( ($TX_BYTES_NEW - $TX_BYTES) / 1024 ))
        
        echo -e "  接收: ${GREEN}${RX_RATE} KB/s${NC}  发送: ${YELLOW}${TX_RATE} KB/s${NC}"
        echo ""
    fi
    
    echo -e "${CYAN}========================================${NC}"
    echo -e "  按 ${YELLOW}Ctrl+C${NC} 退出监控"
    echo -e "${CYAN}========================================${NC}"
    
    sleep 2
done

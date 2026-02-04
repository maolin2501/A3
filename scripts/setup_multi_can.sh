#!/bin/bash
# RS-A3 多 CAN 接口配置脚本
# 用于配置多条机械臂的 CAN 通信接口

# 配置参数
BITRATE=1000000  # 1Mbps
CAN_INTERFACES=(can0 can1 can2 can3)

# 颜色输出
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo "=================================="
echo "RS-A3 多 CAN 接口配置"
echo "=================================="

# 检查是否具有 root 权限
if [ "$EUID" -ne 0 ]; then
    echo -e "${YELLOW}警告: 需要 sudo 权限来配置 CAN 接口${NC}"
    exec sudo "$0" "$@"
fi

# 获取要配置的接口数量
NUM_INTERFACES=${1:-2}  # 默认配置 2 个接口

echo -e "配置 ${GREEN}$NUM_INTERFACES${NC} 个 CAN 接口..."
echo ""

# 配置指定数量的 CAN 接口
for i in $(seq 0 $((NUM_INTERFACES - 1))); do
    CAN_IF="${CAN_INTERFACES[$i]}"
    
    echo -n "配置 $CAN_IF... "
    
    # 检查接口是否存在
    if ! ip link show "$CAN_IF" &> /dev/null; then
        echo -e "${RED}接口不存在${NC}"
        continue
    fi
    
    # 关闭接口
    ip link set "$CAN_IF" down 2>/dev/null
    
    # 配置波特率
    if ip link set "$CAN_IF" type can bitrate $BITRATE; then
        # 启动接口
        if ip link set "$CAN_IF" up; then
            echo -e "${GREEN}成功${NC}"
        else
            echo -e "${RED}启动失败${NC}"
        fi
    else
        echo -e "${RED}配置失败${NC}"
    fi
done

echo ""
echo "=================================="
echo "CAN 接口状态"
echo "=================================="

# 显示所有 CAN 接口状态
for i in $(seq 0 $((NUM_INTERFACES - 1))); do
    CAN_IF="${CAN_INTERFACES[$i]}"
    
    if ip link show "$CAN_IF" &> /dev/null; then
        STATE=$(ip link show "$CAN_IF" | grep -oP '(?<=state )\w+')
        if [ "$STATE" == "UP" ]; then
            echo -e "$CAN_IF: ${GREEN}$STATE${NC}"
        else
            echo -e "$CAN_IF: ${RED}$STATE${NC}"
        fi
    else
        echo -e "$CAN_IF: ${YELLOW}不存在${NC}"
    fi
done

echo ""
echo "=================================="
echo "使用示例"
echo "=================================="
echo "# 启动双臂系统"
echo "ros2 launch rs_a3_description multi_arm_control.launch.py"
echo ""
echo "# 启动主从遥操作"
echo "ros2 launch rs_a3_teleop master_slave.launch.py master_ns:=arm1 slave_ns:=arm2"
echo ""

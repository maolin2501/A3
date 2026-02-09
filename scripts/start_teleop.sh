#!/bin/bash
#
# RS-A3 主从遥操作启动脚本
#
# 功能：
# 1. 配置 CAN 接口（can0-can4）
# 2. 启动主从遥操作程序
#
# 使用方法：
#   ./start_teleop.sh                    # 使用默认配置
#   ./start_teleop.sh --slaves can1      # 只使用 can1 作为从臂
#   ./start_teleop.sh --help             # 显示帮助
#

set -e

# 默认配置
MASTER_CAN="can0"
SLAVE_CANS="can1,can2,can3,can4"
BITRATE=1000000
MASTER_KD=1.0
SLAVE_KP=150.0
SLAVE_KD=1.0
RATE=200.0
SKIP_CAN_SETUP=false

# 颜色输出
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

print_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

print_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# 显示帮助
show_help() {
    echo "RS-A3 主从遥操作启动脚本"
    echo ""
    echo "用法: $0 [选项]"
    echo ""
    echo "选项:"
    echo "  --master CAN      主臂 CAN 接口 (默认: $MASTER_CAN)"
    echo "  --slaves CANs     从臂 CAN 接口列表，逗号分隔 (默认: $SLAVE_CANS)"
    echo "  --bitrate RATE    CAN 波特率 (默认: $BITRATE)"
    echo "  --master-kd KD    主臂阻尼系数 (默认: $MASTER_KD)"
    echo "  --slave-kp KP     从臂位置增益 (默认: $SLAVE_KP)"
    echo "  --slave-kd KD     从臂速度增益 (默认: $SLAVE_KD)"
    echo "  --rate HZ         控制频率 (默认: $RATE)"
    echo "  --skip-can-setup  跳过 CAN 接口配置"
    echo "  -h, --help        显示此帮助信息"
    echo ""
    echo "示例:"
    echo "  $0                              # 使用默认配置"
    echo "  $0 --slaves can1                # 只使用 can1 作为从臂"
    echo "  $0 --slaves can1,can2           # 使用 can1 和 can2 作为从臂"
    echo "  $0 --slave-kp 100 --slave-kd 2  # 自定义从臂控制参数"
    echo ""
}

# 解析命令行参数
while [[ $# -gt 0 ]]; do
    case $1 in
        --master)
            MASTER_CAN="$2"
            shift 2
            ;;
        --slaves)
            SLAVE_CANS="$2"
            shift 2
            ;;
        --bitrate)
            BITRATE="$2"
            shift 2
            ;;
        --master-kd)
            MASTER_KD="$2"
            shift 2
            ;;
        --slave-kp)
            SLAVE_KP="$2"
            shift 2
            ;;
        --slave-kd)
            SLAVE_KD="$2"
            shift 2
            ;;
        --rate)
            RATE="$2"
            shift 2
            ;;
        --skip-can-setup)
            SKIP_CAN_SETUP=true
            shift
            ;;
        -h|--help)
            show_help
            exit 0
            ;;
        *)
            print_error "未知选项: $1"
            show_help
            exit 1
            ;;
    esac
done

# 获取脚本所在目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 配置单个 CAN 接口
setup_can_interface() {
    local iface=$1
    local bitrate=$2
    
    # 检查接口是否存在
    if ! ip link show "$iface" &>/dev/null; then
        print_warn "CAN 接口 $iface 不存在，跳过"
        return 1
    fi
    
    # 先关闭接口（如果已经打开）
    sudo ip link set "$iface" down 2>/dev/null || true
    
    # 设置波特率并启动
    if sudo ip link set "$iface" type can bitrate "$bitrate"; then
        if sudo ip link set "$iface" up; then
            print_info "CAN 接口 $iface 已配置 (bitrate=$bitrate)"
            return 0
        fi
    fi
    
    print_error "配置 CAN 接口 $iface 失败"
    return 1
}

# 配置所有 CAN 接口
setup_all_can_interfaces() {
    print_info "配置 CAN 接口..."
    
    # 配置主臂
    if ! setup_can_interface "$MASTER_CAN" "$BITRATE"; then
        print_error "主臂 CAN 接口配置失败"
        exit 1
    fi
    
    # 配置从臂
    IFS=',' read -ra SLAVES <<< "$SLAVE_CANS"
    for slave in "${SLAVES[@]}"; do
        slave=$(echo "$slave" | xargs)  # 去除空格
        if [[ -n "$slave" ]]; then
            setup_can_interface "$slave" "$BITRATE" || true
        fi
    done
    
    print_info "CAN 接口配置完成"
}

# 主函数
main() {
    echo "============================================================"
    echo "  RS-A3 主从遥操作"
    echo "============================================================"
    echo "  主臂: $MASTER_CAN (零力矩模式, Kd=$MASTER_KD)"
    echo "  从臂: $SLAVE_CANS (位置跟随, Kp=$SLAVE_KP, Kd=$SLAVE_KD)"
    echo "  波特率: $BITRATE"
    echo "  控制频率: $RATE Hz"
    echo "============================================================"
    echo ""
    
    # 配置 CAN 接口
    if [[ "$SKIP_CAN_SETUP" == "false" ]]; then
        setup_all_can_interfaces
    else
        print_info "跳过 CAN 接口配置"
    fi
    
    # 启动遥操作程序
    print_info "启动遥操作程序..."
    echo ""
    
    python3 "$SCRIPT_DIR/teleop_master_slave.py" \
        --master "$MASTER_CAN" \
        --slaves "$SLAVE_CANS" \
        --master-kd "$MASTER_KD" \
        --slave-kp "$SLAVE_KP" \
        --slave-kd "$SLAVE_KD" \
        --rate "$RATE"
}

# 运行
main

#!/bin/bash
# Phase A.1.1 + A.1.2: 编译检查 + Python SDK 导入验证
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

PASS=0
FAIL=0

pass() { echo "  [PASS] $1"; ((PASS++)); }
fail() { echo "  [FAIL] $1"; ((FAIL++)); }

echo "============================================"
echo " A.1.1  C++ / ROS2 编译检查"
echo "============================================"

cd "$PROJECT_ROOT/ros2_ws"

if colcon build \
    --packages-select el_a3_hardware el_a3_description el_a3_moveit_config \
    --cmake-args -DCMAKE_BUILD_TYPE=Release 2>&1 | tee /tmp/test_build_output.log; then
    pass "colcon build 成功"
else
    fail "colcon build 失败"
    echo "  查看详情: /tmp/test_build_output.log"
fi

echo ""
echo "============================================"
echo " A.1.2  Python SDK 导入检查"
echo "============================================"

cd "$PROJECT_ROOT"

python3 -c "
import sys
sys.path.insert(0, '.')

from el_a3_sdk.protocol import MotorType, MOTOR_PARAMS, DEFAULT_MOTOR_TYPE_MAP, ArmState
from el_a3_sdk.protocol import DEFAULT_JOINT_LIMITS, DEFAULT_JOINT_DIRECTIONS

# MotorType enum
assert MotorType.RS00 == 0, 'RS00 != 0'
assert MotorType.EL05 == 1, 'EL05 != 1'
assert MotorType.RS05 == 2, 'RS05 != 2'
print('  [PASS] MotorType 枚举值正确 (RS00=0, EL05=1, RS05=2)')

# RS05 params
assert MotorType.RS05 in MOTOR_PARAMS, 'RS05 not in MOTOR_PARAMS'
p = MOTOR_PARAMS[MotorType.RS05]
assert p.t_min == -5.5, f'RS05 t_min={p.t_min}, expected -5.5'
assert p.t_max == 5.5, f'RS05 t_max={p.t_max}, expected 5.5'
assert p.v_min == -50.0, f'RS05 v_min={p.v_min}'
assert p.v_max == 50.0, f'RS05 v_max={p.v_max}'
print('  [PASS] RS05 参数正确 (t=±5.5, v=±50)')

# EL05 params
p2 = MOTOR_PARAMS[MotorType.EL05]
assert p2.t_min == -6.0 and p2.t_max == 6.0
print('  [PASS] EL05 参数正确 (t=±6.0)')

# Default map
for mid in [1, 2, 3]:
    assert DEFAULT_MOTOR_TYPE_MAP[mid] == MotorType.RS00, f'Motor {mid} should be RS00'
for mid in [4, 5, 6, 7]:
    assert DEFAULT_MOTOR_TYPE_MAP[mid] == MotorType.EL05, f'Motor {mid} should be EL05'
print('  [PASS] DEFAULT_MOTOR_TYPE_MAP 正确 (1-3=RS00, 4-7=EL05)')

# ArmState
assert ArmState.DISCONNECTED == 0
assert ArmState.IDLE == 1
assert ArmState.ENABLED == 2
assert ArmState.ZERO_TORQUE == 4
assert ArmState.ERROR == 5
print('  [PASS] ArmState 枚举值正确')

# Joint config
assert len(DEFAULT_JOINT_LIMITS) == 7, f'Expected 7 joints, got {len(DEFAULT_JOINT_LIMITS)}'
assert len(DEFAULT_JOINT_DIRECTIONS) == 7
print('  [PASS] 关节配置完整 (7 关节)')

# utils import
from el_a3_sdk.utils import float_to_uint16, uint16_to_float
raw = float_to_uint16(0.0, -5.5, 5.5)
assert abs(raw - 32768) < 2, f'float_to_uint16(0, -5.5, 5.5)={raw}'
print('  [PASS] utils 编解码函数可导入')

print('  [PASS] Python SDK 全部导入检查通过')
" && pass "Python SDK 导入检查" || fail "Python SDK 导入检查"

echo ""
echo "============================================"
echo " 结果: $PASS passed, $FAIL failed"
echo "============================================"

[ "$FAIL" -eq 0 ] || exit 1

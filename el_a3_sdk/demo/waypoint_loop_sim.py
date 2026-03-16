#!/usr/bin/env python3
"""
10 路径点循环测试 -- 仿真版

使用 ELA3ROSInterface 通过 ros2_control (mock hardware) 驱动机械臂模型，
在 RViz 中实时显示运动轨迹。

前置条件:
  1. ROS2 环境已 source
  2. 已启动 mock ros2_control + RViz:
     ros2 launch el_a3_description el_a3_control.launch.py use_mock_hardware:=true

用法:
  python3 el_a3_sdk/demo/waypoint_loop_sim.py [--loops 3]
"""

import sys
import os
import argparse
import time
import math
import signal

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))

from el_a3_sdk import get_ros_interface, LogLevel
from el_a3_sdk.demo.waypoints_config import WAYPOINTS, get_waypoint_summary


def main():
    parser = argparse.ArgumentParser(
        description="EL-A3 路径点循环测试 (仿真版)")
    parser.add_argument("--loops", type=int, default=0,
                        help="循环次数，0=无限循环 (默认: 0)")
    parser.add_argument("--speed", type=float, default=1.0,
                        help="速度倍率，影响等待时间 (默认: 1.0)")
    args = parser.parse_args()

    print("=" * 60)
    print("  EL-A3 路径点循环测试 -- 仿真版")
    print(f"  循环次数: {'无限' if args.loops == 0 else args.loops}")
    print(f"  速度倍率: {args.speed}x")
    print("=" * 60)
    print(f"\n{get_waypoint_summary()}\n")

    ELA3ROSInterface = get_ros_interface()
    arm = ELA3ROSInterface(
        node_name="waypoint_loop_sim",
        logger_level=LogLevel.INFO,
    )

    if not arm.ConnectPort():
        print("[ERROR] ROS 接口连接失败，请确认:")
        print("  1. 已 source ROS2 工作空间")
        print("  2. 已启动: ros2 launch el_a3_description el_a3_control.launch.py use_mock_hardware:=true")
        return

    print("[OK] ROS 接口已连接")

    time.sleep(1.0)
    js = arm.GetArmJointMsgs()
    print(f"[INFO] 当前关节角度: {[f'{v:.3f}' for v in js.to_list()[:6]]}")

    # mock 模式下控制器已由 launch 文件激活，EnableArm (switch_controller)
    # 对已激活控制器可能超时，容忍失败继续运行
    arm.EnableArm()
    print("[OK] 仿真模式就绪\n")
    time.sleep(0.5)

    shutdown = False

    def on_signal(_sig, _frame):
        nonlocal shutdown
        shutdown = True
    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    loop_count = 0
    try:
        while not shutdown:
            loop_count += 1
            if args.loops > 0 and loop_count > args.loops:
                break

            loops_str = f"{loop_count}" if args.loops == 0 else f"{loop_count}/{args.loops}"
            print(f"\n{'━' * 60}")
            print(f"  循环 {loops_str}")
            print(f"{'━' * 60}")

            for i, wp in enumerate(WAYPOINTS):
                if shutdown:
                    break

                degs = [f"{p * 180.0 / math.pi:.1f}°" for p in wp.positions]
                print(f"\n  [{i}/{len(WAYPOINTS)-1}] -> {wp.name}  {degs}")

                hold = wp.hold_time / args.speed
                dur_ns = int(hold * 0.8 * 1e9)
                arm.JointCtrl(*wp.positions, duration_ns=dur_ns)

                time.sleep(hold)

                js = arm.GetArmJointMsgs()
                current = js.to_list()[:6]
                errors = [abs(current[j] - wp.positions[j]) * 180.0 / math.pi
                          for j in range(6)]
                max_err = max(errors)
                current_degs = [f"{c * 180.0 / math.pi:.1f}°" for c in current]
                print(f"       到达: {current_degs}  最大误差: {max_err:.2f}°")

        print(f"\n{'=' * 60}")
        print(f"  循环完成 (共 {loop_count - 1} 轮)")
        print(f"{'=' * 60}")

    finally:
        print("\n[INFO] 回零位...")
        arm.JointCtrl(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, duration_ns=1_000_000_000)
        time.sleep(1.5)
        arm.DisconnectPort()
        print("[OK] 仿真测试结束")


if __name__ == "__main__":
    main()

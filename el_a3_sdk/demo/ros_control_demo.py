#!/usr/bin/env python3
"""
EL-A3 SDK ROS Control 模式示例

使用前确保：
  1. 已 source ROS2 工作空间
  2. 已启动 el_a3_control.launch.py 或 robot.launch.py
  3. arm_controller 已激活

用法:
    python3 ros_control_demo.py
"""

import time
import math

from el_a3_sdk import get_ros_interface

ELA3ROSInterface = get_ros_interface()


def main():
    arm = ELA3ROSInterface(
        node_name="el_a3_demo",
        controller_name="arm_controller",
    )

    if not arm.ConnectPort():
        print("连接失败，请检查 ROS2 环境和控制器是否已启动")
        return

    print(f"SDK 版本: {arm.GetCurrentSDKVersion()}")
    print(f"协议: {arm.GetCurrentProtocolVersion()}")

    time.sleep(1.0)
    js = arm.GetArmJointMsgs()
    print(f"当前关节角度: {[f'{v:.3f}' for v in js.to_list()]}")

    print("\n--- 使能机械臂 ---")
    if not arm.EnableArm():
        print("使能失败")
        arm.DisconnectPort()
        return

    time.sleep(0.5)

    print("发送零位指令...")
    arm.JointCtrl(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    time.sleep(3.0)

    js = arm.GetArmJointMsgs()
    print(f"到达位置: {[f'{v:.3f}' for v in js.to_list()]}")

    print("\n发送测试位姿...")
    arm.JointCtrl(0.0, math.radians(30), math.radians(-45), 0.0, 0.0, 0.0)
    time.sleep(3.0)

    js = arm.GetArmJointMsgs()
    print(f"到达位置: {[f'{v:.3f}' for v in js.to_list()]}")

    vel = arm.GetArmJointVelocities()
    print(f"当前速度: {[f'{v:.3f}' for v in vel.to_list()]}")

    eff = arm.GetArmJointEfforts()
    print(f"当前力矩: {[f'{v:.3f}' for v in eff.to_list()]}")

    status = arm.GetArmStatus()
    print(f"臂状态: arm_status={status.arm_status}, enabled={status.all_enabled}")

    print("\n--- 失能机械臂 ---")
    arm.DisableArm()
    time.sleep(0.5)

    arm.DisconnectPort()
    print("完成")


if __name__ == "__main__":
    main()

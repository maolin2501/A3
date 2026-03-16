#!/usr/bin/env python3
"""
基础通信测试 Demo: Type 1 发送 → Type 2 反馈 + RViz 实时显示

不使能电机（不发送 Type 3），仅发送 Type 1 运控指令（Kp=0, Kd=0, torque=0），
接收 Type 2 反馈帧并将关节角度发布为 ROS2 /joint_states，供 RViz 实时显示模型。

前置条件:
  1. CAN 接口已激活: bash scripts/setup_can.sh can0 1000000
  2. 机械臂已上电（电机未使能也可接收 Type 2 反馈）
  3. ROS2 环境已 source

用法:
  # 终端 1: 启动 RViz + robot_state_publisher
  ros2 launch el_a3_description comm_test.launch.py

  # 终端 2: 运行通信测试
  python3 el_a3_sdk/demo/comm_test_rviz.py --can can0
"""

import sys
import os
import argparse
import time
import math
import signal
import threading

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))

from el_a3_sdk.can_driver import RobstrideCanDriver, _busy_wait_us
from el_a3_sdk.protocol import (
    MotorType, ModeState, MOTOR_PARAMS,
    DEFAULT_JOINT_DIRECTIONS, DEFAULT_JOINT_OFFSETS,
)

MODE_STATE_NAMES = {0: "Reset", 1: "Cali", 2: "Motor"}
JOINT_NAMES = [f"L{i}_joint" for i in range(1, 8)]
NUM_MOTORS = 7


def build_motor_type_map(wrist_type_str: str) -> dict:
    wrist = MotorType.RS05 if wrist_type_str == "RS05" else MotorType.EL05
    return {
        1: MotorType.RS00, 2: MotorType.RS00, 3: MotorType.RS00,
        4: wrist, 5: wrist, 6: wrist, 7: wrist,
    }


class CommTestRVizNode:
    """Type 1/2 通信测试 + ROS2 JointState 发布节点"""

    def __init__(self, can_name: str, motor_type_map: dict, rate_hz: float = 100.0):
        self.can_name = can_name
        self.rate_hz = rate_hz

        self._driver = RobstrideCanDriver(
            can_name=can_name,
            host_can_id=0xFD,
            motor_type_map=motor_type_map,
        )
        self._directions = dict(DEFAULT_JOINT_DIRECTIONS)
        self._offsets = dict(DEFAULT_JOINT_OFFSETS)
        self._running = False

        # ROS2
        self._node = None
        self._pub_joint_states = None
        self._timer = None
        self._JointState = None

        self._print_count = 0

    def start(self) -> bool:
        if not self._driver.connect():
            print(f"[ERROR] CAN 接口 {self.can_name} 连接失败")
            return False

        self._driver.start_receive_thread()
        print(f"[OK] CAN 接口 {self.can_name} 已连接，接收线程已启动")

        if not self._init_ros():
            print("[WARN] ROS2 初始化失败，仅终端模式运行")

        self._running = True

        # 发送一轮 Type 1 激活反馈
        self._send_type1_all()
        time.sleep(0.1)

        return True

    def _init_ros(self) -> bool:
        try:
            import rclpy
            from sensor_msgs.msg import JointState
        except ImportError:
            return False

        rclpy.init()
        self._node = rclpy.create_node('comm_test_rviz')
        self._JointState = JointState
        self._pub_joint_states = self._node.create_publisher(
            JointState, '/joint_states', 10)

        self._timer = self._node.create_timer(
            1.0 / self.rate_hz, self._timer_callback)

        threading.Thread(
            target=lambda: rclpy.spin(self._node),
            daemon=True, name="ros_spin"
        ).start()

        self._node.get_logger().info(
            f"通信测试节点已启动: CAN={self.can_name}, 频率={self.rate_hz}Hz")
        return True

    def _timer_callback(self):
        """定时器回调: 发送 Type 1 + 读取 Type 2 + 发布 JointState"""
        self._send_type1_all()
        self._publish_joint_states()
        self._print_count += 1
        if self._print_count % int(self.rate_hz) == 0:
            self._print_feedback()

    def _send_type1_all(self):
        """向所有电机发送 Type 1 (Kp=0, Kd=0, torque=0)"""
        for mid in range(1, NUM_MOTORS + 1):
            self._driver.send_motion_control(
                mid, position=0.0, velocity=0.0,
                kp=0.0, kd=0.0, torque=0.0)
            _busy_wait_us(50)

    def _publish_joint_states(self):
        """读取 Type 2 反馈并发布 /joint_states"""
        if self._pub_joint_states is None:
            return

        msg = self._JointState()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.name = list(JOINT_NAMES)

        positions = []
        velocities = []
        efforts = []

        for mid in range(1, NUM_MOTORS + 1):
            fb = self._driver.get_feedback(mid)
            direction = self._directions.get(mid, 1.0)
            offset = self._offsets.get(mid, 0.0)

            if fb and fb.is_valid:
                positions.append((fb.position - offset) * direction)
                velocities.append(fb.velocity * direction)
                efforts.append(fb.torque * direction)
            else:
                positions.append(0.0)
                velocities.append(0.0)
                efforts.append(0.0)

        msg.position = positions
        msg.velocity = velocities
        msg.effort = efforts
        self._pub_joint_states.publish(msg)

    def _print_feedback(self):
        """终端打印反馈信息（每秒一次）"""
        feedbacks = self._driver.get_all_feedbacks()
        fps = self._driver.get_can_fps()

        lines = []
        lines.append(f"\n{'═' * 95}")
        lines.append(
            f"  Type 1→2 通信测试  |  CAN: {self.can_name}  |  "
            f"CAN FPS: {fps:.0f}  |  电机未使能（安全模式）")
        lines.append(f"{'─' * 95}")
        lines.append(
            f"  {'电机':>4}  {'角度(deg)':>10}  {'速度(rad/s)':>12}  "
            f"{'力矩(Nm)':>10}  {'温度(°C)':>9}  {'模式':>6}  "
            f"{'故障':>4}  {'有效':>4}")
        lines.append(f"{'─' * 95}")

        for mid in range(1, NUM_MOTORS + 1):
            fb = feedbacks.get(mid)
            if fb and fb.is_valid:
                direction = self._directions.get(mid, 1.0)
                offset = self._offsets.get(mid, 0.0)
                joint_pos_deg = (fb.position - offset) * direction * 180.0 / math.pi
                mode_name = MODE_STATE_NAMES.get(fb.mode_state, f"?{fb.mode_state}")
                fault_str = f"{fb.fault_code}" if fb.fault_code else "OK"

                lines.append(
                    f"   M{mid}    {joint_pos_deg:>9.2f}°  {fb.velocity:>11.3f}  "
                    f"{fb.torque:>9.3f}  {fb.temperature:>8.1f}  "
                    f"{mode_name:>6}  {fault_str:>4}    ✓")
            else:
                lines.append(
                    f"   M{mid}    {'---':>9}   {'---':>11}  "
                    f"{'---':>9}  {'---':>8}  {'---':>6}  {'---':>4}    ✗")

        lines.append(f"{'═' * 95}")
        print("\n".join(lines))

    def run_terminal_only(self):
        """无 ROS2 时的纯终端循环"""
        dt = 1.0 / self.rate_hz
        print_interval = int(self.rate_hz)
        count = 0
        while self._running:
            self._send_type1_all()
            count += 1
            if count % print_interval == 0:
                self._print_feedback()
            time.sleep(dt)

    def stop(self):
        self._running = False
        self._driver.stop_receive_thread()
        self._driver.disconnect()
        print("\n[OK] 通信测试已停止，CAN 已断开")

        if self._node:
            import rclpy
            self._node.destroy_node()
            rclpy.try_shutdown()


def main():
    parser = argparse.ArgumentParser(
        description="EL-A3 基础通信测试: Type 1 发送 → Type 2 反馈 + RViz 显示")
    parser.add_argument("--can", default="can0",
                        help="CAN 接口名 (默认: can0)")
    parser.add_argument("--rate", type=float, default=100.0,
                        help="Type 1 发送频率 Hz (默认: 100)")
    parser.add_argument("--motor-type", default="EL05",
                        choices=["EL05", "RS05"],
                        help="腕部电机型号 (默认: EL05)")
    args = parser.parse_args()

    motor_map = build_motor_type_map(args.motor_type)

    print("=" * 60)
    print("  EL-A3 基础通信测试 Demo")
    print(f"  CAN: {args.can}  |  频率: {args.rate}Hz  |  腕部: {args.motor_type}")
    print("  模式: 仅发送 Type 1 (Kp=0) → 接收 Type 2 反馈")
    print("  安全: 不发送 Type 3 使能指令，电机不会运动")
    print("=" * 60)

    node = CommTestRVizNode(
        can_name=args.can,
        motor_type_map=motor_map,
        rate_hz=args.rate,
    )

    shutdown_event = threading.Event()

    def on_signal(_sig, _frame):
        shutdown_event.set()
    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    if not node.start():
        sys.exit(1)

    try:
        if node._node is not None:
            shutdown_event.wait()
        else:
            while not shutdown_event.is_set():
                node.run_terminal_only()
                break
    finally:
        node.stop()


if __name__ == "__main__":
    main()

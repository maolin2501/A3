#!/usr/bin/env python3
"""
EL-A3 Web UI 的 SDK 桥接层。
通过 el_a3_sdk 的 ArmManager + ELA3ROSInterface 统一控制机械臂，
取代原先 ros2_bridge.py 中的直接 ROS2 话题/服务调用。
"""

import threading
import time
import subprocess
import signal
import os
import math
import logging
from typing import Callable, Dict, List, Optional

logger = logging.getLogger("el_a3_web_ui.sdk_bridge")


class SDKBridge:
    """基于 el_a3_sdk 的 Web 桥接层。"""

    JOINT_NAMES = ['L1_joint', 'L2_joint', 'L3_joint', 'L4_joint', 'L5_joint', 'L6_joint']

    JOINT_LIMITS = {
        'L1_joint': {'lower': -2.79253, 'upper': 2.79253},
        'L2_joint': {'lower': -3.14159, 'upper': 3.14159},
        'L3_joint': {'lower': -3.14159, 'upper': 3.14159},
        'L4_joint': {'lower': -3.14159, 'upper': 3.14159},
        'L5_joint': {'lower': -3.14159, 'upper': 3.14159},
        'L6_joint': {'lower': -3.14159, 'upper': 3.14159},
    }

    def __init__(self, namespace: str = "", controller_name: str = "arm_controller"):
        self._namespace = namespace
        self._controller_name = controller_name

        self._sdk = None
        self._sdk_connected = False
        self._system_status = 'disconnected'
        self._motors_enabled = False
        self._last_update_time = 0.0

        self._teleop_process: Optional[subprocess.Popen] = None
        self._teleop_status = 'stopped'
        self._current_can_interface = 'can0'

        self._state_callback: Optional[Callable] = None
        self._log_callback: Optional[Callable] = None

        self._lock = threading.Lock()
        self._poll_thread: Optional[threading.Thread] = None
        self._running = False

    # ================================================================
    # Lifecycle
    # ================================================================

    def start(self):
        """Initialise SDK and start background polling."""
        self._running = True
        try:
            self._init_sdk()
        except Exception as e:
            self._log(f'SDK 初始化失败: {e}', 'error')

        self._poll_thread = threading.Thread(target=self._poll_loop, daemon=True)
        self._poll_thread.start()

    def stop(self):
        """Shut down polling and disconnect SDK."""
        self._running = False
        if self._poll_thread:
            self._poll_thread.join(timeout=3.0)
        self.stop_teleop()
        if self._sdk:
            try:
                self._sdk.DisconnectPort()
            except Exception:
                pass

    def _init_sdk(self):
        """Register and connect the arm via ArmManager."""
        import rclpy
        if not rclpy.ok():
            rclpy.init()

        from el_a3_sdk.arm_manager import ArmManager
        mgr = ArmManager.get_instance()
        self._sdk = mgr.register_ros_arm(
            "webui",
            namespace=self._namespace,
            controller_name=self._controller_name,
        )
        ok = self._sdk.ConnectPort()
        self._sdk_connected = ok
        if ok:
            self._log('SDK 已连接', 'info')
        else:
            self._log('SDK ConnectPort 失败', 'error')

    # ================================================================
    # Background polling (replaces ROS2 subscriber + timer)
    # ================================================================

    def _poll_loop(self):
        """Periodically read joint states from SDK and push to frontend."""
        while self._running:
            try:
                if self._sdk and self._sdk_connected:
                    js = self._sdk.GetArmJointMsgs()
                    if js and js.timestamp > 0:
                        self._last_update_time = time.time()
                        if self._system_status != 'connected':
                            self._system_status = 'connected'
                            self._log('已连接到机器人', 'info')
                    else:
                        if time.time() - self._last_update_time > 3.0:
                            if self._system_status != 'disconnected':
                                self._system_status = 'disconnected'
                                self._log('与机器人连接已断开', 'warn')

                    if self._state_callback:
                        self._state_callback(self.get_state())
                else:
                    if time.time() - self._last_update_time > 3.0:
                        if self._system_status != 'disconnected':
                            self._system_status = 'disconnected'

                self._check_teleop_process()
            except Exception as e:
                logger.debug("poll error: %s", e)
            time.sleep(0.1)

    # ================================================================
    # Callbacks
    # ================================================================

    def set_state_callback(self, callback: Callable):
        self._state_callback = callback

    def set_log_callback(self, callback: Callable):
        self._log_callback = callback

    def _log(self, message: str, level: str = 'info'):
        getattr(logger, level if level != 'warn' else 'warning', logger.info)(message)
        if self._log_callback:
            self._log_callback({'timestamp': time.time(), 'level': level, 'message': message})

    # ================================================================
    # State reading (all via SDK)
    # ================================================================

    def get_state(self) -> Dict:
        """Build full robot state dict for the frontend."""
        joints: List[Dict] = []
        gripper_angle = 0.0
        end_effector = {}
        sdk_version = ""
        arm_state = "UNKNOWN"
        gravity_torques: List[float] = []

        if self._sdk and self._sdk_connected:
            try:
                js = self._sdk.GetArmJointMsgs()
                vel = self._sdk.GetArmJointVelocities()
                eff = self._sdk.GetArmJointEfforts()

                pos_list = js.to_list() if js else [0.0] * 7
                vel_list = vel.to_list() if vel else [0.0] * 7
                eff_list = eff.to_list() if eff else [0.0] * 7

                for i, name in enumerate(self.JOINT_NAMES):
                    joints.append({
                        'name': name,
                        'position': pos_list[i],
                        'position_deg': math.degrees(pos_list[i]),
                        'velocity': vel_list[i],
                        'effort': eff_list[i],
                        'gravity_torque': 0.0,
                        'temperature': 0.0,
                        'limits': self.JOINT_LIMITS[name],
                    })

                if len(pos_list) >= 7:
                    gripper_angle = pos_list[6]

                try:
                    pose = self._sdk.GetArmEndPoseMsgs()
                    end_effector = {
                        'x': pose.x, 'y': pose.y, 'z': pose.z,
                        'rx': pose.rx, 'ry': pose.ry, 'rz': pose.rz,
                    }
                except Exception:
                    pass

                try:
                    gt = self._sdk.ComputeGravityTorques()
                    gravity_torques = list(gt)
                    for idx, name in enumerate(self.JOINT_NAMES):
                        if idx < len(gravity_torques):
                            joints[idx]['gravity_torque'] = gravity_torques[idx]
                except Exception:
                    pass

                sdk_version = self._sdk.GetCurrentSDKVersion()
                arm_state = self._sdk.arm_state.name
            except Exception as e:
                logger.debug("get_state SDK error: %s", e)

        if not joints:
            for name in self.JOINT_NAMES:
                joints.append({
                    'name': name, 'position': 0.0, 'position_deg': 0.0,
                    'velocity': 0.0, 'effort': 0.0, 'gravity_torque': 0.0,
                    'temperature': 0.0, 'limits': self.JOINT_LIMITS[name],
                })

        return {
            'timestamp': time.time(),
            'status': self._system_status,
            'controller': 'sdk',
            'motors_enabled': self._motors_enabled,
            'teleop_status': self.get_teleop_status(),
            'can_interface': self._current_can_interface,
            'joints': joints,
            'gripper_angle': gripper_angle,
            'end_effector': end_effector,
            'sdk_version': sdk_version,
            'arm_state': arm_state,
            'gravity_torques': gravity_torques,
        }

    # ================================================================
    # Motion commands (all via SDK)
    # ================================================================

    def send_joint_command(self, positions: List[float], duration: float = 2.0) -> bool:
        if not self._sdk or not self._sdk_connected:
            self._log('SDK 未连接', 'error')
            return False
        if len(positions) != 6:
            self._log(f'关节数量非法：{len(positions)}，期望 6', 'error')
            return False

        pos7 = list(positions) + [0.0]
        try:
            ok = self._sdk.MoveJ(pos7, duration=duration)
            self._log(f'已发送 MoveJ（时长 {duration:.1f}s）', 'info')
            return ok
        except Exception as e:
            self._log(f'MoveJ 异常: {e}', 'error')
            return False

    def send_single_joint_command(self, joint_index: int, position: float, duration: float = 1.0) -> bool:
        if not self._sdk or not self._sdk_connected:
            self._log('SDK 未连接', 'error')
            return False
        if not 0 <= joint_index < 6:
            self._log(f'关节索引非法：{joint_index}', 'error')
            return False

        js = self._sdk.GetArmJointMsgs()
        pos_list = js.to_list() if js else [0.0] * 7
        current = list(pos_list[:6])
        current[joint_index] = position
        duration_ns = int(duration * 1e9)
        try:
            ok = self._sdk.JointCtrlList(current, duration_ns=duration_ns)
            self._log(f'已发送单关节指令 L{joint_index+1}', 'info')
            return ok
        except Exception as e:
            self._log(f'JointCtrlList 异常: {e}', 'error')
            return False

    def go_home(self, duration: float = 3.0) -> bool:
        """Plan and execute to zero via MoveIt (safe)."""
        if not self._sdk or not self._sdk_connected:
            self._log('SDK 未连接', 'error')
            return False

        self._log('正在通过 MoveIt 规划回零...', 'info')
        try:
            ok = self._sdk.PlanToJointGoal(
                joint_positions=[0.0] * 6,
                velocity_scale=0.3,
                accel_scale=0.3,
                planning_time=5.0,
            )
            if ok:
                self._log('已回到零位', 'info')
            else:
                self._log('MoveIt 回零失败', 'error')
            return ok
        except Exception as e:
            self._log(f'PlanToJointGoal 异常: {e}', 'error')
            return False

    def go_zero(self, duration: float = 3.0) -> bool:
        return self.go_home(duration)

    def plan_joint_goal(self, positions: List[float], velocity_scale: float = 0.3) -> bool:
        if not self._sdk or not self._sdk_connected:
            self._log('SDK 未连接', 'error')
            return False
        try:
            ok = self._sdk.PlanToJointGoal(
                joint_positions=positions,
                velocity_scale=velocity_scale,
                accel_scale=velocity_scale,
            )
            self._log(f'PlanToJointGoal {"成功" if ok else "失败"}', 'info' if ok else 'error')
            return ok
        except Exception as e:
            self._log(f'PlanToJointGoal 异常: {e}', 'error')
            return False

    def move_to_pose(self, x: float, y: float, z: float,
                     rx: float, ry: float, rz: float,
                     duration: float = 2.0) -> bool:
        if not self._sdk or not self._sdk_connected:
            self._log('SDK 未连接', 'error')
            return False
        try:
            ok = self._sdk.EndPoseCtrl(x, y, z, rx, ry, rz, duration=duration)
            self._log(f'EndPoseCtrl {"成功" if ok else "失败"}', 'info' if ok else 'error')
            return ok
        except Exception as e:
            self._log(f'EndPoseCtrl 异常: {e}', 'error')
            return False

    def set_gripper(self, angle: float) -> bool:
        if not self._sdk or not self._sdk_connected:
            self._log('SDK 未连接', 'error')
            return False
        try:
            ok = self._sdk.GripperCtrl(gripper_angle=angle)
            self._log(f'夹爪角度 -> {math.degrees(angle):.1f}°', 'info')
            return ok
        except Exception as e:
            self._log(f'GripperCtrl 异常: {e}', 'error')
            return False

    # ================================================================
    # Safety
    # ================================================================

    def emergency_stop(self) -> bool:
        if not self._sdk or not self._sdk_connected:
            self._log('SDK 未连接，无法急停', 'error')
            return False
        self._log('急停（EMERGENCY STOP）', 'warn')
        try:
            return self._sdk.EmergencyStop()
        except Exception as e:
            self._log(f'EmergencyStop 异常: {e}', 'error')
            return False

    # ================================================================
    # Mode control
    # ================================================================

    def enable_motors(self) -> bool:
        if not self._sdk or not self._sdk_connected:
            self._log('SDK 未连接', 'error')
            return False
        try:
            ok = self._sdk.EnableArm()
            if ok:
                self._motors_enabled = True
                self._log('电机已使能', 'info')
            else:
                self._log('电机使能失败', 'error')
            return ok
        except Exception as e:
            self._log(f'EnableArm 异常: {e}', 'error')
            return False

    def disable_motors(self) -> bool:
        if not self._sdk or not self._sdk_connected:
            self._log('SDK 未连接', 'error')
            return False
        try:
            ok = self._sdk.DisableArm()
            if ok:
                self._motors_enabled = False
                self._log('电机已失能', 'info')
            else:
                self._log('电机失能失败', 'error')
            return ok
        except Exception as e:
            self._log(f'DisableArm 异常: {e}', 'error')
            return False

    def set_zero_torque_mode(self, enable: bool) -> bool:
        if not self._sdk or not self._sdk_connected:
            self._log('SDK 未连接', 'error')
            return False
        try:
            ok = self._sdk.ZeroTorqueMode(enable)
            if ok:
                self._motors_enabled = not enable
                self._log(f'零力矩模式：{"已开启" if enable else "已关闭"}', 'info')
            else:
                self._log('设置零力矩模式失败', 'error')
            return ok
        except Exception as e:
            self._log(f'ZeroTorqueMode 异常: {e}', 'error')
            return False

    def get_motors_enabled(self) -> bool:
        return self._motors_enabled

    # ================================================================
    # SDK info
    # ================================================================

    def get_sdk_info(self) -> Dict:
        info: Dict = {
            'connected': self._sdk_connected,
            'version': '',
            'arm_state': 'UNKNOWN',
            'protocol': '',
        }
        if self._sdk and self._sdk_connected:
            try:
                info['version'] = self._sdk.GetCurrentSDKVersion()
                info['arm_state'] = self._sdk.arm_state.name
                info['protocol'] = self._sdk.GetCurrentProtocolVersion()
            except Exception:
                pass
        return info

    def get_end_effector(self) -> Dict:
        if self._sdk and self._sdk_connected:
            try:
                pose = self._sdk.GetArmEndPoseMsgs()
                return {
                    'x': pose.x, 'y': pose.y, 'z': pose.z,
                    'rx': pose.rx, 'ry': pose.ry, 'rz': pose.rz,
                }
            except Exception:
                pass
        return {}

    def get_dynamics(self) -> Dict:
        if self._sdk and self._sdk_connected:
            try:
                gt = self._sdk.ComputeGravityTorques()
                return {'gravity_torques': list(gt)}
            except Exception:
                pass
        return {'gravity_torques': []}

    # ================================================================
    # Teleop (subprocess — unchanged from ros2_bridge)
    # ================================================================

    def _check_teleop_process(self):
        if self._teleop_process is not None:
            if self._teleop_process.poll() is not None:
                self._teleop_status = 'stopped'
                self._teleop_process = None
                self._log('Teleop 进程已退出', 'info')

    def start_teleop(self) -> bool:
        if self._teleop_process is not None and self._teleop_process.poll() is None:
            self._log('Teleop 已在运行', 'warn')
            return True
        try:
            cmd = ['ros2', 'run', 'el_a3_teleop', 'xbox_teleop_node',
                   '--ros-args', '-p', 'use_fast_ik_mode:=true']
            self._teleop_process = subprocess.Popen(
                cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                preexec_fn=os.setsid)
            self._teleop_status = 'running'
            self._log('Teleop 已启动', 'info')
            return True
        except Exception as e:
            self._log(f'启动 Teleop 失败：{e}', 'error')
            self._teleop_status = 'error'
            return False

    def stop_teleop(self) -> bool:
        if self._teleop_process is None:
            self._teleop_status = 'stopped'
            return True
        try:
            os.killpg(os.getpgid(self._teleop_process.pid), signal.SIGTERM)
            try:
                self._teleop_process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                os.killpg(os.getpgid(self._teleop_process.pid), signal.SIGKILL)
                self._teleop_process.wait()
            self._teleop_process = None
            self._teleop_status = 'stopped'
            self._log('Teleop 已停止', 'info')
            return True
        except Exception as e:
            self._log(f'停止 Teleop 失败：{e}', 'error')
            self._teleop_process = None
            self._teleop_status = 'error'
            return False

    def get_teleop_status(self) -> str:
        if self._teleop_process is not None and self._teleop_process.poll() is None:
            return 'running'
        return self._teleop_status

    # ================================================================
    # CAN interface config (kept for UI compatibility)
    # ================================================================

    def get_can_interface(self) -> str:
        return self._current_can_interface

    def set_can_interface(self, interface: str) -> bool:
        if not interface.startswith('can'):
            self._log(f'CAN 接口名非法：{interface}', 'error')
            return False
        self._current_can_interface = interface
        self._log(f'CAN 接口已设置为 {interface}（需重启生效）', 'info')
        return True

    @staticmethod
    def get_available_can_interfaces() -> List[str]:
        interfaces: List[str] = []
        try:
            result = subprocess.run(
                ['ip', 'link', 'show', 'type', 'can'],
                capture_output=True, text=True, timeout=5.0)
            for line in result.stdout.split('\n'):
                if ':' in line and 'can' in line:
                    parts = line.split(':')
                    if len(parts) >= 2:
                        iface = parts[1].strip()
                        if iface.startswith('can'):
                            interfaces.append(iface)
        except Exception:
            pass
        return interfaces or ['can0']


# ================================================================
# Singleton helpers (drop-in for web_server.py)
# ================================================================

_bridge_instance: Optional[SDKBridge] = None


def get_bridge() -> SDKBridge:
    global _bridge_instance
    if _bridge_instance is None:
        _bridge_instance = SDKBridge()
        _bridge_instance.start()
    return _bridge_instance


def shutdown_bridge():
    global _bridge_instance
    if _bridge_instance:
        _bridge_instance.stop()
        _bridge_instance = None

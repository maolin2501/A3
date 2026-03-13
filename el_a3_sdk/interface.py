"""
EL-A3 机械臂 SDK 主接口

对标 Piper SDK 的 C_PiperInterface，提供：
  - 连接管理 (ConnectPort / DisconnectPort)
  - 电机使能/失能 (EnableArm / DisableArm)
  - 运动控制 (JointCtrl / ModeCtrl / GripperCtrl / EndPoseCtrl)
  - 状态反馈 (GetArmJointMsgs / GetArmStatus / GetMotorStates / ...)
  - 安全控制 (EmergencyStop / ResetArm)
  - 参数查询 (SearchMotorMaxAngleSpdAccLimit / GetFirmwareVersion / ...)
  - 高级模式 (ZeroTorqueMode / MasterSlaveConfig)

底层使用 Robstride 私有协议（CAN 2.0 扩展帧），单位全部采用 SI（rad, m, Nm）。
"""

import time
import threading
import logging
from typing import Dict, List, Optional, Literal

import numpy as np

from el_a3_sdk.can_driver import RobstrideCanDriver, _busy_wait_us
from el_a3_sdk.protocol import (
    MotorType, RunMode, ControlMode, MoveMode, ModeState, ArmState,
    MotorParams, ParamIndex, LogLevel,
    MOTOR_PARAMS, DEFAULT_MOTOR_TYPE_MAP,
    DEFAULT_JOINT_DIRECTIONS, DEFAULT_JOINT_OFFSETS, DEFAULT_JOINT_LIMITS,
)
from el_a3_sdk.data_types import (
    MotorFeedback, ArmJointStates, ArmEndPose, ArmStatus,
    MotorHighSpdInfo, MotorLowSpdInfo,
    MotorAngleLimitMaxVel, MotorMaxAccLimit,
    ParamReadResult, FirmwareVersion,
    DynamicsInfo, TrajectoryResult,
)
from el_a3_sdk.utils import clamp


logger = logging.getLogger("el_a3_sdk")


class ELA3Interface:
    """
    EL-A3 机械臂主接口

    用法示例::

        from el_a3_sdk import ELA3Interface

        arm = ELA3Interface(can_name="can0")
        arm.ConnectPort()
        arm.EnableArm()
        arm.ModeCtrl(ctrl_mode=1, move_mode=0)
        arm.JointCtrl(0.0, 1.57, -0.78, 0.0, 0.0, 0.0)
        print(arm.GetArmJointMsgs())
        arm.DisableArm()
        arm.DisconnectPort()
    """

    NUM_JOINTS = 7
    NUM_ARM_JOINTS = 6

    def __init__(
        self,
        can_name: str = "can0",
        host_can_id: int = 0xFD,
        motor_type_map: Optional[Dict[int, MotorType]] = None,
        joint_directions: Optional[Dict[int, float]] = None,
        joint_offsets: Optional[Dict[int, float]] = None,
        joint_limits: Optional[Dict[int, tuple]] = None,
        start_sdk_joint_limit: bool = True,
        default_kp: float = 80.0,
        default_kd: float = 4.0,
        urdf_path: Optional[str] = None,
        inertia_config_path: Optional[str] = None,
        logger_level: LogLevel = LogLevel.WARNING,
    ):
        """
        Args:
            can_name: CAN 接口名（如 "can0"）
            host_can_id: 主机 CAN ID（默认 0xFD）
            motor_type_map: 电机 ID -> 型号映射（默认 1-3=RS00, 4-6=EL05）
            joint_directions: 关节方向映射（1.0 或 -1.0）
            joint_offsets: 关节偏移映射（rad）
            joint_limits: 关节限位映射 {id: (lower, upper)} (rad)
            start_sdk_joint_limit: 是否启用 SDK 关节限位检查
            default_kp: 默认位置增益（需根据实际负载调整）
            default_kd: 默认速度增益（需根据实际负载调整）
            urdf_path: URDF 路径（用于 Pinocchio 运动学/动力学，None=自动查找）
            inertia_config_path: 标定惯量参数 YAML 路径（可选）
            logger_level: 日志级别
        """
        logging.basicConfig(
            level=int(logger_level),
            format="[%(name)s][%(levelname)s] %(message)s",
        )

        self._can_name = can_name
        self._driver = RobstrideCanDriver(
            can_name=can_name,
            host_can_id=host_can_id,
            motor_type_map=motor_type_map,
        )
        self._joint_directions = joint_directions or dict(DEFAULT_JOINT_DIRECTIONS)
        self._joint_offsets = joint_offsets or dict(DEFAULT_JOINT_OFFSETS)
        self._joint_limits = joint_limits or dict(DEFAULT_JOINT_LIMITS)
        self._joint_limit_enabled = start_sdk_joint_limit

        self._connected = False
        self._state = ArmState.DISCONNECTED
        self._ctrl_mode = ControlMode.STANDBY
        self._move_mode = MoveMode.MOVE_J
        self._move_spd_rate = 50

        self._zero_torque_mode = False
        self._zero_torque_kd = 1.0
        self._zero_torque_thread: Optional[threading.Thread] = None
        self._zero_torque_running = False

        # 位置控制参数（运控模式默认 PD 增益）
        self._position_kp = default_kp
        self._position_kd = default_kd

        # Pinocchio 运动学/动力学（延迟初始化）
        self._kin = None
        self._urdf_path = urdf_path
        self._inertia_config_path = inertia_config_path

    # ================================================================
    # 连接管理（对标 ConnectPort / DisconnectPort）
    # ================================================================

    def ConnectPort(self) -> bool:
        """
        连接机械臂并启动收发线程

        Returns:
            是否连接成功
        """
        if self._connected:
            logger.warning("已经连接，无需重复调用 ConnectPort")
            return True

        if not self._driver.connect():
            return False

        self._driver.start_receive_thread()
        self._connected = True
        self._state = ArmState.IDLE
        logger.info("机械臂已连接: %s", self._can_name)
        return True

    def DisconnectPort(self):
        """断开连接并释放资源"""
        if not self._connected:
            return
        self.DisableArm()
        time.sleep(0.1)
        self._driver.disconnect()
        self._connected = False
        self._state = ArmState.DISCONNECTED
        logger.info("机械臂已断开: %s", self._can_name)

    def get_connect_status(self) -> bool:
        """获取连接状态"""
        return self._connected and self._driver.is_connected

    @property
    def arm_state(self) -> ArmState:
        """获取当前机械臂状态"""
        return self._state

    # ================================================================
    # 电机使能 / 失能（对标 EnableArm / DisableArm）
    # ================================================================

    def EnableArm(self, motor_num: int = 7, run_mode: RunMode = RunMode.MOTION_CONTROL,
                  startup_kd: float = 4.0) -> bool:
        """
        使能电机

        Args:
            motor_num: 电机编号（1-6 单个，7=全部）
            run_mode: 运行模式（默认运控模式）
            startup_kd: 软启动阻尼系数（Kp=0 防止突跳）

        Returns:
            是否全部成功
        """
        if not self._connected:
            logger.error("未连接，请先调用 ConnectPort()")
            return False

        motor_ids = self._resolve_motor_ids(motor_num)
        success = True

        for mid in motor_ids:
            self._driver.disable_motor(mid, clear_fault=True)
            time.sleep(0.03)

            if not self._driver.set_run_mode(mid, run_mode):
                logger.error("电机 %d 设置运行模式失败", mid)
                success = False
                continue
            time.sleep(0.03)

            if not self._driver.enable_motor(mid):
                logger.error("电机 %d 使能失败", mid)
                success = False
                continue

            # 运控模式下发送 Kp=0 软启动指令，防止突跳
            if run_mode == RunMode.MOTION_CONTROL:
                self._driver.send_motion_control(mid, 0.0, 0.0, 0.0, startup_kd, 0.0)

            logger.info("电机 %d 已使能 (mode=%s)", mid, run_mode.name)
            time.sleep(0.03)

        if success:
            self._state = ArmState.ENABLED
        return success

    def DisableArm(self, motor_num: int = 7) -> bool:
        """
        失能电机

        Args:
            motor_num: 电机编号（1-6 单个，7=全部）
        """
        if not self._connected:
            return False

        motor_ids = self._resolve_motor_ids(motor_num)
        success = True

        for mid in motor_ids:
            if not self._driver.disable_motor(mid):
                success = False
            time.sleep(0.005)

        if success and motor_num in (7, 0xFF):
            self._state = ArmState.IDLE
        return success

    # ================================================================
    # 安全控制（对标 EmergencyStop / ResetPiper）
    # ================================================================

    def EmergencyStop(self) -> bool:
        """
        急停：立即失能所有电机并清除故障

        机械臂会立即掉电落下，仅在紧急情况使用。
        """
        if not self._connected:
            return False

        success = True
        for mid in range(1, self.NUM_JOINTS + 1):
            if not self._driver.disable_motor(mid, clear_fault=True):
                success = False

        self._zero_torque_mode = False
        self._state = ArmState.IDLE
        logger.warning("急停已执行！所有电机已失能")
        return success

    def ResetArm(self) -> bool:
        """
        复位机械臂：失能所有电机、清除故障、重置内部状态

        对标 Piper ResetPiper。
        """
        success = self.EmergencyStop()
        self._ctrl_mode = ControlMode.STANDBY
        self._move_mode = MoveMode.MOVE_J
        self._state = ArmState.IDLE
        logger.info("机械臂已复位")
        return success

    # ================================================================
    # 模式控制（对标 ModeCtrl）
    # ================================================================

    def ModeCtrl(
        self,
        ctrl_mode: int = 0x01,
        move_mode: int = 0x00,
        move_spd_rate_ctrl: int = 50,
    ):
        """
        设置控制模式和运动模式

        Args:
            ctrl_mode: 0x00=待机, 0x01=CAN 控制
            move_mode: 0x00=MOVE_J(运控), 0x01=MOVE_CSP, 0x02=MOVE_VELOCITY, 0x03=MOVE_CURRENT
            move_spd_rate_ctrl: 运动速度百分比 (0-100)

        Robstride 映射:
            - MOVE_J   -> RunMode.MOTION_CONTROL (Type 1 运控指令)
            - MOVE_CSP -> RunMode.POSITION_CSP   (Type 18 写 loc_ref)
        """
        self._ctrl_mode = ControlMode(ctrl_mode)
        self._move_mode = MoveMode(move_mode)
        self._move_spd_rate = clamp(move_spd_rate_ctrl, 0, 100)

        if self._ctrl_mode == ControlMode.STANDBY:
            logger.info("切换到待机模式")
            return

        run_mode_map = {
            MoveMode.MOVE_J: RunMode.MOTION_CONTROL,
            MoveMode.MOVE_CSP: RunMode.POSITION_CSP,
            MoveMode.MOVE_VELOCITY: RunMode.VELOCITY,
            MoveMode.MOVE_CURRENT: RunMode.CURRENT,
        }
        target_run_mode = run_mode_map.get(self._move_mode, RunMode.MOTION_CONTROL)

        for mid in range(1, self.NUM_JOINTS + 1):
            self._driver.disable_motor(mid, clear_fault=False)
            time.sleep(0.03)
            self._driver.set_run_mode(mid, target_run_mode)
            time.sleep(0.03)
            self._driver.enable_motor(mid)
            time.sleep(0.03)

        logger.info("模式已设置: ctrl=%s, move=%s, spd_rate=%d%%",
                     self._ctrl_mode.name, self._move_mode.name, self._move_spd_rate)

    # ================================================================
    # 关节控制（对标 JointCtrl）
    # ================================================================

    def JointCtrl(
        self,
        joint_1: float, joint_2: float, joint_3: float,
        joint_4: float, joint_5: float, joint_6: float,
        kp: Optional[float] = None,
        kd: Optional[float] = None,
        velocity: float = 0.0,
        torque_ff: Optional[List[float]] = None,
    ) -> bool:
        """
        关节角度控制（运控模式）

        Args:
            joint_1~6: 目标关节角度 (rad)，关节坐标系
            kp: 位置增益（None=使用默认值）
            kd: 速度增益（None=使用默认值）
            velocity: 速度前馈 (rad/s)
            torque_ff: 各关节前馈力矩列表 (Nm)，长度 6，None=全 0

        Returns:
            是否全部发送成功
        """
        if not self._connected:
            logger.error("未连接")
            return False
        if self._state not in (ArmState.ENABLED, ArmState.RUNNING):
            logger.error("当前状态 %s 不允许关节控制，请先 EnableArm()", self._state.name)
            return False

        self._state = ArmState.RUNNING
        positions = [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6]
        use_kp = kp if kp is not None else self._position_kp
        use_kd = kd if kd is not None else self._position_kd
        ff_torques = torque_ff or [0.0] * self.NUM_ARM_JOINTS

        success = True
        for i, mid in enumerate(range(1, self.NUM_ARM_JOINTS + 1)):
            target_pos = positions[i]

            if self._joint_limit_enabled:
                limits = self._joint_limits.get(mid)
                if limits:
                    target_pos = clamp(target_pos, limits[0], limits[1])

            direction = self._joint_directions.get(mid, 1.0)
            offset = self._joint_offsets.get(mid, 0.0)
            motor_pos = target_pos * direction + offset

            motor_vel = velocity * direction
            motor_torque = ff_torques[i] * direction if i < len(ff_torques) else 0.0

            if not self._driver.send_motion_control(
                mid, motor_pos, motor_vel, use_kp, use_kd, motor_torque
            ):
                success = False

            _busy_wait_us(50)

        return success

    def JointCtrlList(self, positions: List[float], **kwargs) -> bool:
        """便捷接口：列表形式的关节控制（6=仅手臂，7=手臂+夹爪）"""
        n = len(positions)
        if n < self.NUM_ARM_JOINTS or n > self.NUM_JOINTS:
            logger.error("positions 长度必须为 %d 或 %d", self.NUM_ARM_JOINTS, self.NUM_JOINTS)
            return False
        result = self.JointCtrl(*positions[:6], **kwargs)
        if n >= 7:
            result = self.GripperCtrl(gripper_angle=positions[6]) and result
        return result

    # ================================================================
    # 夹爪控制（对标 GripperCtrl）
    # ================================================================

    def GripperCtrl(
        self,
        gripper_angle: float = 0.0,
        gripper_effort: float = 0.0,
        gripper_enable: bool = True,
        set_zero: bool = False,
        kp: Optional[float] = None,
        kd: Optional[float] = None,
    ) -> bool:
        """
        夹爪控制（如有第 7 号电机）

        Args:
            gripper_angle: 夹爪目标位置 (rad)
            gripper_effort: 夹爪力矩 (Nm)
            gripper_enable: 是否使能
            set_zero: 是否设置当前位置为零点
            kp: 位置增益（None=使用默认值）
            kd: 速度增益（None=使用默认值）
        """
        gripper_motor_id = 7

        if set_zero:
            return self._driver.set_zero_position(gripper_motor_id)

        if not gripper_enable:
            return self._driver.disable_motor(gripper_motor_id)

        use_kp = kp if kp is not None else self._position_kp
        use_kd = kd if kd is not None else self._position_kd

        return self._driver.send_motion_control(
            gripper_motor_id, gripper_angle, 0.0,
            use_kp, use_kd, gripper_effort
        )

    # ================================================================
    # 零力矩模式（对标 Piper 示教模式 / 主臂零力矩拖动）
    # ================================================================

    def ZeroTorqueMode(self, enable: bool, kd: float = 1.0,
                       gravity_torques: Optional[List[float]] = None) -> bool:
        """
        零力矩模式（Kp=0，可手动拖动机械臂）

        Args:
            enable: True=启用, False=恢复位置控制
            kd: 阻尼系数
            gravity_torques: 各关节重力补偿力矩 (Nm)，None=不补偿

        对应 el_a3_hardware.cpp 中的零力矩模式实现。
        """
        if not self._connected:
            return False
        if enable and self._state not in (ArmState.ENABLED, ArmState.RUNNING, ArmState.ZERO_TORQUE):
            logger.error("当前状态 %s 不允许零力矩，请先 EnableArm()", self._state.name)
            return False

        self._zero_torque_mode = enable
        self._zero_torque_kd = kd

        if enable:
            self._state = ArmState.ZERO_TORQUE
            grav = list(gravity_torques or [0.0] * self.NUM_JOINTS)
            for i, mid in enumerate(range(1, self.NUM_JOINTS + 1)):
                fb = self._driver.get_feedback(mid)
                current_pos = fb.position if (fb and fb.is_valid) else 0.0
                direction = self._joint_directions.get(mid, 1.0)
                motor_torque = grav[i] * direction if i < len(grav) else 0.0

                self._driver.send_motion_control(
                    mid, current_pos, 0.0, 0.0, kd, motor_torque
                )
                _busy_wait_us(50)

            logger.warning("零力矩模式已启用 (Kd=%.3f)", kd)
        else:
            self._state = ArmState.ENABLED
            logger.info("零力矩模式已关闭，恢复位置控制")

        return True

    # ================================================================
    # 主从配置（对标 MasterSlaveConfig）
    # ================================================================

    def MasterSlaveConfig(self, mode: int = 0xFC, *args) -> bool:
        """
        主从模式配置

        Args:
            mode: 0xFC=从模式（正常工作），0xFD=主模式（零力矩拖动读取）

        Robstride 实现：
        - 从模式: 使能电机 + 正常 PD 控制
        - 主模式: 使能电机 + Kp=0 零力矩模式
        """
        if mode == 0xFD:
            return self.ZeroTorqueMode(True, kd=self._zero_torque_kd)
        else:
            return self.ZeroTorqueMode(False)

    # ================================================================
    # 反馈函数（对标 GetArmJointMsgs / GetArmStatus / ...）
    # ================================================================

    def GetArmJointMsgs(self) -> ArmJointStates:
        """
        获取关节角度反馈

        Returns:
            ArmJointStates，角度单位 rad（关节坐标系）
        """
        positions = []
        latest_ts = 0.0

        for mid in range(1, self.NUM_JOINTS + 1):
            fb = self._driver.get_feedback(mid)
            if fb and fb.is_valid:
                direction = self._joint_directions.get(mid, 1.0)
                offset = self._joint_offsets.get(mid, 0.0)
                joint_pos = (fb.position - offset) * direction
                positions.append(joint_pos)
                latest_ts = max(latest_ts, fb.timestamp)
            else:
                positions.append(0.0)

        return ArmJointStates.from_list(positions, timestamp=latest_ts)

    def GetArmJointVelocities(self) -> ArmJointStates:
        """获取关节速度反馈 (rad/s)"""
        velocities = []
        latest_ts = 0.0

        for mid in range(1, self.NUM_JOINTS + 1):
            fb = self._driver.get_feedback(mid)
            if fb and fb.is_valid:
                direction = self._joint_directions.get(mid, 1.0)
                velocities.append(fb.velocity * direction)
                latest_ts = max(latest_ts, fb.timestamp)
            else:
                velocities.append(0.0)

        return ArmJointStates.from_list(velocities, timestamp=latest_ts)

    def GetArmJointEfforts(self) -> ArmJointStates:
        """获取关节力矩反馈 (Nm)"""
        efforts = []
        latest_ts = 0.0

        for mid in range(1, self.NUM_JOINTS + 1):
            fb = self._driver.get_feedback(mid)
            if fb and fb.is_valid:
                direction = self._joint_directions.get(mid, 1.0)
                efforts.append(fb.torque * direction)
                latest_ts = max(latest_ts, fb.timestamp)
            else:
                efforts.append(0.0)

        return ArmJointStates.from_list(efforts, timestamp=latest_ts)

    def GetArmStatus(self) -> ArmStatus:
        """
        获取机械臂综合状态

        Returns:
            ArmStatus 包含使能状态、故障码、模式等
        """
        status = ArmStatus()
        status.ctrl_mode = int(self._ctrl_mode)
        status.move_mode = int(self._move_mode)

        for i, mid in enumerate(range(1, self.NUM_JOINTS + 1)):
            fb = self._driver.get_feedback(mid)
            if fb and fb.is_valid:
                status.joint_enabled[i] = (fb.mode_state == ModeState.MOTOR)
                status.joint_faults[i] = fb.fault_code
                status.joint_mode_states[i] = fb.mode_state
                status.timestamp = max(status.timestamp, fb.timestamp)

        # 汇总 arm_status
        if status.has_fault:
            status.arm_status = 0x05  # 异常
        elif not status.all_enabled:
            status.arm_status = 0x01  # 未使能（类似急停状态）
        else:
            status.arm_status = 0x00  # 正常

        return status

    def GetArmEnableStatus(self) -> List[bool]:
        """
        获取各电机使能状态

        Returns:
            长度为 6 的 bool 列表
        """
        result = []
        for mid in range(1, self.NUM_JOINTS + 1):
            fb = self._driver.get_feedback(mid)
            result.append(fb.mode_state == ModeState.MOTOR if fb and fb.is_valid else False)
        return result

    def GetArmHighSpdInfoMsgs(self) -> List[MotorHighSpdInfo]:
        """
        获取高速反馈信息（速度/力矩/位置）

        Returns:
            6 个 MotorHighSpdInfo 对象的列表
        """
        result = []
        for mid in range(1, self.NUM_JOINTS + 1):
            fb = self._driver.get_feedback(mid)
            info = MotorHighSpdInfo(motor_id=mid)
            if fb and fb.is_valid:
                direction = self._joint_directions.get(mid, 1.0)
                info.speed = fb.velocity * direction
                info.position = (fb.position - self._joint_offsets.get(mid, 0.0)) * direction
                info.torque = fb.torque * direction
                info.timestamp = fb.timestamp
            result.append(info)
        return result

    def GetArmLowSpdInfoMsgs(self) -> List[MotorLowSpdInfo]:
        """
        获取低速反馈信息（温度/电压/故障）

        温度从 Type 2 反馈直接获取，电压需要参数读取。
        """
        result = []
        for mid in range(1, self.NUM_JOINTS + 1):
            fb = self._driver.get_feedback(mid)
            info = MotorLowSpdInfo(motor_id=mid)
            if fb and fb.is_valid:
                info.motor_temp = fb.temperature
                info.fault_code = fb.fault_code
                info.timestamp = fb.timestamp
            result.append(info)
        return result

    def GetMotorStates(self) -> Dict[int, MotorFeedback]:
        """获取所有电机原始反馈（电机坐标系）"""
        return self._driver.get_all_feedbacks()

    # ================================================================
    # 参数查询（对标 Search / Get 参数系列）
    # ================================================================

    def SearchMotorMaxAngleSpdAccLimit(
        self, motor_num: int = 1, search_content: int = 0x01
    ) -> Optional[ParamReadResult]:
        """
        查询电机参数

        Args:
            motor_num: 电机编号 (1-6)
            search_content: 0x01=速度限制, 0x02=加速度限制
        """
        if search_content == 0x01:
            return self._driver.read_parameter(motor_num, ParamIndex.LIMIT_SPD)
        elif search_content == 0x02:
            return self._driver.read_parameter(motor_num, ParamIndex.ACC_RAD)
        return None

    def GetCurrentMotorAngleLimitMaxVel(self) -> List[MotorAngleLimitMaxVel]:
        """获取所有电机角度限位和最大速度（从配置读取）"""
        result = []
        for mid in range(1, self.NUM_JOINTS + 1):
            limits = self._joint_limits.get(mid, (-6.28, 6.28))
            motor_type = self._driver.motor_type_map.get(mid, MotorType.RS00)
            params = MOTOR_PARAMS[motor_type]
            result.append(MotorAngleLimitMaxVel(
                motor_num=mid,
                min_angle_limit=limits[0],
                max_angle_limit=limits[1],
                max_joint_spd=params.v_max,
            ))
        return result

    def GetAllMotorMaxAccLimit(self) -> List[MotorMaxAccLimit]:
        """获取所有电机最大加速度限制（通过参数读取）"""
        result = []
        for mid in range(1, self.NUM_JOINTS + 1):
            pr = self._driver.read_parameter(mid, ParamIndex.ACC_RAD, timeout=0.3)
            result.append(MotorMaxAccLimit(
                motor_num=mid,
                max_joint_acc=pr.value if (pr and pr.success) else 20.0,
            ))
        return result

    def ReadMotorParameter(self, motor_id: int, param_index: int) -> Optional[ParamReadResult]:
        """通用参数读取"""
        return self._driver.read_parameter(motor_id, param_index)

    def WriteMotorParameter(self, motor_id: int, param_index: int, value: float) -> bool:
        """通用参数写入（掉电丢失）"""
        return self._driver.write_parameter(motor_id, param_index, value)

    def GetMotorVoltage(self, motor_id: int) -> Optional[float]:
        """读取电机母线电压 (V)"""
        result = self._driver.read_parameter(motor_id, ParamIndex.VBUS, timeout=0.3)
        return result.value if (result and result.success) else None

    def GetFirmwareVersion(self, motor_id: int = 1) -> Optional[FirmwareVersion]:
        """查询电机固件版本"""
        return self._driver.query_firmware_version(motor_id)

    def GetAllFirmwareVersions(self) -> Dict[int, FirmwareVersion]:
        """查询所有电机固件版本"""
        result = {}
        for mid in range(1, self.NUM_JOINTS + 1):
            ver = self._driver.query_firmware_version(mid, timeout=0.3)
            if ver:
                result[mid] = ver
        return result

    # ================================================================
    # 零位设置
    # ================================================================

    def SetZeroPosition(self, motor_num: int = 7) -> bool:
        """
        设置当前位置为零位

        Args:
            motor_num: 电机编号（1-6 单个，7=全部）
        """
        motor_ids = self._resolve_motor_ids(motor_num)
        success = True
        for mid in motor_ids:
            if not self._driver.set_zero_position(mid):
                success = False
            time.sleep(0.05)
        return success

    # ================================================================
    # SDK 信息
    # ================================================================

    def GetCanFps(self) -> float:
        """获取 CAN 帧率"""
        return self._driver.get_can_fps()

    def GetCanName(self) -> str:
        """获取当前 CAN 端口名"""
        return self._can_name

    def GetCurrentSDKVersion(self) -> str:
        """获取 SDK 版本"""
        from el_a3_sdk import __version__
        return __version__

    def GetCurrentProtocolVersion(self) -> str:
        """获取协议版本描述"""
        return "Robstride Private Protocol v1.0 (CAN 2.0 Extended Frame)"

    # ================================================================
    # 控制参数设置
    # ================================================================

    def SetPositionPD(self, kp: float, kd: float):
        """设置全局位置控制 PD 增益"""
        self._position_kp = clamp(kp, 0.0, 500.0)
        self._position_kd = clamp(kd, 0.0, 5.0)
        logger.info("PD 增益已设置: Kp=%.1f, Kd=%.2f", self._position_kp, self._position_kd)

    def SetJointLimitEnabled(self, enabled: bool):
        """启用/禁用 SDK 关节限位检查"""
        self._joint_limit_enabled = enabled
        logger.info("关节限位检查: %s", "启用" if enabled else "禁用")

    # ================================================================
    # 笛卡尔控制
    # ================================================================

    def EndPoseCtrl(
        self, x: float, y: float, z: float,
        rx: float, ry: float, rz: float,
        duration: float = 2.0,
        kp: Optional[float] = None,
        kd: Optional[float] = None,
    ) -> bool:
        """
        笛卡尔位姿控制（Pinocchio IK -> S-curve 轨迹 -> JointCtrl）

        Args:
            x, y, z: 目标位置 (m)
            rx, ry, rz: 目标姿态欧拉角 (rad, XYZ 内旋)
            duration: 运动持续时间 (s)
            kp, kd: PD 增益（None=默认值）
        """
        if not self._connected:
            logger.error("未连接")
            return False

        kin = self._get_kinematics()
        if kin is None:
            return False

        target = ArmEndPose(x=x, y=y, z=z, rx=rx, ry=ry, rz=rz)
        current_q = self.GetArmJointMsgs().to_list()

        q_target = kin.inverse_kinematics(target, q_init=current_q)
        if q_target is None:
            logger.error("IK 求解失败，目标位姿可能不可达")
            return False

        return self.MoveJ(q_target, duration=duration, kp=kp, kd=kd)

    def CartesianVelocityCtrl(
        self, vx: float, vy: float, vz: float,
        wx: float, wy: float, wz: float,
        kp: Optional[float] = None,
        kd: Optional[float] = None,
    ) -> bool:
        """
        笛卡尔速度控制（Jacobian 伪逆 -> 关节速度 -> motion_control）

        Args:
            vx, vy, vz: 线速度 (m/s)
            wx, wy, wz: 角速度 (rad/s)
        """
        if not self._connected:
            return False

        kin = self._get_kinematics()
        if kin is None:
            return False

        current_q = self.GetArmJointMsgs().to_list()
        J = kin.compute_jacobian(current_q)

        v_des = np.array([vx, vy, vz, wx, wy, wz])
        damping = 1e-4
        JtJ = J.T @ J + damping * np.eye(J.shape[1])
        dq = np.linalg.solve(JtJ, J.T @ v_des)

        dt = 0.02
        target_q = [current_q[i] + dq[i] * dt for i in range(self.NUM_ARM_JOINTS)]

        return self.JointCtrl(
            *target_q, kp=kp, kd=kd,
            velocity=0.0,
            torque_ff=None,
        )

    def GetArmEndPoseMsgs(self) -> ArmEndPose:
        """获取末端位姿（Pinocchio FK）"""
        kin = self._get_kinematics()
        if kin is None:
            return ArmEndPose()

        q = self.GetArmJointMsgs().to_list()
        return kin.forward_kinematics(q)

    # ================================================================
    # 轨迹运动
    # ================================================================

    def MoveJ(
        self, positions: List[float], duration: float = 2.0,
        v_max: Optional[float] = None, a_max: Optional[float] = None,
        kp: Optional[float] = None, kd: Optional[float] = None,
    ) -> bool:
        """
        关节空间运动（S-curve 规划 + 逐点发送）

        Args:
            positions: 目标关节角度 (rad), 长度 6
            duration: 期望运动持续时间 (s)
            v_max: 最大关节速度 (rad/s)
            a_max: 最大关节加速度 (rad/s^2)
        """
        if not self._connected:
            logger.error("未连接")
            return False

        from el_a3_sdk.trajectory import MultiJointPlanner

        current_q = self.GetArmJointMsgs().to_list()
        vm = v_max or 3.0
        am = a_max or 10.0

        planner = MultiJointPlanner(
            n_joints=self.NUM_ARM_JOINTS, v_max=vm, a_max=am, j_max=50.0,
        )
        profiles = planner.plan_sync(current_q, positions)

        dt = 0.005
        traj = planner.generate_trajectory(profiles, dt=dt)

        gravity_torques = None
        kin = self._get_kinematics()

        start_time = time.time()
        for pt in traj:
            if kin:
                gravity_torques = kin.compute_gravity(pt.positions)

            self.JointCtrl(
                *pt.positions, kp=kp, kd=kd,
                velocity=0.0,
                torque_ff=gravity_torques,
            )

            elapsed = time.time() - start_time
            sleep_time = pt.time - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

        return True

    def MoveL(
        self, target_pose: ArmEndPose, duration: float = 2.0,
        n_waypoints: int = 50,
        kp: Optional[float] = None, kd: Optional[float] = None,
    ) -> bool:
        """
        直线运动（笛卡尔空间线性插值 + IK + 逐点发送）

        Args:
            target_pose: 目标末端位姿
            duration: 运动持续时间 (s)
            n_waypoints: 笛卡尔空间插值点数
        """
        if not self._connected:
            logger.error("未连接")
            return False

        kin = self._get_kinematics()
        if kin is None:
            logger.error("Pinocchio 未初始化，MoveL 不可用")
            return False

        current_q = self.GetArmJointMsgs().to_list()
        start_pose = kin.forward_kinematics(current_q)

        dt = duration / n_waypoints
        q_prev = current_q

        start_time = time.time()
        for i in range(1, n_waypoints + 1):
            s = i / n_waypoints

            x = start_pose.x + s * (target_pose.x - start_pose.x)
            y = start_pose.y + s * (target_pose.y - start_pose.y)
            z = start_pose.z + s * (target_pose.z - start_pose.z)
            rx = start_pose.rx + s * (target_pose.rx - start_pose.rx)
            ry = start_pose.ry + s * (target_pose.ry - start_pose.ry)
            rz = start_pose.rz + s * (target_pose.rz - start_pose.rz)

            wp = ArmEndPose(x=x, y=y, z=z, rx=rx, ry=ry, rz=rz)
            q_sol = kin.inverse_kinematics(wp, q_init=q_prev)
            if q_sol is None:
                logger.error("MoveL IK 失败 at waypoint %d/%d", i, n_waypoints)
                return False

            gravity_torques = kin.compute_gravity(q_sol)
            self.JointCtrl(*q_sol, kp=kp, kd=kd, torque_ff=gravity_torques)
            q_prev = q_sol

            elapsed = time.time() - start_time
            target_time = i * dt
            sleep_time = target_time - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

        return True

    # ================================================================
    # 动力学接口
    # ================================================================

    def ComputeGravityTorques(self, positions: Optional[List[float]] = None) -> List[float]:
        """
        计算重力补偿力矩 (Pinocchio RNEA, 6-DOF arm)

        Args:
            positions: 关节角度 (rad)，None=使用当前位置
        """
        kin = self._get_kinematics()
        if kin is None:
            return [0.0] * self.NUM_ARM_JOINTS

        q = positions or self.GetArmJointMsgs().to_list()
        return kin.compute_gravity(q)

    def GetJacobian(self, positions: Optional[List[float]] = None) -> np.ndarray:
        """获取末端 Jacobian 矩阵 (6xN_arm)"""
        kin = self._get_kinematics()
        if kin is None:
            return np.zeros((6, self.NUM_ARM_JOINTS))

        q = positions or self.GetArmJointMsgs().to_list()
        return kin.compute_jacobian(q)

    def GetMassMatrix(self, positions: Optional[List[float]] = None) -> np.ndarray:
        """获取质量矩阵 M(q) (N_arm x N_arm)"""
        kin = self._get_kinematics()
        if kin is None:
            return np.eye(self.NUM_ARM_JOINTS)

        q = positions or self.GetArmJointMsgs().to_list()
        return kin.mass_matrix(q)

    def InverseDynamics(
        self, q: List[float], v: List[float], a: List[float],
    ) -> List[float]:
        """逆动力学 RNEA: (q, v, a) -> tau"""
        kin = self._get_kinematics()
        if kin is None:
            return [0.0] * self.NUM_ARM_JOINTS
        return kin.inverse_dynamics(q, v, a)

    def ForwardDynamics(
        self, q: List[float], v: List[float], tau: List[float],
    ) -> List[float]:
        """正动力学 ABA: (q, v, tau) -> a"""
        kin = self._get_kinematics()
        if kin is None:
            return [0.0] * self.NUM_ARM_JOINTS
        return kin.forward_dynamics(q, v, tau)

    def GetDynamicsInfo(self, positions: Optional[List[float]] = None) -> DynamicsInfo:
        """获取完整动力学信息"""
        q = positions or self.GetArmJointMsgs().to_list()
        kin = self._get_kinematics()
        if kin is None:
            return DynamicsInfo()

        return DynamicsInfo(
            gravity_torques=kin.compute_gravity(q),
            mass_matrix=kin.mass_matrix(q),
            jacobian=kin.compute_jacobian(q),
            timestamp=time.time(),
        )

    # ================================================================
    # 增强零力矩模式（带 Pinocchio 重力补偿后台线程）
    # ================================================================

    def ZeroTorqueModeWithGravity(
        self, enable: bool, kd=1.0, update_rate: float = 100.0,
    ) -> bool:
        """
        带重力补偿的零力矩模式（后台线程持续发送 Kp=0 + gravity_torque）

        Args:
            enable: True=启用, False=关闭
            kd: 阻尼系数，标量（所有关节相同）或列表（逐关节）
            update_rate: 重力补偿更新频率 (Hz)
        """
        if not self._connected:
            return False

        if enable:
            if isinstance(kd, (list, tuple)):
                kd_list = list(kd)
                if len(kd_list) < self.NUM_JOINTS:
                    kd_list += [kd_list[-1]] * (self.NUM_JOINTS - len(kd_list))
            else:
                kd_list = [float(kd)] * self.NUM_JOINTS

            kin = self._get_kinematics()
            if kin is None:
                logger.warning("Pinocchio 不可用，使用无重力补偿零力矩模式")
                return self.ZeroTorqueMode(True, kd=kd_list[0])

            self._zero_torque_running = True
            self._zero_torque_kd = kd_list[0]
            self._zero_torque_mode = True
            self._zero_torque_thread = threading.Thread(
                target=self._zero_torque_gravity_loop,
                args=(kd_list, 1.0 / update_rate),
                daemon=True,
                name="zero_torque_gravity",
            )
            self._zero_torque_thread.start()
            kd_str = ', '.join(f'{v:.4f}' for v in kd_list)
            logger.warning("零力矩模式已启用 (Kd=[%s], 重力补偿=Pinocchio)", kd_str)
            return True
        else:
            self._zero_torque_running = False
            if self._zero_torque_thread:
                self._zero_torque_thread.join(timeout=1.0)
                self._zero_torque_thread = None
            self._zero_torque_mode = False
            logger.info("零力矩模式已关闭")
            return True

    def _zero_torque_gravity_loop(self, kd_list: list, dt: float):
        """零力矩 + 重力补偿后台循环"""
        kin = self._get_kinematics()
        while self._zero_torque_running and self._connected:
            q = self.GetArmJointMsgs().to_list()
            arm_grav = kin.compute_gravity(q) if kin else [0.0] * self.NUM_ARM_JOINTS
            grav = list(arm_grav) + [0.0] * (self.NUM_JOINTS - len(arm_grav))

            for i, mid in enumerate(range(1, self.NUM_JOINTS + 1)):
                fb = self._driver.get_feedback(mid)
                current_pos = fb.position if (fb and fb.is_valid) else 0.0
                direction = self._joint_directions.get(mid, 1.0)
                motor_torque = grav[i] * direction

                self._driver.send_motion_control(
                    mid, current_pos, 0.0, 0.0, kd_list[i], motor_torque,
                )

            time.sleep(dt)

    # ================================================================
    # 内部辅助
    # ================================================================

    def _get_kinematics(self):
        """延迟初始化 Pinocchio 运动学模块"""
        if self._kin is not None:
            return self._kin
        try:
            from el_a3_sdk.kinematics import ELA3Kinematics
            self._kin = ELA3Kinematics(
                urdf_path=self._urdf_path,
                inertia_config_path=self._inertia_config_path,
                joint_directions=self._joint_directions,
            )
            return self._kin
        except Exception as e:
            logger.debug("Pinocchio 初始化失败 (可选功能): %s", e)
            return None

    def _resolve_motor_ids(self, motor_num: int) -> List[int]:
        """将 motor_num 解析为电机 ID 列表（7=全部）"""
        if motor_num == 7 or motor_num == 0xFF:
            return list(range(1, self.NUM_JOINTS + 1))
        return [motor_num]

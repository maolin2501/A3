"""
EL-A3 机械臂 ROS Control 模式接口

通过 ROS2 话题 / Action / Service 控制机械臂，底层由 el_a3_hardware
(RsA3HardwareInterface) + ros2_control 框架驱动。

与 ELA3Interface (Direct CAN Mode) 保持同名 API，用户可按需选择后端。
"""

import time
import threading
import logging
from typing import Dict, List, Optional

import numpy as np

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from rclpy.action import ActionClient
    from sensor_msgs.msg import JointState
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    from controller_manager_msgs.srv import SwitchController
    from control_msgs.action import FollowJointTrajectory
    from builtin_interfaces.msg import Duration
    from geometry_msgs.msg import Pose, Point, Quaternion
    from std_srvs.srv import SetBool
except ImportError as _e:
    raise ImportError(
        "el_a3_sdk.ros_interface requires rclpy and ROS2 message packages. "
        "Please source your ROS2 workspace before importing."
    ) from _e

try:
    from moveit_msgs.srv import GetPositionIK, GetCartesianPath
    from moveit_msgs.msg import (
        RobotState, PositionIKRequest, Constraints,
        JointConstraint,
    )
    from moveit_msgs.action import MoveGroup
    MOVEIT_AVAILABLE = True
except ImportError:
    MOVEIT_AVAILABLE = False

try:
    from tf2_ros import Buffer as TFBuffer, TransformListener
    from tf2_ros import TransformException
    TF2_AVAILABLE = True
except ImportError:
    TF2_AVAILABLE = False

from el_a3_sdk.protocol import (
    ControlMode, MoveMode, ArmState, LogLevel,
    DEFAULT_JOINT_LIMITS, DEFAULT_JOINT_DIRECTIONS,
)
from el_a3_sdk.data_types import (
    ArmJointStates, ArmEndPose, ArmStatus,
    MotorHighSpdInfo, MotorLowSpdInfo,
    MotorAngleLimitMaxVel, MotorMaxAccLimit,
    ParamReadResult, FirmwareVersion,
    DynamicsInfo, TrajectoryResult,
)
from el_a3_sdk.utils import clamp

logger = logging.getLogger("el_a3_sdk.ros")

ARM_JOINT_NAMES = [
    "L1_joint", "L2_joint", "L3_joint",
    "L4_joint", "L5_joint", "L6_joint",
]
GRIPPER_JOINT_NAMES = ["L7_joint"]
JOINT_NAMES_6 = list(ARM_JOINT_NAMES)
JOINT_NAMES_7 = ARM_JOINT_NAMES + GRIPPER_JOINT_NAMES
JOINT_NAMES = JOINT_NAMES_7


class ELA3ROSInterface:
    """
    EL-A3 机械臂 ROS Control 模式接口

    用法示例::

        from el_a3_sdk.ros_interface import ELA3ROSInterface

        arm = ELA3ROSInterface()
        arm.ConnectPort()
        arm.EnableArm()
        arm.JointCtrl(0.0, 1.57, -0.78, 0.0, 0.0, 0.0)
        print(arm.GetArmJointMsgs())
        arm.DisableArm()
        arm.DisconnectPort()
    """

    NUM_JOINTS = 7
    NUM_ARM_JOINTS = 6
    CONTROLLER_NAME = "arm_controller"
    GRIPPER_CONTROLLER_NAME = "gripper_controller"

    def __init__(
        self,
        node_name: str = "el_a3_sdk_node",
        namespace: str = "",
        joint_names: Optional[List[str]] = None,
        joint_limits: Optional[Dict[int, tuple]] = None,
        start_sdk_joint_limit: bool = True,
        controller_name: str = "arm_controller",
        gripper_controller_name: str = "gripper_controller",
        move_group_name: str = "arm",
        ee_link: str = "end_effector",
        base_link: str = "base_link",
        urdf_path: Optional[str] = None,
        inertia_config_path: Optional[str] = None,
        logger_level: LogLevel = LogLevel.WARNING,
        use_internal_executor: bool = True,
    ):
        """
        Args:
            node_name: ROS2 节点名
            namespace: ROS2 命名空间（多臂场景使用，如 "arm1"）
            joint_names: 关节名列表（默认 L1~L7_joint）
            joint_limits: 关节限位映射 {id: (lower, upper)} (rad)
            start_sdk_joint_limit: 是否启用 SDK 关节限位检查
            controller_name: 手臂轨迹控制器名称 (L1-L6)
            gripper_controller_name: 夹爪控制器名称 (L7)
            move_group_name: MoveIt move group 名称
            ee_link: 末端执行器 link 名称
            base_link: 基座 link 名称
            urdf_path: URDF 路径（用于 Pinocchio，None=自动查找）
            inertia_config_path: 标定惯量参数 YAML 路径（可选）
            logger_level: 日志级别
            use_internal_executor: 是否创建内部 executor 和 spin thread。
                设为 False 时，调用方需将 get_node() 返回的节点加入自己的 executor。
        """
        logging.basicConfig(
            level=int(logger_level),
            format="[%(name)s][%(levelname)s] %(message)s",
        )

        self._node_name = node_name
        self._namespace = namespace
        self._joint_names = joint_names or list(JOINT_NAMES)
        self._arm_joint_names = [j for j in self._joint_names if j in ARM_JOINT_NAMES]
        self._gripper_joint_names = [j for j in self._joint_names if j in GRIPPER_JOINT_NAMES]
        self._joint_limits = joint_limits or dict(DEFAULT_JOINT_LIMITS)
        self._joint_limit_enabled = start_sdk_joint_limit
        self.CONTROLLER_NAME = controller_name
        self.GRIPPER_CONTROLLER_NAME = gripper_controller_name
        self._move_group_name = move_group_name
        self._ee_link = ee_link
        self._base_link = base_link

        self._node: Optional[Node] = None
        self._executor: Optional[SingleThreadedExecutor] = None
        self._spin_thread: Optional[threading.Thread] = None

        self._traj_pub = None
        self._gripper_traj_pub = None
        self._joint_state_sub = None
        self._switch_ctrl_client = None
        self._zero_torque_srv_client = None
        self._follow_traj_client = None
        self._gripper_follow_traj_client = None
        self._ik_client = None
        self._cartesian_path_client = None
        self._move_group_client = None
        self._tf_buffer = None
        self._tf_listener = None

        self._connected = False
        self._state = ArmState.DISCONNECTED
        self._ctrl_mode = ControlMode.STANDBY
        self._move_mode = MoveMode.MOVE_J

        self._latest_joint_state: Optional[JointState] = None
        self._js_lock = threading.Lock()

        self._use_internal_executor = use_internal_executor

        self._kin = None
        self._urdf_path = urdf_path
        self._inertia_config_path = inertia_config_path

    # ================================================================
    # 连接管理
    # ================================================================

    def ConnectPort(self) -> bool:
        """初始化 ROS2 节点并创建通信接口"""
        if self._connected:
            logger.warning("已经连接，无需重复调用 ConnectPort")
            return True

        try:
            if not rclpy.ok():
                rclpy.init()

            self._node = Node(self._node_name, namespace=self._namespace)

            traj_qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
            )
            def _ns_prefix(topic: str) -> str:
                return f"/{self._namespace}{topic}" if self._namespace else topic

            self._traj_pub = self._node.create_publisher(
                JointTrajectory,
                _ns_prefix(f"/{self.CONTROLLER_NAME}/joint_trajectory"),
                traj_qos,
            )

            self._gripper_traj_pub = self._node.create_publisher(
                JointTrajectory,
                _ns_prefix(f"/{self.GRIPPER_CONTROLLER_NAME}/joint_trajectory"),
                traj_qos,
            )

            self._joint_state_sub = self._node.create_subscription(
                JointState,
                _ns_prefix("/joint_states"),
                self._joint_state_callback, 10,
            )

            self._switch_ctrl_client = self._node.create_client(
                SwitchController,
                _ns_prefix("/controller_manager/switch_controller"),
            )

            self._zero_torque_srv_client = self._node.create_client(
                SetBool,
                _ns_prefix("/rs_a3/set_zero_torque_mode"),
            )

            self._follow_traj_client = ActionClient(
                self._node, FollowJointTrajectory,
                _ns_prefix(f"/{self.CONTROLLER_NAME}/follow_joint_trajectory"),
            )

            self._gripper_follow_traj_client = ActionClient(
                self._node, FollowJointTrajectory,
                _ns_prefix(f"/{self.GRIPPER_CONTROLLER_NAME}/follow_joint_trajectory"),
            )

            # MoveIt service / action clients (optional)
            if MOVEIT_AVAILABLE:
                ik_srv = _ns_prefix("/compute_ik")
                cp_srv = _ns_prefix("/compute_cartesian_path")
                mg_action = _ns_prefix("/move_action")
                self._ik_client = self._node.create_client(GetPositionIK, ik_srv)
                self._cartesian_path_client = self._node.create_client(GetCartesianPath, cp_srv)
                self._move_group_client = ActionClient(
                    self._node, MoveGroup, mg_action)
                logger.info(
                    "MoveIt 客户端已创建: IK=%s, CartesianPath=%s, MoveGroup=%s",
                    ik_srv, cp_srv, mg_action,
                )
            else:
                logger.warning(
                    "moveit_msgs 未安装，MoveIt 功能（PlanToJointGoal/EndPoseCtrl/"
                    "CartesianPathCtrl 等）不可用"
                )

            # TF2 listener (optional)
            if TF2_AVAILABLE:
                self._tf_buffer = TFBuffer()
                self._tf_listener = TransformListener(self._tf_buffer, self._node)

            if self._use_internal_executor:
                self._executor = SingleThreadedExecutor()
                self._executor.add_node(self._node)
                self._spin_thread = threading.Thread(
                    target=self._spin_loop, daemon=True, name="ros_spin",
                )
                self._spin_thread.start()
            else:
                logger.info("外部 executor 模式：跳过内部 spin thread")

            self._connected = True
            self._state = ArmState.IDLE
            logger.info("ROS 接口已连接 (node=%s)", self._node_name)
            return True

        except Exception as e:
            logger.error("ROS 接口连接失败: %s", e)
            return False

    @property
    def arm_state(self) -> ArmState:
        """获取当前机械臂状态"""
        return self._state

    def DisconnectPort(self):
        """销毁 ROS2 节点并释放资源"""
        if not self._connected:
            return
        self._connected = False
        self._state = ArmState.DISCONNECTED

        if self._use_internal_executor:
            if self._executor:
                self._executor.shutdown(timeout_sec=1.0)
            if self._spin_thread:
                self._spin_thread.join(timeout=2.0)
                self._spin_thread = None
            self._executor = None

        if self._node:
            self._node.destroy_node()
            self._node = None

        logger.info("ROS 接口已断开")

    def get_node(self) -> Optional[Node]:
        """返回内部 ROS2 节点（用于外部 executor 模式）"""
        return self._node

    def get_connect_status(self) -> bool:
        return self._connected

    def _spin_loop(self):
        while self._connected and rclpy.ok():
            try:
                self._executor.spin_once(timeout_sec=0.01)
            except Exception:
                if self._connected:
                    pass

    def _joint_state_callback(self, msg: JointState):
        with self._js_lock:
            self._latest_joint_state = msg

    # ================================================================
    # 电机使能 / 失能
    # ================================================================

    def EnableArm(self, motor_num: int = 7, **kwargs) -> bool:
        """
        使能机械臂（通过 controller_manager 激活控制器）

        Args:
            motor_num: 兼容 CAN 模式接口，ROS 模式下忽略
        """
        if not self._connected:
            logger.error("未连接，请先调用 ConnectPort()")
            return False
        controllers = [self.CONTROLLER_NAME, self.GRIPPER_CONTROLLER_NAME]
        ok = self._switch_controllers(activate=controllers, deactivate=[])
        if ok:
            self._state = ArmState.ENABLED
        return ok

    def DisableArm(self, motor_num: int = 7) -> bool:
        """失能机械臂（通过 controller_manager 停用控制器）"""
        if not self._connected:
            return False
        controllers = [self.CONTROLLER_NAME, self.GRIPPER_CONTROLLER_NAME]
        ok = self._switch_controllers(activate=[], deactivate=controllers)
        if ok:
            self._state = ArmState.IDLE
        return ok

    # ================================================================
    # 安全控制
    # ================================================================

    def EmergencyStop(self) -> bool:
        """急停：停用所有控制器"""
        if not self._connected:
            return False
        controllers = [self.CONTROLLER_NAME, self.GRIPPER_CONTROLLER_NAME]
        result = self._switch_controllers(activate=[], deactivate=controllers)
        self._state = ArmState.IDLE
        logger.warning("急停已执行（ROS 模式）：控制器已停用")
        return result

    def ResetArm(self) -> bool:
        """复位：停用控制器并重置内部状态"""
        result = self.EmergencyStop()
        self._ctrl_mode = ControlMode.STANDBY
        self._move_mode = MoveMode.MOVE_J
        logger.info("机械臂已复位（ROS 模式）")
        return result

    # ================================================================
    # 模式控制
    # ================================================================

    def ModeCtrl(
        self,
        ctrl_mode: int = 0x01,
        move_mode: int = 0x00,
        move_spd_rate_ctrl: int = 50,
    ):
        """
        设置控制模式（ROS 模式下仅更新内部状态，实际模式由控制器决定）
        """
        self._ctrl_mode = ControlMode(ctrl_mode)
        self._move_mode = MoveMode(move_mode)
        logger.info(
            "模式已设置（ROS 模式）: ctrl=%s, move=%s",
            self._ctrl_mode.name, self._move_mode.name,
        )

    # ================================================================
    # 关节控制
    # ================================================================

    def JointCtrl(
        self,
        joint_1: float, joint_2: float, joint_3: float,
        joint_4: float, joint_5: float, joint_6: float,
        joint_7: Optional[float] = None,
        duration_ns: int = 0,
        velocities: Optional[List[float]] = None,
        **kwargs,
    ) -> bool:
        """
        关节角度控制

        L1-L6 通过 arm_controller 发布，L7 (如果指定) 通过 gripper_controller 发布。

        Args:
            joint_1~6: 目标关节角度 (rad)
            joint_7: 夹爪关节角度 (rad)，None=不发送夹爪指令
            duration_ns: 轨迹点 time_from_start (纳秒)，>0 时启用插值
            velocities: 目标关节速度 (rad/s) 列表，用于 spline 插值速度匹配；
                        None 时如果 duration_ns>0 则填零速度
            **kwargs: 兼容 CAN 模式参数（ROS 模式忽略）
        """
        if not self._connected:
            logger.error("未连接")
            return False

        arm_positions = [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6]

        if self._joint_limit_enabled:
            for i, mid in enumerate(range(1, self.NUM_ARM_JOINTS + 1)):
                limits = self._joint_limits.get(mid)
                if limits:
                    arm_positions[i] = clamp(arm_positions[i], limits[0], limits[1])

        traj = JointTrajectory()
        traj.joint_names = list(self._arm_joint_names)
        point = JointTrajectoryPoint()
        point.positions = arm_positions
        point.time_from_start = Duration(sec=0, nanosec=duration_ns)
        if velocities is not None:
            point.velocities = list(velocities[:len(arm_positions)])
        elif duration_ns > 0:
            point.velocities = [0.0] * len(arm_positions)
        traj.points = [point]
        self._traj_pub.publish(traj)

        if joint_7 is not None and self._gripper_traj_pub:
            self.GripperCtrl(gripper_angle=joint_7)

        if self._state in (ArmState.ENABLED, ArmState.RUNNING):
            self._state = ArmState.RUNNING
        return True

    def JointCtrlList(
        self,
        positions: List[float],
        duration_ns: int = 0,
        velocities: Optional[List[float]] = None,
        **kwargs,
    ) -> bool:
        """便捷接口：列表形式的关节控制（支持 6 或 7 关节）

        Args:
            positions: 目标关节角度列表 (6 或 7 个)
            duration_ns: 轨迹点 time_from_start (纳秒)
            velocities: 目标关节速度列表 (6 个, arm joints only)
        """
        n = len(positions)
        if n < 6:
            logger.error("positions 长度至少为 6 (got %d)", n)
            return False
        joint_7 = positions[6] if n >= 7 else None
        return self.JointCtrl(
            *positions[:6], joint_7=joint_7,
            duration_ns=duration_ns, velocities=velocities, **kwargs,
        )

    # ================================================================
    # 夹爪控制
    # ================================================================

    def GripperCtrl(self, gripper_angle: float = 0.0, gripper_effort: float = 0.0,
                    gripper_enable: bool = True, set_zero: bool = False,
                    **kwargs) -> bool:
        """夹爪控制（通过 gripper_controller 发布 L7_joint 位置指令）

        Args:
            gripper_angle: 目标夹爪角度 (rad)
            gripper_effort: 力矩限制（ROS 模式目前忽略）
            gripper_enable: 是否使能
            set_zero: 是否设置当前位置为零点（ROS 模式忽略）
        """
        if not self._connected:
            logger.error("未连接")
            return False

        if not self._gripper_traj_pub:
            logger.warning("gripper_controller 话题未创建")
            return False

        angle = gripper_angle
        if self._joint_limit_enabled:
            limits = self._joint_limits.get(7)
            if limits:
                angle = clamp(angle, limits[0], limits[1])

        traj = JointTrajectory()
        traj.joint_names = list(self._gripper_joint_names)
        point = JointTrajectoryPoint()
        point.positions = [angle]
        point.time_from_start = Duration(sec=0, nanosec=500_000_000)
        traj.points = [point]

        self._gripper_traj_pub.publish(traj)
        logger.debug("GripperCtrl -> L7_joint = %.4f rad", angle)
        return True

    # ================================================================
    # 零力矩模式
    # ================================================================

    ZERO_TORQUE_CONTROLLER_NAME = "zero_torque_controller"

    def ZeroTorqueMode(self, enable: bool, **kwargs) -> bool:
        """
        零力矩模式 — 优先通过硬件内置 SetBool 服务 (/rs_a3/set_zero_torque_mode)，
        不再停用 arm_controller，避免控制器状态错乱。
        仅当硬件服务不可用时回退到 switch_controller 方式。

        Args:
            enable: True=启用零力矩, False=恢复位置控制
        """
        if not self._connected:
            return False

        ok = False
        if self._zero_torque_srv_client and self._zero_torque_srv_client.wait_for_service(timeout_sec=2.0):
            ok = self._call_zero_torque_service(enable)
        else:
            logger.warning("硬件零力矩服务不可用，回退到 switch_controller 方式")
            if enable:
                ok = self._switch_controllers(
                    activate=[self.ZERO_TORQUE_CONTROLLER_NAME],
                    deactivate=[self.CONTROLLER_NAME, self.GRIPPER_CONTROLLER_NAME],
                )
            else:
                ok = self._switch_controllers(
                    activate=[self.CONTROLLER_NAME, self.GRIPPER_CONTROLLER_NAME],
                    deactivate=[self.ZERO_TORQUE_CONTROLLER_NAME],
                )

        if ok:
            self._state = ArmState.ZERO_TORQUE if enable else ArmState.ENABLED
        return ok

    def _call_zero_torque_service(self, enable: bool) -> bool:
        """调用硬件层 /rs_a3/set_zero_torque_mode (SetBool) 服务"""
        req = SetBool.Request()
        req.data = enable
        future = self._zero_torque_srv_client.call_async(req)

        deadline = time.time() + 5.0
        while not future.done() and time.time() < deadline:
            time.sleep(0.05)

        if future.done():
            resp = future.result()
            if resp and resp.success:
                logger.info("零力矩模式 %s", "已开启" if enable else "已关闭")
                return True
            msg = resp.message if resp else "无响应"
            logger.error("零力矩服务返回失败: %s", msg)
        else:
            logger.error("零力矩服务调用超时")
        return False

    def MasterSlaveConfig(self, mode: int = 0xFC, *args) -> bool:
        """主从模式配置（ROS 模式映射到零力矩控制器切换）"""
        if mode == 0xFD:
            return self.ZeroTorqueMode(True)
        else:
            return self.ZeroTorqueMode(False)

    # ================================================================
    # 反馈函数
    # ================================================================

    def GetArmJointMsgs(self) -> ArmJointStates:
        """获取关节角度反馈（从 /joint_states 订阅）"""
        with self._js_lock:
            js = self._latest_joint_state

        if js is None:
            return ArmJointStates()

        positions = [0.0] * self.NUM_JOINTS
        for i, name in enumerate(self._joint_names):
            if name in js.name:
                idx = js.name.index(name)
                if idx < len(js.position):
                    positions[i] = js.position[idx]

        ts = js.header.stamp.sec + js.header.stamp.nanosec * 1e-9
        return ArmJointStates.from_list(positions, timestamp=ts)

    def GetArmJointVelocities(self) -> ArmJointStates:
        """获取关节速度反馈 (rad/s)"""
        with self._js_lock:
            js = self._latest_joint_state

        if js is None:
            return ArmJointStates()

        velocities = [0.0] * self.NUM_JOINTS
        for i, name in enumerate(self._joint_names):
            if name in js.name:
                idx = js.name.index(name)
                if idx < len(js.velocity):
                    velocities[i] = js.velocity[idx]

        ts = js.header.stamp.sec + js.header.stamp.nanosec * 1e-9
        return ArmJointStates.from_list(velocities, timestamp=ts)

    def GetArmJointEfforts(self) -> ArmJointStates:
        """获取关节力矩反馈 (Nm)"""
        with self._js_lock:
            js = self._latest_joint_state

        if js is None:
            return ArmJointStates()

        efforts = [0.0] * self.NUM_JOINTS
        for i, name in enumerate(self._joint_names):
            if name in js.name:
                idx = js.name.index(name)
                if idx < len(js.effort):
                    efforts[i] = js.effort[idx]

        ts = js.header.stamp.sec + js.header.stamp.nanosec * 1e-9
        return ArmJointStates.from_list(efforts, timestamp=ts)

    def GetArmStatus(self) -> ArmStatus:
        """获取机械臂综合状态（ROS 模式简化版）"""
        status = ArmStatus()
        status.ctrl_mode = int(self._ctrl_mode)
        status.move_mode = int(self._move_mode)

        with self._js_lock:
            js = self._latest_joint_state
        if js is not None:
            status.joint_enabled = [True] * self.NUM_JOINTS
            status.arm_status = 0x00
            status.timestamp = js.header.stamp.sec + js.header.stamp.nanosec * 1e-9
        else:
            status.arm_status = 0x01

        return status

    def GetArmEnableStatus(self) -> List[bool]:
        """获取各电机使能状态（ROS 模式下有 joint_state 即视为使能）"""
        with self._js_lock:
            js = self._latest_joint_state
        if js is not None:
            return [True] * self.NUM_JOINTS
        return [False] * self.NUM_JOINTS

    def GetArmHighSpdInfoMsgs(self) -> List[MotorHighSpdInfo]:
        """获取高速反馈信息（从 joint_states 构建）"""
        with self._js_lock:
            js = self._latest_joint_state

        result = []
        for i, mid in enumerate(range(1, self.NUM_JOINTS + 1)):
            info = MotorHighSpdInfo(motor_id=mid)
            if js is not None:
                name = self._joint_names[i]
                if name in js.name:
                    idx = js.name.index(name)
                    info.position = js.position[idx] if idx < len(js.position) else 0.0
                    info.speed = js.velocity[idx] if idx < len(js.velocity) else 0.0
                    info.torque = js.effort[idx] if idx < len(js.effort) else 0.0
                    info.timestamp = js.header.stamp.sec + js.header.stamp.nanosec * 1e-9
            result.append(info)
        return result

    def GetArmLowSpdInfoMsgs(self) -> List[MotorLowSpdInfo]:
        """获取低速反馈信息（ROS 模式下不可用）"""
        logger.warning("GetArmLowSpdInfoMsgs 在 ROS 模式下不可用（需要直接 CAN 访问）")
        return [MotorLowSpdInfo(motor_id=mid) for mid in range(1, self.NUM_JOINTS + 1)]

    # ================================================================
    # 参数查询（ROS 模式下不可用）
    # ================================================================

    def SearchMotorMaxAngleSpdAccLimit(self, motor_num: int = 1,
                                       search_content: int = 0x01) -> Optional[ParamReadResult]:
        logger.warning("SearchMotorMaxAngleSpdAccLimit 在 ROS 模式下不可用")
        return None

    def GetCurrentMotorAngleLimitMaxVel(self) -> List[MotorAngleLimitMaxVel]:
        """从 SDK 配置读取角度限位"""
        result = []
        for mid in range(1, self.NUM_JOINTS + 1):
            limits = self._joint_limits.get(mid, (-6.28, 6.28))
            result.append(MotorAngleLimitMaxVel(
                motor_num=mid,
                min_angle_limit=limits[0],
                max_angle_limit=limits[1],
            ))
        return result

    def GetAllMotorMaxAccLimit(self) -> List[MotorMaxAccLimit]:
        logger.warning("GetAllMotorMaxAccLimit 在 ROS 模式下不可用")
        return [MotorMaxAccLimit(motor_num=mid) for mid in range(1, self.NUM_JOINTS + 1)]

    def ReadMotorParameter(self, motor_id: int, param_index: int) -> Optional[ParamReadResult]:
        logger.warning("ReadMotorParameter 在 ROS 模式下不可用")
        return None

    def WriteMotorParameter(self, motor_id: int, param_index: int, value: float) -> bool:
        logger.warning("WriteMotorParameter 在 ROS 模式下不可用")
        return False

    def GetMotorVoltage(self, motor_id: int) -> Optional[float]:
        logger.warning("GetMotorVoltage 在 ROS 模式下不可用")
        return None

    def GetFirmwareVersion(self, motor_id: int = 1) -> Optional[FirmwareVersion]:
        logger.warning("GetFirmwareVersion 在 ROS 模式下不可用")
        return None

    def GetAllFirmwareVersions(self) -> Dict[int, FirmwareVersion]:
        logger.warning("GetAllFirmwareVersions 在 ROS 模式下不可用")
        return {}

    # ================================================================
    # 零位设置
    # ================================================================

    def SetZeroPosition(self, motor_num: int = 7) -> bool:
        logger.warning("SetZeroPosition 在 ROS 模式下不可用（需要直接 CAN 访问）")
        return False

    # ================================================================
    # SDK 信息
    # ================================================================

    def GetCanFps(self) -> float:
        logger.warning("GetCanFps 在 ROS 模式下不可用")
        return 0.0

    def GetCanName(self) -> str:
        return f"ros://{self._namespace or 'default'}"

    def GetCurrentSDKVersion(self) -> str:
        from el_a3_sdk import __version__
        return __version__

    def GetCurrentProtocolVersion(self) -> str:
        return "ROS2 Control (JointTrajectoryController)"

    # ================================================================
    # 控制参数设置
    # ================================================================

    def SetPositionPD(self, kp: float, kd: float):
        """ROS 模式下 PD 增益由 hardware_interface 管理"""
        logger.warning("SetPositionPD 在 ROS 模式下不生效（PD 增益由 hardware_interface 配置）")

    def SetJointLimitEnabled(self, enabled: bool):
        self._joint_limit_enabled = enabled
        logger.info("关节限位检查: %s", "启用" if enabled else "禁用")

    # ================================================================
    # 笛卡尔控制 (MoveIt IK)
    # ================================================================

    def EndPoseCtrl(
        self, x: float, y: float, z: float,
        rx: float, ry: float, rz: float,
        duration: float = 2.0,
        **kwargs,
    ) -> bool:
        """
        笛卡尔位姿控制（MoveIt IK -> FollowJointTrajectory）

        如果 MoveIt 不可用，回退到 Pinocchio IK + JointTrajectory 发布。
        """
        if not self._connected:
            logger.error("未连接")
            return False

        target_pose = ArmEndPose(x=x, y=y, z=z, rx=rx, ry=ry, rz=rz)

        if MOVEIT_AVAILABLE and self._ik_client:
            q = self._moveit_ik(target_pose)
        else:
            kin = self._get_kinematics()
            if kin is None:
                logger.error("MoveIt 和 Pinocchio 均不可用，无法执行 EndPoseCtrl")
                return False
            current_q = self.GetArmJointMsgs().to_list()
            q = kin.inverse_kinematics(target_pose, q_init=current_q)

        if q is None:
            logger.error("IK 求解失败")
            return False

        return self.MoveJ(q, duration=duration)

    def CartesianPathCtrl(
        self, waypoints: List[ArmEndPose], eef_step: float = 0.01,
        duration: float = 5.0,
    ) -> bool:
        """
        笛卡尔路径控制（MoveIt /compute_cartesian_path -> FollowJointTrajectory）
        """
        if not self._connected:
            return False

        if not MOVEIT_AVAILABLE or not self._cartesian_path_client:
            logger.error("MoveIt 不可用，CartesianPathCtrl 需要 MoveIt")
            return False

        if not self._cartesian_path_client.wait_for_service(timeout_sec=5.0):
            logger.error("/compute_cartesian_path 服务不可用")
            return False

        try:
            import pinocchio as pin
        except ImportError:
            pin = None

        poses = []
        for wp in waypoints:
            pose = Pose()
            pose.position = Point(x=wp.x, y=wp.y, z=wp.z)
            if pin is not None:
                R = pin.rpy.rpyToMatrix(wp.rx, wp.ry, wp.rz)
                quat = pin.Quaternion(R)
                pose.orientation = Quaternion(
                    x=float(quat.x()), y=float(quat.y()),
                    z=float(quat.z()), w=float(quat.w()),
                )
            else:
                from math import cos, sin
                cr, cp, cy = cos(wp.rx/2), cos(wp.ry/2), cos(wp.rz/2)
                sr, sp, sy = sin(wp.rx/2), sin(wp.ry/2), sin(wp.rz/2)
                pose.orientation = Quaternion(
                    x=sr*cp*cy - cr*sp*sy,
                    y=cr*sp*cy + sr*cp*sy,
                    z=cr*cp*sy - sr*sp*cy,
                    w=cr*cp*cy + sr*sp*sy,
                )
            poses.append(pose)

        req = GetCartesianPath.Request()
        req.header.frame_id = self._base_link
        req.group_name = self._move_group_name
        req.link_name = self._ee_link
        req.waypoints = poses
        req.max_step = eef_step
        req.avoid_collisions = True

        with self._js_lock:
            js = self._latest_joint_state
        if js:
            req.start_state.joint_state = js

        future = self._cartesian_path_client.call_async(req)
        result = self._wait_for_future(future, timeout=10.0)
        if result is None or result.fraction < 0.9:
            logger.error("笛卡尔路径规划失败 (fraction=%.2f)",
                         result.fraction if result else 0.0)
            return False

        return self._execute_trajectory(result.solution, duration)

    # ================================================================
    # MoveIt 异步接口 (供实时控制使用)
    # ================================================================

    def ComputeIKAsync(self, target_pose, seed_positions=None,
                       avoid_collisions=True, timeout_ns=10_000_000):
        """异步 IK 求解，返回 rclpy.Future

        Args:
            target_pose: geometry_msgs/Pose 目标位姿
            seed_positions: 种子关节角度列表 (6-DOF)，None 时使用当前关节状态
            avoid_collisions: 是否启用碰撞检测
            timeout_ns: IK 超时（纳秒）

        Returns:
            rclpy.Future — 调用方可通过 future.add_done_callback() 获取结果
        """
        if not MOVEIT_AVAILABLE or not self._ik_client:
            raise RuntimeError("MoveIt IK 服务不可用")

        if not self._ik_client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError("/compute_ik 服务未就绪（等待 5s 超时），请确认 move_group 节点已启动")

        req = GetPositionIK.Request()
        req.ik_request.group_name = self._move_group_name
        req.ik_request.robot_state.is_diff = False

        if seed_positions is not None:
            seed = JointState()
            seed.name = list(self._arm_joint_names)
            seed.position = list(seed_positions)
            req.ik_request.robot_state.joint_state = seed
        else:
            with self._js_lock:
                js = self._latest_joint_state
            if js:
                req.ik_request.robot_state.joint_state = js

        req.ik_request.pose_stamped.header.frame_id = self._base_link
        req.ik_request.pose_stamped.pose = target_pose
        req.ik_request.timeout.sec = 0
        req.ik_request.timeout.nanosec = timeout_ns
        req.ik_request.avoid_collisions = avoid_collisions

        return self._ik_client.call_async(req)

    def PlanToJointGoal(self, joint_positions, velocity_scale=0.3,
                        accel_scale=0.3, planning_time=5.0,
                        num_attempts=5, replan=True, replan_attempts=3) -> bool:
        """同步 MoveGroup 规划 + 执行到目标关节角度（阻塞直到完成）

        Args:
            joint_positions: 目标关节角度列表（6-DOF）
            velocity_scale: 最大速度缩放 (0~1)
            accel_scale: 最大加速度缩放 (0~1)
            planning_time: 规划时间限制（秒）
            num_attempts: 规划尝试次数
            replan: 是否允许重规划
            replan_attempts: 重规划次数

        Returns:
            bool — 执行成功返回 True
        """
        if not MOVEIT_AVAILABLE or not self._move_group_client:
            logger.error("MoveGroup action 不可用")
            return False

        if not self._move_group_client.wait_for_server(timeout_sec=10.0):
            logger.error("MoveGroup action server 不可用")
            return False

        goal = self._build_move_group_goal(
            joint_positions, velocity_scale, accel_scale,
            planning_time, num_attempts, replan, replan_attempts)

        future = self._move_group_client.send_goal_async(goal)
        goal_handle = self._wait_for_future(future, timeout=10.0)
        if goal_handle is None or not goal_handle.accepted:
            logger.error("MoveGroup goal 被拒绝")
            return False

        result_future = goal_handle.get_result_async()
        result = self._wait_for_future(result_future, timeout=planning_time + 60.0)
        if result is None:
            logger.error("MoveGroup 执行超时")
            return False

        return result.result.error_code.val == 1

    def PlanToJointGoalAsync(self, joint_positions, velocity_scale=0.3,
                             accel_scale=0.3, planning_time=5.0,
                             num_attempts=5, replan=True, replan_attempts=3,
                             result_callback=None):
        """异步 MoveGroup 规划 + 执行，完成后调用 result_callback(success: bool)"""
        if not MOVEIT_AVAILABLE or not self._move_group_client:
            logger.error("MoveGroup action 不可用")
            if result_callback:
                result_callback(False)
            return

        if not self._move_group_client.wait_for_server(timeout_sec=10.0):
            logger.error("MoveGroup action server 未就绪（等待 10s 超时）")
            if result_callback:
                result_callback(False)
            return

        goal = self._build_move_group_goal(
            joint_positions, velocity_scale, accel_scale,
            planning_time, num_attempts, replan, replan_attempts)

        future = self._move_group_client.send_goal_async(goal)

        def _on_response(f):
            try:
                gh = f.result()
                if gh is None or not gh.accepted:
                    logger.warning("MoveGroup goal 被拒绝")
                    if result_callback:
                        result_callback(False)
                    return
                rf = gh.get_result_async()
                rf.add_done_callback(_on_result)
            except Exception as e:
                logger.error("MoveGroup response 异常: %s", e)
                if result_callback:
                    result_callback(False)

        def _on_result(f):
            try:
                res = f.result().result
                success = (res.error_code.val == 1)
            except Exception:
                success = False
            if result_callback:
                result_callback(success)

        future.add_done_callback(_on_response)

    def PlanCartesianPathAsync(self, waypoints, max_step=0.01,
                               avoid_collisions=True, result_callback=None):
        """异步笛卡尔路径规划 + 执行

        Args:
            waypoints: geometry_msgs/Pose 列表
            max_step: 笛卡尔插值步长 (m)
            avoid_collisions: 是否启用碰撞检测
            result_callback: 完成回调 callback(success: bool)
        """
        if not MOVEIT_AVAILABLE or not self._cartesian_path_client:
            logger.error("MoveIt CartesianPath 服务不可用")
            if result_callback:
                result_callback(False)
            return

        req = GetCartesianPath.Request()
        req.header.frame_id = self._base_link
        req.group_name = self._move_group_name
        req.link_name = self._ee_link
        req.waypoints = list(waypoints)
        req.max_step = max_step
        req.avoid_collisions = avoid_collisions

        with self._js_lock:
            js = self._latest_joint_state
        if js:
            req.start_state.joint_state = js

        future = self._cartesian_path_client.call_async(req)

        def _on_path(f):
            try:
                result = f.result()
                if result.fraction < 0.9:
                    logger.warning("笛卡尔路径覆盖率不足: %.2f", result.fraction)
                    if result_callback:
                        result_callback(False)
                    return
                traj = result.solution.joint_trajectory
                self._execute_traj_async(traj, result_callback)
            except Exception as e:
                logger.error("笛卡尔路径回调异常: %s", e)
                if result_callback:
                    result_callback(False)

        future.add_done_callback(_on_path)

    def _build_move_group_goal(self, joint_positions, velocity_scale, accel_scale,
                               planning_time, num_attempts, replan, replan_attempts):
        """构建 MoveGroup.Goal（JointConstraint 方式）"""
        goal = MoveGroup.Goal()
        goal.request.group_name = self._move_group_name
        goal.request.num_planning_attempts = num_attempts
        goal.request.allowed_planning_time = planning_time
        goal.request.max_velocity_scaling_factor = velocity_scale
        goal.request.max_acceleration_scaling_factor = accel_scale

        constraints = Constraints()
        for name, value in zip(self._arm_joint_names, joint_positions):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = float(value)
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)

        goal.request.goal_constraints = [constraints]
        goal.planning_options.plan_only = False
        goal.planning_options.replan = replan
        goal.planning_options.replan_attempts = replan_attempts

        return goal

    def _execute_traj_async(self, traj, result_callback=None):
        """异步执行 JointTrajectory (FollowJointTrajectory action)"""
        if not self._follow_traj_client:
            self._traj_pub.publish(traj)
            if result_callback:
                result_callback(True)
            return

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj
        future = self._follow_traj_client.send_goal_async(goal)

        def _on_response(f):
            try:
                gh = f.result()
                if gh is None or not gh.accepted:
                    logger.warning("FollowJointTrajectory goal 被拒绝")
                    if result_callback:
                        result_callback(False)
                    return
                rf = gh.get_result_async()
                rf.add_done_callback(_on_result)
            except Exception as e:
                logger.error("FollowJointTrajectory response 异常: %s", e)
                if result_callback:
                    result_callback(False)

        def _on_result(f):
            try:
                success = (f.result().result.error_code == 0)
            except Exception:
                success = False
            if result_callback:
                result_callback(success)

        future.add_done_callback(_on_response)

    def GetArmEndPoseMsgs(self) -> ArmEndPose:
        """获取末端位姿（优先 TF2，回退 Pinocchio FK）"""
        if TF2_AVAILABLE and self._tf_buffer:
            try:
                t = self._tf_buffer.lookup_transform(
                    self._base_link, self._ee_link, rclpy.time.Time(),
                )
                pos = t.transform.translation
                rot = t.transform.rotation

                import math
                x, y, z, w = rot.x, rot.y, rot.z, rot.w
                sinr_cosp = 2 * (w * x + y * z)
                cosr_cosp = 1 - 2 * (x * x + y * y)
                rx = math.atan2(sinr_cosp, cosr_cosp)
                sinp = 2 * (w * y - z * x)
                ry = math.asin(max(-1, min(1, sinp)))
                siny_cosp = 2 * (w * z + x * y)
                cosy_cosp = 1 - 2 * (y * y + z * z)
                rz = math.atan2(siny_cosp, cosy_cosp)

                return ArmEndPose(x=pos.x, y=pos.y, z=pos.z, rx=rx, ry=ry, rz=rz)
            except Exception:
                pass

        kin = self._get_kinematics()
        if kin:
            q = self.GetArmJointMsgs().to_list()
            return kin.forward_kinematics(q)

        return ArmEndPose()

    # ================================================================
    # 轨迹运动 (FollowJointTrajectory Action)
    # ================================================================

    def MoveJ(self, positions: List[float], duration: float = 2.0, **kwargs) -> bool:
        """
        关节空间运动（FollowJointTrajectory action）
        """
        if not self._connected:
            logger.error("未连接")
            return False

        if len(positions) != self.NUM_JOINTS:
            logger.error("positions 长度必须为 %d", self.NUM_JOINTS)
            return False

        if self._joint_limit_enabled:
            for i, mid in enumerate(range(1, self.NUM_JOINTS + 1)):
                limits = self._joint_limits.get(mid)
                if limits:
                    positions[i] = clamp(positions[i], limits[0], limits[1])

        traj = JointTrajectory()
        traj.joint_names = list(self._joint_names)

        point = JointTrajectoryPoint()
        point.positions = list(positions)
        point.velocities = [0.0] * self.NUM_JOINTS
        sec = int(duration)
        nsec = int((duration - sec) * 1e9)
        point.time_from_start = Duration(sec=sec, nanosec=nsec)
        traj.points = [point]

        return self._send_follow_trajectory(traj, timeout=duration + 5.0)

    def MoveL(self, target_pose: ArmEndPose, duration: float = 2.0, **kwargs) -> bool:
        """
        直线运动（CartesianPath -> FollowJointTrajectory）

        如果 MoveIt 不可用，回退到 Pinocchio 笛卡尔插值。
        """
        if not self._connected:
            return False

        if MOVEIT_AVAILABLE and self._cartesian_path_client:
            return self.CartesianPathCtrl([target_pose], duration=duration)

        kin = self._get_kinematics()
        if kin is None:
            logger.error("MoveIt 和 Pinocchio 均不可用，无法执行 MoveL")
            return False

        current_q = self.GetArmJointMsgs().to_list()
        start_pose = kin.forward_kinematics(current_q)

        n_wp = 50
        traj = JointTrajectory()
        traj.joint_names = list(self._joint_names)
        q_prev = current_q

        for i in range(1, n_wp + 1):
            s = i / n_wp
            wp = ArmEndPose(
                x=start_pose.x + s * (target_pose.x - start_pose.x),
                y=start_pose.y + s * (target_pose.y - start_pose.y),
                z=start_pose.z + s * (target_pose.z - start_pose.z),
                rx=start_pose.rx + s * (target_pose.rx - start_pose.rx),
                ry=start_pose.ry + s * (target_pose.ry - start_pose.ry),
                rz=start_pose.rz + s * (target_pose.rz - start_pose.rz),
            )
            q_sol = kin.inverse_kinematics(wp, q_init=q_prev)
            if q_sol is None:
                logger.error("MoveL IK 失败 at waypoint %d/%d", i, n_wp)
                return False

            pt = JointTrajectoryPoint()
            pt.positions = list(q_sol)
            t = duration * s
            sec = int(t)
            nsec = int((t - sec) * 1e9)
            pt.time_from_start = Duration(sec=sec, nanosec=nsec)
            traj.points.append(pt)
            q_prev = q_sol

        return self._send_follow_trajectory(traj, timeout=duration + 5.0)

    # ================================================================
    # 动力学接口 (Pinocchio)
    # ================================================================

    def ComputeGravityTorques(self, positions: Optional[List[float]] = None) -> List[float]:
        """计算重力补偿力矩 (Pinocchio RNEA)"""
        kin = self._get_kinematics()
        if kin is None:
            return [0.0] * self.NUM_JOINTS
        q = positions or self.GetArmJointMsgs().to_list()
        return kin.compute_gravity(q)

    def GetJacobian(self, positions: Optional[List[float]] = None) -> np.ndarray:
        """获取末端 Jacobian 矩阵 (6xN)"""
        kin = self._get_kinematics()
        if kin is None:
            return np.zeros((6, self.NUM_JOINTS))
        q = positions or self.GetArmJointMsgs().to_list()
        return kin.compute_jacobian(q)

    def GetMassMatrix(self, positions: Optional[List[float]] = None) -> np.ndarray:
        """获取质量矩阵 M(q)"""
        kin = self._get_kinematics()
        if kin is None:
            return np.eye(self.NUM_JOINTS)
        q = positions or self.GetArmJointMsgs().to_list()
        return kin.mass_matrix(q)

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
    # 增强零力矩模式
    # ================================================================

    def ZeroTorqueModeWithGravity(self, enable: bool, **kwargs) -> bool:
        """
        带重力补偿的零力矩模式。

        ROS 模式下通过 switch_controller 切换到 zero_torque_controller，
        该控制器内置 Pinocchio RNEA 重力补偿。
        """
        return self.ZeroTorqueMode(enable, **kwargs)

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
            )
            return self._kin
        except Exception as e:
            logger.debug("Pinocchio 初始化失败: %s", e)
            return None

    def _moveit_ik(self, target_pose: ArmEndPose) -> Optional[List[float]]:
        """通过 MoveIt /compute_ik 服务求解 IK"""
        if not self._ik_client or not self._ik_client.wait_for_service(timeout_sec=3.0):
            return None

        try:
            import pinocchio as pin
            R = pin.rpy.rpyToMatrix(target_pose.rx, target_pose.ry, target_pose.rz)
            quat = pin.Quaternion(R)
            qx, qy, qz, qw = float(quat.x()), float(quat.y()), float(quat.z()), float(quat.w())
        except ImportError:
            from math import cos, sin
            cr, cp, cy = cos(target_pose.rx/2), cos(target_pose.ry/2), cos(target_pose.rz/2)
            sr, sp, sy = sin(target_pose.rx/2), sin(target_pose.ry/2), sin(target_pose.rz/2)
            qx = sr*cp*cy - cr*sp*sy
            qy = cr*sp*cy + sr*cp*sy
            qz = cr*cp*sy - sr*sp*cy
            qw = cr*cp*cy + sr*sp*sy

        req = GetPositionIK.Request()
        ik_req = PositionIKRequest()
        ik_req.group_name = self._move_group_name
        ik_req.ik_link_name = self._ee_link
        ik_req.pose_stamped.header.frame_id = self._base_link
        ik_req.pose_stamped.pose.position = Point(
            x=target_pose.x, y=target_pose.y, z=target_pose.z,
        )
        ik_req.pose_stamped.pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)

        with self._js_lock:
            js = self._latest_joint_state
        if js:
            ik_req.robot_state.joint_state = js

        req.ik_request = ik_req

        future = self._ik_client.call_async(req)
        result = self._wait_for_future(future, timeout=5.0)
        if result is None or result.error_code.val != 1:
            return None

        q = [0.0] * self.NUM_JOINTS
        sol_js = result.solution.joint_state
        for i, name in enumerate(self._joint_names):
            if name in sol_js.name:
                idx = sol_js.name.index(name)
                q[i] = sol_js.position[idx]
        return q

    def _send_follow_trajectory(
        self, traj: JointTrajectory, timeout: float = 10.0,
    ) -> bool:
        """通过 FollowJointTrajectory action 执行轨迹"""
        if not self._follow_traj_client:
            self._traj_pub.publish(traj)
            return True

        if not self._follow_traj_client.wait_for_server(timeout_sec=5.0):
            logger.warning("FollowJointTrajectory action 不可用，使用话题发布")
            self._traj_pub.publish(traj)
            return True

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj

        future = self._follow_traj_client.send_goal_async(goal)
        goal_handle = self._wait_for_future(future, timeout=5.0)
        if goal_handle is None or not goal_handle.accepted:
            logger.error("FollowJointTrajectory goal 被拒绝")
            return False

        result_future = goal_handle.get_result_async()
        result = self._wait_for_future(result_future, timeout=timeout)
        if result is None:
            logger.error("FollowJointTrajectory 执行超时")
            return False

        return result.result.error_code == 0

    def _execute_trajectory(
        self, robot_trajectory, duration: float = 5.0,
    ) -> bool:
        """从 MoveIt RobotTrajectory 提取 JointTrajectory 并执行"""
        traj = robot_trajectory.joint_trajectory
        return self._send_follow_trajectory(traj, timeout=duration + 5.0)

    def _wait_for_future(self, future, timeout: float = 5.0):
        """阻塞等待 future 完成"""
        deadline = time.time() + timeout
        while not future.done() and time.time() < deadline:
            time.sleep(0.05)
        if future.done():
            return future.result()
        if self._use_internal_executor and (
            self._spin_thread is None or not self._spin_thread.is_alive()
        ):
            logger.error(
                "Future 等待超时且内部 spin thread 已停止，"
                "ROS 回调无法被处理。请检查连接状态。"
            )
        else:
            logger.warning("Future 等待超时 (%.1fs)，服务端可能尚未就绪或处理缓慢", timeout)
        return None

    def _switch_controllers(self, activate: List[str], deactivate: List[str]) -> bool:
        """调用 controller_manager/switch_controller 服务"""
        if not self._switch_ctrl_client:
            return False

        if not self._switch_ctrl_client.wait_for_service(timeout_sec=5.0):
            logger.error("controller_manager/switch_controller 服务不可用")
            return False

        req = SwitchController.Request()
        req.activate_controllers = activate
        req.deactivate_controllers = deactivate
        req.strictness = SwitchController.Request.BEST_EFFORT
        req.activate_asap = True

        future = self._switch_ctrl_client.call_async(req)

        deadline = time.time() + 10.0
        while not future.done() and time.time() < deadline:
            time.sleep(0.05)

        if future.done():
            resp = future.result()
            if resp and resp.ok:
                return True
            logger.error("switch_controller 失败")
        else:
            logger.error("switch_controller 服务调用超时")
        return False

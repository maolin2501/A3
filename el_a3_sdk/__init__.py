from el_a3_sdk.interface import ELA3Interface
from el_a3_sdk.arm_manager import ArmManager
from el_a3_sdk.data_types import (
    MotorFeedback,
    ArmStatus,
    ArmJointStates,
    ArmEndPose,
    MotorHighSpdInfo,
    MotorLowSpdInfo,
    MotorAngleLimitMaxVel,
    DynamicsInfo,
    TrajectoryResult,
)
from el_a3_sdk.protocol import (
    MotorType,
    RunMode,
    ControlMode,
    MoveMode,
    ArmState,
    LogLevel,
)

__version__ = "0.4.0"


def get_ros_interface():
    """延迟导入 ELA3ROSInterface（避免无 rclpy 环境下 import 失败）"""
    from el_a3_sdk.ros_interface import ELA3ROSInterface
    return ELA3ROSInterface


def get_kinematics():
    """延迟导入 ELA3Kinematics（避免无 pinocchio 环境下 import 失败）"""
    from el_a3_sdk.kinematics import ELA3Kinematics
    return ELA3Kinematics


__all__ = [
    "ELA3Interface",
    "ArmManager",
    "get_ros_interface",
    "get_kinematics",
    "MotorFeedback",
    "ArmStatus",
    "ArmJointStates",
    "ArmEndPose",
    "MotorHighSpdInfo",
    "MotorLowSpdInfo",
    "MotorAngleLimitMaxVel",
    "DynamicsInfo",
    "TrajectoryResult",
    "MotorType",
    "RunMode",
    "ControlMode",
    "MoveMode",
    "ArmState",
    "LogLevel",
]

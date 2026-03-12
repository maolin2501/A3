#!/usr/bin/env python3
"""
Xbox 手柄实时控制节点
使用笛卡尔空间控制机械臂末端执行器

控制映射：
- 左摇杆：X/Y 平移
- LT/RT：Z 方向平移
- 右摇杆：Yaw/Roll 旋转
- LB/RB：Pitch 旋转
- A 键：切换速度档位（5 档）
- B 键：回到 home 位置
- Y 键：切换重力补偿模式（手动示教/拖动）
- Menu 键：切换主从遥操作（can0 主臂可拖动 / can1 从臂实时跟随）
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from sensor_msgs.msg import Joy, JointState
from geometry_msgs.msg import PoseStamped, Pose
import tf2_ros
import tf2_geometry_msgs
import math
import numpy as np
import copy
import threading
import time

from scipy.spatial.transform import Rotation as R

from el_a3_sdk.arm_manager import ArmManager
from el_a3_sdk.data_types import ArmEndPose

def quaternion_from_euler(roll, pitch, yaw):
    """将欧拉角转换为四元数 (x, y, z, w)"""
    rot = R.from_euler('xyz', [roll, pitch, yaw])
    quat = rot.as_quat()  # Returns (x, y, z, w)
    return quat

def euler_from_quaternion(quaternion):
    """将四元数 (x, y, z, w) 转换为欧拉角"""
    rot = R.from_quat(quaternion)
    euler = rot.as_euler('xyz')
    return euler[0], euler[1], euler[2]  # roll, pitch, yaw


class XboxTeleopNode(Node):
    def __init__(self):
        super().__init__('xbox_teleop_node')
        
        # Declare parameters
        self.declare_parameter('update_rate', 100.0)  # Hz - Jacobian IK supports 100Hz+
        self.declare_parameter('planning_group', 'arm')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('end_effector_frame', 'end_effector')
        self.declare_parameter('deadzone', 0.15)  # Joystick deadzone
        self.declare_parameter('debug_input', False)  # Debug mode
        self.declare_parameter('use_fast_ik_mode', True)  # Use fast IK mode for 100Hz control
        
        # Cartesian velocity parameters (actual physical units)
        self.declare_parameter('max_linear_velocity', 0.15)   # m/s max linear velocity
        self.declare_parameter('max_angular_velocity', 1.5)   # rad/s max angular velocity
        
        # Joint output smoothing parameters
        self.declare_parameter('joint_smoothing_alpha', 0.35)  # Joint smoothing coefficient
        self.declare_parameter('max_joint_velocity', 1.5)      # rad/s max single joint velocity
        self.declare_parameter('max_joint_acceleration', 5.0)  # rad/s² max single joint acceleration
        
        # Input smoothing parameters
        self.declare_parameter('input_smoothing_factor', 0.15)  # Input smoothing coefficient
        
        # Singularity protection parameters
        self.declare_parameter('max_ik_jump_threshold', 0.5)   # rad max allowed single joint jump
        self.declare_parameter('singularity_warning_count', 5) # Consecutive rejection warning threshold
        
        # Collision detection parameters
        self.declare_parameter('enable_collision_check', True)  # Enable collision detection after initialization
        
        # Master-slave teleoperation parameters (Menu button triggered)
        self.declare_parameter('master_slave_enabled', True)
        self.declare_parameter('master_namespace', '')
        self.declare_parameter('slave_namespace', 'arm2')
        self.declare_parameter('master_controller', 'arm_controller')
        self.declare_parameter('slave_controller', 'arm2_arm_controller')
        self.declare_parameter('master_joint_state_topic', '/arm1/joint_states')
        self.declare_parameter('master_trajectory_topic', '/arm1/arm1_arm_controller/joint_trajectory')
        self.declare_parameter('zero_torque_controller', 'zero_torque_controller')
        self.declare_parameter('gripper_controller', 'gripper_controller')
        self.declare_parameter('slave_joint_state_topic', '/arm2/joint_states')
        self.declare_parameter('slave_trajectory_topic', '/arm2/arm2_arm_controller/joint_trajectory')
        self.declare_parameter('go_zero_duration', 3.0)
        self.declare_parameter('master_slave_smoothing_alpha', 1.0)  # 1.0 = no smoothing
        
        # Get parameters
        self.update_rate = self.get_parameter('update_rate').value
        self.planning_group = self.get_parameter('planning_group').value
        self.base_frame = self.get_parameter('base_frame').value
        self.end_effector_frame = self.get_parameter('end_effector_frame').value
        self.deadzone = self.get_parameter('deadzone').value
        self.debug_input = self.get_parameter('debug_input').value
        self.use_fast_ik_mode = self.get_parameter('use_fast_ik_mode').value
        
        # Cartesian velocity parameters
        self.max_linear_velocity = self.get_parameter('max_linear_velocity').value
        self.max_angular_velocity = self.get_parameter('max_angular_velocity').value
        
        # Joint smoothing parameters
        self.joint_smoothing_alpha = self.get_parameter('joint_smoothing_alpha').value
        self.max_joint_velocity = self.get_parameter('max_joint_velocity').value
        self.max_joint_acceleration = self.get_parameter('max_joint_acceleration').value
        self.last_joint_velocities = None  # Previous frame joint velocities (for acceleration limiting)
        
        # Input smoothing parameters
        self.input_smoothing_factor = self.get_parameter('input_smoothing_factor').value
        
        # Singularity protection parameters
        self.max_ik_jump_threshold = self.get_parameter('max_ik_jump_threshold').value
        self.singularity_warning_count = self.get_parameter('singularity_warning_count').value
        self.consecutive_ik_rejects = 0  # Consecutive rejection counter
        self.ik_seed_just_initialized = False  # IK seed just initialized flag, skip first frame jump detection
        
        # Collision detection parameters
        self.enable_collision_check = self.get_parameter('enable_collision_check').value
        self.collision_check_active = False  # Disabled during initialization, enabled after completion
        
        # Master-slave teleoperation parameters
        self.master_slave_enabled = self.get_parameter('master_slave_enabled').value
        self.master_namespace = self.get_parameter('master_namespace').value
        self.slave_namespace = self.get_parameter('slave_namespace').value
        self.master_controller = self.get_parameter('master_controller').value
        self.slave_controller = self.get_parameter('slave_controller').value
        master_joint_state_topic = self.get_parameter('master_joint_state_topic').value
        self.zero_torque_ctrl_name = self.get_parameter('zero_torque_controller').value
        self.gripper_ctrl_name = self.get_parameter('gripper_controller').value
        slave_joint_state_topic = self.get_parameter('slave_joint_state_topic').value
        self.go_zero_duration = self.get_parameter('go_zero_duration').value
        self.ms_smoothing_alpha = self.get_parameter('master_slave_smoothing_alpha').value
        
        # Nominal time step (used as fallback; overridden by measured dt each callback)
        self.dt = 1.0 / self.update_rate
        self._last_update_time = None
        
        # Debug counter - avoid log spam
        self.debug_counter = 0
        
        # Speed level settings (5 levels) - using speed scale factor
        # Actual speed = max_velocity * factor
        self.speed_levels = [
            {'name': '极慢', 'factor': 0.1},   # 10% 最大速度
            {'name': '慢', 'factor': 0.25},    # 25% 最大速度
            {'name': '中', 'factor': 0.5},     # 50% 最大速度
            {'name': '快', 'factor': 0.75},    # 75% 最大速度
            {'name': '最大', 'factor': 1.0},   # 100% 最大速度
        ]
        self.current_speed_level = 2  # Default medium speed (0-indexed)
        self.speed_factor = self.speed_levels[self.current_speed_level]['factor']
        
        # Button state tracking (for debouncing)
        self.last_a_button = 0
        self.last_b_button = 0
        self.last_x_button = 0
        self.last_y_button = 0
        self.is_going_home = False  # Whether currently returning to home/zero
        self.home_start_time = None  # Home return start time
        self.home_timeout = 5.0  # Safety timeout (timer fires earlier)
        self._move_done_timer = None
        self._pending_move_target = None
        
        # Zero-torque mode (via SDK ZeroTorqueMode)
        self.zero_torque_mode = False
        
        # Master-slave teleoperation mode state
        self.master_slave_mode = False
        self.entering_master_slave = False  # Switching in progress (go-to-zero process)
        self.last_menu_button = 0
        
        # Slave arm joint state subscription (trajectory publishing now via _slave_sdk)
        self.slave_joint_state = None
        self.slave_joint_sub = self.create_subscription(
            JointState, slave_joint_state_topic,
            self.slave_joint_state_callback, 10)
        
        # Master arm namespace interface (for master arm communication in multi_arm mode)
        self.master_joint_state_ms = None  # Master-slave mode dedicated
        self.master_joint_sub_ms = self.create_subscription(
            JointState, master_joint_state_topic,
            self.master_joint_state_ms_callback, 10)
        # Master-slave mode joint names (L1-L7 including gripper)
        self.slave_joint_names = [f"L{i}_joint" for i in range(1, 8)]
        self.master_joint_names_ms = [f"L{i}_joint" for i in range(1, 8)]
        
        # Master-slave safety / smoothing state
        self._last_slave_cmd = None  # last command sent to slave (for velocity clamping)
        self._smoothed_ms_positions = None  # EMA-smoothed master positions
        self._ms_follow_count = 0

        # D-pad controls gripper via L7_joint trajectory
        self.last_dpad_up = 0
        self.last_dpad_down = 0
        self.gripper_target_angle = 0.0
        self.gripper_angle_step = 0.2  # rad per press
        
        # ArmManager for multi-arm management (SDK-based, no direct CAN)
        self._arm_mgr = ArmManager.get_instance()
        self._master_sdk = None
        self._slave_sdk = None
        
        # Input smoothing filter state (exponential moving average)
        self.smoothed_vx = 0.0
        self.smoothed_vy = 0.0
        self.smoothed_vz = 0.0
        self.smoothed_vroll = 0.0
        self.smoothed_vpitch = 0.0
        self.smoothed_vyaw = 0.0
        
        # Joint output smoothing state
        self.smoothed_joint_positions = None  # Smoothed joint positions
        
        # Accumulated deltas (for accumulating input during motion)
        self.pending_dx = 0.0
        self.pending_dy = 0.0
        self.pending_dz = 0.0
        self.pending_droll = 0.0
        self.pending_dpitch = 0.0
        self.pending_dyaw = 0.0
        
        # Subscribe to controller input
        self.joy_sub = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10
        )
        
        # TF2 listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Target pose publisher
        self.target_pose_pub = self.create_publisher(
            PoseStamped,
            '/target_pose',
            10
        )
        
        # IK and Cartesian path now handled by SDK (self._master_sdk)
        
        # Subscribe to current joint states.
        # Use TRANSIENT_LOCAL durability so we ONLY match the joint_state_broadcaster
        # (TRANSIENT_LOCAL) and ignore the external ROS2UnityJointNode (VOLATILE)
        # which publishes all-zero positions and causes oscillation.
        self.current_joint_state = None
        js_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            js_qos
        )
        
        # Joint names (L1-L7 including gripper; IK uses L1-L6 only)
        self.joint_names_7 = ['L1_joint', 'L2_joint', 'L3_joint', 'L4_joint', 'L5_joint', 'L6_joint', 'L7_joint']
        self.joint_names = ['L1_joint', 'L2_joint', 'L3_joint', 'L4_joint', 'L5_joint', 'L6_joint']
        
        # [Debug] IK solution publisher - for Foxglove tracking
        self.ik_solution_pub = self.create_publisher(
            JointState,
            '/debug/ik_solution',
            10
        )
        
        # [Debug] Raw IK solution publisher (before processing) - for comparison
        self.ik_raw_solution_pub = self.create_publisher(
            JointState,
            '/debug/ik_raw_solution',
            10
        )
        
        # IK request state tracking (for fast IK mode)
        self.pending_ik_request = False  # Whether there is a pending IK request
        self._ik_resend_needed = False  # True when a newer target arrived while IK was in flight
        self.last_ik_joint_positions = None  # Cache of last IK joint positions
        self._latest_ik_raw = None      # IK callback writes latest raw solution here
        self._latest_ik_consumed = True  # True once the timer has consumed the result
        self._ik_smooth_target = None   # 2nd-order filter state: position
        self._ik_smooth_vel = None      # 2nd-order filter state: velocity
        self._last_ik_raw_for_seed = None  # Raw IK solution used as seed for next request (avoids smoothing drift)
        
        # MoveGroup and ExecuteTrajectory now handled by SDK (self._master_sdk)
        
        # State variables
        self.current_joy = None
        self.target_pose = None
        self.is_moving = False
        self.last_update_time = self.get_clock().now()
        self.pose_initialized = False  # Whether pose is properly initialized
        self.last_sent_pose = None  # Last sent pose, for change detection
        self.min_move_threshold = 0.00005  # Minimum move threshold (meters)
        self.move_start_time = None  # Motion start time, for timeout detection
        self.move_timeout = 2.0  # Motion timeout (seconds)
        
        # ArmManager SDK integration (must be before service waits and startup_go_zero)
        try:
            self._master_sdk = self._arm_mgr.register_ros_arm(
                "master",
                namespace=self.master_namespace,
                controller_name=self.master_controller,
                move_group_name=self.planning_group,
                ee_link=self.end_effector_frame,
                base_link=self.base_frame,
                use_internal_executor=False,
            )
            self._master_sdk.ConnectPort()
            self.get_logger().info(f'主臂 SDK 已注册（namespace={self.master_namespace!r}，外部 executor 模式）')
        except Exception as e:
            self.get_logger().warn(f'主臂 SDK 初始化失败：{e}')
            self._master_sdk = None

        self._local_kin = None
        if self._master_sdk:
            try:
                self._local_kin = self._master_sdk._get_kinematics()
                if self._local_kin:
                    self.get_logger().info('Pinocchio 本地 IK 已就绪（将绕过 MoveIt 服务）')
            except Exception as e:
                self.get_logger().warn(f'Pinocchio 初始化失败，将回退到 MoveIt IK：{e}')

        if self.master_slave_enabled:
            try:
                self._slave_sdk = self._arm_mgr.register_ros_arm(
                    "slave",
                    namespace=self.slave_namespace,
                    controller_name=self.slave_controller,
                    use_internal_executor=False,
                )
                self._slave_sdk.ConnectPort()
                self.get_logger().info(f'从臂 SDK 已注册（namespace={self.slave_namespace!r}，外部 executor 模式）')
            except Exception as e:
                self.get_logger().warn(f'从臂 SDK 初始化失败：{e}')
                self._slave_sdk = None

        # Wait for MoveIt services via SDK (with timeout, non-blocking if unavailable)
        # With local Pinocchio IK, MoveIt service is only needed for go_home/go_zero trajectory planning
        self._moveit_available = False
        _svc_timeout = 5.0  # seconds
        if self._master_sdk:
            if self.use_fast_ik_mode:
                if self._local_kin:
                    self.get_logger().info('Jacobian 微分 IK 就绪（100Hz，跳过 MoveIt IK 服务等待）')
                    self._moveit_available = True
                else:
                    self.get_logger().info('等待 MoveIt IK 服务（回退模式，超时 %.0fs）...' % _svc_timeout)
                    if self._master_sdk._ik_client and self._master_sdk._ik_client.wait_for_service(timeout_sec=_svc_timeout):
                        self.get_logger().info('MoveIt IK 服务已连接（回退路径）')
                        self._moveit_available = True
                    else:
                        self.get_logger().warn('IK 服务不可用，笛卡尔控制将被禁用')
            else:
                self.get_logger().info('等待笛卡尔路径服务（超时 %.0fs）...' % _svc_timeout)
                if self._master_sdk._cartesian_path_client and self._master_sdk._cartesian_path_client.wait_for_service(timeout_sec=_svc_timeout):
                    self._moveit_available = True
                else:
                    self.get_logger().warn('笛卡尔路径服务不可用，笛卡尔控制将被禁用')
            if self._moveit_available:
                self.get_logger().info('等待 MoveGroup 服务...')
                if self._master_sdk._move_group_client:
                    if not self._master_sdk._move_group_client.wait_for_server(timeout_sec=_svc_timeout):
                        self.get_logger().warn('MoveGroup 服务不可用')
                        self._moveit_available = False
            if self._moveit_available:
                self.get_logger().info('MoveIt 服务已全部连接')
            else:
                self.get_logger().warn('MoveIt 服务未就绪，仅关节控制/主从模式可用')
        else:
            self.get_logger().warn('SDK 未初始化，跳过 MoveIt 服务等待')
        
        # [Auto home on startup]
        # 1. Wait for joint states to be available (ensure motor positions are read)
        self.get_logger().info('等待关节状态（读取电机位置）...')
        self.wait_for_joint_states()
        
        # 2. Print current joint positions
        if self.current_joint_state:
            self.get_logger().info('已读取当前电机位置：')
            for i, name in enumerate(self.current_joint_state.name):
                if i < len(self.current_joint_state.position):
                    self.get_logger().info(f'  {name}: {self.current_joint_state.position[i]:.4f} rad')
        
        # 3. Automatically move to zero position
        self.get_logger().info('正在规划运动到零位...')
        self.startup_go_zero()
        
        # Dedicated high-precision control thread (bypasses rclpy executor jitter)
        self._control_running = True
        self._control_thread = threading.Thread(
            target=self._control_loop, daemon=True, name='ctrl_100hz')
        self._control_thread.start()
        
        self.get_logger().info('Xbox 手柄控制节点已启动')
        self.get_logger().info(f'规划组：{self.planning_group}')
        self.get_logger().info(f'末端执行器：{self.end_effector_frame}')
        self.get_logger().info(f'更新频率：{self.update_rate} Hz（dt={self.dt*1000:.1f}ms）')
        self.get_logger().info(f'最大线速度：{self.max_linear_velocity} m/s')
        self.get_logger().info(f'最大角速度：{self.max_angular_velocity} rad/s')
        self.get_logger().info(f'关节平滑：alpha={self.joint_smoothing_alpha}，max_vel={self.max_joint_velocity} rad/s')
        if self.use_fast_ik_mode:
            mode = 'Jacobian 微分 IK' if self._local_kin else 'MoveIt 异步'
            self.get_logger().info(f'控制模式：{mode}（{self.update_rate:.0f}Hz 笛卡尔控制）')
        else:
            self.get_logger().info('控制模式：传统路径规划模式')

        self.get_logger().info('=== 控制说明 ===')
        self.get_logger().info('左摇杆：XY 平移 | LT/RT：Z 平移')
        self.get_logger().info('右摇杆：Yaw/Roll | LB/RB：Pitch')
        self.get_logger().info('A：切换速度档 | B：回 home | X：回零位')
        self.get_logger().info('Y：切换重力补偿模式（手动示教/拖动）')
        self.get_logger().info('Menu：切换主从遥操作（ROS namespace 模式）')
        self.log_speed_level()
        
    def sync_current_pose(self):
        """Sync current end effector pose from TF as target pose starting point
        
        Each call gets the latest actual pose, ensuring incremental motion
        is based on the correct starting point.
        
        Returns:
            bool: True on success, False on failure
        """
        try:
            # Try to get TF transform (with shorter timeout)
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.end_effector_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)  # Shorter timeout for faster response
            )
            
            # Update target pose to current actual pose every time
            self.target_pose = PoseStamped()
            self.target_pose.header.frame_id = self.base_frame
            self.target_pose.pose.position.x = transform.transform.translation.x
            self.target_pose.pose.position.y = transform.transform.translation.y
            self.target_pose.pose.position.z = transform.transform.translation.z
            self.target_pose.pose.orientation = transform.transform.rotation
            
            if not self.pose_initialized:
                self.get_logger().info(f'Initial pose sync: x={self.target_pose.pose.position.x:.4f}, '
                                     f'y={self.target_pose.pose.position.y:.4f}, '
                                     f'z={self.target_pose.pose.position.z:.4f}')
                # [Key] If IK seed is preset (e.g., from home/zero callbacks), keep it.
                # Otherwise initialize from current joint state.
                if self.last_ik_joint_positions is None:
                    # Only initialize from joint_states when IK seed is unset
                    if self.current_joint_state:
                        seed_positions = []
                        for joint_name in self.joint_names:
                            if joint_name in self.current_joint_state.name:
                                idx = self.current_joint_state.name.index(joint_name)
                                seed_positions.append(self.current_joint_state.position[idx])
                            else:
                                seed_positions.append(0.0)  # Default
                        self.last_ik_joint_positions = seed_positions
                        self.smoothed_joint_positions = list(seed_positions)
                        self.last_joint_velocities = [0.0] * len(seed_positions)
                        self.get_logger().info(f'IK 种子已从 joint_states 初始化：{[f"{p:.3f}" for p in self.last_ik_joint_positions]}')
                        self.ik_seed_just_initialized = True
                        self.consecutive_ik_rejects = 0
                else:
                    # IK seed already preset (from home/zero callbacks); only sync Cartesian pose
                    self.get_logger().info(f'使用预设 IK 种子：{[f"{p:.3f}" for p in self.last_ik_joint_positions]}')
                self.pose_initialized = True
            
            return True
            
        except Exception as e:
            self.get_logger().debug(f'TF 查询失败：{str(e)}')
            return False
            
    def joint_state_callback(self, msg):
        """Joint state callback.

        QoS TRANSIENT_LOCAL subscription already filters out the VOLATILE
        ROS2UnityJointNode publisher.  Only accept messages that contain
        our expected arm joints.
        """
        if not msg.name or len(msg.position) < len(msg.name):
            return
        self.current_joint_state = msg
    
    def slave_joint_state_callback(self, msg):
        """Slave joint state callback (for master-slave mode)"""
        self.slave_joint_state = msg
    
    def master_joint_state_ms_callback(self, msg):
        """Master namespace joint state callback (for master-slave mode)"""
        self.master_joint_state_ms = msg
        if self.master_slave_mode:
            self._master_slave_follow()
    
    def wait_for_joint_states(self, timeout_sec=10.0):
        """Wait for joint states (ensure motor positions are read)"""
        import time
        start_time = time.time()
        while self.current_joint_state is None:
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - start_time > timeout_sec:
                self.get_logger().warn('等待关节状态超时！将使用默认位置')
                return False
        return True
    
    def startup_go_zero(self):
        """Move to zero position on startup via SDK JointCtrlList"""
        import time

        self.is_going_home = True
        self.home_start_time = self.get_clock().now()

        if not self._master_sdk or not self._master_sdk._connected:
            self.get_logger().error('SDK 未连接，无法规划零位')
            self.is_going_home = False
            self.home_start_time = None
            return False

        time.sleep(1.0)

        self.get_logger().info('发送零位轨迹（SDK JointCtrlList）...')
        ok = self._master_sdk.JointCtrlList([0.0] * 6, duration_ns=3_000_000_000)
        if ok:
            time.sleep(3.5)
            self.get_logger().info('已移动到零位')
            self._finalize_move([0.0] * 6)
            if self.enable_collision_check:
                self.collision_check_active = True
                self.get_logger().info('已启用碰撞检测')
            self.is_going_home = False
            self.home_start_time = None
            return True

        self.get_logger().error('零位轨迹发布失败')
        self.is_going_home = False
        self.home_start_time = None
        return False
    
    def joy_callback(self, msg):
        """Controller input callback"""
        self.current_joy = msg
        
        # Debug mode: periodically print raw input
        if self.debug_input:
            self.debug_counter += 1
            if self.debug_counter >= 40:  # Print every 2s (assumes 20Hz)
                self.debug_counter = 0
                if len(msg.axes) >= 6:
                    self.get_logger().info(
                        f'摇杆原始值：LX={msg.axes[0]:.3f} LY={msg.axes[1]:.3f} '
                        f'LT={msg.axes[2]:.3f} RX={msg.axes[3]:.3f} '
                        f'RY={msg.axes[4]:.3f} RT={msg.axes[5]:.3f}'
                    )
        
    def apply_deadzone(self, value):
        """Apply stick deadzone"""
        if abs(value) < self.deadzone:
            return 0.0
        # Linearly remap values outside deadzone
        sign = 1.0 if value > 0 else -1.0
        return sign * (abs(value) - self.deadzone) / (1.0 - self.deadzone)
    
    def apply_trigger_deadzone(self, trigger_value):
        """Apply trigger deadzone - for LT/RT"""
        # Trigger range is 1.0 (not pressed) to -1.0 (fully pressed)
        # Convert to 0.0 (not pressed) to 1.0 (fully pressed)
        normalized = (1.0 - trigger_value) / 2.0
        
        # Apply deadzone (triggers need a slightly larger deadzone)
        trigger_deadzone = self.deadzone * 1.5  # Larger deadzone for triggers
        if normalized < trigger_deadzone:
            return 0.0
        # Linear mapping
        return (normalized - trigger_deadzone) / (1.0 - trigger_deadzone)
    
    def log_speed_level(self):
        """Log current speed level"""
        level = self.speed_levels[self.current_speed_level]
        actual_linear = self.max_linear_velocity * self.speed_factor
        actual_angular = self.max_angular_velocity * self.speed_factor
        self.get_logger().info(
            f'当前速度档位：{self.current_speed_level + 1}/5 [{level["name"]}] '
            f'(线速度={actual_linear*1000:.0f}mm/s，角速度={actual_angular:.2f}rad/s)'
        )
    
    def switch_speed_level(self):
        """Switch speed level"""
        self.current_speed_level = (self.current_speed_level + 1) % len(self.speed_levels)
        level = self.speed_levels[self.current_speed_level]
        self.speed_factor = level['factor']
        self.log_speed_level()
    
    def _finalize_move(self, target_positions):
        """Reset IK state after a successful direct move"""
        self.last_ik_joint_positions = list(target_positions)
        self.smoothed_joint_positions = list(target_positions)
        self._ik_smooth_target = list(target_positions)
        self._ik_smooth_vel = [0.0] * 6
        self._last_ik_raw_for_seed = list(target_positions)
        self.last_joint_velocities = [0.0] * 6
        self._latest_ik_raw = None
        self._latest_ik_consumed = True
        self.ik_seed_just_initialized = True
        self.consecutive_ik_rejects = 0
        self.pose_initialized = False

    def go_home(self):
        """Return to home position via SDK PlanToJointGoalAsync (with fallback)"""
        if self.is_moving or self.is_going_home:
            self.get_logger().warn('当前正在执行其他动作，请稍后再试')
            return

        if not self._master_sdk or not self._master_sdk._connected:
            self.get_logger().warn('SDK 未连接，无法回到 home')
            return

        home_positions = [0.0, 0.785, -0.785, 0.0, 0.0, 0.0]

        self.is_going_home = True
        self.home_start_time = self.get_clock().now()
        self._pending_move_target = home_positions
        self.get_logger().info('正在回到 home 位置（SDK PlanToJointGoalAsync）...')

        try:
            self._master_sdk.PlanToJointGoalAsync(
                joint_positions=home_positions,
                velocity_scale=0.3,
                accel_scale=0.3,
                result_callback=self._on_home_complete,
            )
        except Exception as e:
            self.get_logger().warn(f'PlanToJointGoalAsync 失败（{e}），使用 SDK JointCtrlList fallback')
            self._master_sdk.JointCtrlList(home_positions, duration_ns=2_000_000_000)
            self._move_done_timer = self.create_timer(2.5, self._on_move_done_timer)

    def _on_home_complete(self, success):
        """SDK PlanToJointGoalAsync callback for home"""
        target = getattr(self, '_pending_move_target', None)
        if success and target is not None:
            self._finalize_move(target)
            self.get_logger().info(f'已回到 home 位置（SDK 规划成功）')
        else:
            self.get_logger().warn('home 运动 SDK 报告失败（机械臂可能已到位）')
            if target is not None:
                self._finalize_move(target)
        self._pending_move_target = None
        self.is_going_home = False
        self.home_start_time = None

    def go_zero(self):
        """Return to zero position via SDK PlanToJointGoalAsync (with fallback)"""
        if self.is_moving or self.is_going_home:
            self.get_logger().warn('当前正在执行其他动作，请稍后再试')
            return

        if not self._master_sdk or not self._master_sdk._connected:
            self.get_logger().warn('SDK 未连接，无法回到零位')
            return

        zero_positions = [0.0] * 6

        self.is_going_home = True
        self.home_start_time = self.get_clock().now()
        self._pending_move_target = zero_positions
        self.get_logger().info('正在回到零位（SDK PlanToJointGoalAsync）...')

        try:
            self._master_sdk.PlanToJointGoalAsync(
                joint_positions=zero_positions,
                velocity_scale=0.3,
                accel_scale=0.3,
                result_callback=self._on_zero_complete,
            )
        except Exception as e:
            self.get_logger().warn(f'PlanToJointGoalAsync 失败（{e}），使用 SDK JointCtrlList fallback')
            self._master_sdk.JointCtrlList(zero_positions, duration_ns=2_000_000_000)
            self._move_done_timer = self.create_timer(2.5, self._on_move_done_timer)

    def _on_zero_complete(self, success):
        """SDK PlanToJointGoalAsync callback for zero"""
        target = getattr(self, '_pending_move_target', None)
        if success and target is not None:
            self._finalize_move(target)
            self.get_logger().info('已回到零位（SDK 规划成功）')
        else:
            self.get_logger().warn('零位运动 SDK 报告失败（机械臂可能已到位）')
            if target is not None:
                self._finalize_move(target)
        self._pending_move_target = None
        self.is_going_home = False
        self.home_start_time = None

    def _on_move_done_timer(self):
        """Timer fallback: finalize home/zero move if SDK callback didn't fire"""
        if hasattr(self, '_move_done_timer') and self._move_done_timer:
            self.destroy_timer(self._move_done_timer)
            self._move_done_timer = None

        target = getattr(self, '_pending_move_target', None)
        if target is not None:
            self._finalize_move(target)
            self.get_logger().info(f'已到达目标位置（fallback timer）：{[f"{p:.3f}" for p in target]}')
            self._pending_move_target = None

        self.is_going_home = False
        self.home_start_time = None
    
    def toggle_zero_torque_mode(self):
        """Toggle gravity compensation (zero torque) mode via SDK ZeroTorqueMode"""
        if not self._master_sdk or not self._master_sdk._connected:
            self.get_logger().warn('SDK 未连接，无法切换零力矩模式')
            return

        new_state = not self.zero_torque_mode

        if not new_state:
            self._sync_controller_to_current_position()

        self.get_logger().info(f'正在{"开启" if new_state else "关闭"}零力矩模式...')
        threading.Thread(
            target=self._sdk_zero_torque_thread,
            args=(new_state,),
            daemon=True,
        ).start()

    def _sync_controller_to_current_position(self):
        """Send current joint positions to arm_controller via SDK to avoid mode-switch jumps"""
        if self.current_joint_state is None:
            return
        positions = []
        for name in self.joint_names:
            if name in self.current_joint_state.name:
                idx = self.current_joint_state.name.index(name)
                positions.append(self.current_joint_state.position[idx])
            else:
                positions.append(0.0)
        if self._master_sdk and self._master_sdk._connected:
            self._master_sdk.JointCtrlList(positions, duration_ns=50_000_000)
        else:
            self.get_logger().warn('SDK 未连接，无法同步位置')

    def _sdk_zero_torque_thread(self, enable):
        """Background thread for SDK ZeroTorqueMode (blocking call)"""
        try:
            ok = self._master_sdk.ZeroTorqueMode(enable)
            if ok:
                self.zero_torque_mode = enable
                if enable:
                    self.get_logger().info('>>> 已开启重力补偿：机械臂可手动拖动 <<<')
                else:
                    self.pose_initialized = False
                    self.last_ik_joint_positions = None
                    self._ik_smooth_target = None
                    self._ik_smooth_vel = None
                    self.get_logger().info('>>> 已关闭重力补偿：恢复控制器控制 <<<')
            else:
                self.get_logger().error('零力矩模式切换失败')
        except Exception as e:
            self.get_logger().error(f'零力矩切换异常：{e}')
        
    # ============ Master-slave teleoperation mode ============

    def toggle_master_slave_mode(self):
        """Toggle master-slave teleoperation mode (Menu button) — ROS namespace mode"""
        if self.entering_master_slave:
            self.get_logger().warn('正在切换主从模式，请稍候')
            return
        
        if not self.master_slave_mode:
            # === Enable master-slave mode ===
            self.get_logger().info('>>> 正在启用主从遥操作模式（ROS namespace） <<<')
            self.entering_master_slave = True
            
            if self.master_joint_state_ms is None:
                self.get_logger().error('主臂关节状态不可用，无法启用主从模式')
                self.entering_master_slave = False
                return

            if self._slave_sdk is None:
                self.get_logger().error('从臂 SDK 不可用，无法启用主从模式')
                self.entering_master_slave = False
                return
            
            self._move_both_to_zero_ros()
            
            delay = self.go_zero_duration + 0.5
            self._ms_enable_timer = self.create_timer(
                delay, self._on_go_zero_done_enable)
        else:
            # === Disable master-slave mode ===
            self.get_logger().info('>>> 正在关闭主从遥操作模式 <<<')
            self.master_slave_mode = False
            self.entering_master_slave = True
            
            self._set_master_zero_torque(False)
            
            self._ms_disable_pre_timer = self.create_timer(
                0.5, self._on_gravity_off_then_go_zero)

    def _on_gravity_off_then_go_zero(self):
        """Send zero trajectory after gravity compensation is off"""
        self._ms_disable_pre_timer.cancel()
        self.destroy_timer(self._ms_disable_pre_timer)
        
        self._move_both_to_zero_ros()
        
        delay = self.go_zero_duration + 0.5
        self._ms_disable_timer = self.create_timer(
            delay, self._on_go_zero_done_disable)

    def _on_go_zero_done_enable(self):
        """Enable master gravity compensation after zeroing (enable flow)"""
        self._ms_enable_timer.cancel()
        self.destroy_timer(self._ms_enable_timer)
        
        self._set_master_zero_torque(True)

        # Reset follow state for clean start
        self._last_slave_cmd = None
        self._smoothed_ms_positions = None
        self._ms_follow_count = 0
        if hasattr(self, '_ms_last_stamp'):
            del self._ms_last_stamp

        self.master_slave_mode = True
        self.entering_master_slave = False
        self.get_logger().info('>>> 主从遥操作已启用（ROS namespace 模式） <<<')
        self.get_logger().info(f'  主臂（{self.master_namespace or "default"}）：L1-L7 重力补偿可拖动')
        self.get_logger().info(f'  从臂（{self.slave_namespace}）：L1-L7 ROS 轨迹跟随')
        self.get_logger().info('  再按一次 Menu 退出')

    def _on_go_zero_done_disable(self):
        """Restore normal control after zeroing (disable flow)"""
        self._ms_disable_timer.cancel()
        self.destroy_timer(self._ms_disable_timer)
        
        self.entering_master_slave = False
        
        self._sync_controller_to_current_position()
        self.pose_initialized = False
        self.last_ik_joint_positions = None
        self.smoothed_joint_positions = None
        self._ik_smooth_target = None
        self._ik_smooth_vel = None
        self.get_logger().info('>>> 主从遥操作已关闭：恢复 Xbox 控制 <<<')

    def _set_master_zero_torque(self, enable):
        """Set master gravity compensation via SDK ZeroTorqueMode (background thread)"""
        if self._master_sdk and self._master_sdk._connected:
            threading.Thread(
                target=self._sdk_master_zero_torque_thread,
                args=(enable,),
                daemon=True,
            ).start()
        else:
            self.get_logger().warn('主臂 SDK 不可用，无法切换零力矩模式')

    def _sdk_master_zero_torque_thread(self, enable):
        """Background thread for master arm SDK ZeroTorqueMode"""
        try:
            ok = self._master_sdk.ZeroTorqueMode(enable)
            state_str = '已开启' if enable else '已关闭'
            if ok:
                self.get_logger().info(f'主臂重力补偿{state_str}')
            else:
                self.get_logger().error('主臂重力补偿切换失败')
        except Exception as e:
            self.get_logger().error(f'主臂重力补偿切换异常：{e}')

    def _move_both_to_zero_ros(self):
        """Send slow zero trajectories to both arms via SDK JointCtrlList"""
        duration_ns = int(self.go_zero_duration * 1_000_000_000)

        if self._master_sdk and self._master_sdk._connected:
            self._master_sdk.JointCtrlList([0.0] * 6, duration_ns=duration_ns)
            self.get_logger().info(f'主臂已发送回零轨迹（SDK，{self.go_zero_duration}s）')
        else:
            self.get_logger().warn('SDK 主臂不可用，无法回零')

        if self._slave_sdk and self._slave_sdk._connected:
            self._slave_sdk.JointCtrlList([0.0] * 6, duration_ns=duration_ns)
            self.get_logger().info(f'从臂已发送回零轨迹（SDK，{self.go_zero_duration}s）')
        else:
            self.get_logger().warn('从臂 SDK 不可用，无法控制从臂回零')

    def _extract_joint_positions(self, joint_state, joint_names):
        """Extract joint positions by name from JointState message"""
        positions = []
        for name in joint_names:
            if name in joint_state.name:
                idx = joint_state.name.index(name)
                positions.append(joint_state.position[idx])
            else:
                return None  # Missing joint data
        return positions

    def _extract_joint_velocities(self, joint_state, joint_names):
        """Extract filtered joint velocities by name from JointState message.

        The hardware layer already applies EMA low-pass filtering
        (velocity_filter_alpha=0.3), so these values are suitable for
        direct use as velocity feedforward targets.
        """
        if not joint_state.velocity:
            return None
        velocities = []
        for name in joint_names:
            if name in joint_state.name:
                idx = joint_state.name.index(name)
                if idx < len(joint_state.velocity):
                    velocities.append(joint_state.velocity[idx])
                else:
                    velocities.append(0.0)
            else:
                return None
        return velocities

    def _master_slave_follow(self):
        """Master-slave follow: read master L1-L7 joints, send to slave via SDK.

        Called directly from master_joint_state_ms_callback at master
        publish rate (~200Hz) for lowest-latency following.
        """
        if self.master_joint_state_ms is None:
            return

        master_positions = self._extract_joint_positions(
            self.master_joint_state_ms, self.master_joint_names_ms)
        if master_positions is None:
            return

        # Optional EMA smoothing on master positions (alpha=1.0 means pass-through)
        alpha = self.ms_smoothing_alpha
        if alpha < 1.0:
            if self._smoothed_ms_positions is None:
                self._smoothed_ms_positions = list(master_positions)
            else:
                self._smoothed_ms_positions = [
                    alpha * cur + (1.0 - alpha) * prev
                    for cur, prev in zip(master_positions, self._smoothed_ms_positions)
                ]
            master_positions = list(self._smoothed_ms_positions)

        # Extract hardware-filtered velocities for feedforward
        master_velocities = self._extract_joint_velocities(
            self.master_joint_state_ms, self.master_joint_names_ms)
        arm_velocities = list(master_velocities[:6]) if master_velocities else None

        if not (self._slave_sdk and self._slave_sdk._connected):
            self.get_logger().warn('从臂 SDK 不可用，跳过主从跟随', throttle_duration_sec=5.0)
            return

        # Compute actual dt from callback timing
        now = self.get_clock().now()
        if not hasattr(self, '_ms_last_stamp'):
            self._ms_last_stamp = now
        dt_duration = now - self._ms_last_stamp
        dt = max(dt_duration.nanoseconds / 1e9, 1e-4)  # guard against zero
        self._ms_last_stamp = now

        # Safety velocity clamping
        max_joint_vel = 8.0  # rad/s
        max_delta = max_joint_vel * dt
        if self._last_slave_cmd is not None:
            for i in range(len(master_positions)):
                delta = master_positions[i] - self._last_slave_cmd[i]
                if abs(delta) > max_delta:
                    master_positions[i] = self._last_slave_cmd[i] + max_delta * (1.0 if delta > 0 else -1.0)
        self._last_slave_cmd = list(master_positions)

        dt_ns = max(int(dt * 1e9), 1_000_000)  # at least 1ms
        self._slave_sdk.JointCtrlList(
            master_positions, duration_ns=dt_ns, velocities=arm_velocities)

        self._ms_follow_count += 1
        if self._ms_follow_count % 200 == 1:
            pos_str = ', '.join([f'{p:.3f}' for p in master_positions])
            self.get_logger().info(
                f'[主从跟随-SDK] dt={dt*1000:.1f}ms L1-L7=[{pos_str}]')

    def _control_loop(self):
        """Dedicated 100Hz control thread with precise timing."""
        period = 1.0 / self.update_rate
        next_tick = time.monotonic()
        while self._control_running and rclpy.ok():
            next_tick += period
            try:
                self.update_callback()
            except Exception as e:
                self.get_logger().error(
                    f'update_callback 异常：{e}', throttle_duration_sec=2.0)
            now = time.monotonic()
            sleep_s = next_tick - now
            if sleep_s < -period:
                next_tick = now + period
            elif sleep_s > 0:
                time.sleep(sleep_s)

    def update_callback(self):
        """Periodic update callback"""
        now = time.monotonic()
        nominal_dt = 1.0 / self.update_rate
        self.dt = nominal_dt
        if self._last_update_time is not None:
            measured_dt = now - self._last_update_time
        else:
            measured_dt = nominal_dt
        self._last_update_time = now

        self._diag_tick = getattr(self, '_diag_tick', 0) + 1
        self._diag_last_report = getattr(self, '_diag_last_report', now)
        self._diag_dt_min = min(getattr(self, '_diag_dt_min', 1.0), measured_dt)
        self._diag_dt_max = max(getattr(self, '_diag_dt_max', 0.0), measured_dt)
        if now - self._diag_last_report >= 5.0:
            jc = getattr(self, '_diag_jac_calls', 0)
            jo = getattr(self, '_diag_jac_ok', 0)
            je = getattr(self, '_diag_jac_err', 0)
            mc = getattr(self, '_diag_moveit_calls', 0)
            dt_min_ms = self._diag_dt_min * 1000
            dt_max_ms = self._diag_dt_max * 1000
            raw_deltas = getattr(self, '_diag_raw_deltas', [])
            nz = sum(1 for d in raw_deltas if abs(d) > 1e-9)
            self.get_logger().info(
                f'[DIAG] ticks={self._diag_tick} jac={jc}/{jo}/{je} '
                f'dt={dt_min_ms:.1f}-{dt_max_ms:.1f}ms '
                f'raw_delta_nz={nz}/{len(raw_deltas)} '
                f'pose_init={self.pose_initialized}')
            self._diag_tick = 0
            self._diag_jac_calls = 0
            self._diag_jac_ok = 0
            self._diag_jac_err = 0
            self._diag_moveit_calls = 0
            self._diag_dt_min = 1.0
            self._diag_dt_max = 0.0
            self._diag_raw_deltas = []
            self._diag_last_report = now

        if self.current_joy is None:
            return
        
        joy = self.current_joy
        
        # Ensure indices are valid
        if len(joy.axes) < 6 or len(joy.buttons) < 6:
            return
        
        # A button: switch speed level (edge-triggered)
        a_button = joy.buttons[0]
        if a_button == 1 and self.last_a_button == 0:
            self.switch_speed_level()
        self.last_a_button = a_button
        
        # B button: go home (edge-triggered)
        b_button = joy.buttons[1]
        if b_button == 1 and self.last_b_button == 0:
            self.go_home()
        self.last_b_button = b_button
        
        # X button: go zero (edge-triggered)
        x_button = joy.buttons[2]
        if x_button == 1 and self.last_x_button == 0:
            self.go_zero()
        self.last_x_button = x_button
        
        # Y button: toggle gravity compensation (edge-triggered)
        y_button = joy.buttons[3]
        if y_button == 1 and self.last_y_button == 0:
            self.toggle_zero_torque_mode()
        self.last_y_button = y_button
        
        # Menu button: toggle master-slave teleop (edge-triggered)
        if len(joy.buttons) > 7 and self.master_slave_enabled:
            menu_button = joy.buttons[7]
            if menu_button == 1 and self.last_menu_button == 0:
                self.toggle_master_slave_mode()
            self.last_menu_button = menu_button
        
        # D-pad: control gripper via L7_joint position (skip in master-slave mode)
        if len(joy.axes) >= 8 and not self.master_slave_mode:
            dpad_vertical = joy.axes[7]  # Up=1, down=-1
            
            dpad_up = 1 if dpad_vertical > 0.5 else 0
            if dpad_up == 1 and self.last_dpad_up == 0:
                self.gripper_target_angle += self.gripper_angle_step
                self.gripper_target_angle = min(self.gripper_target_angle, 1.5708)
                self._send_gripper_position(self.gripper_target_angle)
                self.get_logger().info(f'夹爪角度：{self.gripper_target_angle:.2f} rad')
            self.last_dpad_up = dpad_up
            
            dpad_down = 1 if dpad_vertical < -0.5 else 0
            if dpad_down == 1 and self.last_dpad_down == 0:
                self.gripper_target_angle -= self.gripper_angle_step
                self.gripper_target_angle = max(self.gripper_target_angle, -1.5708)
                self._send_gripper_position(self.gripper_target_angle)
                self.get_logger().info(f'夹爪角度：{self.gripper_target_angle:.2f} rad')
            self.last_dpad_down = dpad_down
        
        # In master-slave mode: follow is driven by joint_state callback, skip normal input
        if self.master_slave_mode:
            return
        if self.entering_master_slave:
            return
        
        # Skip motion control in zero-torque mode (only when NOT in master-slave mode)
        if self.zero_torque_mode:
            return
        
        # If returning home/zero, skip normal motion control (with timeout protection)
        if self.is_going_home:
            # Timeout check to prevent permanent block
            if self.home_start_time is not None:
                elapsed = (self.get_clock().now() - self.home_start_time).nanoseconds / 1e9
                if elapsed > self.home_timeout:
                    self.get_logger().warn(f'home/零位超时（{elapsed:.1f}s），正在重置状态并恢复控制')
                    self.is_going_home = False
                    self.home_start_time = None
                    self.pose_initialized = False
                    self._ik_smooth_target = None
                    self._ik_smooth_vel = None
                else:
                    return
            else:
                return
            
        # Parse controller input -> Cartesian velocity mapping
        # Xbox mapping:
        # Left stick X: axes[0], Left stick Y: axes[1]
        # LT: axes[2], RT: axes[5] (range 1.0 to -1.0, pressed = -1.0)
        # Right stick X: axes[3], Right stick Y: axes[4]
        # LB: buttons[4], RB: buttons[5]
        # A: buttons[0], B: buttons[1]
        
        # ============ Cartesian velocity mapping ============
        # Stick input → velocity (m/s, rad/s) → position delta
        
        # Compute actual max speed at current level
        current_max_linear = self.max_linear_velocity * self.speed_factor
        current_max_angular = self.max_angular_velocity * self.speed_factor
        
        # Translation - swap and invert left stick XY
        raw_vx = -self.apply_deadzone(joy.axes[1]) * current_max_linear  # 左摇杆Y -> X速度 (m/s)
        raw_vy = -self.apply_deadzone(joy.axes[0]) * current_max_linear  # 左摇杆X -> Y速度 (m/s)
        
        # LT/RT control Z velocity
        lt = self.apply_trigger_deadzone(joy.axes[2])  # 转换为0-1范围并应用死区
        rt = self.apply_trigger_deadzone(joy.axes[5])  # 转换为0-1范围并应用死区
        raw_vz = (rt - lt) * current_max_linear  # RT up, LT down (m/s)
        
        # Rotation velocities
        raw_vyaw = self.apply_deadzone(joy.axes[3]) * current_max_angular   # 右摇杆X -> Yaw速度 (rad/s)
        raw_vroll = self.apply_deadzone(joy.axes[4]) * current_max_angular  # 右摇杆Y -> Roll速度 (rad/s)
        
        # LB/RB control pitch velocity
        raw_vpitch = (joy.buttons[5] - joy.buttons[4]) * current_max_angular  # RB positive, LB negative (rad/s)
        
        # Rapid-stop: when all raw inputs are zero, snap EMA state to zero
        raw_lin_mag = abs(raw_vx) + abs(raw_vy) + abs(raw_vz)
        raw_ang_mag = abs(raw_vroll) + abs(raw_vpitch) + abs(raw_vyaw)
        if raw_lin_mag < 1e-6 and raw_ang_mag < 1e-6:
            self.smoothed_vx = 0.0
            self.smoothed_vy = 0.0
            self.smoothed_vz = 0.0
            self.smoothed_vroll = 0.0
            self.smoothed_vpitch = 0.0
            self.smoothed_vyaw = 0.0
        else:
            alpha = self.input_smoothing_factor
            self.smoothed_vx = alpha * raw_vx + (1 - alpha) * self.smoothed_vx
            self.smoothed_vy = alpha * raw_vy + (1 - alpha) * self.smoothed_vy
            self.smoothed_vz = alpha * raw_vz + (1 - alpha) * self.smoothed_vz
            self.smoothed_vroll = alpha * raw_vroll + (1 - alpha) * self.smoothed_vroll
            self.smoothed_vpitch = alpha * raw_vpitch + (1 - alpha) * self.smoothed_vpitch
            self.smoothed_vyaw = alpha * raw_vyaw + (1 - alpha) * self.smoothed_vyaw
        
        # Velocity × dt = position delta
        dx = self.smoothed_vx * self.dt
        dy = self.smoothed_vy * self.dt
        dz = self.smoothed_vz * self.dt
        droll = self.smoothed_vroll * self.dt
        dpitch = self.smoothed_vpitch * self.dt
        dyaw = self.smoothed_vyaw * self.dt
        
        # Check for input based on velocity thresholds
        velocity_threshold = 0.001  # Below 1mm/s treated as no input
        angular_threshold = 0.01    # Below 0.01 rad/s treated as no input
        translation_threshold = velocity_threshold * self.dt
        rotation_threshold = angular_threshold * self.dt
        
        has_translation = abs(dx) > translation_threshold or abs(dy) > translation_threshold or abs(dz) > translation_threshold
        has_rotation = abs(dyaw) > rotation_threshold or abs(dpitch) > rotation_threshold or abs(droll) > rotation_threshold
        
        if not has_translation and not has_rotation:
            if self.use_fast_ik_mode and self.pose_initialized:
                self.send_cartesian_goal()
            self._send_fixed_rate_ik_command()
            return
        
        # Accumulate deltas
        self.pending_dx += dx
        self.pending_dy += dy
        self.pending_dz += dz
        self.pending_droll += droll
        self.pending_dpitch += dpitch
        self.pending_dyaw += dyaw
        
        # [Key change] Sync pose only once, then accumulate deltas onto target pose
        # This avoids Z drift from tracking errors (e.g., sagging under gravity).
        if not self.pose_initialized:
            if not self.sync_current_pose():
                self.get_logger().warn('等待 TF 树就绪...')
                return  # TF not ready, skip this update
        
        # Use accumulated deltas
        dx = self.pending_dx
        dy = self.pending_dy
        dz = self.pending_dz
        droll = self.pending_droll
        dpitch = self.pending_dpitch
        dyaw = self.pending_dyaw
        
        # Clear accumulated deltas
        self.pending_dx = 0.0
        self.pending_dy = 0.0
        self.pending_dz = 0.0
        self.pending_droll = 0.0
        self.pending_dpitch = 0.0
        self.pending_dyaw = 0.0
            
        # Save last target to detect jumps
        if hasattr(self, 'last_target_x'):
            position_jump = math.sqrt(
                (self.target_pose.pose.position.x - self.last_target_x) ** 2 +
                (self.target_pose.pose.position.y - self.last_target_y) ** 2 +
                (self.target_pose.pose.position.z - self.last_target_z) ** 2
            )
            # Detect position jumps (warn if > 50mm)
            if position_jump > 0.05:
                self.get_logger().warn(f'检测到目标位姿跳变！距离={position_jump*1000:.1f}mm')
        
        # Debug: print actual inputs (only when there is input)
        if self.debug_input:
            self.get_logger().info(
                f'输入：dx={dx:.6f} dy={dy:.6f} dz={dz:.6f} '
                f'droll={droll:.6f} dpitch={dpitch:.6f} dyaw={dyaw:.6f}'
            )
            
        # Update target pose
        # Translation
        self.target_pose.pose.position.x += dx
        self.target_pose.pose.position.y += dy
        self.target_pose.pose.position.z += dz
        
        # Rotation - convert current quaternion to Euler
        q = self.target_pose.pose.orientation
        current_roll, current_pitch, current_yaw = euler_from_quaternion([
            q.x, q.y, q.z, q.w
        ])
        
        # Update Euler angles
        new_roll = current_roll + droll
        new_pitch = current_pitch + dpitch
        new_yaw = current_yaw + dyaw
        
        # Convert back to quaternion
        new_q = quaternion_from_euler(new_roll, new_pitch, new_yaw)
        self.target_pose.pose.orientation.x = new_q[0]
        self.target_pose.pose.orientation.y = new_q[1]
        self.target_pose.pose.orientation.z = new_q[2]
        self.target_pose.pose.orientation.w = new_q[3]
        
        # Save current target for next jump detection
        self.last_target_x = self.target_pose.pose.position.x
        self.last_target_y = self.target_pose.pose.position.y
        self.last_target_z = self.target_pose.pose.position.z
        
        # Publish target pose for visualization
        self.target_pose.header.stamp = self.get_clock().now().to_msg()
        self.target_pose_pub.publish(self.target_pose)
        
        # Send motion command (triggers async IK request)
        self.send_cartesian_goal()

        # Fixed-rate IK command output (consumes buffered IK result)
        self._send_fixed_rate_ik_command()

    def pose_changed_significantly(self):
        """Check if target pose changed significantly"""
        if self.last_sent_pose is None:
            return True
        
        # Compute position change
        dx = abs(self.target_pose.pose.position.x - self.last_sent_pose.pose.position.x)
        dy = abs(self.target_pose.pose.position.y - self.last_sent_pose.pose.position.y)
        dz = abs(self.target_pose.pose.position.z - self.last_sent_pose.pose.position.z)
        
        # If position change exceeds threshold, treat as significant change
        if dx > self.min_move_threshold or dy > self.min_move_threshold or dz > self.min_move_threshold:
            return True
        
        # Check orientation change (simplified: quaternion w component)
        dw = abs(self.target_pose.pose.orientation.w - self.last_sent_pose.pose.orientation.w)
        if dw > 0.01:  # About 1 degree
            return True
        
        return False
    
    def send_cartesian_goal(self):
        """Send Cartesian goal - use fast IK or path planning based on mode"""
        if self.use_fast_ik_mode:
            self.send_ik_goal()
        else:
            self.send_cartesian_path_goal()
    
    def _get_ik_seed(self):
        """Return the best available IK seed (raw > filtered > last IK > joint_states).

        Prefer the last raw IK solution so the Jacobian is evaluated at the
        most recent solve point, avoiding the phase-lag oscillation that occurs
        when the seed trails the raw output through the 2nd-order filter.
        """
        seed = (self._last_ik_raw_for_seed
                or self._ik_smooth_target
                or self.last_ik_joint_positions)
        if seed is None and self.current_joint_state:
            seed = []
            for name in self.joint_names:
                if name in self.current_joint_state.name:
                    idx = self.current_joint_state.name.index(name)
                    seed.append(self.current_joint_state.position[idx])
                else:
                    seed.append(0.0)
        return seed

    def _process_ik_result(self, ik_positions):
        """Validate and store an IK solution (jump protection + buffer write).

        Returns True if the solution was accepted, False otherwise.
        """
        self.publish_ik_raw_debug(ik_positions)

        prev_raw = getattr(self, '_diag_prev_raw_j1', None)
        if prev_raw is not None and len(ik_positions) > 1:
            delta = ik_positions[1] - prev_raw
            raw_deltas = getattr(self, '_diag_raw_deltas', [])
            raw_deltas.append(delta)
            self._diag_raw_deltas = raw_deltas
        if len(ik_positions) > 1:
            self._diag_prev_raw_j1 = ik_positions[1]

        ik_ref = self._last_ik_raw_for_seed or self.last_ik_joint_positions
        if ik_ref is not None:
            max_diff = max(abs(ik_positions[i] - ik_ref[i])
                           for i in range(len(ik_positions)))

            if max_diff > self.max_ik_jump_threshold:
                if self.ik_seed_just_initialized:
                    self.get_logger().info(
                        f'IK 种子初始化后的首帧：接受 IK 解（diff={max_diff:.3f}rad）')
                    self.ik_seed_just_initialized = False
                else:
                    self.consecutive_ik_rejects += 1
                    if self.consecutive_ik_rejects >= self.singularity_warning_count:
                        self.get_logger().warn(
                            f'疑似奇异区：IK 跳变={max_diff:.3f}rad，'
                            f'已保护 {self.consecutive_ik_rejects} 帧')
                    if self.consecutive_ik_rejects >= 50:
                        self.get_logger().warn(
                            'IK 解已连续拒绝 50+ 帧，自动重新同步位姿...')
                        self.pose_initialized = False
                        self.smoothed_joint_positions = None
                        self.last_ik_joint_positions = None
                        self._ik_smooth_target = None
                        self._ik_smooth_vel = None
                        self.consecutive_ik_rejects = 0
                    return False
            else:
                if self.consecutive_ik_rejects > 0:
                    self.consecutive_ik_rejects = 0
                if self.ik_seed_just_initialized:
                    self.ik_seed_just_initialized = False

        self._last_ik_raw_for_seed = list(ik_positions)
        self._latest_ik_raw = list(ik_positions)
        self._latest_ik_consumed = False
        self.last_sent_pose = copy.deepcopy(self.target_pose)
        return True

    def send_ik_goal(self):
        """Jacobian differential IK (< 0.1ms) with MoveIt fallback."""
        if self._local_kin is not None:
            self._diag_jac_calls = getattr(self, '_diag_jac_calls', 0) + 1
            q = self._get_ik_seed()
            if q is None:
                self.get_logger().warn('Jacobian: seed is None', throttle_duration_sec=2.0)
                return

            try:
                J_full = self._local_kin.compute_jacobian(q)
                J = J_full[:, :6]

                v = np.array([
                    self.smoothed_vx, self.smoothed_vy, self.smoothed_vz,
                    self.smoothed_vroll, self.smoothed_vpitch, self.smoothed_vyaw,
                ])

                v_norm = np.linalg.norm(v[:3])
                w_norm = np.linalg.norm(v[3:])
                if v_norm < 5e-4 and w_norm < 5e-3:
                    v[:] = 0.0

                sigma = np.linalg.svd(J, compute_uv=False)
                sigma_min = sigma[-1]
                sigma_max = sigma[0]
                cond = sigma_max / max(sigma_min, 1e-12)
                sigma_thresh = 0.035
                lambda_base = 5e-3
                lambda_sing = 8e-2
                cond_thresh = 80.0
                lambda_cond = 2e-2
                if sigma_min < sigma_thresh:
                    r = (sigma_min / sigma_thresh) ** 2
                    damping = lambda_base + lambda_sing * (1.0 - r)
                else:
                    damping = lambda_base
                if cond > cond_thresh:
                    rc = min((cond - cond_thresh) / cond_thresh, 1.0)
                    damping = max(damping, lambda_base + lambda_cond * rc)

                JJt = J @ J.T + damping * np.eye(6)
                dq = J.T @ np.linalg.solve(JJt, v)

                max_dq = self.max_joint_velocity
                dq = np.clip(dq, -max_dq, max_dq)

                q_new = [q[i] + float(dq[i]) * self.dt for i in range(6)]
                self._process_ik_result(q_new)
                self._diag_jac_ok = getattr(self, '_diag_jac_ok', 0) + 1
            except Exception as e:
                self._diag_jac_err = getattr(self, '_diag_jac_err', 0) + 1
                self.get_logger().error(
                    f'Jacobian IK 异常：{e}', throttle_duration_sec=1.0)
            return
        else:
            self._diag_moveit_calls = getattr(self, '_diag_moveit_calls', 0) + 1

        self._send_ik_goal_moveit()

    def _send_ik_goal_moveit(self):
        """Fallback: async IK via MoveIt /compute_ik service."""
        import time as _time

        if self.pending_ik_request:
            if hasattr(self, '_ik_request_time') and (_time.time() - self._ik_request_time) > 0.5:
                self.get_logger().warn('IK 请求超时 0.5s，强制重置', throttle_duration_sec=2.0)
                self.pending_ik_request = False
            else:
                self._ik_resend_needed = True
                return
        self._ik_resend_needed = False

        if not self._master_sdk or not self._master_sdk._connected:
            self.get_logger().warn('SDK 未连接，跳过 IK 请求', throttle_duration_sec=5.0)
            return

        seed = self._get_ik_seed()

        self.pending_ik_request = True
        self._ik_request_time = _time.time()
        try:
            future = self._master_sdk.ComputeIKAsync(
                target_pose=self.target_pose.pose,
                seed_positions=seed,
                avoid_collisions=self.collision_check_active,
                timeout_ns=10_000_000,
            )
            future.add_done_callback(self._ik_callback_moveit)
        except Exception as e:
            self.pending_ik_request = False
            self.get_logger().error(f'ComputeIKAsync 调用失败：{e}')

    def _ik_callback_moveit(self, future):
        """MoveIt IK async callback (fallback path)."""
        self.pending_ik_request = False

        if self.master_slave_mode or self.entering_master_slave:
            return

        try:
            response = future.result()
            if response.error_code.val != 1:
                self.get_logger().debug(f'IK 失败：error_code={response.error_code.val}')
                return

            solution = response.solution.joint_state
            ik_positions = []
            for joint_name in self.joint_names:
                if joint_name in solution.name:
                    idx = solution.name.index(joint_name)
                    ik_positions.append(solution.position[idx])
                else:
                    return

            self._process_ik_result(ik_positions)

        except Exception as e:
            self.get_logger().error(f'IK 回调异常：{e}')

        if self._ik_resend_needed:
            self._ik_resend_needed = False
            self._send_ik_goal_moveit()

    def _send_fixed_rate_ik_command(self):
        """Consume buffered IK result and send motor command at the timer rate.

        With local Pinocchio IK, a fresh solution arrives every tick.
        """
        if self.master_slave_mode or self.entering_master_slave:
            return
        if self.is_going_home:
            return
        if self.zero_torque_mode:
            return

        if self._latest_ik_raw is not None and not self._latest_ik_consumed:
            self._latest_ik_consumed = True

        # 2nd-order critically-damped filter (exact discrete matrix-exponential solution)
        # Continuous: ẍ + 2ω·ẋ + ω²·x = ω²·target   (ζ=1, no overshoot)
        # Exact discrete step avoids Euler instability at any ω·dt value.
        if self._latest_ik_raw is not None:
            if self._ik_smooth_target is None:
                self._ik_smooth_target = list(self._latest_ik_raw)
                self._ik_smooth_vel = [0.0] * len(self._latest_ik_raw)
            else:
                omega = 14.0
                dt = self.dt
                a = omega * dt
                ea = math.exp(-a)
                for i in range(len(self._latest_ik_raw)):
                    err = self._latest_ik_raw[i] - self._ik_smooth_target[i]
                    vel = self._ik_smooth_vel[i]
                    err_new = ea * ((1.0 + a) * err - dt * vel)
                    vel_new = ea * (omega * omega * dt * err + (1.0 - a) * vel)
                    self._ik_smooth_target[i] = self._latest_ik_raw[i] - err_new
                    self._ik_smooth_vel[i] = vel_new

        if self._ik_smooth_target is not None:
            target_positions = list(self._ik_smooth_target)
            self.smoothed_joint_positions = target_positions
            self.publish_ik_debug(target_positions)
        else:
            return

        # Use analytic velocity from 2nd-order filter (much cleaner than numeric differentiation)
        ik_velocities = list(self._ik_smooth_vel) if self._ik_smooth_vel else None
        self.last_ik_joint_positions = target_positions

        if self._master_sdk and self._master_sdk._connected:
            self._master_sdk.JointCtrlList(
                target_positions, duration_ns=10_000_000,
                velocities=ik_velocities)

    def smooth_joint_positions(self, target_positions):
        """Smooth IK joint positions (critical damping + deadzone).
        
        Uses critical damping + position deadzone + asymmetric decel to avoid
        joint jumps and idle oscillations.
        
        Args:
            target_positions: Target joint positions from IK
            
        Returns:
            Smoothed joint positions
        """
        # Initialize on first call
        if self.smoothed_joint_positions is None:
            self.smoothed_joint_positions = list(target_positions)
            self.last_joint_velocities = [0.0] * len(target_positions)
            return target_positions
        
        # Ensure velocity array initialized
        if self.last_joint_velocities is None:
            self.last_joint_velocities = [0.0] * len(target_positions)
        
        smoothed = []
        new_velocities = []
        alpha = self.joint_smoothing_alpha
        max_delta_v = self.max_joint_acceleration * self.dt  # Max velocity change per cycle
        
        # [New] Position deadzone (below this is treated as reached)
        position_deadzone = 0.0005  # 0.5 mrad ≈ 0.03 deg
        
        for i in range(len(target_positions)):
            pos_error = target_positions[i] - self.smoothed_joint_positions[i]
            current_velocity = self.last_joint_velocities[i]
            
            # [Improvement 1] Position deadzone: zero when error and velocity are tiny
            if abs(pos_error) < position_deadzone and abs(current_velocity) < 0.01:
                smoothed.append(self.smoothed_joint_positions[i])
                new_velocities.append(0.0)
                continue
            
            # [Improvement 2] Critical damping: desired_velocity considers current velocity
            # PD-like: v_desired = Kp * pos_error - Kd * current_velocity
            # Here Kp is alpha/dt, Kd is damping coefficient.
            damping = 0.5  # Damping coefficient, slightly underdamped for faster response
            
            # Desired velocity = position error term - damping term
            desired_velocity = alpha * pos_error / self.dt - damping * current_velocity
            
            # [Improvement 3] Asymmetric accel limit: allow 2x accel when decelerating
            velocity_change = desired_velocity - current_velocity
            is_decelerating = (current_velocity * desired_velocity < 0) or \
                              (abs(desired_velocity) < abs(current_velocity))
            
            if is_decelerating:
                effective_max_delta_v = max_delta_v * 2.0  # Double accel when decelerating
            else:
                effective_max_delta_v = max_delta_v
            
            if abs(velocity_change) > effective_max_delta_v:
                desired_velocity = current_velocity + effective_max_delta_v * (1 if velocity_change > 0 else -1)
            
            # Velocity limit
            if abs(desired_velocity) > self.max_joint_velocity:
                desired_velocity = self.max_joint_velocity * (1 if desired_velocity > 0 else -1)
            
            # Compute final position
            final_pos = self.smoothed_joint_positions[i] + desired_velocity * self.dt
            
            smoothed.append(final_pos)
            new_velocities.append(desired_velocity)
        
        self.smoothed_joint_positions = smoothed
        self.last_joint_velocities = new_velocities
        return smoothed
    
    
    def publish_ik_debug(self, positions):
        """Publish IK debug info to /debug/ik_solution (continuous, even without input)"""
        ik_debug_msg = JointState()
        ik_debug_msg.header.stamp = self.get_clock().now().to_msg()
        ik_debug_msg.name = list(self.joint_names_7)
        ik_debug_msg.position = list(positions) + [0.0] * (7 - len(positions))
        self.ik_solution_pub.publish(ik_debug_msg)
    
    def publish_ik_raw_debug(self, positions):
        """Publish raw IK solution (pre-processing) to /debug/ik_raw_solution for comparison"""
        ik_raw_msg = JointState()
        ik_raw_msg.header.stamp = self.get_clock().now().to_msg()
        ik_raw_msg.name = list(self.joint_names_7)
        ik_raw_msg.position = list(positions) + [0.0] * (7 - len(positions))
        self.ik_raw_solution_pub.publish(ik_raw_msg)
    
    def send_cartesian_path_goal(self):
        """Plan Cartesian path with MoveIt via SDK (traditional mode)"""
        if self.is_moving:
            if self.move_start_time is not None:
                elapsed = (self.get_clock().now() - self.move_start_time).nanoseconds / 1e9
                if elapsed > self.move_timeout:
                    self.get_logger().warn(f'运动超时（{elapsed:.1f}s），正在重置状态')
                    self.is_moving = False
                else:
                    return
            else:
                return
        
        if not self._master_sdk or not self._master_sdk._connected:
            self.get_logger().warn('SDK 未连接，跳过笛卡尔路径规划', throttle_duration_sec=5.0)
            return
        
        self.is_moving = True
        self.move_start_time = self.get_clock().now()
        
        waypoint = Pose()
        waypoint.position = self.target_pose.pose.position
        waypoint.orientation = self.target_pose.pose.orientation
        
        self._master_sdk.PlanCartesianPathAsync(
            waypoints=[waypoint],
            avoid_collisions=self.collision_check_active,
            result_callback=self._on_cartesian_complete,
        )
    
    def _on_cartesian_complete(self, success):
        """Cartesian path execution complete callback"""
        self.is_moving = False
        if success:
            self.last_sent_pose = copy.deepcopy(self.target_pose)
        else:
            self.get_logger().debug('笛卡尔路径执行失败')


    def _send_gripper_position(self, angle):
        """Send gripper position via SDK GripperCtrl"""
        if self._master_sdk and self._master_sdk._connected:
            self._master_sdk.GripperCtrl(gripper_angle=float(angle))
        else:
            self.get_logger().warn('SDK 未连接，跳过夹爪控制')


def main(args=None):
    rclpy.init(args=args)
    node = XboxTeleopNode()

    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    if node._master_sdk and node._master_sdk.get_node():
        executor.add_node(node._master_sdk.get_node())
        node.get_logger().info('共享 executor：已添加主臂 SDK 节点')
    if node._slave_sdk and node._slave_sdk.get_node():
        executor.add_node(node._slave_sdk.get_node())
        node.get_logger().info('共享 executor：已添加从臂 SDK 节点')

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


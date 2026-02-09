/**
 * @file rs_a3_hardware.cpp
 * @brief Implementation of ROS2 Control hardware interface for RS-A3 robot arm
 */

#include "rs_a3_hardware/rs_a3_hardware.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_srvs/srv/set_bool.hpp"

namespace rs_a3_hardware
{

RsA3HardwareInterface::RsA3HardwareInterface()
  : can_interface_("can0")
  , host_can_id_(0xFD)
  , position_kp_(60.0)    // 降低Kp减少振荡
  , position_kd_(3.5)     // 增加Kd提高阻尼
  , velocity_limit_(10.0)
  , control_mode_(ControlMode::POSITION)
  , use_mock_hardware_(false)
  , s_curve_enabled_(true)       // 默认启用S曲线规划
  , zero_torque_mode_(false)
  , zero_torque_kd_(1.0)
  , gravity_comp_enabled_(false)
  , gravity_feedforward_ratio_(0.5)  // 默认50%重力补偿前馈
  , use_pinocchio_gravity_(false)    // 默认使用简化重力模型
  , pinocchio_initialized_(false)
  , use_calibrated_inertia_(false)   // 默认不使用标定后的惯性参数
  , spin_thread_running_(false)      // 线程运行标志初始化
  , limit_margin_(0.15)          // 15度开始减速 (~8.6°)
  , limit_stop_margin_(0.02)     // 1度硬停止 (~1.1°)
  , limit_decel_factor_(0.3)     // 减速到30%
  , max_jerk_(50.0)              // 默认最大加加速度 50 rad/s³ (S曲线规划)
{
}

RsA3HardwareInterface::~RsA3HardwareInterface()
{
  // 停止 spin 线程
  spin_thread_running_ = false;
  if (spin_thread_.joinable()) {
    spin_thread_.join();
  }
  on_shutdown(rclcpp_lifecycle::State());
}

hardware_interface::CallbackReturn RsA3HardwareInterface::on_init(
  const hardware_interface::HardwareInfo& info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Parse parameters
  if (info_.hardware_parameters.count("can_interface")) {
    can_interface_ = info_.hardware_parameters.at("can_interface");
  }
  if (info_.hardware_parameters.count("host_can_id")) {
    host_can_id_ = std::stoi(info_.hardware_parameters.at("host_can_id"));
  }
  if (info_.hardware_parameters.count("position_kp")) {
    position_kp_ = std::stod(info_.hardware_parameters.at("position_kp"));
  }
  if (info_.hardware_parameters.count("position_kd")) {
    position_kd_ = std::stod(info_.hardware_parameters.at("position_kd"));
  }
  if (info_.hardware_parameters.count("velocity_limit")) {
    velocity_limit_ = std::stod(info_.hardware_parameters.at("velocity_limit"));
  }
  if (info_.hardware_parameters.count("use_mock_hardware")) {
    use_mock_hardware_ = info_.hardware_parameters.at("use_mock_hardware") == "true";
  }

  // Parse joint configurations
  if (!parseJointConfig(info)) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Resize state and command vectors
  size_t num_joints = joint_configs_.size();
  hw_positions_.resize(num_joints, 0.0);
  hw_velocities_.resize(num_joints, 0.0);
  hw_efforts_.resize(num_joints, 0.0);
  hw_temperatures_.resize(num_joints, 0.0);
  hw_commands_positions_.resize(num_joints, 0.0);
  hw_commands_velocities_.resize(num_joints, 0.0);
  hw_commands_efforts_.resize(num_joints, 0.0);
  
  // 初始化位置命令平滑滤波器（带速度/加速度/加加速度限制 - S曲线规划）
  smoothed_positions_.resize(num_joints, 0.0);
  smoothed_velocities_.resize(num_joints, 0.0);
  smoothed_accelerations_.resize(num_joints, 0.0);  // S曲线规划用
  
  // 初始化速度前馈计算相关变量
  last_cmd_positions_.resize(num_joints, 0.0);       // 上一周期命令位置
  last_hw_commands_positions_.resize(num_joints, 0.0); // 上一帧hw_commands（用于检测命令更新）
  cmd_velocities_.resize(num_joints, 0.0);           // 计算的命令速度
  filtered_cmd_velocities_.resize(num_joints, 0.0);  // 滤波后的命令速度（一阶）
  velocity_ff_stage2_.resize(num_joints, 0.0);       // 速度前馈二阶滤波中间值
  velocity_filter_alpha_ = 0.3;                      // 速度滤波系数
  
  // 默认参数
  smoothing_alpha_ = 0.08;      // 平滑系数（降低使运动更平滑）
  max_velocity_ = 2.0;          // 最大速度 2 rad/s
  max_acceleration_ = 8.0;      // 最大加速度 8 rad/s²
  max_jerk_ = 50.0;             // 最大加加速度 50 rad/s³（S曲线规划）
  control_period_ = 0.005;      // 默认200Hz -> 5ms周期
  first_command_ = true;
  gravity_feedforward_ratio_ = 0.5;  // 默认50%重力补偿前馈
  s_curve_enabled_ = true;      // 默认启用S曲线
  
  // 从参数读取平滑系数
  if (info_.hardware_parameters.count("smoothing_alpha")) {
    smoothing_alpha_ = std::stod(info_.hardware_parameters.at("smoothing_alpha"));
    smoothing_alpha_ = std::clamp(smoothing_alpha_, 0.01, 1.0);
  }
  
  // 从参数读取速度限制
  if (info_.hardware_parameters.count("max_velocity")) {
    max_velocity_ = std::stod(info_.hardware_parameters.at("max_velocity"));
  }
  
  // 从参数读取加速度限制
  if (info_.hardware_parameters.count("max_acceleration")) {
    max_acceleration_ = std::stod(info_.hardware_parameters.at("max_acceleration"));
  }
  
  // 从参数读取加加速度限制（S曲线规划）
  if (info_.hardware_parameters.count("max_jerk")) {
    max_jerk_ = std::stod(info_.hardware_parameters.at("max_jerk"));
  }
  
  // 从参数读取S曲线使能
  if (info_.hardware_parameters.count("s_curve_enabled")) {
    s_curve_enabled_ = info_.hardware_parameters.at("s_curve_enabled") == "true";
  }
  
  // 初始化S曲线生成器（每个关节一个）
  s_curve_generators_.clear();
  for (size_t i = 0; i < num_joints; ++i) {
    s_curve_generators_.push_back(
      std::make_unique<SCurveGenerator>(max_velocity_, max_acceleration_, max_jerk_));
  }
  
  // 从参数读取重力补偿前馈比例
  if (info_.hardware_parameters.count("gravity_feedforward_ratio")) {
    gravity_feedforward_ratio_ = std::stod(info_.hardware_parameters.at("gravity_feedforward_ratio"));
    gravity_feedforward_ratio_ = std::clamp(gravity_feedforward_ratio_, 0.0, 1.0);
  }
  
  // 从参数读取限位保护参数
  if (info_.hardware_parameters.count("limit_margin")) {
    limit_margin_ = std::stod(info_.hardware_parameters.at("limit_margin"));
  }
  if (info_.hardware_parameters.count("limit_stop_margin")) {
    limit_stop_margin_ = std::stod(info_.hardware_parameters.at("limit_stop_margin"));
  }
  if (info_.hardware_parameters.count("limit_decel_factor")) {
    limit_decel_factor_ = std::stod(info_.hardware_parameters.at("limit_decel_factor"));
    limit_decel_factor_ = std::clamp(limit_decel_factor_, 0.0, 1.0);
  }
  
  // 初始化限位保护状态
  joint_at_limit_.resize(num_joints, false);
  limit_warn_counter_.resize(num_joints, 0);

  RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
              "Initialized with %zu joints on %s", num_joints, can_interface_.c_str());
  RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
              "  S-curve: %s, max_vel=%.1f rad/s, max_acc=%.1f rad/s², max_jerk=%.1f rad/s³", 
              s_curve_enabled_ ? "ENABLED" : "DISABLED",
              max_velocity_, max_acceleration_, max_jerk_);
  RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
              "  PID: Kp=%.1f, Kd=%.1f, gravity_feedforward_ratio=%.0f%%", 
              position_kp_, position_kd_, gravity_feedforward_ratio_ * 100.0);
  
  RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
              "Joint limit protection: margin=%.3f rad, stop_margin=%.3f rad, decel_factor=%.2f",
              limit_margin_, limit_stop_margin_, limit_decel_factor_);

  // 初始化调试发布器节点
  debug_node_ = rclcpp::Node::make_shared("rs_a3_hw_debug");
  hw_cmd_pub_ = debug_node_->create_publisher<sensor_msgs::msg::JointState>("/debug/hw_command", 10);
  smoothed_cmd_pub_ = debug_node_->create_publisher<sensor_msgs::msg::JointState>("/debug/smoothed_command", 10);
  gravity_torque_pub_ = debug_node_->create_publisher<sensor_msgs::msg::JointState>("/debug/gravity_torque", 10);
  velocity_ff_pub_ = debug_node_->create_publisher<sensor_msgs::msg::JointState>("/debug/velocity_feedforward", 10);
  temperature_pub_ = debug_node_->create_publisher<sensor_msgs::msg::JointState>("/debug/motor_temperature", 10);
  
  RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
              "Debug publishers created: /debug/hw_command, /debug/smoothed_command, /debug/gravity_torque, /debug/velocity_feedforward, /debug/motor_temperature");

  // ============ 初始化重力补偿参数 ============
  gravity_params_.resize(num_joints);
  for (size_t i = 0; i < num_joints; ++i) {
    gravity_params_[i] = {0.0, 0.0, 0.0};  // 默认为0
  }
  
  // 从参数读取重力补偿（如果有）
  // 格式: gravity_comp_L1_sin, gravity_comp_L1_cos, gravity_comp_L1_offset 等
  std::vector<std::string> joint_prefixes = {"L1", "L2", "L3", "L4", "L5", "L6"};
  for (size_t i = 0; i < num_joints && i < joint_prefixes.size(); ++i) {
    std::string prefix = "gravity_comp_" + joint_prefixes[i];
    if (info_.hardware_parameters.count(prefix + "_sin")) {
      gravity_params_[i].sin_coeff = std::stod(info_.hardware_parameters.at(prefix + "_sin"));
    }
    if (info_.hardware_parameters.count(prefix + "_cos")) {
      gravity_params_[i].cos_coeff = std::stod(info_.hardware_parameters.at(prefix + "_cos"));
    }
    if (info_.hardware_parameters.count(prefix + "_offset")) {
      gravity_params_[i].offset = std::stod(info_.hardware_parameters.at(prefix + "_offset"));
    }
  }
  
  // 检查是否有任何非零重力补偿参数
  for (size_t i = 0; i < num_joints; ++i) {
    if (gravity_params_[i].sin_coeff != 0.0 || 
        gravity_params_[i].cos_coeff != 0.0 || 
        gravity_params_[i].offset != 0.0) {
      gravity_comp_enabled_ = true;
      break;
    }
  }
  
  if (gravity_comp_enabled_) {
    RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
                "Gravity compensation enabled with parameters:");
    for (size_t i = 0; i < num_joints && i < joint_prefixes.size(); ++i) {
      RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
                  "  %s: sin=%.4f, cos=%.4f, offset=%.4f",
                  joint_prefixes[i].c_str(),
                  gravity_params_[i].sin_coeff,
                  gravity_params_[i].cos_coeff,
                  gravity_params_[i].offset);
    }
  }
  
  // 读取零力矩模式Kd参数
  if (info_.hardware_parameters.count("zero_torque_kd")) {
    zero_torque_kd_ = std::stod(info_.hardware_parameters.at("zero_torque_kd"));
  }
  
  // ============ Pinocchio 动力学模型初始化 ============
  // 读取 URDF 路径
  if (info_.hardware_parameters.count("urdf_path")) {
    urdf_path_ = info_.hardware_parameters.at("urdf_path");
    
    // 尝试初始化 Pinocchio
    if (initPinocchioModel(urdf_path_)) {
      // 检查是否启用 Pinocchio 重力补偿
      if (info_.hardware_parameters.count("use_pinocchio_gravity")) {
        use_pinocchio_gravity_ = (info_.hardware_parameters.at("use_pinocchio_gravity") == "true");
      }
      
      // 读取惯性参数缩放因子（用于标定微调）
      inertia_scale_params_.resize(joint_configs_.size());
      for (size_t i = 0; i < joint_configs_.size(); ++i) {
        inertia_scale_params_[i].mass_scale = 1.0;
        inertia_scale_params_[i].com_x_offset = 0.0;
        inertia_scale_params_[i].com_y_offset = 0.0;
        inertia_scale_params_[i].com_z_offset = 0.0;
        
        std::string prefix = "inertia_scale_L" + std::to_string(i + 1);
        if (info_.hardware_parameters.count(prefix + "_mass")) {
          inertia_scale_params_[i].mass_scale = std::stod(info_.hardware_parameters.at(prefix + "_mass"));
        }
        if (info_.hardware_parameters.count(prefix + "_com_x")) {
          inertia_scale_params_[i].com_x_offset = std::stod(info_.hardware_parameters.at(prefix + "_com_x"));
        }
        if (info_.hardware_parameters.count(prefix + "_com_y")) {
          inertia_scale_params_[i].com_y_offset = std::stod(info_.hardware_parameters.at(prefix + "_com_y"));
        }
        if (info_.hardware_parameters.count(prefix + "_com_z")) {
          inertia_scale_params_[i].com_z_offset = std::stod(info_.hardware_parameters.at(prefix + "_com_z"));
        }
      }
      
      // 读取惯性参数配置文件路径
      if (info_.hardware_parameters.count("inertia_config_path")) {
        inertia_config_path_ = info_.hardware_parameters.at("inertia_config_path");
        
        // 加载标定后的惯性参数
        if (loadCalibratedInertia(inertia_config_path_)) {
          // 应用到 Pinocchio 模型
          applyCalibratedInertiaToModel();
          use_calibrated_inertia_ = true;
          RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
                      "Calibrated inertia parameters loaded from: %s", inertia_config_path_.c_str());
        }
      }
      
      RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
                  "Pinocchio gravity compensation: %s (calibrated inertia: %s)", 
                  use_pinocchio_gravity_ ? "ENABLED" : "DISABLED",
                  use_calibrated_inertia_ ? "YES" : "NO");
    }
  }
  
  // 创建零力矩模式服务
  zero_torque_srv_ = debug_node_->create_service<std_srvs::srv::SetBool>(
    "/rs_a3/set_zero_torque_mode",
    std::bind(&RsA3HardwareInterface::zeroTorqueModeCallback, this,
              std::placeholders::_1, std::placeholders::_2));
  
  // 启动单独的线程来处理服务回调
  spin_thread_running_ = true;
  spin_thread_ = std::thread([this]() {
    RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
                "Debug node spin thread started");
    while (spin_thread_running_ && rclcpp::ok()) {
      rclcpp::spin_some(debug_node_);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));  // 100Hz
    }
    RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
                "Debug node spin thread stopped");
  });
  
  RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
              "Zero torque mode service created: /rs_a3/set_zero_torque_mode (Kd=%.1f)", zero_torque_kd_);

  return hardware_interface::CallbackReturn::SUCCESS;
}

bool RsA3HardwareInterface::parseJointConfig(const hardware_interface::HardwareInfo& info)
{
  joint_configs_.clear();
  
  for (const auto& joint : info.joints) {
    JointConfig config;
    config.name = joint.name;
    
    // Parse motor_id
    if (joint.parameters.count("motor_id")) {
      config.motor_id = std::stoi(joint.parameters.at("motor_id"));
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("RsA3HardwareInterface"),
                   "Joint %s missing motor_id parameter", joint.name.c_str());
      return false;
    }
    
    // Parse motor_type
    if (joint.parameters.count("motor_type")) {
      std::string type_str = joint.parameters.at("motor_type");
      if (type_str == "RS00") {
        config.motor_type = MotorType::RS00;
      } else if (type_str == "RS05") {
        config.motor_type = MotorType::RS05;
      } else {
        RCLCPP_ERROR(rclcpp::get_logger("RsA3HardwareInterface"),
                     "Unknown motor_type %s for joint %s", type_str.c_str(), joint.name.c_str());
        return false;
      }
    } else {
      // Default based on motor_id: 1-3 = RS00, 4-6 = RS05
      config.motor_type = (config.motor_id <= 3) ? MotorType::RS00 : MotorType::RS05;
    }
    
    // Parse offset
    if (joint.parameters.count("position_offset")) {
      config.position_offset = std::stod(joint.parameters.at("position_offset"));
    } else {
      config.position_offset = 0.0;
    }
    
    // Parse direction
    if (joint.parameters.count("direction")) {
      config.direction = std::stod(joint.parameters.at("direction"));
    } else {
      config.direction = 1.0;
    }
    
    // Parse joint limits (从参数或command_interface获取)
    config.lower_limit = -6.28;  // 默认±360°
    config.upper_limit = 6.28;
    
    if (joint.parameters.count("lower_limit")) {
      config.lower_limit = std::stod(joint.parameters.at("lower_limit"));
    }
    if (joint.parameters.count("upper_limit")) {
      config.upper_limit = std::stod(joint.parameters.at("upper_limit"));
    }
    
    // 尝试从command_interface获取限位（ros2_control标准方式）
    for (const auto& cmd_if : joint.command_interfaces) {
      if (cmd_if.name == "position") {
        if (cmd_if.min != cmd_if.max) {  // 有效的限位
          config.lower_limit = std::stod(cmd_if.min);
          config.upper_limit = std::stod(cmd_if.max);
        }
        break;
      }
    }
    
    // Parse joint-specific Kp/Kd (0表示使用全局值)
    config.kp = 0.0;
    config.kd = 0.0;
    if (joint.parameters.count("kp")) {
      config.kp = std::stod(joint.parameters.at("kp"));
    }
    if (joint.parameters.count("kd")) {
      config.kd = std::stod(joint.parameters.at("kd"));
    }
    
    joint_configs_.push_back(config);
    
    if (config.kp > 0.0 || config.kd > 0.0) {
      RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
                  "Joint %s: motor_id=%d, type=%s, dir=%.1f, limits=[%.1f°~%.1f°], Kp=%.0f, Kd=%.1f",
                  config.name.c_str(), config.motor_id,
                  config.motor_type == MotorType::RS00 ? "RS00" : "RS05",
                  config.direction,
                  config.lower_limit * 180.0 / M_PI, config.upper_limit * 180.0 / M_PI,
                  config.kp, config.kd);
    } else {
      RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
                  "Joint %s: motor_id=%d, type=%s, dir=%.1f, limits=[%.1f°~%.1f°]",
                  config.name.c_str(), config.motor_id,
                  config.motor_type == MotorType::RS00 ? "RS00" : "RS05",
                  config.direction,
                  config.lower_limit * 180.0 / M_PI, config.upper_limit * 180.0 / M_PI);
    }
  }
  
  return true;
}

hardware_interface::CallbackReturn RsA3HardwareInterface::on_configure(
  const rclcpp_lifecycle::State& /*previous_state*/)
{
  if (use_mock_hardware_) {
    RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
                "Using mock hardware - skipping CAN initialization");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  // Create and initialize CAN driver
  can_driver_ = std::make_unique<RobstrideCanDriver>(can_interface_, host_can_id_);
  
  if (!can_driver_->init()) {
    RCLCPP_ERROR(rclcpp::get_logger("RsA3HardwareInterface"),
                 "Failed to initialize CAN driver on %s", can_interface_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Set motor types for each joint (with delay to avoid CAN buffer overflow)
  for (const auto& config : joint_configs_) {
    can_driver_->setMotorType(config.motor_id, config.motor_type);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));  // 添加延迟
  }

  // Start receive thread
  can_driver_->startReceiveThread();

  RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"), "Hardware configured on %s", can_interface_.c_str());
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RsA3HardwareInterface::on_cleanup(
  const rclcpp_lifecycle::State& /*previous_state*/)
{
  if (can_driver_) {
    can_driver_->stopReceiveThread();
    can_driver_->close();
    can_driver_.reset();
  }

  RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"), "Hardware cleaned up");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RsA3HardwareInterface::on_activate(
  const rclcpp_lifecycle::State& /*previous_state*/)
{
  if (use_mock_hardware_) {
    RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
                "Mock hardware activated");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  // ============ 步骤0: 清除所有电机故障 ============
  RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
              "Clearing motor faults...");
  for (const auto& config : joint_configs_) {
    can_driver_->disableMotor(config.motor_id, true);  // clear_fault=true
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  
  // ============ 步骤1: 使能所有电机（用Kp=0，不控制位置）============
  // 电机未使能时不发送反馈，所以必须先使能
  RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
              "Enabling motors in soft mode (Kp=0)...");
  
  for (size_t i = 0; i < joint_configs_.size(); ++i) {
    const auto& config = joint_configs_[i];
    
    // 1. 先停止电机（不清除故障，因为已经清除过了）
    can_driver_->disableMotor(config.motor_id, false);
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    
    // 2. 设置运行模式为运控模式 (run_mode = 0)
    if (!can_driver_->setRunMode(config.motor_id, RunMode::MOTION_CONTROL)) {
      RCLCPP_ERROR(rclcpp::get_logger("RsA3HardwareInterface"),
                   "Failed to set Motion Control mode for motor %d", config.motor_id);
      return hardware_interface::CallbackReturn::ERROR;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    
    // 3. 使能电机
    if (!can_driver_->enableMotor(config.motor_id)) {
      RCLCPP_ERROR(rclcpp::get_logger("RsA3HardwareInterface"),
                   "Failed to enable motor %d", config.motor_id);
      return hardware_interface::CallbackReturn::ERROR;
    }
    
    // 4. 【关键】发送 Kp=0 的命令，使电机处于柔软状态，不追踪任何位置
    //    这样即使发送任意位置命令，电机也不会移动
    can_driver_->sendMotionControl(
        config.motor_id,
        config.motor_type,
        0.0,          // 位置无所谓，因为Kp=0
        0.0,          // velocity = 0
        0.0,          // Kp = 0 (不追踪位置!)
        4.0,          // Kd = 4.0 (增加阻尼防止晃动)
        0.0           // torque = 0
    );
    
    RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
                "Motor %d enabled in soft mode (Kp=0, Kd=4)", config.motor_id);
    
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  // ============ 步骤2: 开环控制 - 使用默认位置(0)，不等待反馈 ============
  RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
              "Open-loop mode: Using default position (0.0) for all joints - skipping feedback");
  
  std::vector<double> initial_positions(joint_configs_.size(), 0.0);
  
  for (size_t i = 0; i < joint_configs_.size(); ++i) {
    const auto& config = joint_configs_[i];
    RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
                "Motor %d initial position: 0.0 rad (open-loop)", config.motor_id);
  }

  // ============ 步骤3: 用真实位置发送保持命令（切换到正常控制）============
  RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
              "Switching to position hold mode...");
  
  for (size_t i = 0; i < joint_configs_.size(); ++i) {
    const auto& config = joint_configs_[i];
    
    // 初始化状态变量
    hw_positions_[i] = initial_positions[i];
    hw_commands_positions_[i] = initial_positions[i];
    smoothed_positions_[i] = initial_positions[i];
    smoothed_velocities_[i] = 0.0;
    smoothed_accelerations_[i] = 0.0;
    
    // 初始化S曲线生成器状态
    if (s_curve_enabled_ && i < s_curve_generators_.size()) {
      s_curve_generators_[i]->initialize(initial_positions[i], 0.0, 0.0);
    }
    
    // 计算电机坐标系下的位置
    double motor_pos = initial_positions[i] * config.direction + config.position_offset;
    
    // 发送保持当前位置的命令（正常Kp/Kd）
    can_driver_->sendMotionControl(
        config.motor_id,
        config.motor_type,
        motor_pos,        // 使用真实位置
        0.0,              // velocity = 0
        position_kp_,     // 正常Kp
        position_kd_,     // 正常Kd
        0.0               // torque = 0
    );
    
    RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
                "Motor %d holding at %.4f rad (Kp=%.0f, Kd=%.1f)", 
                config.motor_id, initial_positions[i], position_kp_, position_kd_);
    
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
  }
  
  RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
              "All %zu motors initialized successfully", joint_configs_.size());
  
  first_command_ = false;

  RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"), 
              "Hardware activated with CSP Position mode");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RsA3HardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State& /*previous_state*/)
{
  if (!use_mock_hardware_ && can_driver_) {
    // Disable all motors
    for (const auto& config : joint_configs_) {
      can_driver_->disableMotor(config.motor_id);
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"), "Hardware deactivated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RsA3HardwareInterface::on_shutdown(
  const rclcpp_lifecycle::State& /*previous_state*/)
{
  on_deactivate(rclcpp_lifecycle::State());
  on_cleanup(rclcpp_lifecycle::State());
  
  RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"), "Hardware shutdown");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RsA3HardwareInterface::on_error(
  const rclcpp_lifecycle::State& /*previous_state*/)
{
  if (!use_mock_hardware_ && can_driver_) {
    // Emergency stop - disable all motors and clear faults
    for (const auto& config : joint_configs_) {
      can_driver_->disableMotor(config.motor_id, true);
    }
  }

  RCLCPP_ERROR(rclcpp::get_logger("RsA3HardwareInterface"), "Hardware error occurred");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> RsA3HardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  
  for (size_t i = 0; i < joint_configs_.size(); ++i) {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        joint_configs_[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        joint_configs_[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        joint_configs_[i].name, hardware_interface::HW_IF_EFFORT, &hw_efforts_[i]));
    // 添加温度状态接口
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        joint_configs_[i].name, "temperature", &hw_temperatures_[i]));
  }
  
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> RsA3HardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  
  for (size_t i = 0; i < joint_configs_.size(); ++i) {
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        joint_configs_[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_positions_[i]));
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        joint_configs_[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_velocities_[i]));
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        joint_configs_[i].name, hardware_interface::HW_IF_EFFORT, &hw_commands_efforts_[i]));
  }
  
  return command_interfaces;
}

hardware_interface::return_type RsA3HardwareInterface::read(
  const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
  // ============ 从电机实际读取所有状态 ============
  if (!use_mock_hardware_ && can_driver_) {
    for (size_t i = 0; i < joint_configs_.size(); ++i) {
      const auto& config = joint_configs_[i];
      auto feedback = can_driver_->getMotorFeedback(config.motor_id);
      
      if (feedback.is_valid) {
        // 从电机坐标系转换到关节坐标系
        // motor_pos = joint_pos * direction + offset
        // joint_pos = (motor_pos - offset) / direction = (motor_pos - offset) * direction
        hw_positions_[i] = (feedback.position - config.position_offset) * config.direction;
        hw_velocities_[i] = feedback.velocity * config.direction;
        hw_efforts_[i] = feedback.torque * config.direction;
        hw_temperatures_[i] = feedback.temperature;  // 电机温度 (°C)
      }
    }
  } else {
    // Mock模式：使用命令值
    for (size_t i = 0; i < joint_configs_.size(); ++i) {
      hw_positions_[i] = smoothed_positions_[i];
      hw_velocities_[i] = smoothed_velocities_[i];
      hw_efforts_[i] = 0.0;
      hw_temperatures_[i] = 25.0;  // Mock 温度
    }
  }
  
  // 发布温度数据（降低频率，每 50 次读取发布一次）
  static int temp_pub_counter = 0;
  if (++temp_pub_counter >= 50) {
    temp_pub_counter = 0;
    sensor_msgs::msg::JointState temp_msg;
    temp_msg.header.stamp = debug_node_->now();
    for (size_t i = 0; i < joint_configs_.size(); ++i) {
      temp_msg.name.push_back(joint_configs_[i].name);
      temp_msg.effort.push_back(hw_temperatures_[i]);  // 使用 effort 字段存储温度
    }
    temperature_pub_->publish(temp_msg);
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RsA3HardwareInterface::write(
  const rclcpp::Time& /*time*/, const rclcpp::Duration& period)
{
  static int write_counter = 0;
  
  if (use_mock_hardware_) {
    return hardware_interface::return_type::OK;
  }

  if (!can_driver_ || !can_driver_->isConnected()) {
    return hardware_interface::return_type::ERROR;
  }

  // 获取实际控制周期
  double dt = period.seconds();
  if (dt <= 0.0 || dt > 0.1) {
    dt = control_period_;  // 使用默认周期
  }

  // 【平滑滤波】第一次命令时直接设置，避免从0开始平滑
  if (first_command_) {
    for (size_t i = 0; i < joint_configs_.size(); ++i) {
      smoothed_positions_[i] = hw_commands_positions_[i];
      smoothed_velocities_[i] = 0.0;
      smoothed_accelerations_[i] = 0.0;
      
      // 初始化速度前馈相关变量，避免第一次计算产生跳变
      last_cmd_positions_[i] = hw_commands_positions_[i];
      filtered_cmd_velocities_[i] = 0.0;
      velocity_ff_stage2_[i] = 0.0;  // 二阶滤波中间值
      
      // 初始化S曲线生成器
      if (s_curve_enabled_ && i < s_curve_generators_.size()) {
        s_curve_generators_[i]->initialize(hw_commands_positions_[i], 0.0, 0.0);
      }
    }
    first_command_ = false;
    RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
                "First command received, initializing positions (S-curve: %s)",
                s_curve_enabled_ ? "ENABLED" : "DISABLED");
  }
  
  // ============ 【整体限位保护】已禁用 ============
  bool global_stop = false;
  
  // ============ 检测命令是否更新（解决IK跳帧导致的速度跳变）============
  bool command_updated = false;
  for (size_t i = 0; i < joint_configs_.size(); ++i) {
    if (std::abs(hw_commands_positions_[i] - last_hw_commands_positions_[i]) > 1e-8) {
      command_updated = true;
      break;
    }
  }
  // 更新记录
  for (size_t i = 0; i < joint_configs_.size(); ++i) {
    last_hw_commands_positions_[i] = hw_commands_positions_[i];
  }
  
  // ============ 预计算 Pinocchio 重力补偿向量（如果启用）============
  std::vector<double> pinocchio_gravity_torques;
  if (use_pinocchio_gravity_ && pinocchio_initialized_) {
    pinocchio_gravity_torques = computePinocchioGravity(hw_positions_);
  }
  
  // 使用运控模式发送命令
  for (size_t i = 0; i < joint_configs_.size(); ++i) {
    const auto& config = joint_configs_[i];
    
    // ============ 位置命令处理 ============
    double new_position = hw_commands_positions_[i];
    
    // 【修正】先计算速度（在更新位置之前！）
    double raw_cmd_velocity = (new_position - smoothed_positions_[i]) / dt;
    
    // 更新状态（用于read()反馈和调试）
    double new_velocity = raw_cmd_velocity;
    double new_acceleration = (new_velocity - smoothed_velocities_[i]) / dt;
    smoothed_positions_[i] = new_position;
    smoothed_velocities_[i] = new_velocity;
    smoothed_accelerations_[i] = new_acceleration;
    
    // 【关节限位保护】已禁用
    double limit_factor = 1.0;
    (void)limit_factor;
    (void)global_stop;
    
    // 将关节坐标转换为电机坐标
    double cmd_position = smoothed_positions_[i] * config.direction + config.position_offset;
    
    // Clamp position to motor valid range
    auto params = getMotorParams(config.motor_type);
    cmd_position = std::clamp(cmd_position, params.p_min, params.p_max);
    
    // ============ 计算速度前馈（根据命令是否更新决定计算方式）============
    double filtered_velocity;
    
    if (command_updated) {
      // 命令已更新，正常计算速度前馈
      // 位置差死区：微小速度不产生输出
      const double velocity_threshold = 0.001;  // 0.001 rad/s
      double cmd_velocity;
      if (std::abs(raw_cmd_velocity) < velocity_threshold) {
        cmd_velocity = 0.0;
      } else {
        cmd_velocity = raw_cmd_velocity;
      }
      
      // 限制速度在合理范围内
      cmd_velocity = std::clamp(cmd_velocity, -velocity_limit_, velocity_limit_);
      
      // 二阶滤波
      double alpha1 = 0.1;  // 一阶滤波
      double first_stage = alpha1 * cmd_velocity + (1.0 - alpha1) * filtered_cmd_velocities_[i];
      filtered_cmd_velocities_[i] = first_stage;
      
      double alpha2 = 0.15;  // 二阶滤波
      filtered_velocity = alpha2 * first_stage + (1.0 - alpha2) * velocity_ff_stage2_[i];
      
      // 加速度限制
      double max_velocity_change = max_acceleration_ * dt;
      double velocity_change = filtered_velocity - velocity_ff_stage2_[i];
      if (std::abs(velocity_change) > max_velocity_change) {
        filtered_velocity = velocity_ff_stage2_[i] + 
                            max_velocity_change * (velocity_change > 0 ? 1.0 : -1.0);
      }
    } else {
      // 命令未更新（IK跳帧），速度平滑衰减而不是重新计算
      filtered_velocity = velocity_ff_stage2_[i] * 0.95;  // 衰减5%
      // 同时更新一阶滤波状态，保持一致性
      filtered_cmd_velocities_[i] *= 0.95;
    }
    
    // 速度死区：极小速度直接归零
    const double velocity_deadzone = 0.005;  // 0.005 rad/s
    if (std::abs(filtered_velocity) < velocity_deadzone) {
      filtered_velocity = 0.0;
    }
    
    velocity_ff_stage2_[i] = filtered_velocity;
    
    // 转换为电机坐标系的速度
    double motor_cmd_velocity = velocity_ff_stage2_[i] * config.direction;
    
    // Debug: 定期输出日志
    if (write_counter % 1000 == 0 && i == 0) {
      RCLCPP_DEBUG(rclcpp::get_logger("RsA3HardwareInterface"),
                  "[Velocity FF] raw=%.3f, filtered=%.3f", 
                  raw_cmd_velocity, filtered_velocity);
    }
    
    // ============ 计算重力补偿力矩（按比例作为前馈）============
    double gravity_torque = 0.0;
    if (gravity_comp_enabled_) {
      if (use_pinocchio_gravity_ && pinocchio_initialized_ && i < pinocchio_gravity_torques.size()) {
        // 使用 Pinocchio 完整动力学计算（考虑所有关节级联效应）
        gravity_torque = pinocchio_gravity_torques[i] * gravity_feedforward_ratio_;
      } else {
        // 使用简化模型（单关节独立）
        gravity_torque = computeGravityTorque(i, hw_positions_[i]) * gravity_feedforward_ratio_;
      }
    }
    
    // ============ 根据模式选择控制参数 ============
    double motor_kp, motor_kd, cmd_torque;
    double final_cmd_position;
    
    if (zero_torque_mode_) {
      // 零力矩模式: Kp=0, Kd=阻尼, 仅重力补偿（100%用于零力矩模式）
      motor_kp = 0.0;
      motor_kd = std::clamp(zero_torque_kd_, 0.0, 5.0);
      
      // 零力矩模式使用100%重力补偿
      if (use_pinocchio_gravity_ && pinocchio_initialized_ && i < pinocchio_gravity_torques.size()) {
        cmd_torque = pinocchio_gravity_torques[i];  // Pinocchio 完整动力学
      } else {
        cmd_torque = computeGravityTorque(i, hw_positions_[i]);  // 简化模型
      }
      
      // 位置发送当前实际位置（不产生位置误差）
      final_cmd_position = hw_positions_[i] * config.direction + config.position_offset;
    } else {
      // 正常位置控制模式 - 使用关节独立Kp/Kd（如果设置了），否则使用全局值
      double joint_kp = (config.kp > 0.0) ? config.kp : position_kp_;
      double joint_kd = (config.kd > 0.0) ? config.kd : position_kd_;
      motor_kp = std::clamp(joint_kp, 0.0, 500.0);
      motor_kd = std::clamp(joint_kd, 0.0, 5.0);
      cmd_torque = gravity_torque;  // 按gravity_feedforward_ratio_比例的重力补偿前馈
      final_cmd_position = cmd_position;
    }
    
    // 计算最终发送的速度前馈（零力矩模式下发送0速度）
    double final_cmd_velocity = zero_torque_mode_ ? 0.0 : motor_cmd_velocity;
    
    if (!can_driver_->sendMotionControl(
          config.motor_id,
          config.motor_type,
          final_cmd_position,
          final_cmd_velocity,  // 速度前馈（位置差分计算）
          motor_kp,
          motor_kd,
          cmd_torque // 重力补偿前馈力矩
        )) {
      // 使用静态计数器替代 THROTTLE 避免 Clock 问题
      static int warn_counter = 0;
      if (warn_counter++ % 1000 == 0) {
        RCLCPP_WARN(rclcpp::get_logger("RsA3HardwareInterface"),
          "Failed to send motion control command to motor %d", config.motor_id);
      }
    }
    
    // 添加帧间微延迟，防止CAN缓冲区拥塞（6帧×50μs=300μs，远小于5ms控制周期）
    usleep(50);
  }

  // 注意：服务回调现在在单独的 spin_thread_ 中处理，无需在此调用 spin_some

  // 发布调试信息（每10次发布一次，约20Hz）
  if (write_counter % 10 == 0 && debug_node_ && hw_cmd_pub_ && smoothed_cmd_pub_) {
    auto now = debug_node_->get_clock()->now();
    
    // 发布控制器命令位置
    sensor_msgs::msg::JointState hw_cmd_msg;
    hw_cmd_msg.header.stamp = now;
    for (const auto& config : joint_configs_) {
      hw_cmd_msg.name.push_back(config.name);
    }
    hw_cmd_msg.position = hw_commands_positions_;
    hw_cmd_pub_->publish(hw_cmd_msg);
    
    // 发布平滑后的命令位置（实际发给电机的）
    sensor_msgs::msg::JointState smoothed_msg;
    smoothed_msg.header.stamp = now;
    smoothed_msg.name = hw_cmd_msg.name;
    smoothed_msg.position = smoothed_positions_;
    smoothed_msg.velocity = smoothed_velocities_;
    smoothed_cmd_pub_->publish(smoothed_msg);
    
    // 发布重力补偿力矩
    if (gravity_comp_enabled_ && gravity_torque_pub_) {
      sensor_msgs::msg::JointState gravity_msg;
      gravity_msg.header.stamp = now;
      gravity_msg.name = hw_cmd_msg.name;
      for (size_t i = 0; i < joint_configs_.size(); ++i) {
        gravity_msg.effort.push_back(computeGravityTorque(i, hw_positions_[i]));
      }
      gravity_torque_pub_->publish(gravity_msg);
    }
  }
  
  // 【改进】速度前馈单独高频发布（每帧200Hz，用于高频分析）
  if (velocity_ff_pub_ && debug_node_) {
    auto now = debug_node_->get_clock()->now();
    sensor_msgs::msg::JointState velocity_ff_msg;
    velocity_ff_msg.header.stamp = now;
    for (const auto& config : joint_configs_) {
      velocity_ff_msg.name.push_back(config.name);
    }
    velocity_ff_msg.velocity = velocity_ff_stage2_;  // 使用二阶滤波后的速度
    velocity_ff_pub_->publish(velocity_ff_msg);
  }

  write_counter++;
  return hardware_interface::return_type::OK;
}

double RsA3HardwareInterface::computeGravityTorque(size_t joint_idx, double position)
{
  if (joint_idx >= gravity_params_.size()) {
    return 0.0;
  }
  
  const auto& params = gravity_params_[joint_idx];
  return params.sin_coeff * std::sin(position)
       + params.cos_coeff * std::cos(position)
       + params.offset;
}

void RsA3HardwareInterface::zeroTorqueModeCallback(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  zero_torque_mode_ = request->data;
  
  if (zero_torque_mode_) {
    RCLCPP_WARN(rclcpp::get_logger("RsA3HardwareInterface"),
                "ZERO TORQUE MODE ENABLED! Kp=0, Kd=%.1f, Gravity Comp=%s",
                zero_torque_kd_, gravity_comp_enabled_ ? "ON" : "OFF");
    response->message = "Zero torque mode enabled - robot can be manually moved";
  } else {
    RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
                "Zero torque mode disabled - returning to position control");
    response->message = "Zero torque mode disabled - position control active";
  }
  
  response->success = true;
}

double RsA3HardwareInterface::computeLimitProtectionFactor(
  size_t joint_idx, double current_pos, double target_pos)
{
  if (joint_idx >= joint_configs_.size()) {
    return 1.0;
  }
  
  const auto& config = joint_configs_[joint_idx];
  double lower = config.lower_limit;
  double upper = config.upper_limit;
  
  // 计算到边界的距离
  double dist_to_lower = current_pos - lower;
  double dist_to_upper = upper - current_pos;
  
  // 确定运动方向
  double motion_dir = target_pos - current_pos;
  
  // 选择相关边界距离
  double relevant_dist;
  if (motion_dir < 0) {
    // 向下限移动
    relevant_dist = dist_to_lower;
  } else if (motion_dir > 0) {
    // 向上限移动
    relevant_dist = dist_to_upper;
  } else {
    return 1.0;  // 没有运动
  }
  
  // 如果在限位边界内，计算减速系数
  if (relevant_dist < limit_margin_) {
    if (relevant_dist <= limit_stop_margin_) {
      // 硬停止区域
      return 0.0;
    }
    // 线性减速区域: 从limit_decel_factor_到1.0
    double ratio = (relevant_dist - limit_stop_margin_) / (limit_margin_ - limit_stop_margin_);
    return limit_decel_factor_ + (1.0 - limit_decel_factor_) * ratio;
  }
  
  return 1.0;
}

bool RsA3HardwareInterface::applyJointLimitProtection(size_t joint_idx, double& target_pos)
{
  if (joint_idx >= joint_configs_.size()) {
    return false;
  }
  
  const auto& config = joint_configs_[joint_idx];
  double lower = config.lower_limit;
  double upper = config.upper_limit;
  
  bool hit_limit = false;
  
  // 检查并钳位目标位置
  if (target_pos < lower + limit_stop_margin_) {
    target_pos = lower + limit_stop_margin_;
    hit_limit = true;
  } else if (target_pos > upper - limit_stop_margin_) {
    target_pos = upper - limit_stop_margin_;
    hit_limit = true;
  }
  
  // 限位警告（每100次触发打印一次，防止刷屏）
  if (hit_limit) {
    if (!joint_at_limit_[joint_idx]) {
      joint_at_limit_[joint_idx] = true;
      RCLCPP_WARN(rclcpp::get_logger("RsA3HardwareInterface"),
                  "⚠️ Joint %s reached limit! pos=%.3f rad (%.1f°), limits=[%.2f, %.2f]",
                  config.name.c_str(), target_pos, target_pos * 180.0 / M_PI,
                  lower, upper);
    }
    limit_warn_counter_[joint_idx]++;
    if (limit_warn_counter_[joint_idx] % 500 == 0) {
      RCLCPP_WARN(rclcpp::get_logger("RsA3HardwareInterface"),
                  "Joint %s still at limit (count=%d)", 
                  config.name.c_str(), limit_warn_counter_[joint_idx]);
    }
  } else {
    if (joint_at_limit_[joint_idx]) {
      joint_at_limit_[joint_idx] = false;
      limit_warn_counter_[joint_idx] = 0;
      RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
                  "✓ Joint %s left limit zone", config.name.c_str());
    }
  }
  
  return hit_limit;
}

// ============ Pinocchio 动力学函数实现 ============

bool RsA3HardwareInterface::initPinocchioModel(const std::string& urdf_path)
{
  try {
    // 从 URDF 文件构建模型
    pinocchio::urdf::buildModel(urdf_path, pinocchio_model_);
    
    // 创建数据对象
    pinocchio_data_ = pinocchio::Data(pinocchio_model_);
    
    pinocchio_initialized_ = true;
    
    RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
                "Pinocchio model initialized successfully:");
    RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
                "  - Model name: %s", pinocchio_model_.name.c_str());
    RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
                "  - Number of joints: %d", pinocchio_model_.njoints);
    RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
                "  - Number of DOFs: %d", pinocchio_model_.nv);
    
    // 打印关节信息
    for (int i = 1; i < pinocchio_model_.njoints; ++i) {
      RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
                  "  - Joint %d: %s", i, pinocchio_model_.names[i].c_str());
    }
    
    return true;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("RsA3HardwareInterface"),
                 "Failed to initialize Pinocchio model from %s: %s",
                 urdf_path.c_str(), e.what());
    pinocchio_initialized_ = false;
    return false;
  }
}

std::vector<double> RsA3HardwareInterface::computePinocchioGravity(
  const std::vector<double>& positions)
{
  std::vector<double> gravity_torques(joint_configs_.size(), 0.0);
  
  if (!pinocchio_initialized_ || positions.size() != joint_configs_.size()) {
    return gravity_torques;
  }
  
  try {
    // 创建位置向量 q（Pinocchio 格式）
    // 注意：需要根据实际的关节映射来填充
    Eigen::VectorXd q = Eigen::VectorXd::Zero(pinocchio_model_.nq);
    
    // 映射硬件关节到 Pinocchio 模型关节
    // 假设顺序一致，根据实际 URDF 调整
    for (size_t i = 0; i < std::min(positions.size(), static_cast<size_t>(pinocchio_model_.nq)); ++i) {
      // 应用方向和偏移（反向转换，因为 Pinocchio 使用 URDF 坐标系）
      q[i] = positions[i];
    }
    
    // 创建零速度和零加速度
    Eigen::VectorXd v = Eigen::VectorXd::Zero(pinocchio_model_.nv);
    Eigen::VectorXd a = Eigen::VectorXd::Zero(pinocchio_model_.nv);
    
    // 使用 RNEA（递归牛顿-欧拉算法）计算逆动力学
    // 当 v=0, a=0 时，结果就是重力补偿力矩
    Eigen::VectorXd tau = pinocchio::rnea(pinocchio_model_, pinocchio_data_, q, v, a);
    
    // 将结果映射回硬件关节
    for (size_t i = 0; i < std::min(gravity_torques.size(), static_cast<size_t>(tau.size())); ++i) {
      // 如果使用标定后的惯性参数，跳过缩放因子（已在模型中应用）
      // 仅当使用 URDF 默认值时才应用缩放因子
      double scale = 1.0;
      if (!use_calibrated_inertia_ && i < inertia_scale_params_.size()) {
        scale = inertia_scale_params_[i].mass_scale;
      }
      
      // 应用关节方向（与硬件接口一致）
      gravity_torques[i] = tau[i] * scale * joint_configs_[i].direction;
    }
    
  } catch (const std::exception& e) {
    RCLCPP_WARN_THROTTLE(rclcpp::get_logger("RsA3HardwareInterface"),
                          *debug_node_->get_clock(), 5000,
                          "Pinocchio gravity computation error: %s", e.what());
  }
  
  return gravity_torques;
}

bool RsA3HardwareInterface::loadCalibratedInertia(const std::string& config_path)
{
  try {
    // 简单的 YAML 解析（不依赖外部库）
    std::ifstream file(config_path);
    if (!file.is_open()) {
      RCLCPP_WARN(rclcpp::get_logger("RsA3HardwareInterface"),
                  "Cannot open inertia config file: %s", config_path.c_str());
      return false;
    }
    
    // 初始化参数容器
    calibrated_inertia_params_.resize(6);  // L1-L6
    
    // 从 Pinocchio 模型读取默认值
    for (size_t i = 0; i < 6 && i + 1 < static_cast<size_t>(pinocchio_model_.nbodies); ++i) {
      const auto& inertia = pinocchio_model_.inertias[i + 1];  // 跳过 universe
      calibrated_inertia_params_[i].mass = inertia.mass();
      calibrated_inertia_params_[i].com_x = inertia.lever()[0];
      calibrated_inertia_params_[i].com_y = inertia.lever()[1];
      calibrated_inertia_params_[i].com_z = inertia.lever()[2];
    }
    
    // 解析 YAML（简化版，按行解析）
    std::string line;
    std::string current_joint;
    bool in_inertia_params = false;
    bool use_calibrated = false;
    
    while (std::getline(file, line)) {
      // 跳过空行和注释
      size_t pos = line.find_first_not_of(" \t");
      if (pos == std::string::npos || line[pos] == '#') continue;
      
      // 检查 use_calibrated_params
      if (line.find("use_calibrated_params:") != std::string::npos) {
        use_calibrated = (line.find("true") != std::string::npos);
        continue;
      }
      
      // 检查 inertia_params 节
      if (line.find("inertia_params:") != std::string::npos) {
        in_inertia_params = true;
        continue;
      }
      
      if (!in_inertia_params) continue;
      
      // 检查关节名称 (L2:, L3:, etc.)
      for (int j = 2; j <= 6; ++j) {
        std::string joint_key = "L" + std::to_string(j) + ":";
        if (line.find(joint_key) != std::string::npos && 
            line.find("mass") == std::string::npos &&
            line.find("com") == std::string::npos) {
          current_joint = "L" + std::to_string(j);
          break;
        }
      }
      
      // 解析 mass
      if (!current_joint.empty() && line.find("mass:") != std::string::npos) {
        size_t colon_pos = line.find("mass:");
        std::string value_str = line.substr(colon_pos + 5);
        // 去掉注释
        size_t comment_pos = value_str.find('#');
        if (comment_pos != std::string::npos) {
          value_str = value_str.substr(0, comment_pos);
        }
        // 去掉空格
        value_str.erase(std::remove_if(value_str.begin(), value_str.end(), ::isspace), value_str.end());
        
        int joint_idx = std::stoi(current_joint.substr(1)) - 1;
        if (joint_idx >= 0 && joint_idx < 6) {
          calibrated_inertia_params_[joint_idx].mass = std::stod(value_str);
        }
      }
      
      // 解析 com
      if (!current_joint.empty() && line.find("com:") != std::string::npos) {
        size_t bracket_start = line.find('[');
        size_t bracket_end = line.find(']');
        if (bracket_start != std::string::npos && bracket_end != std::string::npos) {
          std::string com_str = line.substr(bracket_start + 1, bracket_end - bracket_start - 1);
          // 解析三个值
          std::vector<double> com_values;
          std::stringstream ss(com_str);
          std::string token;
          while (std::getline(ss, token, ',')) {
            token.erase(std::remove_if(token.begin(), token.end(), ::isspace), token.end());
            if (!token.empty()) {
              com_values.push_back(std::stod(token));
            }
          }
          
          int joint_idx = std::stoi(current_joint.substr(1)) - 1;
          if (joint_idx >= 0 && joint_idx < 6 && com_values.size() >= 3) {
            calibrated_inertia_params_[joint_idx].com_x = com_values[0];
            calibrated_inertia_params_[joint_idx].com_y = com_values[1];
            calibrated_inertia_params_[joint_idx].com_z = com_values[2];
          }
        }
      }
    }
    
    file.close();
    
    if (!use_calibrated) {
      RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
                  "Calibrated inertia disabled in config, using URDF defaults");
      return false;
    }
    
    // 打印加载的参数
    RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
                "Loaded calibrated inertia parameters:");
    for (size_t i = 1; i < 6; ++i) {  // L2-L6
      RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
                  "  L%zu: mass=%.4f kg, com=[%.4f, %.4f, %.4f] m",
                  i + 1,
                  calibrated_inertia_params_[i].mass,
                  calibrated_inertia_params_[i].com_x,
                  calibrated_inertia_params_[i].com_y,
                  calibrated_inertia_params_[i].com_z);
    }
    
    return true;
    
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("RsA3HardwareInterface"),
                 "Failed to load inertia config: %s", e.what());
    return false;
  }
}

void RsA3HardwareInterface::applyCalibratedInertiaToModel()
{
  if (!pinocchio_initialized_ || calibrated_inertia_params_.size() < 6) {
    return;
  }
  
  RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
              "Applying calibrated inertia to Pinocchio model...");
  
  // 更新 L2-L6 的惯性参数 (Pinocchio 模型索引从 1 开始，跳过 universe)
  for (size_t i = 1; i < 6 && i + 1 < static_cast<size_t>(pinocchio_model_.nbodies); ++i) {
    const auto& params = calibrated_inertia_params_[i];
    
    // 更新质量 (mass() 返回引用)
    pinocchio_model_.inertias[i + 1].mass() = params.mass;
    
    // 更新质心位置 (lever() 返回向量引用)
    pinocchio_model_.inertias[i + 1].lever()[0] = params.com_x;
    pinocchio_model_.inertias[i + 1].lever()[1] = params.com_y;
    pinocchio_model_.inertias[i + 1].lever()[2] = params.com_z;
    
    RCLCPP_DEBUG(rclcpp::get_logger("RsA3HardwareInterface"),
                 "  Updated L%zu: mass=%.4f, com=[%.4f, %.4f, %.4f]",
                 i + 1, params.mass, params.com_x, params.com_y, params.com_z);
  }
  
  // 重新创建 Pinocchio Data 对象以应用更改
  pinocchio_data_ = pinocchio::Data(pinocchio_model_);
  
  RCLCPP_INFO(rclcpp::get_logger("RsA3HardwareInterface"),
              "Pinocchio model updated with calibrated inertia parameters");
}

}  // namespace rs_a3_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  rs_a3_hardware::RsA3HardwareInterface,
  hardware_interface::SystemInterface)


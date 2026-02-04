/**
 * @file rs_a3_hardware.hpp
 * @brief ROS2 Control hardware interface for RS-A3 robot arm
 */

#ifndef RS_A3_HARDWARE__RS_A3_HARDWARE_HPP_
#define RS_A3_HARDWARE__RS_A3_HARDWARE_HPP_

#include <atomic>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_srvs/srv/set_bool.hpp"

#include "rs_a3_hardware/robstride_can_driver.hpp"
#include "rs_a3_hardware/s_curve_generator.hpp"

// Pinocchio for dynamics computation
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/compute-all-terms.hpp>

namespace rs_a3_hardware
{

/**
 * @brief Joint configuration
 */
struct JointConfig
{
  std::string name;
  uint8_t motor_id;
  MotorType motor_type;
  double position_offset;
  double direction;  // 1.0 or -1.0
  double lower_limit;  // 关节下限 (rad)
  double upper_limit;  // 关节上限 (rad)
  double kp;           // 关节独立Kp (0表示使用全局值)
  double kd;           // 关节独立Kd (0表示使用全局值)
};

/**
 * @brief ROS2 Control hardware interface for RS-A3
 */
class RsA3HardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(RsA3HardwareInterface)

  RsA3HardwareInterface();
  ~RsA3HardwareInterface() override;

  // SystemInterface methods
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo& info) override;
  
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State& previous_state) override;
  
  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State& previous_state) override;
  
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State& previous_state) override;
  
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State& previous_state) override;
  
  hardware_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State& previous_state) override;
  
  hardware_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State& previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time& time, const rclcpp::Duration& period) override;
  
  hardware_interface::return_type write(
    const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
  /**
   * @brief Parse joint configuration from hardware info
   */
  bool parseJointConfig(const hardware_interface::HardwareInfo& info);
  
  // CAN driver
  std::unique_ptr<RobstrideCanDriver> can_driver_;
  std::string can_interface_;
  uint8_t host_can_id_;
  
  // Joint configurations
  std::vector<JointConfig> joint_configs_;
  
  // State interfaces data
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_efforts_;
  std::vector<double> hw_temperatures_;  // 电机温度 (°C)
  
  // Command interfaces data
  std::vector<double> hw_commands_positions_;
  std::vector<double> hw_commands_velocities_;
  std::vector<double> hw_commands_efforts_;
  
  // Control parameters
  double position_kp_;
  double position_kd_;
  double velocity_limit_;
  
  // 位置命令平滑滤波器（带速度/加速度/加加速度限制 - S曲线规划）
  std::vector<double> smoothed_positions_;    // 平滑后的位置命令
  std::vector<double> smoothed_velocities_;   // 平滑后的速度（用于加速度限制）
  std::vector<double> smoothed_accelerations_; // 平滑后的加速度（用于S曲线规划）
  
  // 速度前馈计算相关
  std::vector<double> last_cmd_positions_;     // 上一周期的命令位置（用于计算速度前馈）
  std::vector<double> last_hw_commands_positions_; // 上一帧的hw_commands（用于检测命令更新）
  std::vector<double> cmd_velocities_;         // 计算出的命令速度
  std::vector<double> filtered_cmd_velocities_; // 滤波后的命令速度（一阶）
  std::vector<double> velocity_ff_stage2_;     // 速度前馈二阶滤波中间值
  double velocity_filter_alpha_;               // 速度滤波系数 (0-1)
  double smoothing_alpha_;                    // 平滑系数 (0-1，越小越平滑)
  double max_velocity_;                       // 最大速度限制 (rad/s)
  double max_acceleration_;                   // 最大加速度限制 (rad/s²)
  bool first_command_;                        // 第一次命令标记
  double control_period_;                     // 控制周期 (s)
  
  // Control mode
  enum class ControlMode
  {
    POSITION,    // CSP mode
    VELOCITY,
    EFFORT
  };
  ControlMode control_mode_;
  
  bool use_mock_hardware_;
  
  // S曲线轨迹生成器（每个关节一个）
  std::vector<std::unique_ptr<SCurveGenerator>> s_curve_generators_;
  bool s_curve_enabled_;                      // 是否启用S曲线规划
  
  // ============ 零力矩模式与重力补偿 ============
  // 零力矩模式标志
  bool zero_torque_mode_;           // 是否启用零力矩模式
  double zero_torque_kd_;           // 零力矩模式的默认阻尼系数
  std::vector<double> zero_torque_kp_joints_;  // 零力矩模式关节独立 Kp
  std::vector<double> zero_torque_kd_joints_;  // 零力矩模式关节独立 Kd
  
  // 重力补偿参数 (每个关节: τ = sin_coeff * sin(θ) + cos_coeff * cos(θ) + offset)
  struct GravityCompParams {
    double sin_coeff;
    double cos_coeff;
    double offset;
  };
  std::vector<GravityCompParams> gravity_params_;
  bool gravity_comp_enabled_;       // 是否启用重力补偿
  double gravity_feedforward_ratio_;          // 重力补偿前馈比例 (0-1, 默认0.5=50%)
  
  // ============ Pinocchio 动力学模型 ============
  bool use_pinocchio_gravity_;      // 是否使用 Pinocchio 计算重力补偿
  std::string urdf_path_;           // URDF 文件路径
  pinocchio::Model pinocchio_model_;     // Pinocchio 模型
  pinocchio::Data pinocchio_data_;       // Pinocchio 数据
  bool pinocchio_initialized_;           // Pinocchio 是否已初始化
  
  // 惯性参数（用于标定，完全替换 URDF 默认值）
  struct CalibratedInertiaParams {
    double mass;            // 质量 (kg)
    double com_x;           // 质心x坐标 (m)
    double com_y;           // 质心y坐标 (m)
    double com_z;           // 质心z坐标 (m)
  };
  std::vector<CalibratedInertiaParams> calibrated_inertia_params_;
  std::string inertia_config_path_;   // 惯性参数配置文件路径
  bool use_calibrated_inertia_;       // 是否使用标定后的惯性参数
  
  // 旧的缩放因子结构（保留向后兼容）
  struct InertiaScaleParams {
    double mass_scale;      // 质量缩放因子
    double com_x_offset;    // 质心x偏移
    double com_y_offset;    // 质心y偏移
    double com_z_offset;    // 质心z偏移
  };
  std::vector<InertiaScaleParams> inertia_scale_params_;
  
  /**
   * @brief 初始化 Pinocchio 模型
   * @param urdf_path URDF 文件路径
   * @return 是否成功
   */
  bool initPinocchioModel(const std::string& urdf_path);
  
  /**
   * @brief 从 YAML 配置文件加载标定后的惯性参数
   * @param config_path 配置文件路径
   * @return 是否成功
   */
  bool loadCalibratedInertia(const std::string& config_path);
  
  /**
   * @brief 将标定后的惯性参数应用到 Pinocchio 模型
   */
  void applyCalibratedInertiaToModel();
  
  /**
   * @brief 使用 Pinocchio 计算完整的重力补偿向量
   * @param positions 当前关节位置
   * @return 各关节的重力补偿力矩
   */
  std::vector<double> computePinocchioGravity(const std::vector<double>& positions);
  
  // 计算关节重力补偿力矩（简化模型，单关节独立）
  double computeGravityTorque(size_t joint_idx, double position);
  
  // 零力矩模式服务回调
  void zeroTorqueModeCallback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response);
  
  // 零力矩模式服务
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr zero_torque_srv_;
  
  // 调试发布器
  rclcpp::Node::SharedPtr debug_node_;
  std::thread spin_thread_;                 // 用于处理服务回调的线程
  std::atomic<bool> spin_thread_running_;   // 线程运行标志
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr hw_cmd_pub_;      // 控制器发来的命令
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr smoothed_cmd_pub_; // 平滑后发给电机的命令
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr gravity_torque_pub_; // 重力补偿力矩
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr velocity_ff_pub_;  // 速度前馈（实际发给电机的）
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr temperature_pub_;  // 电机温度
  
  // ============ 关节限位保护 ============
  double limit_margin_;           // 限位边界距离 (rad)，开始减速的距离
  double limit_stop_margin_;      // 硬停止边界 (rad)，完全停止的距离
  double limit_decel_factor_;     // 接近限位时的减速系数 (0-1)
  double max_jerk_;               // 最大加加速度限制 (rad/s³) - S曲线规划
  std::vector<bool> joint_at_limit_;  // 各关节是否处于限位状态
  std::vector<int> limit_warn_counter_;  // 限位警告计数器（防止刷屏）
  
  /**
   * @brief 计算限位保护系数
   * @param joint_idx 关节索引
   * @param current_pos 当前位置
   * @param target_pos 目标位置
   * @return 速度缩放系数 (0.0-1.0)
   */
  double computeLimitProtectionFactor(size_t joint_idx, double current_pos, double target_pos);
  
  /**
   * @brief 应用限位保护，钳位目标位置
   * @param joint_idx 关节索引
   * @param target_pos 目标位置（会被修改）
   * @return 是否触发了限位
   */
  bool applyJointLimitProtection(size_t joint_idx, double& target_pos);
};

}  // namespace rs_a3_hardware

#endif  // RS_A3_HARDWARE__RS_A3_HARDWARE_HPP_


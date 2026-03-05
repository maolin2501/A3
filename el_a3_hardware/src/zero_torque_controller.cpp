/**
 * @file zero_torque_controller.cpp
 * @brief Zero-torque (gravity compensated) controller for EL-A3
 */

#include "el_a3_hardware/zero_torque_controller.hpp"

#include <algorithm>
#include <cmath>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace el_a3_hardware
{

controller_interface::CallbackReturn ZeroTorqueController::on_init()
{
  auto_declare<std::vector<std::string>>("joints", std::vector<std::string>{});
  auto_declare<double>("kd", 1.0);
  auto_declare<std::string>("urdf_path", "");

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ZeroTorqueController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  joint_names_ = get_node()->get_parameter("joints").as_string_array();
  if (joint_names_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "No joints configured");
    return controller_interface::CallbackReturn::ERROR;
  }

  kd_ = get_node()->get_parameter("kd").as_double();
  kd_ = std::clamp(kd_, 0.0, 5.0);

  std::string urdf_path = get_node()->get_parameter("urdf_path").as_string();
  if (!urdf_path.empty()) {
    try {
      pinocchio::urdf::buildModel(urdf_path, model_);
      data_ = pinocchio::Data(model_);
      pinocchio_ok_ = true;
      RCLCPP_INFO(get_node()->get_logger(),
                  "Pinocchio model loaded: nq=%d, nv=%d", model_.nq, model_.nv);
    } catch (const std::exception & e) {
      RCLCPP_WARN(get_node()->get_logger(),
                  "Pinocchio init failed: %s (gravity comp disabled)", e.what());
      pinocchio_ok_ = false;
    }
  }

  gravity_pub_ = get_node()->create_publisher<sensor_msgs::msg::JointState>(
    "~/gravity_torque", 10);

  RCLCPP_INFO(get_node()->get_logger(),
              "ZeroTorqueController configured: %zu joints, kd=%.2f, pinocchio=%s",
              joint_names_.size(), kd_, pinocchio_ok_ ? "yes" : "no");

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
ZeroTorqueController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & joint : joint_names_) {
    conf.names.push_back(joint + "/" + hardware_interface::HW_IF_EFFORT);
  }
  return conf;
}

controller_interface::InterfaceConfiguration
ZeroTorqueController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & joint : joint_names_) {
    conf.names.push_back(joint + "/" + hardware_interface::HW_IF_POSITION);
    conf.names.push_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);
  }
  return conf;
}

controller_interface::CallbackReturn ZeroTorqueController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_WARN(get_node()->get_logger(),
              "Zero-torque mode ACTIVATED (kd=%.2f, gravity=%s)",
              kd_, pinocchio_ok_ ? "on" : "off");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ZeroTorqueController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_node()->get_logger(), "Zero-torque mode DEACTIVATED");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type ZeroTorqueController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  size_t n = joint_names_.size();
  std::vector<double> gravity_torques(n, 0.0);

  if (pinocchio_ok_) {
    try {
      Eigen::VectorXd q = Eigen::VectorXd::Zero(model_.nq);
      Eigen::VectorXd v = Eigen::VectorXd::Zero(model_.nv);
      Eigen::VectorXd a = Eigen::VectorXd::Zero(model_.nv);

      size_t nq = std::min(n, static_cast<size_t>(model_.nq));
      for (size_t i = 0; i < nq; ++i) {
        q[i] = state_interfaces_[i * 2].get_value();  // position
      }

      Eigen::VectorXd tau = pinocchio::rnea(model_, data_, q, v, a);

      for (size_t i = 0; i < std::min(n, static_cast<size_t>(tau.size())); ++i) {
        gravity_torques[i] = tau[i];
      }
    } catch (const std::exception & e) {
      (void)e;
    }
  }

  for (size_t i = 0; i < n; ++i) {
    command_interfaces_[i].set_value(gravity_torques[i]);
  }

  // Publish debug (every ~10th cycle)
  static int pub_counter = 0;
  if (++pub_counter >= 10 && gravity_pub_) {
    pub_counter = 0;
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = get_node()->now();
    msg.name = joint_names_;
    msg.effort = gravity_torques;
    gravity_pub_->publish(msg);
  }

  return controller_interface::return_type::OK;
}

}  // namespace el_a3_hardware

PLUGINLIB_EXPORT_CLASS(
  el_a3_hardware::ZeroTorqueController,
  controller_interface::ControllerInterface)

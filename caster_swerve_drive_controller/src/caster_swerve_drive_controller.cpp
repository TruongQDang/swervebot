#include "caster_swerve_drive_controller/caster_swerve_drive_controller.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "lifecycle_msgs/msg/state.hpp"

namespace caster_swerve_drive_controller {

using controller_interface::interface_configuration_type;
using controller_interface::InterfaceConfiguration;
using hardware_interface::HW_IF_POSITION;
using hardware_interface::HW_IF_VELOCITY;
using lifecycle_msgs::msg::State;

CasterSwerveDriveController::CasterSwerveDriveController()
    : controller_interface::ChainableControllerInterface() {}

controller_interface::CallbackReturn CasterSwerveDriveController::on_init() {
  try {
    // Create the parameter listener and get the parameters
    param_listener_ = std::make_shared<ParamListener>(get_node());
    params_ = param_listener_->get_params();
  } catch (const std::exception &e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n",
            e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

InterfaceConfiguration
CasterSwerveDriveController::command_interface_configuration() const {
  std::vector<std::string> conf_names;
  for (const auto &joint_name : params_.steering_joint_names) {
    conf_names.push_back(joint_name + "/" + HW_IF_POSITION);
  }
  for (const auto &joint_name : params_.driving_joint_names) {
    conf_names.push_back(joint_name + "/" + HW_IF_VELOCITY);
  }
  return {interface_configuration_type::INDIVIDUAL, conf_names};
}

InterfaceConfiguration
CasterSwerveDriveController::state_interface_configuration() const {
  if (params_.open_loop) {
    return {interface_configuration_type::NONE, {}};
  }

  std::vector<std::string> conf_names;
  for (const auto &joint_name : params_.steering_joint_names) {
    conf_names.push_back(joint_name + "/" + HW_IF_POSITION);
    conf_names.push_back(joint_name + "/" + HW_IF_VELOCITY);
  }
  for (const auto &joint_name : params_.driving_joint_names) {
    conf_names.push_back(joint_name + "/" + HW_IF_POSITION);
    conf_names.push_back(joint_name + "/" + HW_IF_VELOCITY);
  }
  return {interface_configuration_type::INDIVIDUAL, conf_names};
}

controller_interface::return_type
CasterSwerveDriveController::update_reference_from_subscribers(
    const rclcpp::Time &time, const rclcpp::Duration & /*period*/) {
  auto logger = get_node()->get_logger();

  auto current_ref_op = received_velocity_msg_.try_get();
  if (current_ref_op.has_value()) {
    command_msg_ = current_ref_op.value();
  }

  const auto age_of_last_command = time - command_msg_.header.stamp;
  // Brake if cmd_vel has timeout, override the stored command
  if (age_of_last_command > cmd_vel_timeout_) {
    reference_interfaces_[0] = 0.0;
    reference_interfaces_[1] = 0.0;
  } else if (std::isfinite(command_msg_.twist.linear.x) &&
             std::isfinite(command_msg_.twist.angular.z)) {
    reference_interfaces_[0] = command_msg_.twist.linear.x;
    reference_interfaces_[1] = command_msg_.twist.angular.z;
  } else {
    RCLCPP_WARN_SKIPFIRST_THROTTLE(
        logger, *get_node()->get_clock(),
        static_cast<rcutils_duration_value_t>(cmd_vel_timeout_.seconds() *
                                              1000),
        "Command message contains NaNs. Not updating reference interfaces.");
  }

  previous_update_timestamp_ = time;

  return controller_interface::return_type::OK;
}

controller_interface::return_type
CasterSwerveDriveController::update_and_write_commands(
    const rclcpp::Time &time, const rclcpp::Duration &period) {

  return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn CasterSwerveDriveController::on_configure(
    const rclcpp_lifecycle::State &previous_state) {
  const int nr_ref_itfs = 2;
  reference_interfaces_.resize(nr_ref_itfs,
                               std::numeric_limits<double>::quiet_NaN());
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CasterSwerveDriveController::on_activate(
    const rclcpp_lifecycle::State &previous_state) {

  return controller_interface::CallbackReturn::SUCCESS;
}

bool CasterSwerveDriveController::on_set_chained_mode(bool /*chained_mode*/) {
  return true;
}

controller_interface::CallbackReturn CasterSwerveDriveController::on_deactivate(
    const rclcpp_lifecycle::State &previous_state) {
  return controller_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::CommandInterface>
CasterSwerveDriveController::on_export_reference_interfaces() {
  std::vector<hardware_interface::CommandInterface> reference_interfaces;

  reference_interfaces.reserve(reference_interfaces_.size());

  reference_interfaces.push_back(hardware_interface::CommandInterface(
      get_node()->get_name() + std::string("/linear"),
      hardware_interface::HW_IF_VELOCITY, &reference_interfaces_[0]));

  reference_interfaces.push_back(hardware_interface::CommandInterface(
      get_node()->get_name() + std::string("/angular"),
      hardware_interface::HW_IF_VELOCITY, &reference_interfaces_[1]));

  return reference_interfaces;
}

} // namespace caster_swerve_drive_controller

#include "class_loader/register_macro.hpp"

CLASS_LOADER_REGISTER_CLASS(
    caster_swerve_drive_controller::CasterSwerveDriveController,
    controller_interface::ChainableControllerInterface)
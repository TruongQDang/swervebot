#include "caster_swerve_drive_controller/caster_swerve_drive_controller.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "controller_interface/helpers.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/parameter.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2/utils.hpp"

namespace {
constexpr auto DEFAULT_COMMAND_TOPIC = "~/cmd_vel";
constexpr auto DEFAULT_COMMAND_OUT_TOPIC = "~/cmd_vel_out";
constexpr auto DEFAULT_ODOMETRY_TOPIC = "~/odom";
constexpr auto DEFAULT_TRANSFORM_TOPIC = "/tf";
} // namespace

namespace caster_swerve_drive_controller {

using controller_interface::interface_configuration_type;
using controller_interface::InterfaceConfiguration;
using hardware_interface::HW_IF_POSITION;
using hardware_interface::HW_IF_VELOCITY;
using rcl_interfaces::msg::ParameterDescriptor;
using rcl_interfaces::msg::ParameterType;

// Reset function
void CasterSwerveDriveController::reset_controller_reference_msg(
    const std::shared_ptr<geometry_msgs::msg::Twist> &msg) {
  msg->linear.x = std::numeric_limits<double>::quiet_NaN();
  msg->linear.y = std::numeric_limits<double>::quiet_NaN();
  msg->linear.z = std::numeric_limits<double>::quiet_NaN();
  msg->angular.x = std::numeric_limits<double>::quiet_NaN();
  msg->angular.y = std::numeric_limits<double>::quiet_NaN();
  msg->angular.z = std::numeric_limits<double>::quiet_NaN();
}

// --- Controller Implementation ---
CasterSwerveDriveController::CasterSwerveDriveController()
    : controller_interface::ControllerInterface() {}
// *****************************************

CallbackReturn CasterSwerveDriveController::on_init() {
  RCLCPP_DEBUG(get_node()->get_logger(),
               "Initializing CasterSwerveDriveController");
  try {
    param_listener_ = std::make_shared<ParamListener>(get_node());
    params_ = param_listener_->get_params();
  } catch (const std::exception &e) {
    RCLCPP_FATAL(get_node()->get_logger(),
                 "Exception during parameter declaration: %s", e.what());
    return CallbackReturn::ERROR;
  }
  RCLCPP_DEBUG(get_node()->get_logger(), "Parameter declaration successful");
  return CallbackReturn::SUCCESS;
}

// command_interface_configuration, state_interface_configuration
InterfaceConfiguration
CasterSwerveDriveController::command_interface_configuration() const {
  std::vector<std::string> conf_names;
  for (const auto &joint_name : params_.steering_joint_names) {
    conf_names.push_back(joint_name + "/" + HW_IF_POSITION);
    conf_names.push_back(joint_name + "/" + HW_IF_VELOCITY);
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
    conf_names.push_back(joint_name + "/" + HW_IF_VELOCITY);
  }
  return {interface_configuration_type::INDIVIDUAL, conf_names};
}

CallbackReturn
CasterSwerveDriveController::on_configure(const rclcpp_lifecycle::State &) {
  auto logger = get_node()->get_logger();
  // update parameters if they have changed
  if (param_listener_->try_update_params(params_)) {
    RCLCPP_INFO(logger, "Parameters were updated");
  }

  if (params_.steering_joint_names.size() !=
      params_.driving_joint_names.size()) {
    RCLCPP_ERROR(
        logger,
        "The number of steering joints [%zu] and the number of driving "
        "joints [%zu] are different",
        params_.steering_joint_names.size(),
        params_.driving_joint_names.size());
    return CallbackReturn::ERROR;
  }

  odometry_.setModuleParams(params_.module_x_offsets, params_.module_y_offsets,
                            params_.wheel_radius);
  num_modules_ = params_.steering_joint_names.size();
  cmd_vel_timeout_ = rclcpp::Duration::from_seconds(params_.cmd_vel_timeout);

  // initialize command subscriber
  cmd_vel_subscriber_ = get_node()->create_subscription<TwistStamped>(
      params_.cmd_vel_topic, rclcpp::SystemDefaultsQoS(),
      [this](const std::shared_ptr<TwistStamped> msg) -> void {
        const auto current_time_diff = get_node()->now() - msg->header.stamp;

        if (cmd_vel_timeout_ == rclcpp::Duration::from_seconds(0.0) ||
            current_time_diff < cmd_vel_timeout_) {
          cmd_vel_buffer_.writeFromNonRT(msg);
        } else {
          RCLCPP_WARN(get_node()->get_logger(),
                      "Ignoring the received message (timestamp %.10f) because "
                      "it is older than "
                      "the current time by %.10f seconds, which exceeds the "
                      "allowed timeout (%.4f)",
                      rclcpp::Time(msg->header.stamp).seconds(),
                      current_time_diff.seconds(), cmd_vel_timeout_.seconds());
        }
      });

  // initialize odometry publisher and message
  odometry_publisher_ = get_node()->create_publisher<nav_msgs::msg::Odometry>(
      DEFAULT_ODOMETRY_TOPIC, rclcpp::SystemDefaultsQoS());
  realtime_odometry_publisher_ = std::make_shared<
      realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>(
      odometry_publisher_);

  auto &odometry_message = realtime_odometry_publisher_->msg_;
  odometry_message.header.frame_id = params_.odom_frame_id;
  odometry_message.child_frame_id = params_.base_frame_id;

  // initialize transform publisher and message
  odometry_transform_publisher_ = get_node()->create_publisher<TfMsg>(
      DEFAULT_TRANSFORM_TOPIC, rclcpp::SystemDefaultsQoS());
  realtime_odometry_transform_publisher_ =
      std::make_shared<realtime_tools::RealtimePublisher<TfMsg>>(
          odometry_transform_publisher_);

  // keeping track of odom and base_link transforms only
  auto &odometry_transform_message =
      realtime_odometry_transform_publisher_->msg_;
  odometry_transform_message.transforms.resize(1);
  odometry_transform_message.transforms.front().header.frame_id =
      params_.odom_frame_id;
  odometry_transform_message.transforms.front().child_frame_id =
      params_.base_frame_id;

  previous_update_timestamp_ = get_node()->get_clock()->now();

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type update(const rclcpp::Time &time,
                                         const rclcpp::Duration &period) {}

} // namespace caster_swerve_drive_controller

// Pluginlib export macro
#include "pluginlib/class_list_macros.hpp"

// Export the controller class as a plugin
PLUGINLIB_EXPORT_CLASS(
    caster_swerve_drive_controller::CasterSwerveDriveController,
    controller_interface::ControllerInterface)
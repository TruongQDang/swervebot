#include "caster_swerve_drive_controller/odometry.hpp"

#include <Eigen/Dense>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

#include "rclcpp/logging.hpp"
#include "tf2/transform_datatypes.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace caster_swerve_drive_controller {

Odometry::Odometry(size_t velocity_rolling_window_size)
    : timestamp_(0, 0, RCL_ROS_TIME), position_x_odom_(0.0),
      position_y_odom_(0.0), orientation_yaw_odom_(0.0),
      velocity_in_base_frame_linear_x_(0.0),
      velocity_in_base_frame_linear_y_(0.0),
      velocity_in_base_frame_angular_z_(0.0), num_modules_(0),
      wheel_radius_(0.0), solver_method_(OdomSolverMethod::SVD),
      velocity_rolling_window_size_(velocity_rolling_window_size),
      linear_x_accumulator_(velocity_rolling_window_size),
      linear_y_accumulator_(velocity_rolling_window_size),
      angular_z_accumulator_(velocity_rolling_window_size) {
  base_frame_offset_.fill(0.0);
}

} // namespace caster_swerve_drive_controller
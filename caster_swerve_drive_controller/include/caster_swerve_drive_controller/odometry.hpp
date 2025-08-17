#ifndef CASTER_SWERVE_DRIVE_CONTROLLER__ODOMETRY_HPP_
#define CASTER_SWERVE_DRIVE_CONTROLLER__ODOMETRY_HPP_

#include <Eigen/Dense>
#include <array>
#include <cmath>
#include <string>
#include <vector>

#include "rclcpp/time.hpp"
#include "rcpputils/rolling_mean_accumulator.hpp"

namespace caster_swerve_drive_controller {
enum class OdomSolverMethod { PSEUDO_INVERSE, QR_DECOMPOSITION, SVD };

class Odometry {
public:
  explicit Odometry(size_t velocity_rolling_window_size = 1);

  void init(const rclcpp::Time &time,
            const std::array<double, 3> &base_frame_offset = {0.0, 0.0, 0.0});

  bool update(const std::vector<double> &steering_positions,
              const std::vector<double> &wheel_velocities, const double dt);
  bool update(double target_vx, double target_vy, double target_w,
              const double dt);

  double getX() const { return position_x_odom_; }
  double getY() const { return position_y_odom_; }
  double getYaw() const { return orientation_yaw_odom_; }
  double getVx() const { return velocity_in_base_frame_linear_x_; }
  double getVy() const { return velocity_in_base_frame_linear_y_; }
  double getWz() const { return velocity_in_base_frame_angular_z_; }

  void setModuleParams(const std::vector<double> &module_x_offsets,
                       const std::vector<double> &module_y_offsets,
                       const double wheel_radius);

  void set_solver_method(OdomSolverMethod method);

  void setVelocityRollingWindowSize(size_t velocity_rolling_window_size);
  void resetAccumulators();
  Eigen::Vector3d updateFromVelocity(double linear_x, double linear_y,
                                     double angular_z);

private:
  // \note The versions conditioning is added here to support the
  // source-compatibility with Humble
  using RollingMeanAccumulator = rcpputils::RollingMeanAccumulator<double>;

  rclcpp::Time timestamp_;
  std::array<double, 3> base_frame_offset_;

  double position_x_odom_;
  double position_y_odom_;
  double orientation_yaw_odom_;

  double velocity_in_base_frame_linear_x_;
  double velocity_in_base_frame_linear_y_;
  double velocity_in_base_frame_angular_z_;

  size_t num_modules_ = 0;
  std::vector<double> module_x_offsets_;
  std::vector<double> module_y_offsets_;
  double wheel_radius_ = 0.0;

  static double normalize_angle(double angle_rad);

  // Rolling mean accumulators for the linear and angular velocities:
  OdomSolverMethod solver_method_;
  size_t velocity_rolling_window_size_;
  RollingMeanAccumulator linear_x_accumulator_;
  RollingMeanAccumulator linear_y_accumulator_;
  RollingMeanAccumulator angular_z_accumulator_;
};
} // namespace caster_swerve_drive_controller

#endif // CASTER_SWERVE_DRIVE_CONTROLLER__ODOMETRY_HPP_
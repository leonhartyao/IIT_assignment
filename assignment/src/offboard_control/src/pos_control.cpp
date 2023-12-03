#include "offboard_control/pos_control.h"

namespace pos_control {
PositionController::PositionController() {
  // Initialize control gains
  kp = Eigen::Vector3d(1.0, 1.0, 1.0);
  ki = Eigen::Vector3d(0.0, 0.0, 0.0);
  kd = Eigen::Vector3d(0.2, 0.2, 0.2);
}

Eigen::Vector4d PositionController::computeAttiSetpoint(
    const Eigen::Vector3d& desired_position, const Eigen::Vector3d current_pose,
    const double dt) {
  Eigen::Vector3d error = desired_position - current_pose;
  integral += error * dt;
  Eigen::Vector3d derivative = (error - prev_error) / dt;

  Eigen::Vector3d result_vec = kp.cwiseProduct(error) +
                               ki.cwiseProduct(integral) +
                               kd.cwiseProduct(derivative);

  // Assuming thrust is proportional to the magnitude of the attitude setpoint
  double thrust = result_vec.norm();
  Eigen::Vector3d direction = result_vec.normalized();

  Eigen::Vector4d control_input;
  control_input.head<3>() = direction;
  control_input(3) = thrust;

  prev_error = error;

  return control_input;
}
}  // namespace pos_control

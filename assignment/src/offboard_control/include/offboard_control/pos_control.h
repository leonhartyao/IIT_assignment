#ifndef OFFBOARD_CONTROL__POS_CONTROL_H__
#define OFFBOARD_CONTROL__POS_CONTROL_H__

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/Thrust.h>
#include <ros/ros.h>

#include <Eigen/Dense>

namespace pos_control {

class PositionController {
 public:
  PositionController();

  /**
   * @brief compute attitude setpoint based on the control law
   *
   * @param desired_position
   * @return geometry_msgs::TwistStamped
   */
  Eigen::Vector4d computeAttiSetpoint(const Eigen::Vector3d& desired_position,
                                      const Eigen::Vector3d current_pose,
                                      const double dt);

 private:
  mavros_msgs::State current_state;
  geometry_msgs::Pose current_pose;
  Eigen::Vector3d kp, ki, kd;
  Eigen::Vector3d integral;
  Eigen::Vector3d prev_error;
};

}  // namespace pos_control

#endif  // OFFBOARD_CONTROL__POS_CONTROL_H__

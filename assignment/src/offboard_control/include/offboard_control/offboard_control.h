#ifndef OFFBOARD_CONTROL__OFFBOARD_CONTROL_H__
#define OFFBOARD_CONTROL__OFFBOARD_CONTROL_H__

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <ros/ros.h>

#include <Eigen/Dense>
#include <memory>

#include "offboard_control/pos_control.h"
#include "offboard_control/trajectory_generator.h"

class OffboardControl {
 public:
  OffboardControl();

  void offboardControl();

 private:
  void initializeRosInterface();

  void stateCallback(const mavros_msgs::State::ConstPtr& msg);

  void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

  void setMode(const std::string& mode);

  void arm(bool arm);

  ros::Subscriber state_sub;
  ros::Subscriber pose_sub;
  ros::Publisher local_pos_pub;
  ros::Publisher vel_pub;
  ros::Publisher attitude_pub;
  ros::ServiceClient set_mode_client;
  ros::ServiceClient arm_client;
  ros::NodeHandle nh;
  ros::Rate rate;
  mavros_msgs::State current_state;
  Eigen::Vector3d current_pose;
  std::unique_ptr<TrajectoryPlanner> planner;
  std::unique_ptr<pos_control::PositionController> pos_controller;
};

#endif  // OFFBOARD_CONTROL__OFFBOARD_CONTROL_H__

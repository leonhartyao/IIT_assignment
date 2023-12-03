/**
 * @file offboard_node.cpp
 * @brief Control node of offboard mode
 */

#include "offboard_control/trajectory_generator.h"

#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <ros/ros.h>
#include <Eigen/Dense>
#include <memory>

mavros_msgs::State g_current_state;

/**
 * @brief Callback to upate FCU state
 *
 * @param msg mavros state msg
 */
void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
  g_current_state = *msg;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "offboard_control");
  ros::NodeHandle nh;

  ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 1, state_cb);
  ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
  ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
  ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

  // the setpoint publishing rate MUST be faster than 2Hz
  ros::Rate rate(20.0);

  // prepare trajectory
  // TODO: load waypoints from config
  std::vector<Eigen::Vector3d> waypoints;
  waypoints.emplace_back(0.0, 0.0, 1.7);
  waypoints.emplace_back(1.0, 0.0, 1.7);
  waypoints.emplace_back(1.0, 0.5, 1.7);
  waypoints.emplace_back(1.5, 0.7, 1.7);
  waypoints.emplace_back(1.3, 1.5, 1.7);
  waypoints.emplace_back(0.5, 1.0, 1.7);

  std::unique_ptr<TrajectoryPlanner> planner = std::make_unique<TrajectoryPlanner>(waypoints, nh);

  // wait for connection
  while (ros::ok() && !g_current_state.connected) {
    ros::spinOnce();
    rate.sleep();
  }

  geometry_msgs::PoseStamped takeoff_pose;
  takeoff_pose.pose.position.x = 0;
  takeoff_pose.pose.position.y = 0;
  takeoff_pose.pose.position.z = 1.7;

  // send a few setpoints before starting
  for (int i = 100; ros::ok() && i > 0; --i) {
    local_pos_pub.publish(takeoff_pose);
    ros::spinOnce();
    rate.sleep();
  }

  mavros_msgs::SetMode mode_req;
  mode_req.request.custom_mode = "OFFBOARD";

  mavros_msgs::CommandBool arm_req;
  arm_req.request.value = true;

  ros::Time last_request_time = ros::Time::now();

  while (ros::ok()) {
    if (g_current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request_time > ros::Duration(5.0))) {
      if (set_mode_client.call(mode_req) && mode_req.response.mode_sent) {
        ROS_INFO("Offboard enabled");
      }
      last_request_time = ros::Time::now();
    } else {
      if (!g_current_state.armed && (ros::Time::now() - last_request_time > ros::Duration(5.0))) {
        if (arming_client.call(arm_req) && arm_req.response.success) {
          ROS_INFO("Vehicle armed");
        }
        last_request_time = ros::Time::now();
      }
    }

    local_pos_pub.publish(takeoff_pose);

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}

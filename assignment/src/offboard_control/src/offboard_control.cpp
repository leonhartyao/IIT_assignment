#include "offboard_control/offboard_control.h"

#include <cmath>
#include <numeric>

geometry_msgs::PoseStamped eigenVector3dToPoseStamped(
    const Eigen::Vector3d& eigenVec) {
  geometry_msgs::PoseStamped poseStamped;
  poseStamped.pose.position.x = eigenVec(0);
  poseStamped.pose.position.y = eigenVec(1);
  poseStamped.pose.position.z = eigenVec(2);

  return poseStamped;
}

OffboardControl::OffboardControl() : nh("~"), rate(ros::Rate(20.0)) {
  initializeRosInterface();

  // prepare trajectory
  // TODO: load waypoints from config
  std::vector<Eigen::Vector3d> waypoints;
  waypoints.emplace_back(0.0, 0.0, 1.7);
  waypoints.emplace_back(1.0, 0.0, 1.7);
  waypoints.emplace_back(1.0, 1.0, 1.7);
  waypoints.emplace_back(0.0, 1.0, 1.7);
  waypoints.emplace_back(-1.0, 1.0, 1.7);
  waypoints.emplace_back(-1.0, 0.0, 1.7);

  planner = std::make_unique<TrajectoryPlanner>(waypoints, nh);
}

void OffboardControl::initializeRosInterface() {
  // Set up subscribers
  state_sub =
      nh.subscribe("/mavros/state", 1, &OffboardControl::stateCallback, this);
  pose_sub = nh.subscribe("/mavros/local_position/pose", 1,
                          &OffboardControl::poseCallback, this);

  // Set up publishers
  local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>(
      "/mavros/setpoint_position/local", 10);
  vel_pub = nh.advertise<geometry_msgs::TwistStamped>(
      "/mavros/setpoint_velocity/cmd_vel", 10);
  attitude_pub = nh.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_attitude/cmd_vel", 10);

  // Set up services
  set_mode_client =
      nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
  arm_client =
      nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
}

void OffboardControl::stateCallback(const mavros_msgs::State::ConstPtr& msg) {
  current_state = *msg;
}

void OffboardControl::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  current_pose << msg->pose.position.x, msg->pose.position.y,
      msg->pose.position.z;
  // TODO: add the customized position control here.
  // controller->computeAttiSetpoint(pose_desired, current_pose)
}

void OffboardControl::setMode(const std::string& mode) {
  mavros_msgs::SetMode set_mode;
  set_mode.request.custom_mode = mode;

  if (set_mode_client.call(set_mode) && set_mode.response.mode_sent) {
    ROS_INFO("Mode set to: %s", mode.c_str());
  } else {
    ROS_ERROR("Failed to set mode");
  }
}

void OffboardControl::arm(bool arm) {
  mavros_msgs::CommandBool arm_cmd;
  arm_cmd.request.value = arm;

  if (arm_client.call(arm_cmd) && arm_cmd.response.success) {
    if (arm) {
      ROS_INFO("vehicle armed");
    } else {
      ROS_INFO("vehicle disarmed");
    }
  } else {
    ROS_ERROR("Failed to arm/disarm the vehicle");
  }
}

void OffboardControl::offboardControl() {
  // Add a delay before setting the mode and arming
  ros::Duration(1.0).sleep();

  // Safe hovering altitude 1.7m
  geometry_msgs::PoseStamped takeoff_pose;
  double hovering_altitude = 1.7;
  takeoff_pose.pose.position.z = hovering_altitude;

  // prepare to switch to OFFBOARD
  for (int i = 50; ros::ok() && i > 0; --i) {
    takeoff_pose.header.stamp = ros::Time::now();
    local_pos_pub.publish(takeoff_pose);
    rate.sleep();
    ros::spinOnce();
  }

  setMode("OFFBOARD");
  // Wait for the OFFBOARD mode to be engaged
  while (ros::ok() && current_state.mode != "OFFBOARD") {
    takeoff_pose.header.stamp = ros::Time::now();
    local_pos_pub.publish(takeoff_pose);
    rate.sleep();
    ros::spinOnce();
  }
  ros::Time offboard_start_time = ros::Time::now();

  // Delay before arm
  while (ros::ok() &&
          (ros::Time::now() - offboard_start_time < ros::Duration(1.0))) {
    takeoff_pose.header.stamp = ros::Time::now();
    local_pos_pub.publish(takeoff_pose);
    rate.sleep();
    ros::spinOnce();
  }

  // Arm the vehicle
  arm(true);
  rate.sleep();
  ros::spinOnce();

  // Wait for the vehicle to be armed
  while (!current_state.armed && ros::ok()) {
    takeoff_pose.header.stamp = ros::Time::now();
    local_pos_pub.publish(takeoff_pose);
    rate.sleep();
    ros::spinOnce();
  }

  // Wait until the quadrotor reaches the desired altitude
  while (ros::ok() && abs(hovering_altitude - current_pose(2)) > 0.02) {
    takeoff_pose.header.stamp = ros::Time::now();
    local_pos_pub.publish(takeoff_pose);
    rate.sleep();
    ros::spinOnce();
  }
  ROS_INFO("vehicle reached hovering altitude");

  // Keep the vehicle hovering for 1 seconds
  ros::Time hover_start_time = ros::Time::now();
  while (ros::ok() &&
          ((ros::Time::now() - hover_start_time) < ros::Duration(1.0))) {
    takeoff_pose.header.stamp = ros::Time::now();
    local_pos_pub.publish(takeoff_pose);
    rate.sleep();
    ros::spinOnce();
  }

  // ROS_INFO("vehicle goes to desired waypoint");
  // Eigen::Vector3d waypoint;
  // waypoint << 0.1, 0.0, 1.7;  // Adjust the waypoint coordinates
  // geometry_msgs::PoseStamped pose_desired =
  //     eigenVector3dToPoseStamped(waypoint);

  // while (ros::ok() && ((current_pose - waypoint).norm() > 0.05)) {
  //   ROS_INFO("distance error norm: %f", (current_pose - waypoint).norm());
  //   pose_desired.header.stamp = ros::Time::now();
  //   local_pos_pub.publish(pose_desired);
  //   rate.sleep();
  //   ros::spinOnce();
  // }

  ROS_INFO("vehicle follows trajectory defined by waypoints");
  // TODO: enable the customized position control here.
  geometry_msgs::PoseStamped pose_desired;
  mav_trajectory_generation::Trajectory trajectory = planner->getTrajectory();
  std::vector<double> segment_times = trajectory.getSegmentTimes();
  // Calculate the total duration of the trajectory
  double total_duration = std::accumulate(segment_times.begin(), segment_times.end(), 0.0);
  ROS_INFO("trajectory duration total: %f", total_duration);
  double sampling_time = 0;
  int derivative_order = mav_trajectory_generation::derivative_order::POSITION;
  ros::Time traj_start_time = ros::Time::now();
  // TODO: traj duration condition
  while (ros::ok() && (sampling_time < total_duration)) {
    sampling_time = (ros::Time::now() - traj_start_time).toSec();
    Eigen::VectorXd sample = planner->sampleTrajectory(sampling_time, derivative_order);
    pose_desired = eigenVector3dToPoseStamped(sample.head<3>());
    // ROS_INFO("distance error norm: %f", (current_pose - sample.head<3>()).norm());
    pose_desired.header.stamp = ros::Time::now();
    local_pos_pub.publish(pose_desired);


    ROS_INFO("desired pos x: %f y: %f z: %f", pose_desired.pose.position.x, pose_desired.pose.position.y, pose_desired.pose.position.z);
    rate.sleep();
    ros::spinOnce();
  }

    ROS_INFO("vehicle reached desired waypoint");
    hover_start_time = ros::Time::now();
    while (ros::ok() &&
           ((ros::Time::now() - hover_start_time) < ros::Duration(1.0))) {
      pose_desired.header.stamp = ros::Time::now();
      local_pos_pub.publish(pose_desired);
      rate.sleep();
      ros::spinOnce();
    }

    setMode("AUTO.LOITER");
    rate.sleep();
    ros::spinOnce();
    // FIXME: Failed to switch from OFFBOARD to LOITER
    // while (ros::ok() && current_state.mode != "AUTO.LOITER") {
    //   ROS_INFO("current_mode is %s", current_state.mode.c_str());
    //   pose_desired.header.stamp = ros::Time::now();
    //   local_pos_pub.publish(pose_desired);
    //   rate.sleep();
    //   ros::spinOnce();
    // }
    ROS_INFO("vehicle is holding");

    ros::Duration(2.0).sleep();
    setMode("AUTO.LAND");

    ros::Duration(8.0).sleep();

    // Disarm the vehicle
    arm(false);

    // Keep the loop running for a moment to ensure that the disarming command
    // is sent
    for (int i = 0; i < 50; ++i) {
      rate.sleep();
      ros::spinOnce();
    }
}

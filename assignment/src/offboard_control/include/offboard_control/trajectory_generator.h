#ifndef OFFBOARD_CONTROL__TRAJECTORY_GENERATOR_H__
#define OFFBOARD_CONTROL__TRAJECTORY_GENERATOR_H__

#include <ros/ros.h>
#include <mav_trajectory_generation/polynomial_optimization_nonlinear.h>
#include <mav_planning_msgs/PolynomialTrajectory.h>
#include <Eigen/Dense>

class TrajectoryPlanner {
public:
    TrajectoryPlanner(std::vector<Eigen::Vector3d> waypoints, ros::NodeHandle& nh);
    mav_trajectory_generation::Trajectory getTrajectory();
    Eigen::VectorXd sampleTrajectory(double sampling_time, int derivative_order);

private:
    void initializeParams();
    void initializeROS();
    bool generateTrajectory();

    ros::NodeHandle nh_;
    ros::Publisher traj_pub_;
    std::vector<Eigen::Vector3d> waypoints_;
    mav_trajectory_generation::Trajectory trajectory_;
};

#endif // OFFBOARD_CONTROL__TRAJECTORY_GENERATOR_H__

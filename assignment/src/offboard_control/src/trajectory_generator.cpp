#include "offboard_control/trajectory_generator.h"
#include <mav_trajectory_generation_ros/ros_conversions.h>

TrajectoryPlanner::TrajectoryPlanner(std::vector<Eigen::Vector3d> waypoints, ros::NodeHandle& nh) 
: waypoints_(waypoints), nh_(nh) {
    // initializeParams();
    // initializeROS();
    generateTrajectory();
}

void TrajectoryPlanner::initializeParams() {
    std::string waypoints_file;
    nh_.param("waypoints_file", waypoints_file, std::string(""));
}

void TrajectoryPlanner::initializeROS() {
    traj_pub_ = nh_.advertise<mav_planning_msgs::PolynomialTrajectory>("trajectory", 1);
}

bool TrajectoryPlanner::generateTrajectory() {
    if (waypoints_.size() < 2) {
        ROS_ERROR("At least two waypoints are required.");
        return false;
    }

    mav_trajectory_generation::Vertex::Vector vertices;
    mav_trajectory_generation::Vertex start(3), end(3);
    const int derivative_to_optimize = mav_trajectory_generation::derivative_order::SNAP;;
    start.makeStartOrEnd(waypoints_[0], derivative_to_optimize);
    vertices.push_back(start);
    for (size_t i = 1; i < waypoints_.size()-1; ++i) {
        mav_trajectory_generation::Vertex vertex(3);
        vertex.addConstraint(mav_trajectory_generation::derivative_order::POSITION, waypoints_[i]);
        vertices.push_back(vertex);
    }
    end.makeStartOrEnd(waypoints_.back(), derivative_to_optimize);
    vertices.push_back(end);

    std::vector<double> segment_times;
    const double v_max = 0.5;
    const double a_max = 0.5;
    segment_times = estimateSegmentTimes(vertices, v_max, a_max);

    mav_trajectory_generation::PolynomialOptimization<10> opt(3);
    opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);

    if (!opt.solveLinear()) {
        ROS_ERROR("Failed to optimize trajectory.");
        return false;
    }

    opt.getTrajectory(&trajectory_);

    return true;
}

mav_trajectory_generation::Trajectory TrajectoryPlanner::getTrajectory(){
    return trajectory_;
}

Eigen::VectorXd TrajectoryPlanner::sampleTrajectory(double sampling_time, int derivative_order){
    return trajectory_.evaluate(sampling_time, derivative_order);
}

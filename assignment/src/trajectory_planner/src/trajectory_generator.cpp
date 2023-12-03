// trajectory_planner_combined.cpp

#include <ros/ros.h>
#include <mav_trajectory_generation/polynomial_optimization_nonlinear.h>
#include <mav_planning_msgs/PolynomialTrajectory.h>
// #include <mav_trajectory_generation_ros/ros_visualization.h>
#include <mav_trajectory_generation_ros/ros_conversions.h>
// #include <yaml-cpp/yaml.h>

class TrajectoryPlanner {
public:
    TrajectoryPlanner();

private:
    void initializeParams();
    void initializeROS();
    void generateAndPublishTrajectory();

    ros::NodeHandle nh_;
    ros::Publisher traj_pub_;
    std::vector<Eigen::Vector3d> waypoints_;
};

TrajectoryPlanner::TrajectoryPlanner() : nh_("~") {
    initializeParams();
    initializeROS();
}

void TrajectoryPlanner::initializeParams() {
    std::string waypoints_file;
    nh_.param("waypoints_file", waypoints_file, std::string(""));
    waypoints_.emplace_back(0.0, 0.0, 1.7);
    waypoints_.emplace_back(1.0, 0.0, 1.7);
    waypoints_.emplace_back(1.0, 0.5, 1.7);
    waypoints_.emplace_back(1.5, 0.7, 1.7);
    waypoints_.emplace_back(1.3, 1.5, 1.7);
    waypoints_.emplace_back(0.5, 1.0, 1.7);
}

void TrajectoryPlanner::initializeROS() {
    traj_pub_ = nh_.advertise<mav_planning_msgs::PolynomialTrajectory4D>("trajectory", 1);
    generateAndPublishTrajectory();
}

void TrajectoryPlanner::generateAndPublishTrajectory() {
    if (waypoints_.size() < 2) {
        ROS_ERROR("At least two waypoints are required.");
        return;
    }

    mav_trajectory_generation::Vertex::Vector vertices;
    mav_trajectory_generation::Vertex start(3), end(3);
    const int derivative_to_optimize = derivative_to_optimize;
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
    const double v_max = 2.0;
    const double a_max = 2.0;
    segment_times = estimateSegmentTimes(vertices, v_max, a_max);

    mav_trajectory_generation::PolynomialOptimization<8> opt(3);
    opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);

    if (!opt.solveLinear()) {
        ROS_ERROR("Failed to optimize trajectory.");
        return;
    }

    mav_trajectory_generation::Trajectory trajectory;
    opt.getTrajectory(&trajectory);

    mav_planning_msgs::PolynomialTrajectory trajectory_msg;
    mav_trajectory_generation::trajectoryToPolynomialTrajectoryMsg(trajectory, &trajectory_msg);

    trajectory_msg.header.frame_id = "world";
    traj_pub_.publish(trajectory_msg);

    ROS_INFO("Trajectory published.");
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "trajectory_generator");
    TrajectoryPlanner planner;
    ros::spin();
    return 0;
}

// trajectory_sampler_node.cpp

#include <ros/ros.h>
#include <mav_planning_msgs/PolynomialTrajectory.h>
#include <geometry_msgs/Point.h>

class TrajectorySamplerNode {
public:
    TrajectorySamplerNode();

private:
    void trajectoryCallback(const mav_planning_msgs::PolynomialTrajectory::ConstPtr& trajectory_msg);
    void sampleTrajectory();
    void publishSampledPoint(const ros::TimerEvent& event);

    ros::NodeHandle nh_;
    ros::Subscriber traj_sub_;
    ros::Publisher sampled_point_pub_;
    mav_planning_msgs::PolynomialTrajectory current_trajectory_;
    size_t current_sample_index_;
    ros::Timer sample_timer_;
};

TrajectorySamplerNode::TrajectorySamplerNode() : nh_("~"), current_sample_index_(0) {
    traj_sub_ = nh_.subscribe("trajectory", 1, &TrajectorySamplerNode::trajectoryCallback, this);
    sampled_point_pub_ = nh_.advertise<geometry_msgs::Point>("sampled_point", 1);

    // Set the sampling rate to 20Hz
    sample_timer_ = nh_.createTimer(ros::Duration(0.05), &TrajectorySamplerNode::publishSampledPoint, this);
}

void TrajectorySamplerNode::trajectoryCallback(const mav_planning_msgs::PolynomialTrajectory::ConstPtr& trajectory_msg) {
    current_trajectory_ = *trajectory_msg;
    current_sample_index_ = 0;
}

void TrajectorySamplerNode::sampleTrajectory() {
    if (current_sample_index_ < current_trajectory_.segments.size()) {
        const mav_planning_msgs::PolynomialSegment& segment = current_trajectory_.segments[current_sample_index_];
        const mav_planning_msgs::PolynomialTrajectory::_segments_type::_control_points_type& control_points =
            segment.control_points;

        // Assuming the trajectory is 3D (position)
        geometry_msgs::Point sampled_point;
        sampled_point.x = control_points[0].x;
        sampled_point.y = control_points[0].y;
        sampled_point.z = control_points[0].z;

        // Publish the sampled point
        sampled_point_pub_.publish(sampled_point);

        // Move to the next segment for the next sample
        ++current_sample_index_;
    }
}

void TrajectorySamplerNode::publishSampledPoint(const ros::TimerEvent& event) {
    sampleTrajectory();
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "trajectory_sampler_node");
    TrajectorySamplerNode sampler_node;
    ros::spin();
    return 0;
}

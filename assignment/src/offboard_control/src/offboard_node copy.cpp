#include <ros/ros.h>
#include <Eigen/Dense>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

class OffboardControl {
public:
    OffboardControl() {
        ros::NodeHandle nh;

        // Set up subscribers
        state_sub = nh.subscribe("/mavros/state", 10, &OffboardControl::stateCallback, this);

        // Set up publishers
        local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
        vel_pub = nh.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 10);

        // Set up services
        set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
        arm_client = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");

        // Set the rate for the control loop
        rate = ros::Rate(20.0);

        // Initialize the state
        current_state = mavros_msgs::State();
    }

    void stateCallback(const mavros_msgs::State::ConstPtr& msg) {
        current_state = *msg;
    }

    void odometryCallback(const nav_msgs::Odometry::ConstPtr& odom) {
        current_pose.header = odom->header;
        current_pose.pose = odom->pose.pose;
    }

    void setMode(const std::string& mode) {
        if (current_state.mode == mode) {
            ROS_INFO("Vehicle is already in %s mode", mode.c_str());
            return;
        }

        mavros_msgs::SetMode set_mode;
        set_mode.request.custom_mode = mode;

        if (set_mode_client.call(set_mode) && set_mode.response.mode_sent) {
            ROS_INFO("Offboard mode set to: %s", mode.c_str());
        } else {
            ROS_ERROR("Failed to set offboard mode");
        }
    }

    void arm(bool arm) {
        mavros_msgs::CommandBool arm_cmd;
        arm_cmd.request.value = arm;

        if (arm_client.call(arm_cmd) && arm_cmd.response.success) {
            if (arm) {
                ROS_INFO("Vehicle armed");
            } else {
                ROS_INFO("Vehicle disarmed");
            }
        } else {
            ROS_ERROR("Failed to arm/disarm the vehicle");
        }
    }

    void offboardControl() {
        // Add a delay before setting the mode and arming
        ros::Duration(5.0).sleep();

        // Set the desired mode to OFFBOARD
        setMode("OFFBOARD");

        // Arm the vehicle
        arm(true);

        // Wait for the OFFBOARD mode to be engaged and the vehicle to be armed
        while (!current_state.armed || current_state.mode != "OFFBOARD") {
            rate.sleep();
        }

        // Subscribe to odometry for current position information
        ros::Subscriber odom_sub = nh.subscribe("/mavros/local_position/odom", 10, &OffboardControl::odometryCallback, this);

        // Take off and hover at 1.7m
        geometry_msgs::PoseStamped takeoff_pose;
        takeoff_pose.pose.position.z = 1.7;

        // Wait until the quadrotor reaches the desired altitude
        while (ros::ok() && current_pose.pose.position.z < 1.7) {
            takeoff_pose.header.stamp = ros::Time::now();  // Update the timestamp
            local_pos_pub.publish(takeoff_pose);
            rate.sleep();
            ros::spinOnce();  // Allow callbacks (e.g., odometry) to be processed
        }

        // Keep the vehicle hovering for 5 seconds
        ros::Time start_time = ros::Time::now();
        while (ros::ok() && (ros::Time::now() - start_time).toSec() < 5.0) {
            vel_pub.publish(geometry_msgs::TwistStamped());  // Publish zero velocity
            rate.sleep();
            ros::spinOnce();
        }

        // Specify a waypoint using Eigen::VectorXd (assuming it's a 3D vector)
        Eigen::VectorXd waypoint(3);
        waypoint << 2.0, 3.0, 1.7;  // Adjust the waypoint coordinates

        // Switch to POSCTL mode for waypoint following
        setMode("POSCTL");

        // Control loop for waypoint following
        while (ros::ok() && (current_pose.pose.position - waypoint).norm() > 0.1) {
            geometry_msgs::TwistStamped cmd_vel;
            
            // Implement your control algorithm to move towards the waypoint
            // Here, we use a simple proportional controller as an example
            cmd_vel.twist.linear.x = 0.1 * (waypoint(0) - current_pose.pose.position.x);
            cmd_vel.twist.linear.y = 0.1 * (waypoint(1) - current_pose.pose.position.y);
            cmd_vel.twist.linear.z = 0.1 * (waypoint(2) - current_pose.pose.position.z);

            vel_pub.publish(cmd_vel);
            rate.sleep();
            ros::spinOnce();
        }

        // Switch back to OFFBOARD mode for automatic control
        setMode("OFFBOARD");

        // Switch to HOLD mode for a safe landing
        setMode("HOLD");

        // Disarm the vehicle
        arm(false);

        // Keep the loop running for a moment to ensure that the disarming command is sent
        for (int i = 0; i < 50; ++i) {
            rate.sleep();
            ros::spinOnce();
        }
    }

private:
    ros::Subscriber state_sub;
    ros::Publisher local_pos_pub;
    ros::Publisher vel_pub;
    ros::ServiceClient set_mode_client;
    ros::ServiceClient arm_client;
    ros::Rate rate;
    mavros_msgs::State current_state;
    geometry_msgs::PoseStamped current_pose;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "offboard_control_node");
    OffboardControl offboard_control;

    offboard_control.offboardControl();

    return 0;
}

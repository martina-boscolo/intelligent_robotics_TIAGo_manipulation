#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <algorithm>
#include <vector>

// Parameters for controlling the robot
const double MIN_DISTANCE = 0.2; // Minimum allowable distance from walls (meters)
const double MAX_SPEED = 2;   // Maximum linear speed (m/s)
const double TURN_GAIN = 1.0;   // Gain for angular velocity adjustment

ros::Publisher cmd_vel_pub;


bool isInCorridor(const sensor_msgs::LaserScan::ConstPtr& msg, double& corridor_width) {
    int n_ranges = msg->ranges.size();
    if (n_ranges == 0) return false;

    // Initialize distances
    double left_dist = 0.0, right_dist = 0.0;
    int left_count = 0, right_count = 0;

    // Divide the scan into left and right halves
    for (int i = 0; i < n_ranges / 2; ++i) {
        if (msg->ranges[i] < msg->range_max && msg->ranges[i] > msg->range_min) {
            left_dist += msg->ranges[i];
            left_count++;
        }
    }
    for (int i = n_ranges / 2; i < n_ranges; ++i) {
        if (msg->ranges[i] < msg->range_max && msg->ranges[i] > msg->range_min) {
            right_dist += msg->ranges[i];
            right_count++;
        }
    }

    // Compute average distances
    double avg_left_dist = (left_count > 0) ? left_dist / left_count : msg->range_max;
    double avg_right_dist = (right_count > 0) ? right_dist / right_count : msg->range_max;

    // Compute corridor width
    corridor_width = avg_left_dist + avg_right_dist;

    // Conditions for detecting a corridor
    bool parallel_walls = fabs(avg_left_dist - avg_right_dist) < 0.3; // Walls are roughly equidistant
    bool narrow_width = corridor_width < 4.0;                         // Width less than 2 meters

    return parallel_walls && narrow_width;
}


void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {

    double corridor_width = 0.0;
    if (isInCorridor(msg, corridor_width)) {
        ROS_INFO("Robot is in a corridor. Width: %.2f meters", corridor_width);
    } else {
        ROS_INFO("Robot is NOT in a corridor.");
    }

    // Extract the laser data
    int n_ranges = msg->ranges.size();
    if (n_ranges == 0) return;

    double left_dist = 0.0, right_dist = 0.0;
    int left_count = 0, right_count = 0;

    // Divide the scan into left and right sides
    for (int i = 0; i < n_ranges / 2; ++i) {
        if (msg->ranges[i] < msg->range_max && msg->ranges[i] > msg->range_min) {
            left_dist += msg->ranges[i];
            left_count++;
        }
    }
    for (int i = n_ranges / 2; i < n_ranges; ++i) {
        if (msg->ranges[i] < msg->range_max && msg->ranges[i] > msg->range_min) {
            right_dist += msg->ranges[i];
            right_count++;
        }
    }

    // Compute average distances
    if (left_count > 0) left_dist /= left_count;
    if (right_count > 0) right_dist /= right_count;

    // Adjust based on wall proximity
    double angular_z = 0.0; // Angular velocity
    if (left_count > 0 && right_count > 0) {
        angular_z = TURN_GAIN * (right_dist - left_dist); // Correct towards center
    }

    // Compute linear speed based on obstacle proximity
    double min_distance = *std::min_element(msg->ranges.begin(), msg->ranges.end());
    double linear_x = (min_distance > MIN_DISTANCE) ? MAX_SPEED : 0.0;

    // Publish velocity command
    geometry_msgs::Twist cmd_vel_msg;
    cmd_vel_msg.linear.x = linear_x;
    cmd_vel_msg.angular.z = angular_z;
    cmd_vel_pub.publish(cmd_vel_msg);

    // Debug messages
    ROS_INFO("Left Dist: %.2f, Right Dist: %.2f, Min Dist: %.2f", left_dist, right_dist, min_distance);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "corridor_navigation_node");
    ros::NodeHandle nh;

    // Publisher for velocity commands
    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    // Subscriber to laser scan data
    ros::Subscriber laser_sub = nh.subscribe("/scan", 10, laserCallback);

    ROS_INFO("Corridor Navigation Node Started");
    ros::spin();
    return 0;
}

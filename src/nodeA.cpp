#include "ros/ros.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "nodeA");
    ros::NodeHandle nh;

    ROS_INFO("Node A is running...");

    ros::spin(); // Keep the node alive.
    return 0;
}


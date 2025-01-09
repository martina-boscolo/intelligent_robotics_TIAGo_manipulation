#include <ros/ros.h>
#include "ir2425_group_08/PlaceGoal.h"

std::string NODE_A_SRV = "/place_goal";

bool handlePlaceGoal(ir2425_group_08::PlaceGoal::Request& req, ir2425_group_08::PlaceGoal::Response& res) {
    ROS_INFO_STREAM("Received " << req.num_goals << " goals for pick and place");
    res.success = true;  // Assuming a response member named `success`.
    return true;         // Return true to indicate the service was processed successfully.
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "node_b");
    ros::NodeHandle nh;

    ros::ServiceServer server = nh.advertiseService(NODE_A_SRV, handlePlaceGoal);

    ROS_INFO("Node B is ready to handle place_goal service.");
    ros::spin();

    return 0;
}
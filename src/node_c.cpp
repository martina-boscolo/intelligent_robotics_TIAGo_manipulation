#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

#include "ir2425_group_08/PickAndPlaceAction.h"

actionlib::SimpleActionServer<ir2425_group_08::PickAndPlaceAction>* as_ptr;

void pickAndPlaceCallback(const ir2425_group_08::PickAndPlaceGoalConstPtr& goal)
{
    ROS_INFO_STREAM("Got tag " << goal->id << " with pose " << goal->goal_pose);
    ir2425_group_08::PickAndPlaceResult result;
    result.success = true;
    as_ptr->setSucceeded(result);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "node_c");
    ros::NodeHandle nh;

    actionlib::SimpleActionServer<ir2425_group_08::PickAndPlaceAction> as(nh, "/pick_and_place", pickAndPlaceCallback, false);
    as_ptr = &as;

    as.start();
    ROS_INFO("Server started!");

    ros::spin();

    return 0;
}
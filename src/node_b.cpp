#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <apriltag_ros/AprilTagDetectionArray.h>

#include "ir2425_group_08/PlaceService.h"
#include "ir2425_group_08/PickAndPlaceAction.h"

std::string NODE_A_SRV = "/place_goal";
std::vector<geometry_msgs::Point> PlaceServicePoints;
int numPlaceServicePoints;

std::vector<geometry_msgs::Pose> foundTags;
std::vector<int> foundTagIds;

actionlib::SimpleActionClient<ir2425_group_08::PickAndPlaceAction>* ac_ptr;

bool handlePlaceService(ir2425_group_08::PlaceService::Request& req, ir2425_group_08::PlaceService::Response& res) {
    ROS_INFO_STREAM("Received " << req.num_goals << " goals for pick and place");

    PlaceServicePoints = req.target_points;
    numPlaceServicePoints = req.num_goals; // remove it, redoundant

    res.success = true;  // Assuming a response member named `success`.
    return true;         // Return true to indicate the service was processed successfully.
}

void tagsCallback(const apriltag_ros::AprilTagDetectionArrayConstPtr& msg)
{
    for(auto detection : msg->detections)
    {
        if (detection.id[0] != 10)
        {
            if (std::find(foundTagIds.begin(), foundTagIds.end(), detection.id[0]) == foundTagIds.end())
            {
                ROS_INFO_STREAM("Found apriltag " << detection.id[0] << " at pose " << detection.pose.pose.pose);
                foundTags.push_back(detection.pose.pose.pose);
                foundTagIds.push_back(detection.id[0]);

                ir2425_group_08::PickAndPlaceGoal goal;
                goal.goal_pose = detection.pose.pose.pose;
                goal.id = detection.id[0];

                ac_ptr->sendGoal(goal);
            }
        }
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "node_b");
    ros::NodeHandle nh;

    ros::ServiceServer server = nh.advertiseService(NODE_A_SRV, handlePlaceService);

    ROS_INFO("Node B is ready to handle place_goal service.");

    actionlib::SimpleActionClient<ir2425_group_08::PickAndPlaceAction> ac("/pick_and_place", true);
    ac_ptr = &ac;

    ROS_INFO("Waiting for pick_and_place server to start (launch node C!)");
    ac.waitForServer();
    ROS_INFO("pick_and_place server started!");

    ros::Subscriber sub = nh.subscribe("/tag_detections", 10, tagsCallback);

    ros::spin();

    return 0;
}
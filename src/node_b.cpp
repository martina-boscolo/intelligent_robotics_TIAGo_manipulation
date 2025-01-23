#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <tf/transform_listener.h>
#include "ir2425_group_08/RouteHandler.h"
#include "ir2425_group_08/PlaceService.h"
#include "ir2425_group_08/PickAndPlaceAction.h"

std::vector<geometry_msgs::Pose> target_poses = {
    // First pose
    []() {
        geometry_msgs::Pose pose;
        pose.position.x = 8.807;
        pose.position.y = -2.777;
        pose.position.z = 0.0;
        pose.orientation.x = 0.0;
        pose.orientation.y = 0.0;
        pose.orientation.z = 1.0;
        pose.orientation.w = 0.009;
        return pose;
     }(),
    // []() {
    //     geometry_msgs::Pose pose;
    //     pose.position.x = 8.828;
    //     pose.position.y = -1.972;
    //     pose.position.z = 0.0;
    //     pose.orientation.x = 0.0;
    //     pose.orientation.y = 0.0;
    //     pose.orientation.z = -1.0;
    //     pose.orientation.w = 0.0;
    //     return pose;
    // }()
};

int placed_tags = 0;
std::vector<int> foundTagIds;

std::string NODE_A_SRV = "/place_goal";
std::vector<geometry_msgs::Point> PlaceServicePoints;
int numPlaceServicePoints;

ir2425_group_08::RouteHandler* rh_ptr;
actionlib::SimpleActionClient<ir2425_group_08::PickAndPlaceAction> *ac_ptr;
ros::NodeHandle* nh_ptr;

geometry_msgs::PoseStamped transformTagPose(const geometry_msgs::Pose& tag_pose, const std::string& source_frame) {
    tf::TransformListener listener;
    geometry_msgs::PoseStamped pos_in, pos_out;
    std::string target_frame = "map";

    while (!listener.canTransform(target_frame, source_frame, ros::Time(0))) {
        ros::Duration(0.5).sleep();
    }

    pos_in.header.frame_id = source_frame;
    pos_in.pose = tag_pose;
    listener.transformPose(target_frame, pos_in, pos_out);

    return pos_out;
}

bool scanForTags() {
    ROS_INFO("Scanning for AprilTags...");
    apriltag_ros::AprilTagDetectionArrayConstPtr msg = 
        ros::topic::waitForMessage<apriltag_ros::AprilTagDetectionArray>("/tag_detections", *nh_ptr, ros::Duration(5.0));
    
    if (!msg) {
        ROS_WARN("No AprilTag detections received within timeout");
        return false;
    }

    bool tagsFound = false;
    for (const auto& detection : msg->detections) {
        if (!(detection.id[0] == 10) && placed_tags < numPlaceServicePoints) {
            geometry_msgs::PoseStamped transformed_pose = 
                transformTagPose(detection.pose.pose.pose, detection.pose.header.frame_id);

                ROS_INFO_STREAM("Found apriltag " << detection.id[0]);

            if (std::find(foundTagIds.begin(), foundTagIds.end(), detection.id[0]) == foundTagIds.end()){
                foundTagIds.push_back(detection.id[0]);
                ir2425_group_08::PickAndPlaceGoal goal;
                goal.goal_pose = transformed_pose.pose;
                goal.id = detection.id[0];
                ROS_INFO_STREAM("Sending apriltag " << detection.id[0] << "as goal.");
                ac_ptr->sendGoal(goal);
                ROS_INFO("Waiting for result...");
                bool finished_before_timeout = ac_ptr->waitForResult(ros::Duration(90.0)); //might need more time
                if (finished_before_timeout) {
                    ROS_INFO_STREAM("Tag " << detection.id[0] << " placed on the 'place table'.");
                    placed_tags++;
                }
                else {
                    ROS_INFO("Action did not finish before the time out."); // maybe wait for node c to reset
                }

                tagsFound = true;
            }
        }
    }

    return tagsFound;
}



void poseReachedCallback(const actionlib::SimpleClientGoalState& state) {
    if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
        scanForTags();  // Your tag scanning function
    }
}

bool handlePlaceService(ir2425_group_08::PlaceService::Request &req, ir2425_group_08::PlaceService::Response &res) {
    ROS_INFO_STREAM("Received " << req.num_goals << " goals for pick and place");

    PlaceServicePoints = req.target_points;
    numPlaceServicePoints = req.num_goals;

    if(placed_tags < numPlaceServicePoints) {
        // Set up the done callback before following poses
        rh_ptr->followPosesAsync(target_poses, poseReachedCallback);
    }

    res.success = true;
    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "node_b");
    ros::NodeHandle nh;
    nh_ptr = &nh;

    ros::ServiceServer server = nh.advertiseService(NODE_A_SRV, handlePlaceService);
    ROS_INFO("Node B is ready to handle place_goal service.");

    ir2425_group_08::RouteHandler rh;
    rh_ptr = &rh;

    actionlib::SimpleActionClient<ir2425_group_08::PickAndPlaceAction> ac("/pick_and_place", true);
    ac_ptr = &ac;

    ROS_INFO("Waiting for pick_and_place server to start (launch node C!)");
    ac.waitForServer();
    ROS_INFO("pick_and_place server started!");

    ros::spin();
    return 0;
}
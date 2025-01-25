#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <tf/transform_listener.h>
#include <cmath>
#include "ir2425_group_08/RouteHandler.h"
#include "ir2425_group_08/PlaceService.h"
#include "ir2425_group_08/PickAndPlaceAction.h"

using NodeHandleShared = std::shared_ptr<ros::NodeHandle>;

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

int PLACED_TAGS = 0;
constexpr double TIAGO_ARM_MAX_REACH = 0.70;
std::vector<int> foundTagIds;
bool tagsFound = true;

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


bool isPoseWithinArmReachXY(const geometry_msgs::PoseStamped &target_pose_map, double max_reach = TIAGO_ARM_MAX_REACH)
{
    static tf::TransformListener tf_listener;

    try
    {
        tf::StampedTransform transform;
        tf_listener.waitForTransform("base_link", "map", ros::Time(0), ros::Duration(2.0));
        tf_listener.lookupTransform("base_link", "map", ros::Time(0), transform);

        double robot_x = transform.getOrigin().x();
        double robot_y = transform.getOrigin().y();

        double target_x = target_pose_map.pose.position.x;
        double target_y = target_pose_map.pose.position.y;

        double distance_xy = std::sqrt(
            std::pow(target_x - robot_x, 2) +
            std::pow(target_y - robot_y, 2));

        return distance_xy <= max_reach;
    }
    catch (tf::TransformException &ex)
    {
        ROS_WARN("Failed to lookup transform from 'map' to 'base_link': %s", ex.what());
        return false;
    }
}

void sendGoalTag(geometry_msgs::PoseStamped transformed_pose, int id) {
    ir2425_group_08::PickAndPlaceGoal goal;
    goal.goal_pose = transformed_pose.pose;
    goal.id = id;
    //goal.target_point = PlaceServicePoints[PLACED_TAGS];
    ROS_INFO_STREAM("Sending apriltag " << id << " as goal.");
    ac_ptr->sendGoal(goal);
    ROS_INFO("Waiting for result...");
    bool finished_before_timeout = ac_ptr->waitForResult(ros::Duration(90.0)); //might need more time
    if (finished_before_timeout) {
        ROS_INFO_STREAM("Apriltag " << id << " placed on the 'place table'.");
        PLACED_TAGS++;
    }
    else {
        ROS_INFO("Action did not finish before the time out."); // maybe wait for node c to reset
    }
}

void scanForTags() {
    ROS_INFO("Scanning for AprilTags...");
    apriltag_ros::AprilTagDetectionArrayConstPtr msg = 
        ros::topic::waitForMessage<apriltag_ros::AprilTagDetectionArray>("/tag_detections", *nh_ptr, ros::Duration(5.0));
    
    if (!msg) {
        ROS_WARN("No AprilTag detections received within timeout");
        return;
    }

    // for (const auto& detection : msg->detections) {
    //     if (!(detection.id[0] == 10) && PLACED_TAGS < numPlaceServicePoints) {
    //         geometry_msgs::PoseStamped transformed_pose = transformTagPose(detection.pose.pose.pose, detection.pose.header.frame_id);

    //         ROS_INFO_STREAM("Found apriltag " << detection.id[0]);

    //         if(true) { /*isPoseWithinArmReachXY(transformed_pose)*/
    //             if (std::find(foundTagIds.begin(), foundTagIds.end(), detection.id[0]) == foundTagIds.end()){
    //                 foundTagIds.push_back(detection.id[0]);
                    
    //                 sendGoalTag(transformed_pose, detection.id[0]);

    //                 tagsFound = true;
    //             }
    //         }
    //         else {
    //             ROS_INFO_STREAM("Apriltag " << detection.id[0] << " out of reach.");
    //         }
    //     }
    // }

    std::vector<int> new_ids; // potentially valid tags
    for (const auto& detection : msg->detections)
    {
        if (!(detection.id[0] == 10) && std::find(foundTagIds.begin(), foundTagIds.end(), detection.id[0]) == foundTagIds.end())
        {
            new_ids.push_back(detection.id[0]);
        }
    }

    if (new_ids.empty())
    {
        tagsFound = false;
        ROS_INFO("No valid tags found in this scan");
    } else 
    {
        ROS_INFO_STREAM("Found apriltag " << new_ids[0]);

        for (auto detection : msg->detections)
        {
            if (detection.id[0] == new_ids[0])
            {
                geometry_msgs::PoseStamped transformed_pose = transformTagPose(detection.pose.pose.pose, detection.pose.header.frame_id);
                foundTagIds.push_back(new_ids[0]);

                sendGoalTag(transformed_pose, new_ids[0]);
            }
        }
    }
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

    //implement here the loop untill numPlaceServicePoints are placed, go to next anchor point, scan and send, repeat if needed
    // if(PLACED_TAGS < numPlaceServicePoints) {
    //     // Set up the done callback before following poses
    //     //rh_ptr->followPosesAsync(target_poses, poseReachedCallback);
    //     rh_ptr->goFrontPick(poseReachedCallback);
    // }

    ROS_INFO("Moving front...");
    tagsFound = true;

    while(tagsFound) // && PLACED_TAGS < numPlaceServicePoints
    {
        rh_ptr->goFrontPick(poseReachedCallback);
    }
    ROS_INFO_STREAM("All reachable apriltags detected from front sent. " << numPlaceServicePoints - PLACED_TAGS 
        << " apriltags remain in order to complete the task.");
    tagsFound = true;

    while(tagsFound) // && PLACED_TAGS < numPlaceServicePoints
    {
        rh_ptr->goAsidePick(poseReachedCallback);
    }
    ROS_INFO_STREAM("All reachable apriltags detected from aside sent. " << numPlaceServicePoints - PLACED_TAGS 
        << " apriltags remain in order to complete the task.");
    tagsFound = true;

    while(tagsFound) // && PLACED_TAGS < numPlaceServicePoints
    {
        rh_ptr->goBackPick(poseReachedCallback);
    }
    ROS_INFO_STREAM("All reachable apriltags detected from back sent. " << numPlaceServicePoints - PLACED_TAGS 
        << " apriltags remain in order to complete the task.");

    // do
    // {
    //     mi muovo in questo lato usando RouteHandler => passo findTags()
    //     salvo in findTags() tutti i tag che ho visto in un vettore apposito per questo lato del tavolo (vedi while)
    // } while(ci sono ancora tag prendibili da questo lato);

    // per ogni lato

    res.success = true;
    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "node_b");
    auto nh = std::make_shared<ros::NodeHandle>();
    NodeHandleShared nh_ptr_shared(nh);
    nh_ptr = nh.get();

    ros::ServiceServer server = nh->advertiseService(NODE_A_SRV, handlePlaceService);
    ROS_INFO("Node B is ready to handle place_goal service.");

    ir2425_group_08::RouteHandler rh(nh_ptr_shared);
    rh_ptr = &rh;

    actionlib::SimpleActionClient<ir2425_group_08::PickAndPlaceAction> ac("/pick_and_place", true);
    ac_ptr = &ac;

    ROS_INFO("Waiting for pick_and_place server to start (launch node C!)");
    ac.waitForServer();
    ROS_INFO("pick_and_place server started!");

    ros::spin();
    return 0;
}
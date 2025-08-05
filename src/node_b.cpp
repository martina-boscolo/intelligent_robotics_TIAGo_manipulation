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
#include <ir2425_group_08/DetectedObj.h>

using NodeHandleShared = std::shared_ptr<ros::NodeHandle>;

int placed_tags = 0; // apriltags already correctly placed
std::vector<int> foundTagIds;   // apriltags already found from this side of the table
bool tagsFound = true;

std::string NODE_A_SRV = "/place_goal";
std::vector<geometry_msgs::Point> placeServicePoints;
int numPlaceServicePoints;

ir2425_group_08::RouteHandler* rh_ptr;
actionlib::SimpleActionClient<ir2425_group_08::PickAndPlaceAction> *ac_ptr;
ros::NodeHandle* nh_ptr;

// transform tag_pose in map frame
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

// sends the goal of PickAndPlaceAction and reacts accordingly if the action succeeded or failed
void sendGoalTag(geometry_msgs::PoseStamped transformed_pose, int id, std::vector<ir2425_group_08::DetectedObj> objs ) {
    ir2425_group_08::PickAndPlaceGoal goal;
    goal.goal_pose = transformed_pose.pose;
    goal.id = id;
    goal.detectedObj = objs;
    goal.current_waypoint = rh_ptr->getCurrentWaypointIndex();

    goal.target_points = placeServicePoints;
    ROS_INFO_STREAM("Sending apriltag " << id << " as goal.");
    ac_ptr->sendGoal(goal);
    ROS_INFO("Waiting for result...");
    ac_ptr->waitForResult();
    auto actionResult = ac_ptr->getResult();
    // action succeeded, update the number of placed tags and remove the used target point from the list
    if (actionResult->success) {
        ROS_INFO_STREAM("Apriltag " << id << " placed on the 'place table'.");
        placed_tags++;
        placeServicePoints.erase(placeServicePoints.begin() + actionResult->selected_target_index);
    }
    else {
        ROS_INFO("Action failed."); 
    }
    rh_ptr->setCurrentWaypointIndex(actionResult->new_current_waypoint);
}

// checks if there are new pickable tags from this side of the table
void scanForTags() {
    ROS_INFO("Scanning for AprilTags...");
    apriltag_ros::AprilTagDetectionArrayConstPtr msg = 
        ros::topic::waitForMessage<apriltag_ros::AprilTagDetectionArray>("/tag_detections", *nh_ptr, ros::Duration(5.0));
    
    if (!msg) {
        ROS_WARN("No AprilTag detections received within timeout");
        return;
    }

    std::vector<ir2425_group_08::DetectedObj> objs;

    std::vector<int> new_ids; // potentially valid tags
    std::vector<geometry_msgs::PoseStamped> new_collision_objects_poses;


    for (const auto& detection : msg->detections)
    {
        // check if the detected apriltag does not have id 10 and is not yet been discovered (tiago has never tried to pick it up from this side)
        if (!(detection.id[0] == 10) && std::find(foundTagIds.begin(), foundTagIds.end(), detection.id[0]) == foundTagIds.end())
        {
            ir2425_group_08::DetectedObj obj;
            obj.id = detection.id[0];
            obj.pose =transformTagPose(detection.pose.pose.pose, detection.pose.header.frame_id).pose;
            objs.push_back(obj);
        }
    }

    if (objs.empty())
    {
        // no valid tags found from this side of the table
        tagsFound = false;
        ROS_INFO("No valid tags found in this scan");
    } else 
    {
        ROS_INFO_STREAM("Found apriltag " << objs[0].id);

        for (auto detection : msg->detections)
        {
            if (detection.id[0] == objs[0].id)
            {
                geometry_msgs::PoseStamped transformed_pose = transformTagPose(detection.pose.pose.pose, detection.pose.header.frame_id);
                foundTagIds.push_back(objs[0].id);

                // if valid tag found send it
                sendGoalTag(transformed_pose, objs[0].id, objs);
            }
        }
    }
}

bool handlePlaceService(ir2425_group_08::PlaceService::Request &req, ir2425_group_08::PlaceService::Response &res) {
    ROS_INFO_STREAM("Received: " << req.num_goals << " goals for pick and place / " << req.target_points.size() << " available locations.");

    placeServicePoints = req.target_points;
    numPlaceServicePoints = req.num_goals;

    ROS_INFO("Moving front...");
    tagsFound = true;
    foundTagIds.clear();

    while(tagsFound && placed_tags < numPlaceServicePoints)
    {
        rh_ptr->goFrontPick();
        scanForTags();
    }
    if (placed_tags < numPlaceServicePoints)
    {
        ROS_INFO_STREAM("All reachable apriltags detected from front sent. " << numPlaceServicePoints - placed_tags 
            << " apriltags remain in order to complete the task.");
    }
    tagsFound = true;
    foundTagIds.clear();

    while(tagsFound && placed_tags < numPlaceServicePoints)
    {
        rh_ptr->goAsidePick();
        scanForTags();
    }
    if (placed_tags < numPlaceServicePoints)
    {
        ROS_INFO_STREAM("All reachable apriltags detected from aside sent. " << numPlaceServicePoints - placed_tags 
            << " apriltags remain in order to complete the task.");
    }
    tagsFound = true;
    foundTagIds.clear();

    while(tagsFound && placed_tags < numPlaceServicePoints)
    {
        rh_ptr->goBackPick();
        scanForTags();
    }
    if (placed_tags < numPlaceServicePoints)
    {
        ROS_INFO_STREAM("All reachable apriltags detected from back sent. " << numPlaceServicePoints - placed_tags 
            << " apriltags remain in order to complete the task.");
    }
    ROS_INFO_STREAM("IT'S OVER, PLACED " << placed_tags << " APRILTAGS");

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
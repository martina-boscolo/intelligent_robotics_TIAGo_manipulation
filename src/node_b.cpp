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
     }()
};

int placed_tags = 0;
constexpr double TIAGO_ARM_MAX_REACH = 0.75;
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

void sendGoalTag(geometry_msgs::PoseStamped transformed_pose, int id, std::vector<ir2425_group_08::DetectedObj> objs ) {
    ir2425_group_08::PickAndPlaceGoal goal;
    goal.goal_pose = transformed_pose.pose;
    goal.id = id;
    goal.detectedObj = objs;
    goal.current_waypoint = rh_ptr->getCurrentWaypointIndex();

    goal.target_point = PlaceServicePoints[placed_tags];
    ROS_INFO_STREAM("Sending apriltag " << id << " as goal.");
    ac_ptr->sendGoal(goal);
    ROS_INFO("Waiting for result...");
    ac_ptr->waitForResult();
    auto actionResult = ac_ptr->getResult();
    if (actionResult->success) {
        ROS_INFO_STREAM("Apriltag " << id << " placed on the 'place table'.");
        rh_ptr->setCurrentWaypointIndex(actionResult->new_current_waypoint);
        placed_tags++;
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

    std::vector<ir2425_group_08::DetectedObj> objs;

    std::vector<int> new_ids; // potentially valid tags
    std::vector<geometry_msgs::PoseStamped> new_collision_objects_poses;


    for (const auto& detection : msg->detections)
    {
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
        tagsFound = false;
        ROS_INFO("No valid tags found in this scan");
    } else 
    {
        ROS_INFO_STREAM("Found apriltag " << objs[0].id);

        for (auto detection : msg->detections)
        {
            if (detection.id[0] == objs[0].id)
            {
                // add check for distance, if too far away don't consider it since it may be pickable from another side of the table
                geometry_msgs::PoseStamped transformed_pose = transformTagPose(detection.pose.pose.pose, detection.pose.header.frame_id);
                foundTagIds.push_back(objs[0].id);

                sendGoalTag(transformed_pose, objs[0].id, objs);
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

    ROS_INFO("Moving front...");
    tagsFound = true;

    while(tagsFound) // && placed_tags < numPlaceServicePoints
    {
        rh_ptr->goFrontPick(poseReachedCallback);
    }
    ROS_INFO_STREAM("All reachable apriltags detected from front sent. " << numPlaceServicePoints - placed_tags 
        << " apriltags remain in order to complete the task.");
    tagsFound = true;

    while(tagsFound) // && placed_tags < numPlaceServicePoints
    {
        rh_ptr->goAsidePick(poseReachedCallback);
    }
    ROS_INFO_STREAM("All reachable apriltags detected from aside sent. " << numPlaceServicePoints - placed_tags 
        << " apriltags remain in order to complete the task.");
    tagsFound = true;

    while(tagsFound) // && placed_tags < numPlaceServicePoints
    {
        rh_ptr->goBackPick(poseReachedCallback);
    }
    ROS_INFO_STREAM("All reachable apriltags detected from back sent. " << numPlaceServicePoints - placed_tags 
        << " apriltags remain in order to complete the task.");
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
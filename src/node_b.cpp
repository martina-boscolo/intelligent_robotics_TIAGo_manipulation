#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <tf/transform_listener.h>

#include "ir2425_group_08/RouteHandler.h"
#include "ir2425_group_08/PlaceService.h"
#include "ir2425_group_08/PickAndPlaceAction.h"

// tavolo pick: Setting goal: Frame:map, Position(8.907, -2.977, 0.000), Orientation(0.000, 0.000, 1.000, 0.009) = Angle: 3.124

std::vector<geometry_msgs::Pose> target_poses = {
    // First pose
    []() {
        geometry_msgs::Pose pose;
        pose.position.x = 8.807;
        pose.position.y = -2.977;
        pose.position.z = 0.0;
        pose.orientation.x = 0.0;
        pose.orientation.y = 0.0;
        pose.orientation.z = 1.0;
        pose.orientation.w = 0.009;
        return pose;
    }()
};

std::string NODE_A_SRV = "/place_goal";
std::vector<geometry_msgs::Point> PlaceServicePoints;
int numPlaceServicePoints;

bool callbackToNodeCOn = false;

std::vector<geometry_msgs::Pose> foundTags;
std::vector<int> foundTagIds;
ir2425_group_08::RouteHandler* rh_ptr;

actionlib::SimpleActionClient<ir2425_group_08::PickAndPlaceAction> *ac_ptr;

bool handlePlaceService(ir2425_group_08::PlaceService::Request &req, ir2425_group_08::PlaceService::Response &res)
{
    ROS_INFO_STREAM("Received " << req.num_goals << " goals for pick and place");

    PlaceServicePoints = req.target_points;
    numPlaceServicePoints = req.num_goals; // remove it, redoundant

    rh_ptr->followPoses(target_poses);

    res.success = true; // Assuming a response member named `success`.
    callbackToNodeCOn = true;
    return true;        // Return true to indicate the service was processed successfully.
}

// void tagsCallback(const apriltag_ros::AprilTagDetectionArrayConstPtr &msg)
// {
//     if (callbackToNodeCOn)
//     {
//         for (auto detection : msg->detections)
//         {
//             if (detection.id[0] != 10)
//             {
//                 if (std::find(foundTagIds.begin(), foundTagIds.end(), detection.id[0]) == foundTagIds.end())
//                 {

//                     tf::TransformListener listener;
//                     tf::StampedTransform transform;
//                     std::string target_frame = "base_link";
//                 // std::string source_frame = msg->header.frame_id;
//                     std::string source_frame = detection.pose.header.frame_id;

//                     while (!listener.canTransform(target_frame, source_frame, ros::Time(0)))
//                         ros::Duration(0.5).sleep();

//                     // Transform available
//                     geometry_msgs::PoseStamped pos_in;
//                     geometry_msgs::PoseStamped pos_out;

                    

//                         // if (msg->detections.at(i).pose.header.frame_id == "tag_10")
//                         // { detection.pose.pose.pose;
//                         pos_in.header.frame_id = detection.pose.header.frame_id;
//                         pos_in.pose.position.x = detection.pose.pose.pose.position.x;
//                         pos_in.pose.position.y = detection.pose.pose.pose.position.y;
//                         pos_in.pose.position.z = detection.pose.pose.pose.position.z;
//                         pos_in.pose.orientation.x = detection.pose.pose.pose.orientation.x;
//                         pos_in.pose.orientation.y = detection.pose.pose.pose.orientation.y;
//                         pos_in.pose.orientation.z = detection.pose.pose.pose.orientation.z;
//                         pos_in.pose.orientation.w = detection.pose.pose.pose.orientation.w;

//                         listener.transformPose(target_frame, pos_in, pos_out);

//                         ROS_INFO_STREAM("Obj with ID: " << detection.id[0]);
//                         ROS_INFO_STREAM("Original pose\n"
//                                         << pos_in);
//                         ROS_INFO_STREAM("Transformed pose\n"
//                                         << pos_out);
//                         // tag10Processed = true;
//                         // }
                    

//                     ROS_INFO_STREAM("Found apriltag " << detection.id[0] << " at pose " << pos_out.pose);
//                     foundTags.push_back(pos_out.pose);
//                     foundTagIds.push_back(detection.id[0]);

//                     ir2425_group_08::PickAndPlaceGoal goal;
//                     goal.goal_pose = pos_out.pose;
//                     goal.id = detection.id[0];

//                     ac_ptr->sendGoal(goal);
//                 }
//             }
//         }
//     }
// }

void tagsCallback(const apriltag_ros::AprilTagDetectionArrayConstPtr &msg)
{
    if (!callbackToNodeCOn)
        return;

    for (const auto &detection : msg->detections)
    {
        int tag_id = detection.id[0]; // Extract the tag ID

        // Skip if the tag is already processed
        if (std::find(foundTagIds.begin(), foundTagIds.end(), tag_id) != foundTagIds.end())
            continue;

        // Determine the tag frame name (e.g., "tag_1", "tag_2", etc.)
        std::string tag_frame = "tag_" + std::to_string(tag_id);

        // Create the pose in the tag's own frame
        geometry_msgs::PoseStamped pose_in_tag_frame;
        pose_in_tag_frame.header.frame_id = tag_frame; // Use the tag's frame
        pose_in_tag_frame.header.stamp = ros::Time::now();

        // Position in the tag frame is the origin (0, 0, 0)
        pose_in_tag_frame.pose.position.x = 0.0;
        pose_in_tag_frame.pose.position.y = 0.0;
        pose_in_tag_frame.pose.position.z = 0.0;

        // Orientation in the tag frame is an identity quaternion (no rotation)
        pose_in_tag_frame.pose.orientation.x = 0.0;
        pose_in_tag_frame.pose.orientation.y = 0.0;
        pose_in_tag_frame.pose.orientation.z = 0.0;
        pose_in_tag_frame.pose.orientation.w = 1.0;

        // Log the tag information
        ROS_INFO_STREAM("Found AprilTag " << tag_id << " with pose in frame " << tag_frame);
        ROS_INFO_STREAM("Pose in " << tag_frame << ": " << pose_in_tag_frame);

        // Add the tag to the list of processed tags
        foundTagIds.push_back(tag_id);

        // Send the pose in the tag frame to Node C
        ir2425_group_08::PickAndPlaceGoal goal;
        goal.goal_pose = pose_in_tag_frame.pose;
        goal.id = tag_id;

        // Send the goal to the action client
        ac_ptr->sendGoal(goal);

        ROS_INFO_STREAM("Sent goal for AprilTag " << tag_id << " to Node C.");
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "node_b");
    ros::NodeHandle nh;

    ros::ServiceServer server = nh.advertiseService(NODE_A_SRV, handlePlaceService);

    ROS_INFO("Node B is ready to handle place_goal service.");

    ir2425_group_08::RouteHandler rh;
    rh_ptr = &rh;

    actionlib::SimpleActionClient<ir2425_group_08::PickAndPlaceAction> ac("/pick_and_place", true);
    ac_ptr = &ac;

    ROS_INFO("Waiting for pick_and_place server to start (launch node C!)");
    ac.waitForServer();
    ROS_INFO("pick_and_place server started!");

    ros::Subscriber sub = nh.subscribe("/tag_detections", 10, tagsCallback);

    ros::spin();

    return 0;
}
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>

#include <ir2425_group_08/FindTagsAction.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <apriltag_ros/AprilTagDetection.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <control_msgs/PointHeadAction.h>
#include <control_msgs/PointHeadGoal.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_listener.h>

#include <string>
#include <math.h>

#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>

#include <thread> 
#include <atomic> 

geometry_msgs::Point createPoint(double x, double y, double z)
{
    geometry_msgs::Point p;
    p.x = x;
    p.y = y;
    p.z = z;
    return p;
}

std::vector<geometry_msgs::Point> WAYPOINT_LIST =
    {
        createPoint(0.0, 0.0, 0.0),
        createPoint(9.5, 0.0, 0.0),
        createPoint(12, 1.0, 0.0),
        createPoint(12.0, -1.0, 0.0),
        createPoint(9.5, -4.0, 0.0),
        createPoint(8.5, -1.5, 0.0),
        createPoint(12.5, -3.0, 0.0),
        createPoint(13.5, -1.5, 0.0),
        createPoint(0.0, 0.0, 0.0),
};

class FindTags
{
protected:
    actionlib::SimpleActionServer<ir2425_group_08::FindTagsAction> as_;
    ir2425_group_08::FindTagsFeedback feedback_;
    ir2425_group_08::FindTagsResult result_;
    ros::Subscriber sub_;
    tf::TransformListener tf_listener_;

    std::vector <apriltag_ros::AprilTagDetection> foundTags;

    std::vector<int> Ids;
    std::vector<int> AlreadyFoundIds;

    std::atomic<bool> keepHeadMoving;
    std::thread headMovementThread;
    bool isNewAndValidTag(apriltag_ros::AprilTagDetection tag)
    {
        if (tag.id.empty())
            return false;

        int tagId = tag.id[0];
        bool isValid = std::find(this->Ids.begin(), this->Ids.end(), tagId) != this->Ids.end();
        bool isNew = std::find(this->AlreadyFoundIds.begin(), this->AlreadyFoundIds.end(), tagId) == this->AlreadyFoundIds.end();

        return isValid && isNew;
    }

public:
    FindTags(ros::NodeHandle &nh,  std::string server_name) : as_(nh, server_name, boost::bind(&FindTags::mainCycle, this, _1), false)
    {
        this->as_.start();

        this->sub_ = nh.subscribe("tag_detections", 1, &FindTags::tagsCallback, this);
        ROS_INFO("Server and tag_detections subscriber started");
    }

    void tagsCallback(const apriltag_ros::AprilTagDetectionArrayConstPtr &msg)
    {
        for (const auto &tag : msg->detections)
        {
            if (tag.id.empty())
                continue;

            int tagId = tag.id[0];
            if (this->isNewAndValidTag(tag))
            {
                bool isPresent = false;
                // Add the tag ID to AlreadyFoundIds only if not already present
                for (int i =0; i< this->AlreadyFoundIds.size(); i++){
                    if (AlreadyFoundIds[i]== tagId)
                      isPresent = true;
                      break;
                }
                if(!isPresent)
                    this->AlreadyFoundIds.push_back(tagId);

                // if (std::find(this->AlreadyFoundIds.begin(), this->AlreadyFoundIds.end(), tagId) == this->AlreadyFoundIds.end())
                // {
                //     this->AlreadyFoundIds.push_back(tagId);
                // }

                // Check if this tag already exists in feedback_.current_detection
                // bool alreadyExists = std::any_of(
                //     this->feedback_.current_detection.begin(),
                //     this->feedback_.alreacurrent_detectiondyFoundTags.end(),
                //     [&](const apriltag_ros::AprilTagDetection &existingTag)
                //     {
                //         return existingTag.id[0] == tagId;
                //     });

                //LE STAMPE DEVONO ESSERE SISTEMATE
                if (!isPresent)
                {
                    
                    ROS_INFO("New valid tag detected: %d", tagId);
                    ROS_INFO_STREAM("Position (respect to camera) - x: " << tag.pose.pose.pose.position.x
                                                                         << ", y: " << tag.pose.pose.pose.position.y
                                                                         << ", z: " << tag.pose.pose.pose.position.z);

                    // Transform the pose from camera frame to map frame
                    geometry_msgs::PoseStamped pose_in_camera_frame;
                    geometry_msgs::PoseStamped pose_in_map_frame;

                    pose_in_camera_frame.header.frame_id = tag.pose.header.frame_id;
                    pose_in_camera_frame.header.stamp = ros::Time(0);  // Copy the frame_id and timestamp
                    pose_in_camera_frame.pose = tag.pose.pose.pose; // Copy the pose

                    try
                    {
                        // Use the TF listener to transform the pose
                        tf_listener_.waitForTransform("map", pose_in_camera_frame.header.frame_id,
                                                      ros::Time(0), ros::Duration(1.0));
                        tf_listener_.transformPose("map", pose_in_camera_frame, pose_in_map_frame);

                        // Print the transformed pose 
                        ROS_INFO_STREAM("Tag ID: " << tagId << " Position (map frame) - x: "
                                                   << pose_in_map_frame.pose.position.x
                                                   << ", y: " << pose_in_map_frame.pose.position.y
                                                   << ", z: " << pose_in_map_frame.pose.position.z);

                        apriltag_ros::AprilTagDetection newTag;

                        // Assign the transformed pose properly
                        newTag.pose.pose.pose = pose_in_map_frame.pose; // Copy both position and orientation

                        // Set the header correctly (frame ID and timestamp)
                        newTag.pose.header.frame_id = "map"; // Target frame
                        newTag.pose.header.stamp = ros::Time(0);
                        // Copy the tag ID
                        newTag.id = tag.id;

                        foundTags.push_back(newTag);


                        // Publish the feedback
                        this->feedback_.current_detection = newTag;
                        this->feedback_.progress_status = AlreadyFoundIds.size();
                        this->as_.publishFeedback(this->feedback_);
                    }
                    catch (tf::TransformException &ex)
                    {
                        ROS_WARN("Failed to transform pose: %s", ex.what());
                    }
                }
            }
            // else
            // {
            //     ROS_INFO("Invalid or already detected tag: %d", tagId);
            // }
        }
    }

    void mainCycle(const ir2425_group_08::FindTagsGoalConstPtr &goal)
    {
        // Deduplicate the IDs in the goal
        this->Ids.clear();
        for (int id : goal->target_ids)
        {
            if (std::find(this->Ids.begin(), this->Ids.end(), id) == this->Ids.end())
            {
                this->Ids.push_back(id);
            }
        }

        int waypointIndex = 0;

        actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> moveClient("move_base", true);

        ROS_INFO("Waiting for move_base...");
        moveClient.waitForServer();

        while (this->AlreadyFoundIds.size() < this->Ids.size() && waypointIndex < WAYPOINT_LIST.size())
        {
            move_base_msgs::MoveBaseGoal waypointGoal;
            waypointGoal.target_pose.header.stamp = ros::Time::now();
            waypointGoal.target_pose.header.frame_id = "map";
            waypointGoal.target_pose.pose.position = WAYPOINT_LIST[waypointIndex];
            waypointGoal.target_pose.pose.orientation.w = 1.0;

            ROS_INFO("Moving to waypoint %d: x=%.2f, y=%.2f",
                     waypointIndex,
                     waypointGoal.target_pose.pose.position.x,
                     waypointGoal.target_pose.pose.position.y);

            moveClient.sendGoal(waypointGoal);

            if (moveClient.waitForResult(ros::Duration(10.0)))
            {
                ROS_INFO("Waypoint %d reached", waypointIndex);
                waypointIndex++;
            }
            else
            {
                ROS_WARN("Timeout reached for waypoint %d", waypointIndex);
            }
        }

        if (waypointIndex >= WAYPOINT_LIST.size())
        {
            ROS_INFO("All waypoints visited");
        }

        ROS_INFO("Publishing result...");
        this->result_.finished = (this->AlreadyFoundIds.size() == this->Ids.size());
        if (this->AlreadyFoundIds.size() == this->Ids.size()){
            this->result_.status_message = "All the apriltags have been found!"; 
            this->result_.detected_tags = foundTags;
        } else
           this->result_.status_message = "Not all the apriltags have been found!";
           
        this->as_.setSucceeded(this->result_);
    }
};

void extendTorso()
{
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac("torso_controller/follow_joint_trajectory", true);

    ROS_INFO("Waiting for torso_controller...");
    ac.waitForServer();

    control_msgs::FollowJointTrajectoryGoal goal;
    trajectory_msgs::JointTrajectoryPoint point;

    goal.trajectory.joint_names.push_back("torso_lift_joint");
    point.positions.push_back(0.35);            // Maximum height (adjust based on Tiago specs)
    point.time_from_start = ros::Duration(2.0); // Time to reach the position

    goal.trajectory.points.push_back(point);
    goal.trajectory.header.stamp = ros::Time::now();

    ROS_INFO("Sending goal to torso_controller...");
    ac.sendGoal(goal);
    ac.waitForResult();
    ROS_INFO("Torso fully extended");
}

void lookDown(ros::NodeHandle &nh)
{
    actionlib::SimpleActionClient<control_msgs::PointHeadAction> ac("head_controller/point_head_action", true);

    ROS_INFO("Waiting for head_controller...");
    ac.waitForServer();

    control_msgs::PointHeadGoal goal;
    goal.target.header.stamp = ros::Time(0);
    goal.target.header.frame_id = "xtion_rgb_optical_frame";
    goal.target.point.x = 0.0;
    goal.target.point.y = tan(M_PI / 6); // inclination of 30 degrees
    goal.target.point.z = 1.0;
    goal.pointing_axis.z = 1.0;
    goal.pointing_frame = "xtion_rgb_optical_frame";
    goal.min_duration = ros::Duration(1.0);
    goal.max_velocity = 0.6;

    ROS_INFO("Sending goal to head_controller...");
    ac.sendGoal(goal);
    ac.waitForResult();
    ROS_INFO("Robot is looking down");
}


void cameraCallback(const sensor_msgs::ImageConstPtr &msg)
{
    cv::imshow("camera", cv_bridge::toCvCopy(msg, msg->encoding)->image);
    cv::waitKey(1);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "node_b");
    ros::NodeHandle nh;
    tf::TransformListener tf_listener; //not used here

    // Debug section for camera
    cv::namedWindow("camera");
    ros::Subscriber sub = nh.subscribe("xtion/rgb/image_raw", 1, cameraCallback);

    lookDown(nh);
    extendTorso();

    FindTags findTags(nh, "find_tags");

   ros::spin(); //why it's not finishig at the end? could be the camera
    return 0;
}
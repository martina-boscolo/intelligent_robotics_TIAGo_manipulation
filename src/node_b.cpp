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
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>

#include <string>
#include <math.h>

// just for camera debug
#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>

#include <thread>  // For managing parallel execution

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
    //createPoint(8.312910079956055, 0.2933077812194824, 0.0),
    //createPoint(10.939831733703613, 0.7441291809082031, 0.0),
    createPoint(0.0, 0.0, 0.0),
    createPoint(9.5, 0.0, 0.0),
    createPoint(12.0, -1.0, 0.0),
    createPoint(9.5, -4.0, 0.0),
    createPoint(12.5, -3.0, 0.0),
    createPoint(0.0, 0.0, 0.0),
};

class FindTags
{
protected:
    actionlib::SimpleActionServer<ir2425_group_08::FindTagsAction> as_;
    ir2425_group_08::FindTagsFeedback feedback_;
    ir2425_group_08::FindTagsResult result_;
    ros::Subscriber sub_;

    std::vector<int> Ids;
    std::set<int> AlreadyFoundIds;
    //std::vector<int> AlreadyFoundIds;

    bool isNewAndValidTag(apriltag_ros::AprilTagDetection tag)
    {
        if (tag.id.empty())
            return false;

        int tagId = tag.id[0];
        bool isValid = std::find(this->Ids.begin(), this->Ids.end(), tagId) != this->Ids.end();
        bool isNew = this->AlreadyFoundIds.find(tagId) == this->AlreadyFoundIds.end();

        return isValid && isNew;
    }

public:
    FindTags(ros::NodeHandle& nh, std::string server_name) : as_(nh, server_name, boost::bind(&FindTags::mainCycle, this, _1), false)
    {
        this->as_.start();

        this->sub_ = nh.subscribe("tag_detections", 1, &FindTags::tagsCallback, this);
        ROS_INFO("Server and tag_detections subscriber started");
    }

void tagsCallback(const apriltag_ros::AprilTagDetectionArrayConstPtr& msg)
{
    for (const auto& tag : msg->detections)
    {
        if (tag.id.empty()) continue;

        int tagId = tag.id[0];
        if (this->isNewAndValidTag(tag))
        {
            this->feedback_.alreadyFoundTags.push_back(tag);
            this->AlreadyFoundIds.insert(tag.id[0]);  // Use insert instead of push_back

            this->as_.publishFeedback(this->feedback_);
            ROS_INFO("New valid tag detected: %d", tagId);
        }
        else
        {
            ROS_INFO("Invalid or already detected tag: %d", tagId);
        }
    }
}


void moveHead( double angle, bool toLeft)
{
    actionlib::SimpleActionClient<control_msgs::PointHeadAction> ac("head_controller/point_head_action", true);

    ROS_INFO("Waiting for head_controller...");
    ac.waitForServer();

    control_msgs::PointHeadGoal goal;
    goal.target.header.stamp = ros::Time::now();
    goal.target.header.frame_id = "base_link";  // Adjust to your frame
    goal.target.point.x = 1.0;                 // Focus 1 meter in front
    goal.target.point.y = toLeft ? tan(angle) : -tan(angle);  // Swaying left and right
    goal.target.point.z = 0.0;                 // Keep level
    goal.pointing_axis.z = 1.0;                // Point in Z direction
    goal.pointing_frame = "head_frame";        // Adjust to your robot's head frame
    goal.min_duration = ros::Duration(0.5);    // Minimal duration for head movement
    goal.max_velocity = 1.0;                  // Head movement speed

    ROS_INFO("Sending head movement goal...");
    ac.sendGoal(goal);
    ac.waitForResult();  // Wait for the head to finish moving
}




void mainCycle(const ir2425_group_08::FindTagsGoalConstPtr& goal)
{
    this->Ids = goal->ids;
    int waypointIndex = 0;

    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> moveClient("move_base", true);

    ROS_INFO("Waiting for move_base...");
    moveClient.waitForServer();

    // Move to each waypoint
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

        // Alternate head movement: left or right, every time a new waypoint is reached
        moveHead(M_PI / 6, waypointIndex % 2 == 0);  // Alternate between left and right

        if (moveClient.waitForResult(ros::Duration(10.0)))
        {
            ROS_INFO("Goal reached");
            waypointIndex++;
        }
        else
        {
            ROS_WARN("Timeout reached for waypoint %d", waypointIndex);
        }
    }

    if (waypointIndex >= WAYPOINT_LIST.size())
    {
        ROS_INFO("All waypoints visited...");
    }

    ROS_INFO("Publishing result...");
    this->result_.finished = (this->AlreadyFoundIds.size() == this->Ids.size());
    this->as_.setSucceeded(this->result_);
}

};


void lookDown(ros::NodeHandle& nh)
{
    actionlib::SimpleActionClient<control_msgs::PointHeadAction> ac("head_controller/point_head_action", true);

    ROS_INFO("Waiting for head_controller...");
    ac.waitForServer();

    // move down head by 30 degrees
    control_msgs::PointHeadGoal goal;
    goal.target.header.stamp = ros::Time(0);
    goal.target.header.frame_id = "xtion_rgb_optical_frame";
    goal.target.point.x = 0.0;
    goal.target.point.y = tan(M_PI / 4); // inclination of 45 degree;
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

void cameraCallback(const sensor_msgs::ImageConstPtr& msg)
{
    cv::imshow("camera", cv_bridge::toCvCopy(msg, msg->encoding)->image);
    cv::waitKey(1);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "node_b");
    ros::NodeHandle nh;

    // debug section for camera
    cv::namedWindow("camera");
    ros::Subscriber sub = nh.subscribe("xtion/rgb/image_raw", 1, cameraCallback);

    lookDown(nh);

    FindTags findTags(nh, "find_tags");

    ros::spin();
    return 0;
}
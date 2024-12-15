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
    createPoint(8.312910079956055, 0.2933077812194824, 0.0),
    createPoint(10.939831733703613, 0.7441291809082031, 0.0)
};

class FindTags
{
protected:
    actionlib::SimpleActionServer<ir2425_group_08::FindTagsAction> as_;
    ir2425_group_08::FindTagsFeedback feedback_;
    ir2425_group_08::FindTagsResult result_;
    ros::Subscriber sub_;

    std::vector<int> Ids;
    std::vector<int> AlreadyFoundIds;

    bool isNewAndValidTag(apriltag_ros::AprilTagDetection tag)
    {
        bool isValid = false;
        bool isNew = true;
        for (int targetId : this->Ids)
        {
            if (tag.id[0] == targetId)
            {
                isValid = true;
            }
        }

        if (isValid)
        {
            for (int knownId : this->AlreadyFoundIds)
            {
                if (tag.id[0] == knownId)
                {
                    isNew = false;
                }
            }
        }

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
        for (apriltag_ros::AprilTagDetection tag : msg->detections)
        {
            if(this->isNewAndValidTag(tag))
            {
                this->feedback_.alreadyFoundTags.push_back(tag);
                this->AlreadyFoundIds.push_back(tag.id[0]);

                this->as_.publishFeedback(this->feedback_);
                ROS_INFO("Valid tag detected");
            }
            else
            {
                ROS_INFO("Invalid or already seen tag detected, n %d", tag.id[0]);
            }
        }
    }

    void mainCycle(const ir2425_group_08::FindTagsGoalConstPtr& goal)
    {
        this->Ids = goal->ids;
        int waypointIndex = 0;

        actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> moveClient("move_base", true);

        ROS_INFO("Waiting for move_base...");
        moveClient.waitForServer();

        // until goal is reached <=> all ids are detected
            // move to the next checkpoint (action client) =>
            // until next checkpoint is reached
                // update progresses if there are some

        while (this->AlreadyFoundIds.size() < this->Ids.size() and waypointIndex < WAYPOINT_LIST.size())
        {
            move_base_msgs::MoveBaseGoal waypointGoal;
            waypointGoal.target_pose.header.stamp = ros::Time(0);
            waypointGoal.target_pose.header.frame_id = "map";
            waypointGoal.target_pose.pose.position = WAYPOINT_LIST[waypointIndex];
            waypointGoal.target_pose.pose.orientation.w = 1.0;

            ROS_INFO("Moving to waypoint n %d; x=%f, y=%f", waypointIndex, waypointGoal.target_pose.pose.position.x, waypointGoal.target_pose.pose.position.y);
            moveClient.sendGoal(waypointGoal);

            bool finishedInTime = moveClient.waitForResult(ros::Duration(10.0));
            if (finishedInTime)
            {
                ROS_INFO("Goal reached! Moving to the next");
                waypointIndex++;
            }
            else
            {
                ROS_INFO("Goal not reached in time");
            }
        }

        if (waypointIndex >= WAYPOINT_LIST.size())
        {
            ROS_INFO("All waypoint visited...");
        }

        ROS_INFO("Publishing result...");
        this->result_.finished = this->AlreadyFoundIds.size() == this->Ids.size();
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
    goal.target.point.y = tan(M_PI / 6); // inclination of 30 degree;
    goal.target.point.z = 1.0;
    goal.pointing_axis.z = 1.0;
    goal.pointing_frame = "xtion_rgb_optical_frame";
    goal.min_duration = ros::Duration(1.0);
    goal.max_velocity = 0.5;

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
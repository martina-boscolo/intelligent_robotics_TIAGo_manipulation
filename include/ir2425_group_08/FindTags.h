#ifndef IR2425GROUP08_FINDTAGS_H
#define IR2425GROUP08_FINDTAGS_H

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
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>

#include <string>
#include <math.h>

#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>

using NodeHandleShared = std::shared_ptr<ros::NodeHandle>;

geometry_msgs::Point createPoint(double x, double y, double z)
{
    geometry_msgs::Point p;
    p.x = x;
    p.y = y;
    p.z = z;
    return p;
}

const double MIN_DISTANCE = 0.2; // Minimum allowable distance from walls (meters)
const double MAX_SPEED = 0.2;    // Maximum linear speed (m/s)
const double TURN_GAIN = 0.2;    // Gain for angular velocity adjustment

std::vector<geometry_msgs::Point> WAYPOINT_LIST =
    {
        //createPoint(-0.1, 0.2, 0.0),
        createPoint(12, 1.0, 0.0),
        createPoint(12.0, -1.0, 0.0),
        createPoint(9, -1.9, 0.0), // ADDED
        createPoint(9.5, -4.0, 0.0),
        createPoint(8.5, -1.5, 0.0),
        createPoint(8.7, -3.3, 0.0), // ADDED
        createPoint(8.5, -3.15, 0.0), // ADDED
        createPoint(11.5, -3.2, 0.0), // ADDED
        createPoint(12.5, -3.0, 0.0),
        createPoint(12, 0.0, 0.0), // ADDED
        createPoint(9.7, 0.7, 0.0), // ADDED
        createPoint(0.0, 0.0, 0.0),
        
};

namespace ir2425_group_08
{
    class FindTags
    {
    protected:
        NodeHandleShared nh_ptr_;
        actionlib::SimpleActionServer<ir2425_group_08::FindTagsAction> as_;
        ir2425_group_08::FindTagsFeedback feedback_;
        ir2425_group_08::FindTagsResult result_;
        ros::Subscriber apriltag_sub_;
        ros::Subscriber laser_sub_;
        ros::Publisher cmd_vel_pub_;
        tf::TransformListener tf_listener_;

        actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> moveClient_;

        sensor_msgs::LaserScan::ConstPtr latest_laser_msg_;
        bool corridor_done_;
        bool corridor_feedback_sent_; 

        std::vector<int> Ids;
        std::vector<int> AlreadyFoundIds;
        std::vector <geometry_msgs::PoseStamped> AlreadyFoundTags;

        bool isNewAndValidTag(apriltag_ros::AprilTagDetection tag);

    public:
        FindTags(NodeHandleShared& nh,  std::string server_name);

        void tagsCallback(const apriltag_ros::AprilTagDetectionArrayConstPtr& msg);

        void laserCallback(const sensor_msgs::LaserScanConstPtr& msg);

        void mainCycle(const ir2425_group_08::FindTagsGoalConstPtr& goal);

        void extendTorso();

        void inclineHead(float pitch);

        void performFullSpin();

        bool isInCorridor(const sensor_msgs::LaserScanConstPtr& msg, double& corridor_width);

        void navigateInCorridor(const sensor_msgs::LaserScan::ConstPtr &msg, ros::Publisher &cmd_vel_pub_);
    };
}

#endif
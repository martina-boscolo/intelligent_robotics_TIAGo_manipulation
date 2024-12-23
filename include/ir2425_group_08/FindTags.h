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

using NodeHandleShared = std::shared_ptr<ros::NodeHandle>;

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
        createPoint(-0.1, 0.2, 0.0),
        //createPoint(9.5, 0.0, 0.0), //this does not work
        createPoint(12, 1.0, 0.0),
        //might need a waypoint to see tag 6 here
        createPoint(12.0, -1.0, 0.0),
        createPoint(9.5, -4.0, 0.0),
        createPoint(8.5, -1.5, 0.0),
        createPoint(12.5, -3.0, 0.0),
        //createPoint(13.5, -1.5, 0.0), //this does not work
        createPoint(0.0, 0.0, 0.0),
        
};

namespace ir2425_group_08
{
    class FindTags
    {
    protected:
        actionlib::SimpleActionServer<ir2425_group_08::FindTagsAction> as_;
        ir2425_group_08::FindTagsFeedback feedback_;
        ir2425_group_08::FindTagsResult result_;
        ros::Subscriber sub_;
        tf::TransformListener tf_listener_;

        actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> moveClient_;

        std::vector<int> Ids;
        std::vector<int> AlreadyFoundIds;
        std::vector <geometry_msgs::PoseStamped> AlreadyFoundTags;

        bool isNewAndValidTag(apriltag_ros::AprilTagDetection tag);

    public:
        FindTags(NodeHandleShared& nh,  std::string server_name);

        void tagsCallback(const apriltag_ros::AprilTagDetectionArrayConstPtr& msg);

        void mainCycle(const ir2425_group_08::FindTagsGoalConstPtr& goal);

        void extendTorso();

        void lookDown();
    };
}

#endif
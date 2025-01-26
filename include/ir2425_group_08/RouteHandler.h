#ifndef IR2425GROUP08_ROUTEHANDLER_H
#define IR2425GROUP08_ROUTEHANDLER_H

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseGoal.h>

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>

#include <nav_msgs/Odometry.h>

using NodeHandleShared = std::shared_ptr<ros::NodeHandle>;

namespace ir2425_group_08
{
    class RouteHandler
    {
    protected:
        NodeHandleShared nh_ptr_;

        ros::Publisher cmd_vel_pub_;
        actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac_;
        tf::TransformListener tf_listener_;
        
        //std::vector<geometry_msgs::Pose> current_poses_;
        //size_t current_pose_index_;
        //boost::function<void(const actionlib::SimpleClientGoalState&)> done_callback_;

        std::vector<geometry_msgs::Pose> tablesWaypoints_;
        size_t currentWaypointIndex_;
        float waypointTolerance_;

        //void sendNextPose();
    private:
        static std::vector<geometry_msgs::Pose> initTablesWaypoints();

        size_t getNextIndex(size_t targetIndex);
        geometry_msgs::Pose getRobotPoseInMap();

        void pointTowards(geometry_msgs::Pose target_pose);
        void moveTowards(geometry_msgs::Pose target_pose);
        void alignWithPose(geometry_msgs::Pose target_pose);

        void goToWaypoint(size_t index, boost::function<void(const actionlib::SimpleClientGoalState&)> done_cb);
    public:
        RouteHandler(NodeHandleShared& nh_ptr);
        bool followPoses(std::vector<geometry_msgs::Pose> poses);
        //void followPosesAsync(std::vector<geometry_msgs::Pose> poses, boost::function<void(const actionlib::SimpleClientGoalState&)> done_cb);

        bool fullPickRotation(boost::function<void(const actionlib::SimpleClientGoalState&)> done_cb);
        bool goFrontPick(boost::function<void(const actionlib::SimpleClientGoalState&)> done_cb);
        bool goAsidePick(boost::function<void(const actionlib::SimpleClientGoalState&)> done_cb);
        bool goBackPick(boost::function<void(const actionlib::SimpleClientGoalState&)> done_cb);
        bool goFrontPlace(boost::function<void(const actionlib::SimpleClientGoalState&)> done_cb);
    };
}

#endif
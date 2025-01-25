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
        // all the points in the arguments are assumed to be in the map frame
    protected:
        NodeHandleShared nh_ptr_;

        ros::Publisher cmd_vel_pub_;
        actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac_;
        tf::TransformListener tf_listener_;
        //bool is_robot_moving_;
        std::vector<geometry_msgs::Pose> current_poses_;
        size_t current_pose_index_;
        boost::function<void(const actionlib::SimpleClientGoalState&)> done_callback_;

        std::vector<geometry_msgs::Pose> tablesWaypoints_;
        size_t currentWaypointIndex_;
        float waypointTolerance_;

        void sendNextPose();
    private:
        static std::vector<geometry_msgs::Pose> initTablesWaypoints();
        geometry_msgs::Point transformPoint(const geometry_msgs::Point& point_in, const std::string& source_frame, const std::string& target_frame);
        geometry_msgs::Pose getRobotPoseInMap();

        void goToWaypoint(size_t index, boost::function<void(const actionlib::SimpleClientGoalState&)> done_cb);
        //void goToNextWaypoint(size_t target_index);
        size_t getNextIndex(size_t targetIndex);

        geometry_msgs::Twist getTwistTowardPose(geometry_msgs::Pose start_pose, geometry_msgs::Pose target_pose, float linear_speed);
        bool waypointReached(geometry_msgs::Pose target_pose, double& e_linear);
        bool orientationReached(geometry_msgs::Pose target_pose, double& e_angular);

        void pointTowards(geometry_msgs::Pose target_pose);
        void moveTowards(geometry_msgs::Pose target_pose);
        void alignWithPose(geometry_msgs::Pose target_pose);
    public:
        RouteHandler(NodeHandleShared& nh_ptr);
        bool followPoses(std::vector<geometry_msgs::Pose> poses);
        bool followWaypoints(std::vector<geometry_msgs::Point> waypoints);
        void followPosesAsync(std::vector<geometry_msgs::Pose> poses, boost::function<void(const actionlib::SimpleClientGoalState&)> done_cb);

        bool fullPickRotation(boost::function<void(const actionlib::SimpleClientGoalState&)> done_cb);
        bool goFrontPick(boost::function<void(const actionlib::SimpleClientGoalState&)> done_cb);
        bool goAsidePick(boost::function<void(const actionlib::SimpleClientGoalState&)> done_cb);
        bool goBackPick(boost::function<void(const actionlib::SimpleClientGoalState&)> done_cb);

        //bool isRobotMoving() const { return is_robot_moving_; }
    };
}

#endif
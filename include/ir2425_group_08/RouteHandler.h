#ifndef IR2425GROUP08_ROUTEHANDLER_H
#define IR2425GROUP08_ROUTEHANDLER_H

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseGoal.h>

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <geometry_msgs/Point.h>

using NodeHandleShared = std::shared_ptr<ros::NodeHandle>;

namespace ir2425_group_08
{
    class RouteHandler
    {
        // all the points in the arguments are assumed to be in the map frame
    protected:
        actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac_;
        tf::TransformListener tf_listener_;
        //bool is_robot_moving_;
        void sendNextPose();
std::vector<geometry_msgs::Pose> current_poses_;
size_t current_pose_index_;
boost::function<void(const actionlib::SimpleClientGoalState&)> done_callback_;
    private:
        geometry_msgs::Point transformPoint(const geometry_msgs::Point& point_in, const std::string& source_frame, const std::string& target_frame);
    public:
        RouteHandler();
        bool followPoses(std::vector<geometry_msgs::Pose> poses);
        bool followWaypoints(std::vector<geometry_msgs::Point> waypoints);
        bool pointTowards(geometry_msgs::Point target);
        void followPosesAsync(
    std::vector<geometry_msgs::Pose> poses,
    boost::function<void(const actionlib::SimpleClientGoalState&)> done_cb);

        //bool isRobotMoving() const { return is_robot_moving_; }
    };
}

#endif
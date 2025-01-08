#include "ir2425_group_08/RouteHandler.h"

namespace ir2425_group_08
{
    // ctor

    RouteHandler::RouteHandler() : ac_("/move_base", true)
    {
        this->ac_.waitForServer();
        ROS_INFO_STREAM("move_base server online!");
    }

    // public

    bool RouteHandler::followWaypoints(std::vector<geometry_msgs::Point> waypoints)
    {
        int index = 0;
        bool last_waypoint_failed = false;
        while(!last_waypoint_failed && index < static_cast<int>(waypoints.size()))
        {
            move_base_msgs::MoveBaseGoal waypointGoal;
            waypointGoal.target_pose.header.stamp = ros::Time::now();
            waypointGoal.target_pose.header.frame_id = "map";
            waypointGoal.target_pose.pose.position = waypoints[index];
            waypointGoal.target_pose.pose.orientation.w = 1.0;

            this->ac_.sendGoal(waypointGoal);

            bool result = this->ac_.waitForResult(ros::Duration(50.0));
            ROS_INFO_STREAM("Waypoint " << index << " reached: " + result);

            last_waypoint_failed = !result;
        }

        return !last_waypoint_failed;
    }

    bool RouteHandler::pointTowards(geometry_msgs::Point target)
    {
        geometry_msgs::Point baselink_point = transformPoint(target, "map", "base_link");

        float distance = sqrt(baselink_point.x * baselink_point.x + baselink_point.y * baselink_point.y);

        // Calculate the yaw angle
        double yaw = atan2(baselink_point.y, baselink_point.x);

        tf::Quaternion tf_quat;
        tf_quat.setRPY(0, 0, yaw); // Roll and pitch are 0, only yaw is set

        geometry_msgs::Quaternion geom_quat;
        tf::quaternionTFToMsg(tf_quat, geom_quat);

        // Set the goal
        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.header.frame_id = "base_link";
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.pose.orientation = geom_quat;

        this->ac_.sendGoal(goal);
        return this->ac_.waitForResult();
    }

    // private

    geometry_msgs::Point RouteHandler::transformPoint(const geometry_msgs::Point& point_in, const std::string& source_frame, const std::string& target_frame) 
    {
        // Wrap the point in a PointStamped
        geometry_msgs::PointStamped point_stamped_in, point_stamped_out;
        point_stamped_in.header.frame_id = source_frame;
        point_stamped_in.header.stamp = ros::Time(0);
        point_stamped_in.point = point_in;

        // Transform the PointStamped
        try {
            this->tf_listener_.waitForTransform(target_frame, source_frame, ros::Time(0), ros::Duration(1.0));
            this->tf_listener_.transformPoint(target_frame, point_stamped_in, point_stamped_out);
        } catch (tf::TransformException& ex) {
            ROS_ERROR("Transform error: %s", ex.what());
            throw;
        }

        // Return the transformed point
        return point_stamped_out.point;
    }
}
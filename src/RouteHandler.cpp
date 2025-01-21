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
    bool RouteHandler::followPoses(std::vector<geometry_msgs::Pose> poses)
    {
        for (size_t i = 0; i < poses.size(); ++i)
        {
            // Define the goal pose
            move_base_msgs::MoveBaseGoal goal;
            goal.target_pose.header.frame_id = "map"; // Target frame
            goal.target_pose.header.stamp = ros::Time::now();
            goal.target_pose.pose = poses[i];        // Set the pose from the vector

            // Log and send the goal
            ROS_INFO("Sending goal %lu: Position(%f, %f, %f), Orientation(%f, %f, %f, %f)",
                    i + 1,
                    poses[i].position.x, poses[i].position.y, poses[i].position.z,
                    poses[i].orientation.x, poses[i].orientation.y, poses[i].orientation.z, poses[i].orientation.w);
            this->ac_.sendGoal(goal);

            // Wait for the result
            bool success = this->ac_.waitForResult(ros::Duration(30.0)); // Timeout after 30 seconds
            if (success && this->ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            {
                ROS_INFO("Successfully reached goal %lu!", i + 1);
            }
            else
            {
                ROS_WARN("Failed to reach goal %lu. Aborting remaining goals.", i + 1);
                return false;
            }
        }

        return true;
    }

    void RouteHandler::followPosesAsync(
        std::vector<geometry_msgs::Pose> poses,
        boost::function<void(const actionlib::SimpleClientGoalState&)> done_cb)
    {
        if (poses.empty()) {
            ROS_WARN("No poses provided for followPosesAsync");
            return;
        }

        current_poses_ = poses;
        current_pose_index_ = 0;
        done_callback_ = done_cb;

        // Start following the first pose
        sendNextPose();
    }

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
            index++;
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
    void RouteHandler::sendNextPose()
    {
        if (current_pose_index_ >= current_poses_.size()) {
            // All poses completed successfully
            if (done_callback_) {
                done_callback_(actionlib::SimpleClientGoalState::SUCCEEDED);
            }
            return;
        }

        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.pose = current_poses_[current_pose_index_];

        ROS_INFO("Sending goal %zu: Position(%f, %f, %f), Orientation(%f, %f, %f, %f)",
                current_pose_index_ + 1,
                goal.target_pose.pose.position.x,
                goal.target_pose.pose.position.y,
                goal.target_pose.pose.position.z,
                goal.target_pose.pose.orientation.x,
                goal.target_pose.pose.orientation.y,
                goal.target_pose.pose.orientation.z,
                goal.target_pose.pose.orientation.w);

        // Set up the callback for when this goal completes
        actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>::SimpleDoneCallback cb =
            [this](const actionlib::SimpleClientGoalState& state,
                  const move_base_msgs::MoveBaseResultConstPtr& result) {
                if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
                    ROS_INFO("Successfully reached goal %zu!", current_pose_index_ + 1);
                    current_pose_index_++;
                    sendNextPose();
                } else {
                    ROS_WARN("Failed to reach goal %zu. Aborting remaining goals.", current_pose_index_ + 1);
                    if (done_callback_) {
                        done_callback_(state);
                    }
                }
            };

        this->ac_.sendGoal(goal, cb);
    }

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
#include "ir2425_group_08/RouteHandler.h"

/*
waypoints
aside pick_table: [ INFO] [1737714488.320085439, 6613.189000000]: Setting goal: Frame:map, Position(8.007, -3.846, 0.000), Orientation(0.000, 0.000, 0.686, 0.728) = Angle: 1.511
back pick_table: [ INFO] [1737719711.942532671, 6459.648000000]: Setting goal: Frame:map, Position(6.761, -2.793, 0.000), Orientation(0.000, 0.000, -0.011, 1.000) = Angle: -0.023

middle point f-a: [ INFO] [1737756393.025888166, 6652.027000000]: Setting goal: Frame:map, Position(8.921, -3.939, 0.000), Orientation(0.000, 0.000, 1.000, 0.010) = Angle: 3.121
middle point a-b: [ INFO] [1737756453.802653619, 6692.130000000]: Setting goal: Frame:map, Position(7.121, -4.108, 0.000), Orientation(0.000, 0.000, 0.713, 0.701) = Angle: 1.587
*/

namespace ir2425_group_08
{
    // static private

    std::vector<geometry_msgs::Pose> RouteHandler::initTablesWaypoints() 
    {
        // TODO add waypoints for place table
        return {
            []() { // back pick
                geometry_msgs::Pose pose;
                pose.position.x = 6.761;
                pose.position.y = -2.793;
                pose.position.z = 0.0;
                pose.orientation.x = 0.0;
                pose.orientation.y = 0.0;
                pose.orientation.z = -0.011;
                pose.orientation.w = 1.000;
                return pose;
            }(),
            []() { // middle point a-b pick
                geometry_msgs::Pose pose;
                pose.position.x = 7.121;
                pose.position.y = -4.108;
                pose.position.z = 0.0;
                pose.orientation.x = 0.0;
                pose.orientation.y = 0.0;
                pose.orientation.z = 0.713;
                pose.orientation.w = 0.701;
                return pose;
            }(),
            []() { // aside pick
                geometry_msgs::Pose pose;
                pose.position.x = 8.007;
                pose.position.y = -3.846;
                pose.position.z = 0.0;
                pose.orientation.x = 0.0;
                pose.orientation.y = 0.0;
                pose.orientation.z = 0.686;
                pose.orientation.w = 0.728;
                return pose;
            }(),
            []() { // middle point f-a pick
                geometry_msgs::Pose pose;
                pose.position.x = 8.921;
                pose.position.y = -3.939;
                pose.position.z = 0.0;
                pose.orientation.x = 0.0;
                pose.orientation.y = 0.0;
                pose.orientation.z = 1.000;
                pose.orientation.w = 0.010;
                return pose;
            }(),
            []() { // front pick
                geometry_msgs::Pose pose;
                pose.position.x = 8.807;
                pose.position.y = -2.777;
                pose.position.z = 0.0;
                pose.orientation.x = 0.0;
                pose.orientation.y = 0.0;
                pose.orientation.z = 1.0;
                pose.orientation.w = 0.009;
                return pose;
            }(),
            []() { // front place
                geometry_msgs::Pose pose;
                pose.position.x = 8.828;
                pose.position.y = -1.972;
                pose.position.z = 0.0;
                pose.orientation.x = 0.0;
                pose.orientation.y = 0.0;
                pose.orientation.z = -1.0;
                pose.orientation.w = 0.0;
                return pose;
            }()
        };
    }

    // ctor
    RouteHandler::RouteHandler(NodeHandleShared& nh_ptr) 
    :
    nh_ptr_(nh_ptr),
    ac_("/move_base", true),
    tablesWaypoints_(initTablesWaypoints()),
    currentWaypointIndex_(5), // assuming the robot starts in front place
    waypointTolerance_(0.001)
    {
        this->cmd_vel_pub_ = this->nh_ptr_->advertise<geometry_msgs::Twist>("mobile_base_controller/cmd_vel", 10);

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

    bool RouteHandler::fullPickRotation(boost::function<void(const actionlib::SimpleClientGoalState&)> done_cb)
    {
        if (currentWaypointIndex_ != 5)
        {
            ROS_WARN("fullPickRotation is a test function. Please assure the robot starts in front place table waypoint");
            return false;
        }

        // ROS_INFO_STREAM("Waypoints: ");
        // for (auto waypoint : tablesWaypoints_) 
        // {
        //     ROS_INFO_STREAM(waypoint);
        // }

        auto last_cb = done_cb;
        if (!done_cb)
        {
            last_cb = [](const actionlib::SimpleClientGoalState& state) { /* do nothing */ };
        }

        goToWaypoint(0, [](const actionlib::SimpleClientGoalState& state) { /* do nothing */ });

        goToWaypoint(4, last_cb);

        return true;
    }

    bool RouteHandler::goFrontPick(boost::function<void(const actionlib::SimpleClientGoalState&)> done_cb)
    {
        auto last_cb = done_cb;
        if (!done_cb)
        {
            last_cb = [](const actionlib::SimpleClientGoalState& state) { /* do nothing */ };
        }

        goToWaypoint(4, last_cb);

        return true;
    }

    bool RouteHandler::goAsidePick(boost::function<void(const actionlib::SimpleClientGoalState&)> done_cb)
    {
        auto last_cb = done_cb;
        if (!done_cb)
        {
            last_cb = [](const actionlib::SimpleClientGoalState& state) { /* do nothing */ };
        }

        goToWaypoint(2, last_cb);

        return true;
    }

    bool RouteHandler::goBackPick(boost::function<void(const actionlib::SimpleClientGoalState&)> done_cb)
    {
        auto last_cb = done_cb;
        if (!done_cb)
        {
            last_cb = [](const actionlib::SimpleClientGoalState& state) { /* do nothing */ };
        }

        goToWaypoint(0, last_cb);

        return true;
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

        ROS_INFO("Sending goal (waypoint) %zu: Position(%f, %f, %f), Orientation(%f, %f, %f, %f)",
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

    size_t RouteHandler::getNextIndex(size_t targetIndex)
    {
        int result = static_cast<int>(this->currentWaypointIndex_);
        int target = static_cast<int>(targetIndex);

        if (result == target) {
            return static_cast<size_t>(result); // Already at target
        }

        int sign = (target > result) ? 1 : -1;

        if (result == 0 && sign == -1) 
        {
            ROS_WARN("You tried to move from waypoint 0 to -1!");
            return 0; // Stay at the first waypoint
        } else if (result == static_cast<int>(this->tablesWaypoints_.size()) - 1 && sign == 1) 
        {
            ROS_WARN_STREAM("You tried to move from waypoint " << (tablesWaypoints_.size() - 1) << " to " << tablesWaypoints_.size());
            return this->tablesWaypoints_.size() - 1; // Stay at the last waypoint
        }

        // Update result and return
        result += sign;
        return static_cast<size_t>(result);
    }

    geometry_msgs::Pose RouteHandler::getRobotPoseInMap()
    {
        // Wrap the pose in a PoseStamped
        geometry_msgs::PoseStamped origin_base_link, robot_pose_in_map;
        origin_base_link.header.frame_id = "base_link";
        origin_base_link.header.stamp = ros::Time(0);
        origin_base_link.pose.orientation.w = 1.0;

        // Transform the PointStamped
        try {
            this->tf_listener_.waitForTransform("map", "base_link", ros::Time(0), ros::Duration(1.0));
            this->tf_listener_.transformPose("map", origin_base_link, robot_pose_in_map);
        } catch (tf::TransformException& ex) {
            ROS_ERROR("Transform error: %s", ex.what());
            throw;
        }

        // Return the transformed point
        return robot_pose_in_map.pose;
    }

    // deprecated
    // geometry_msgs::Twist RouteHandler::getTwistTowardPose(geometry_msgs::Pose start_pose, geometry_msgs::Pose target_pose, float linear_speed)
    // {
    //     geometry_msgs::Twist goalTwist;

    //     double dx = target_pose.position.x - start_pose.position.x;
    //     double dy = target_pose.position.y - start_pose.position.y;

    //     double distance = std::sqrt(dx * dx + dy * dy);
    //     // target_yaw = std::atan2(dy, dx);

    //     // // extract current yaw
    //     // tf::Quaternion start_orientation;
    //     // tf::fromMsg(start_pose.orientation, start_orientation);
    //     // double start_roll, start_pitch, start_yaw;
    //     // tf::Matrix3x3(q).getRPY(start_roll, start_pitch, start_yaw);

    //     // double angular_diff = target_yaw - current_yaw;

    //     // // Normalize angular difference to [-pi, pi]
    //     // while (angular_diff > M_PI)
    //     // {
    //     //     angular_diff -= 2.0 * M_PI;
    //     // }
    //     // while (angular_diff < -M_PI)
    //     // {
    //     //     angular_diff += 2.0 * M_PI;
    //     // }

    //     if (distance > 0.01) { // Tolerance to stop linear movement
    //         goalTwist.linear.x = (dx / distance) * linear_speed;
    //         goalTwist.linear.y = (dy / distance) * linear_speed;
    //     } else {
    //         goalTwist.linear.x = twist.linear.y = 0.0; // Stop if close to the target
    //     }

    //     // if (std::abs(angular_diff) > 0.01) { // Tolerance for angular alignment
    //     //     goalTwist.angular.z = (angular_diff > 0 ? 1 : -1) * angular_speed;
    //     // } else {
    //     //     goalTwist.angular.z = 0.0; // Stop angular motion if aligned
    //     // }

    //     return goalTwist;
    // }

    // bool RoutHandler::waypointReached(geometry_msgs::Pose target_pose, double& e_linear)
    // {
    //     // get current position
    //     const nav_msgs::OdometryConstPtr& msg = ros::topic:waitForMessage<nav_msgs::Odometry>("/odom", *nh_ptr_, ros::Duration(5.0));
    //     if(!msg)
    //     {
    //         ROS_WARN("Failed to retrieve actual robot position during precise movment");
    //         return false;
    //     }

    //     double current_x = msg->pose.pose.position.x;
    //     double current_y = msg->pose.pose.position.y;

    //     double target_x = target_pose.position.x;
    //     double target_y = target_pose.position.y;

    //     // Compute Errors
    //     e_linear = std::sqrt(std::pow(target_x - current_x, 2) + std::pow(target_y - current_y, 2));

    //     return (e_linear <= waypointTolerance_);
    // }

    // bool RoutHandler::orientationReached(geometry_msgs::Pose target_pose, double& e_angular)
    // {
    //     // get current position
    //     const nav_msgs::OdometryConstPtr& msg = ros::topic:waitForMessage<nav_msgs::Odometry>("/odom", *nh_ptr_, ros::Duration(5.0));
    //     if(!msg)
    //     {
    //         ROS_WARN("Failed to retrieve actual robot position during precise movment");
    //         return false;
    //     }

    //     double current_x = msg->pose.pose.position.x;
    //     double current_y = msg->pose.pose.position.y;
    //     double current_theta = tf::getYaw(msg->pose.pose.orientation);

    //     double target_x = target_pose.position.x;
    //     double target_y = target_pose.position.y;

    //     // Compute Errors
    //     double target_angle = std::atan2(target_y - current_y, target_x - current_x);
    //     double e_angular = target_angle - current_theta;
    //     e_angular = std::atan2(std::sin(e_angular), std::cos(e_angular)); // Normalize angle

    //     return (std::fabs(e_angular) <= waypointTolerance_);
    // }

    void RouteHandler::pointTowards(geometry_msgs::Pose target_pose)
    {
        geometry_msgs::Pose start_pose = getRobotPoseInMap();
        double dx = target_pose.position.x - start_pose.position.x;
        double dy = target_pose.position.y - start_pose.position.y;

        // Calculate the target_yaw and start_yaw
        double target_yaw = atan2(dy, dx);
        double start_yaw = tf::getYaw(start_pose.orientation);
        ROS_INFO_STREAM("Target yaw: " << target_yaw << ", Start yaw: " << start_yaw);

        // error
        double e_angular = target_yaw - start_yaw;
        e_angular = std::atan2(std::sin(e_angular), std::cos(e_angular)); // Normalize angle

        geometry_msgs::Twist cmd_vel;
        ros::Rate rate(10);
        double K_angular = 1.0;

        while(std::fabs(e_angular) > waypointTolerance_)
        {
            cmd_vel.angular.z = K_angular * e_angular;
            this->cmd_vel_pub_.publish(cmd_vel);

            //ROS_INFO_STREAM("Rotating of " << cmd_vel.angular.z << "...");
            rate.sleep();

            // updating the error
            geometry_msgs::Pose current_pose = getRobotPoseInMap();
            dx = target_pose.position.x - current_pose.position.x;
            dy = target_pose.position.y - current_pose.position.y;

            double current_yaw = tf::getYaw(current_pose.orientation);

            e_angular = target_yaw - current_yaw;
            e_angular = std::atan2(std::sin(e_angular), std::cos(e_angular)); // Normalize angle            
        }

        // send a 0
        cmd_vel.angular.z = 0.0;
        this->cmd_vel_pub_.publish(cmd_vel);
    }

    void RouteHandler::moveTowards(geometry_msgs::Pose target_pose)
    {
        geometry_msgs::Pose start_pose = getRobotPoseInMap();
        double dx = target_pose.position.x - start_pose.position.x;
        double dy = target_pose.position.y - start_pose.position.y;
        ROS_INFO_STREAM("dx: " << dx << ", dy: " << dy);

        // error
        double e_linear = dx * dx + dy * dy;

        geometry_msgs::Twist cmd_vel;
        ros::Rate rate(10);
        double K_linear = 1.0;

        while(e_linear > waypointTolerance_)
        {
            cmd_vel.linear.x = K_linear * e_linear;
            this->cmd_vel_pub_.publish(cmd_vel);

            //ROS_INFO_STREAM("Moving of " << cmd_vel.linear.x << "...");
            rate.sleep();

            // updating error
            geometry_msgs::Pose current_pose = getRobotPoseInMap();
            dx = target_pose.position.x - current_pose.position.x;
            dy = target_pose.position.y - current_pose.position.y;

            e_linear = dx * dx + dy * dy;
        }

        // send a 0
        cmd_vel.linear.x = 0.0;
        this->cmd_vel_pub_.publish(cmd_vel);
    }

    void RouteHandler::alignWithPose(geometry_msgs::Pose target_pose)
    {
        geometry_msgs::Pose start_pose = getRobotPoseInMap();

        // Calculate the target_yaw and start_yaw
        double target_yaw = tf::getYaw(target_pose.orientation);
        double start_yaw = tf::getYaw(start_pose.orientation);

        // error
        double e_angular = target_yaw - start_yaw;
        e_angular = std::atan2(std::sin(e_angular), std::cos(e_angular)); // Normalize angle

        geometry_msgs::Twist cmd_vel;
        ros::Rate rate(10);
        double K_angular = 1.0;

        while(std::fabs(e_angular) > waypointTolerance_)
        {
            cmd_vel.angular.z = K_angular * e_angular;
            this->cmd_vel_pub_.publish(cmd_vel);

            rate.sleep();

            // updating the error
            geometry_msgs::Pose current_pose = getRobotPoseInMap();

            double current_yaw = tf::getYaw(current_pose.orientation);

            e_angular = target_yaw - current_yaw;
            e_angular = std::atan2(std::sin(e_angular), std::cos(e_angular)); // Normalize angle 
        }

        // send a 0
        cmd_vel.angular.z = 0.0;
        this->cmd_vel_pub_.publish(cmd_vel);
    }

    void RouteHandler::goToWaypoint(size_t index, boost::function<void(const actionlib::SimpleClientGoalState&)> done_cb)
    {
        if (tablesWaypoints_.empty()) {
            ROS_WARN("No waypoints know by RouteHandler");
            return;
        }

        if (index > tablesWaypoints_.size())
        {
            ROS_WARN("Index out of bounds in tablesWaypoints");
            return;
        }

        ROS_INFO_STREAM("Target index: " << index);
        size_t nextIndex;
        while (currentWaypointIndex_ != index)
        {
                nextIndex = getNextIndex(index);
                ROS_INFO_STREAM("Moving to waypoint " << nextIndex);

                // preventing an infinite loop...
                if (nextIndex == currentWaypointIndex_)
                {
                    ROS_WARN("For reasons, the robot tried to access a non-consecutive waypoint. Something bad happened. Aborting...");
                    return;
                }

                // points towards next
                pointTowards(tablesWaypoints_[nextIndex]);

                // go towards next
                moveTowards(tablesWaypoints_[nextIndex]);

                currentWaypointIndex_ = nextIndex;
        }

        // point towards pose of current
        alignWithPose(tablesWaypoints_[currentWaypointIndex_]);

        // debug
        ROS_INFO_STREAM(getRobotPoseInMap());

        // notify the node that called that the movment is done
        done_cb(actionlib::SimpleClientGoalState::SUCCEEDED);
    }
}
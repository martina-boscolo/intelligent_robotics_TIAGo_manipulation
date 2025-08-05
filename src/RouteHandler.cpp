#include "ir2425_group_08/RouteHandler.h"

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
                pose.position.y = -2.877; // -0.10
                pose.position.z = 0.0;
                pose.orientation.x = 0.0;
                pose.orientation.y = 0.0;
                pose.orientation.z = 1.0;
                pose.orientation.w = 0.009;
                return pose;
            }(),
            []() { // front place
                geometry_msgs::Pose pose;
                pose.position.x = 8.578; // -0.15
                pose.position.y = -1.822; // +0.15
                pose.position.z = 0.0;
                pose.orientation.x = 0.0;
                pose.orientation.y = 0.0;
                pose.orientation.z = -1.0;
                pose.orientation.w = 0.0;
                return pose;
            }(),
            []() { // middle point f-a place
                geometry_msgs::Pose pose;
                pose.position.x = 8.719;
                pose.position.y = -1.053;
                pose.position.z = 0.0;
                pose.orientation.x = 0.0;
                pose.orientation.y = 0.0;
                pose.orientation.z = 1.0;
                pose.orientation.w = 0.016;
                return pose;
            }(),
            []() { // aside place
                geometry_msgs::Pose pose;
                pose.position.x = 7.876;
                pose.position.y = -1.162;
                pose.position.z = 0.0;
                pose.orientation.x = 0.0;
                pose.orientation.y = 0.0;
                pose.orientation.z = -0.701;
                pose.orientation.w = 0.713;
                return pose;
            }(),
            []() { // middle point a-b place
                geometry_msgs::Pose pose;
                pose.position.x = 6.819;
                pose.position.y = -0.954;
                pose.position.z = 0.0;
                pose.orientation.x = 0.0;
                pose.orientation.y = 0.0;
                pose.orientation.z = 1.0;
                pose.orientation.w = 0.010;
                return pose;
            }(),
            []() { // back place
                geometry_msgs::Pose pose;
                pose.position.x = 6.984;
                pose.position.y = -1.940;
                pose.position.z = 0.0;
                pose.orientation.x = 0.0;
                pose.orientation.y = 0.0;
                pose.orientation.z = 0.008;
                pose.orientation.w = 1.0;
                return pose;
            }()
        };
    }

    // ctor
    RouteHandler::RouteHandler(NodeHandleShared& nh_ptr) 
    :
    nh_ptr_(nh_ptr),
    tablesWaypoints_(initTablesWaypoints()),
    currentWaypointIndex_(5), // assuming the robot starts in front place
    waypointTolerance_(0.01)
    {
        this->cmd_vel_pub_ = this->nh_ptr_->advertise<geometry_msgs::Twist>("mobile_base_controller/cmd_vel", 10);
        ROS_INFO_STREAM("Publisher on mobile_base_controller/cmd_vel ready!");
    }

    // public

    bool RouteHandler::fullPickRotation()
    {
        if (currentWaypointIndex_ != 5)
        {
            ROS_WARN("fullPickRotation is a test function. Please assure the robot starts in front place table waypoint");
            return false;
        }

        goToWaypoint(0);

        goToWaypoint(4);

        return true;
    }

    bool RouteHandler::fullPlaceRotation()
    {
        if (currentWaypointIndex_ != 5)
        {
            ROS_WARN("fullPickRotation is a test function. Please assure the robot starts in front place table waypoint");
            return false;
        }

        goToWaypoint(9);

        goToWaypoint(5);

        return true;
    }

    bool RouteHandler::goFrontPick()
    {
        goToWaypoint(4);

        return true;
    }

    bool RouteHandler::goAsidePick()
    {
        goToWaypoint(2);

        return true;
    }

    bool RouteHandler::goBackPick()
    {
        goToWaypoint(0);

        return true;
    }

    bool RouteHandler::goFrontPlace()
    {
        goToWaypoint(5);

        return true;
    }

    bool RouteHandler::goAsidePlace()
    {
        goToWaypoint(7);

        return true;
    }

    bool RouteHandler::goBackPlace()
    {
        goToWaypoint(9);

        return true;
    }

    size_t RouteHandler::getCurrentWaypointIndex()
    {
        return this->currentWaypointIndex_;
    }

    void RouteHandler::setCurrentWaypointIndex(size_t new_index)
    {
        currentWaypointIndex_ = new_index;
    }

    // private

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

    void RouteHandler::goToWaypoint(size_t index)
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

        ROS_INFO_STREAM("Target index: " << index << ", Current index: " << currentWaypointIndex_);
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
    }
}
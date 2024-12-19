#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <cmath>
#include <vector>

class CorridorNavigator {
private:
    ros::NodeHandle nh_;
    ros::Subscriber laser_sub_;
    ros::Publisher vel_pub_;
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base_client_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    
    // Parameters
    const double CORRIDOR_WIDTH_THRESHOLD = 2.0;    // Maximum width to be considered a corridor
    const double MIN_WALL_LENGTH = 0.2;            // Minimum length of parallel walls
    const double MAX_LINEAR_VEL = 0.5;             // Maximum linear velocity
    const double MAX_ANGULAR_VEL = 0.5;            // Maximum angular velocity
    const double Kp_angular = 0.5;                 // Proportional gain for angular velocity
    const double MIN_FRONT_CLEARANCE = 0.2;        // Minimum clearance in front to move forward
    
    bool in_corridor_;
    geometry_msgs::PoseStamped goal_;
    ros::Time last_cmd_time_;

public:
    CorridorNavigator() : 
        move_base_client_("move_base", true),
        tf_listener_(tf_buffer_),
        in_corridor_(false) {
        
        // Fix: Initialize publisher correctly
        vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/mobile_base_controller/cmd_vel", 1);
        laser_sub_ = nh_.subscribe("/scan", 1, &CorridorNavigator::laserCallback, this);
        
        ROS_INFO("Waiting for move_base action server...");
        move_base_client_.waitForServer();
        ROS_INFO("Move base server connected!");
        
        last_cmd_time_ = ros::Time::now();
        setGoal(12.0, 0.0);
    }

    void setGoal(double x, double y) {
        goal_.header.frame_id = "map";
        goal_.header.stamp = ros::Time::now();
        goal_.pose.position.x = x;
        goal_.pose.position.y = y;
        goal_.pose.orientation.w = 1.0;
        
        startNavigation();
    }

    void startNavigation() {
        move_base_msgs::MoveBaseGoal move_base_goal;
        move_base_goal.target_pose = goal_;
        move_base_client_.sendGoal(move_base_goal);
    }

    bool detectCorridor(const sensor_msgs::LaserScan::ConstPtr& scan, double& left_dist, double& right_dist) {
        // Get the indices for left and right side scans (approximately ±90 degrees)
        int mid_index = scan->ranges.size() / 2;
        int quarter_index = scan->ranges.size() / 4;
        
        // Get left and right side distances
        std::vector<double> left_samples, right_samples;
        
        // Sample points on both sides
        for (int i = 0; i < quarter_index/2; i++) {
            double left_range = scan->ranges[mid_index + quarter_index/2 + i];
            double right_range = scan->ranges[mid_index - quarter_index/2 - i];
            
            if (!std::isinf(left_range) && !std::isnan(left_range)) {
                left_samples.push_back(left_range);
            }
            if (!std::isinf(right_range) && !std::isnan(right_range)) {
                right_samples.push_back(right_range);
            }
        }
        
        // Check if we have enough valid samples
        if (left_samples.size() < 10 || right_samples.size() < 10) {
            return false;
        }
        
        // Calculate average distances
        left_dist = 0;
        right_dist = 0;
        for (double d : left_samples) left_dist += d;
        for (double d : right_samples) right_dist += d;
        left_dist /= left_samples.size();
        right_dist /= right_samples.size();
        
        // Check if distances indicate a corridor
        double width = left_dist + right_dist;
        return width < CORRIDOR_WIDTH_THRESHOLD;
    }

    bool checkFrontClearance(const sensor_msgs::LaserScan::ConstPtr& scan) {
        // Check the area in front of the robot
        int mid_index = scan->ranges.size() / 2;
        int check_width = scan->ranges.size() / 8; // Check ±22.5 degrees in front
        
        for (int i = mid_index - check_width; i <= mid_index + check_width; i++) {
            if (i >= 0 && i < scan->ranges.size()) {
                double range = scan->ranges[i];
                if (!std::isinf(range) && !std::isnan(range) && range < MIN_FRONT_CLEARANCE) {
                    return false;
                }
            }
        }
        return true;
    }

    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
        double left_dist, right_dist;
        bool corridor_detected = detectCorridor(scan, left_dist, right_dist);
        
        if (corridor_detected) {
            if (!in_corridor_) {
                in_corridor_ = true;
                move_base_client_.cancelAllGoals();
                ROS_INFO("Corridor detected, switching to custom control");
            }
            bool front_clear = checkFrontClearance(scan);
            corridorControl(left_dist, right_dist, front_clear);
        } else {
            if (in_corridor_) {
                in_corridor_ = false;
                startNavigation();
                ROS_INFO("Corridor exit detected, resuming move_base");
            }
        }
        
        // Safety check: if no commands sent for a while, send zero velocity
        if ((ros::Time::now() - last_cmd_time_).toSec() > 0.5) {
            geometry_msgs::Twist zero_cmd;
            vel_pub_.publish(zero_cmd);
        }
    }

    void corridorControl(double left_dist, double right_dist, bool front_clear) {
        geometry_msgs::Twist cmd_vel;
        
        // Calculate center offset
        double center_error = (left_dist - right_dist) / 2.0;
        
        // Set linear velocity if front is clear
        if (front_clear) {
            cmd_vel.linear.x = MAX_LINEAR_VEL;
            cmd_vel.linear.y = MAX_LINEAR_VEL;
         }
        else {
            cmd_vel.linear.x = 0.0;
            cmd_vel.linear.y = 0.0;
            ROS_WARN_THROTTLE(1.0, "Front not clear, stopping forward motion");
        }
        
        // Calculate angular velocity for centering
        cmd_vel.angular.z = -Kp_angular * center_error;
        
        // Limit angular velocity
        cmd_vel.angular.z = std::max(-MAX_ANGULAR_VEL, std::min(MAX_ANGULAR_VEL, cmd_vel.angular.z));
        
        // Publish velocity commands
        vel_pub_.publish(cmd_vel);
        last_cmd_time_ = ros::Time::now();
        
        // Debug output
        ROS_INFO_THROTTLE(1.0, "Corridor control - linear: %.2f, angular: %.2f", cmd_vel.linear.x, cmd_vel.angular.z);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "corridor_navigator");
    CorridorNavigator navigator;
    ros::spin();
    return 0;
}
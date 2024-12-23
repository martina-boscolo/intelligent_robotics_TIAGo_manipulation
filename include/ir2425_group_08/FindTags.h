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
        createPoint(12, 1.0, 0.0),
        createPoint(12.0, -1.0, 0.0),
        createPoint(9, -1.9, 0.0), 
        createPoint(9.5, -4.0, 0.0),
        createPoint(8.5, -1.5, 0.0),
        createPoint(8.7, -3.3, 0.0), 
        createPoint(8.5, -3.15, 0.0), 
        createPoint(11.5, -3.2, 0.0), 
        createPoint(12.5, -3.0, 0.0),
        createPoint(12, 0.0, 0.0), 
        createPoint(9.7, 0.7, 0.0),
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
        std::vector <geometry_msgs::PoseStamped> AlreadyFoundValidTags;
        std::vector <bool> AlreadyFoundTags;

        /**
         * @brief Checks if a detected AprilTag is new and valid.
         * 
         * This function verifies that the tag ID is in the list of valid IDs 
         * and has not been previously detected.
         * 
         * @param tag The AprilTag detection to validate.
         * @return True if the tag is valid and has not been detected before; false otherwise.
         */
        bool isNewAndValidTag(apriltag_ros::AprilTagDetection tag);

    public:
        /**
         * @brief Constructs a FindTags object and initializes its components.
         * 
         * Sets up the action server, AprilTag detection subscriber, and velocity command publisher.
         * Also waits for the move_base action server to become available.
         * 
         * @param nh_ptr A shared pointer to the ROS NodeHandle for managing ROS entities.
         * @param server_name The name of the action server to initialize.
         */
        FindTags(NodeHandleShared& nh,  std::string server_name);

        /**
         * @brief Callback function to handle AprilTag detections.
         * 
         * This function processes the AprilTag detection messages, checks for new valid tags,
         * and transforms their poses from the camera frame to the map frame. The results are
         * published as feedback for an action server and logged for debugging.
         * 
         * @param msg A shared pointer to a message containing an array of detected AprilTags.
         */
        void tagsCallback(const apriltag_ros::AprilTagDetectionArrayConstPtr& msg);

        /**
         * @brief Callback function to process LaserScan messages for corridor navigation.
         * 
         * This function handles laser scan data to detect if the robot is in a corridor and manages
         * navigation within the corridor. It also handles the duration for which the robot stays in 
         * corridor mode and publishes feedback on the navigation progress.
         * 
         * @param msg A shared pointer to a LaserScan message containing the latest scan data.
         */
        void laserCallback(const sensor_msgs::LaserScanConstPtr& msg);

        /**
         * @brief Main cycle for handling the FindTags action server's goal.
         * 
         * This function processes the goal, which includes a list of target AprilTag IDs, and manages 
         * the robot's movement through waypoints while searching for the specified tags. It ensures that 
         * the robot navigates through the corridor (if detected), performs specific actions like head 
         * inclination and torso extension, and handles the movement to each waypoint using the move_base 
         * action server. The cycle continues until all the tags are found or all waypoints are visited.
         * 
         * @param goal A shared pointer to the goal message containing the list of target AprilTag IDs.
         */
        void mainCycle(const ir2425_group_08::FindTagsGoalConstPtr& goal);

        /**
         * @brief Extends the robot's torso to a predefined height.
         * 
         * This function sends a goal to the `torso_controller` to move the torso lift joint to a 
         * specified position, which is the maximum height for the robot's torso. It waits for the 
         * controller server, sends the trajectory goal, and waits for the result. The status of the 
         * action is published as feedback to the action server.
         */
        void extendTorso();

        /**
         * @brief Increases the inclination of the robot's head to a specified pitch angle.
         * 
         * This function sends a goal to the `head_controller` to adjust the robot's head position 
         * based on a specified pitch value. It waits for the controller to be ready, sends the 
         * goal, and waits for the result. The status of the action is published as feedback to 
         * the action server.
         * 
         * @param pitch The desired pitch angle (in radians) for inclining the robot's head.
         */
        void inclineHead(float pitch);

        /**
         * @brief Makes the robot perform a full 360-degree spin.
         * 
         * This function publishes a `Twist` message to rotate the robot counterclockwise with a specified 
         * angular velocity. The robot spins for the time required to complete a full rotation (2π radians) 
         * and then stops. The status of the action is published as feedback to the action server.
         */
        void performFullSpin();

        /**
         * @brief Checks if the robot is in a corridor based on laser scan data.
         * 
         * This function analyzes the laser scan data to detect if the robot is in a corridor. It computes 
         * the average distances to obstacles in the left, right, and forward directions, and uses these 
         * values to determine if the robot is in a corridor. The function checks for parallel walls, narrow 
         * width, and sufficient length ahead to confirm the presence of a corridor.
         * 
         * @param msg The laser scan data from the robot's sensors.
         * @param corridor_width The calculated width of the corridor, returned by reference.
         * 
         * @return `true` if the robot is in a corridor, otherwise `false`.
         */
        bool isInCorridor(const sensor_msgs::LaserScanConstPtr& msg, double& corridor_width);

        /**
         * @brief Controls the robot's movement while navigating in a corridor based on laser scan data.
         * 
         * This function uses the laser scan data to compute the average distances to obstacles on the left 
         * and right sides of the robot. It then adjusts the robot's velocity to ensure it stays centered 
         * within the corridor. The robot moves forward if there is sufficient space, and the turning speed 
         * is adjusted based on the difference between the left and right distances.
         * 
         * @param msg The laser scan data from the robot's sensors.
         * @param cmd_vel_pub_ The publisher used to send velocity commands to the robot.
         */
        void navigateInCorridor(const sensor_msgs::LaserScan::ConstPtr &msg, ros::Publisher &cmd_vel_pub_);
    };
}

#endif
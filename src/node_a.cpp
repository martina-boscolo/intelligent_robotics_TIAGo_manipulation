#include <ros/ros.h>
#include <tiago_iaslab_simulation/Coeffs.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PointStamped.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/PointHeadAction.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <math.h>
#include <vector>

#include "ir2425_group_08/PlaceGoal.h"

//Decent waypoints to scan tag with ID=10
//Position(8,234, 0,505, 0,000), Orientation(0,000, 0,000, -0,374, 0,927)
//Position(9,191, -2,266, 0,000), Orientation(0,000, 0,000, -0,999, 0,037)
/*std::vector<geometry_msgs::Point> WAYPOINT_LIST =
    {
        createPoint(12, 1.0, 0.0),
    };*/

std::vector<geometry_msgs::Point> global_points;
ros::Publisher debug_marker_pub;

//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
void publishDebugPoints(const std::vector<geometry_msgs::Point> &points, ros::NodeHandle &nh)
{
    // Create a publisher inside the function
    static ros::Publisher debug_marker_pub = nh.advertise<visualization_msgs::Marker>("debug_points", 10);

    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";  // Frame in which points are visualized
    marker.header.stamp = ros::Time::now();
    marker.ns = "debug_points";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE_LIST;  // Use a sphere list for multiple points
    marker.action = visualization_msgs::Marker::ADD;

    // Marker properties
    marker.scale.x = 0.05;  // Sphere diameter
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;
    marker.color.r = 1.0;   // Red color
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;   // Fully opaque

    // Add points to the marker
    for (const auto &point : points)
    {
        marker.points.push_back(point);
    }

    // Publish the marker
    debug_marker_pub.publish(marker);
    ROS_INFO("Published %lu points to RViz for debugging.", points.size());
}
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&

void tagDetectionCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg, float m, float q, int n, ros::NodeHandle &nh)
{
    static tf::TransformListener tf_listener;

    for (const auto &detection : msg->detections)
    {
        if (detection.id[0] == 10) // Only process AprilTag with ID 10
        {
            ROS_INFO("Tag 10 detected!");

            try
            {
                // Extract the AprilTag's pose in the camera frame
                geometry_msgs::PoseStamped tag_pose_camera;
                tag_pose_camera.header = detection.pose.header;
                tag_pose_camera.pose = detection.pose.pose.pose;

                ROS_INFO("Tag position in camera frame: [x: %f, y: %f, z: %f]",
                         tag_pose_camera.pose.position.x,
                         tag_pose_camera.pose.position.y,
                         tag_pose_camera.pose.position.z);

                // Compute the rotation matrix from the tag's orientation
                tf::Quaternion quaternion(tag_pose_camera.pose.orientation.x,
                                          tag_pose_camera.pose.orientation.y,
                                          tag_pose_camera.pose.orientation.z,
                                          tag_pose_camera.pose.orientation.w);
                tf::Matrix3x3 rotation_matrix(quaternion);

                // Extract the axes of the tag in the camera frame
                tf::Vector3 x_axis = rotation_matrix.getColumn(0); // Tag's x-axis
                tf::Vector3 y_axis = rotation_matrix.getColumn(1); // Tag's y-axis

                // Tag's origin in the camera frame
                tf::Vector3 origin(tag_pose_camera.pose.position.x,
                                   tag_pose_camera.pose.position.y,
                                   tag_pose_camera.pose.position.z);

                // Generate points in the tag's plane (camera frame)
                std::vector<geometry_msgs::PointStamped> camera_points;
                double step = 0.05; // Step size for points along the line
                for (int i = 0; i < n; ++i)
                {
                    // Calculate x and y in the tag's plane
                    double x_i = i * step;
                    double y_i = m * x_i + q;

                    // Calculate z_i based on the tag's plane orientation
                    double z_i = origin.z() + x_i * x_axis.z() + y_i * y_axis.z();

                    // Construct the point in the camera frame
                    geometry_msgs::PointStamped point_camera;
                    point_camera.header.frame_id = tag_pose_camera.header.frame_id; // Camera frame
                    point_camera.header.stamp = ros::Time(0);
                    point_camera.point.x = origin.x() + x_i * x_axis.x() + y_i * y_axis.x();
                    point_camera.point.y = origin.y() + x_i * x_axis.y() + y_i * y_axis.y();
                    point_camera.point.z = z_i;

                    camera_points.push_back(point_camera);
                }

                // Transform the points from the camera frame to the map frame
                for (const auto &camera_point : camera_points)
                {
                    geometry_msgs::PointStamped map_point;
                    tf_listener.waitForTransform("map", camera_point.header.frame_id, ros::Time(0), ros::Duration(2.0));
                    tf_listener.transformPoint("map", camera_point, map_point);

                    // Save the transformed point in the global vector
                    geometry_msgs::Point global_point;
                    global_point.x = map_point.point.x;
                    global_point.y = map_point.point.y;
                    global_point.z = map_point.point.z;

                    global_points.push_back(global_point);
                }

                ROS_INFO("Generated and transformed %d points to the map frame.", n);

                // Debug points
                publishDebugPoints(global_points, nh);

                // send message, provvisiorio, perché fa schifo
                ros::Publisher pub = nh.advertise<ir2425_group_08::PlaceGoal>("/place_goal", 1);

                ir2425_group_08::PlaceGoal msg;
                msg.target_points = global_points;
                msg.num_goals = n;

                ROS_INFO("Publishing message");
                pub.publish(msg);

            }
            catch (tf::TransformException &ex)
            {
                ROS_WARN("Could not transform points to map frame: %s", ex.what());
            }
            return; // Process only one tag
        }
    }
}



void extendTorso()
{
    // Define an action client for the torso controller
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> torso_client("/torso_controller/follow_joint_trajectory", true);

    // Wait for the action server to become available
    ROS_INFO("Waiting for torso controller action server...");
    torso_client.waitForServer();
    ROS_INFO("Torso controller action server available!");

    // Define a trajectory message
    trajectory_msgs::JointTrajectory trajectory;
    trajectory.joint_names.push_back("torso_lift_joint");

    // Define a trajectory point
    trajectory_msgs::JointTrajectoryPoint point;
    point.positions.push_back(0.35); // Desired height of the torso
    point.time_from_start = ros::Duration(2.0); // Time to reach the position

    // Add the point to the trajectory
    trajectory.points.push_back(point);

    // Create the goal message
    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory = trajectory;

    // Send the goal to the action server
    ROS_INFO("Sending torso extension goal...");
    torso_client.sendGoal(goal);

    // Wait for the result
    bool finished_before_timeout = torso_client.waitForResult(ros::Duration(5.0));
    if (finished_before_timeout)
    {
        ROS_INFO("Torso extended successfully!");
    }
    else
    {
        ROS_WARN("Failed to extend the torso within the timeout period.");
    }
}

void inclineHead(float pitch)
{
    // Define an action client for the head controller
    actionlib::SimpleActionClient<control_msgs::PointHeadAction> head_client("head_controller/point_head_action", true);

    // Wait for the action server to become available
    ROS_INFO("Waiting for head_controller action server...");
    head_client.waitForServer();
    ROS_INFO("Head controller action server available!");

    // Define a PointHead goal
    control_msgs::PointHeadGoal goal;
    goal.target.header.stamp = ros::Time(0);
    goal.target.header.frame_id = "xtion_rgb_optical_frame";
    goal.target.point.x = 0.0;
    goal.target.point.y = tan(pitch); // Convert pitch angle to tangent
    goal.target.point.z = 1.0;
    goal.pointing_axis.z = 1.0;       // Set the pointing axis
    goal.pointing_frame = "xtion_rgb_optical_frame";
    goal.min_duration = ros::Duration(1.0);
    goal.max_velocity = 0.6;

    // Send the goal to the action server
    ROS_INFO("Sending goal to head_controller...");
    head_client.sendGoal(goal);

    // Wait for the result
    // bool finished_before_timeout = head_client.waitForResult(ros::Duration(5.0));
    // if (finished_before_timeout)
    // {
    //     ROS_INFO("Robot is now looking with an inclined head.");
    // }
    // else
    // {
    //     ROS_WARN("Failed to incline head within the timeout period.");
    // }
}

int main(int argc, char **argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "node_a");
    ros::NodeHandle nh;
    
    // Create a service client to request Apriltag IDs
    ros::ServiceClient client = nh.serviceClient<tiago_iaslab_simulation::Coeffs>("/straight_line_srv");

    // Create a service request and response object
    tiago_iaslab_simulation::Coeffs srv;

    // Wait for the service to become available
    ROS_INFO("Waiting for /straight_line_srv service...");
    ros::service::waitForService("/straight_line_srv");
    srv.request.ready = true; 
    ROS_INFO("/straight_line_srv service available!");

    // Call the service to get target coefficents
    if (!client.call(srv))
    {
        ROS_ERROR("Failed to call /straight_line_srv service.");
        return 1;
    }

    ROS_INFO("Received %lu target coefficents.", srv.response.coeffs.size());
    float m = srv.response.coeffs[0];
    float q = srv.response.coeffs[1];
    ROS_INFO("m = %f", m);
    ROS_INFO("q = %f", q);

    extendTorso();
    inclineHead(M_PI / 6);

    int n = 3;
    ros::Subscriber sub = nh.subscribe<apriltag_ros::AprilTagDetectionArray>(
        "/tag_detections", 10,
        [&nh, m, q, n](const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg) {
            tagDetectionCallback(msg, m, q, n, nh);
        });

    ros::spin();
    return 0;
}

//Decent waypoints to scan tag with ID=10
//Position(8,234, 0,505, 0,000), Orientation(0,000, 0,000, -0,374, 0,927)
//Position(9,191, -2,266, 0,000), Orientation(0,000, 0,000, -0,999, 0,037)

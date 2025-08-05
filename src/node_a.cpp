#include <ros/ros.h>
#include <tiago_iaslab_simulation/Coeffs.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PointStamped.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/PointHeadAction.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <math.h>
#include <vector>

#include "ir2425_group_08/PlaceService.h"

using NodeHandleShared = std::shared_ptr<ros::NodeHandle>;

std::vector<geometry_msgs::Pose> target_poses = {
    // first pose
    []() {
        geometry_msgs::Pose pose;
        pose.position.x = 8.234;
        pose.position.y = 0.505;
        pose.position.z = 0.0;
        pose.orientation.x = 0.0;
        pose.orientation.y = 0.0;
        pose.orientation.z = -0.374;
        pose.orientation.w = 0.927;
        return pose;
    }(),
    // second pose
    []() {
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

std::vector<geometry_msgs::Point> global_points;
ros::Publisher debug_marker_pub;
bool is_apriltag_10_processed = false;

//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
// debug function to see the target points
void publishDebugPoints(const std::vector<geometry_msgs::Point> &points, ros::NodeHandle &nh)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "debug_points";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE_LIST;
    marker.action = visualization_msgs::Marker::ADD;

    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    for ( auto point : points)
    {
        point.z+= 0.3;
        marker.points.push_back(point);
    }

    debug_marker_pub.publish(marker);
    ROS_INFO("Published %lu points to RViz for debugging.", points.size());
}
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&

// function to generate points in the AprilTag frame
void generatePointsInAprilTagFrame(float m, float q, int n, ros::NodeHandle &nh)
{
    tf::TransformListener tf_listener;

    try
    {
        tf_listener.waitForTransform("map", "tag_10", ros::Time(0), ros::Duration(2.0));

        // generate points in the AprilTag frame
        std::vector<geometry_msgs::PointStamped> apriltag_points;
        double step = 0.15; // step size for points along the chosen axis
        if (m <= 1)
        {
            for (int i = 0; i < n; ++i)
            {
                geometry_msgs::PointStamped point_apriltag;
                point_apriltag.header.frame_id = "tag_10";
                point_apriltag.header.stamp = ros::Time(0);

                point_apriltag.point.x = i * step;  // increment x
                point_apriltag.point.y = m * (point_apriltag.point.x) + q;  // y = mx + q
                point_apriltag.point.z = 0.0;   // flat plane in the apriltag frame

                apriltag_points.push_back(point_apriltag);
            }
        } else  // the line is too steep so we will iterate over y
        {
            // in order to stay within the table limit we decreased the step
            step = 0.08;

            for (int i = 0; i < n; ++i)
            {
                geometry_msgs::PointStamped point_apriltag;
                point_apriltag.header.frame_id = "tag_10";
                point_apriltag.header.stamp = ros::Time(0);

                point_apriltag.point.y = i * step + q;  // increment x
                point_apriltag.point.x = (point_apriltag.point.y - q) / m;  // x = (y - q) / m
                point_apriltag.point.z = 0.0;   // flat plane in the apriltag frame

                apriltag_points.push_back(point_apriltag);
            }
        }

        // transform the points to the map frame
        for (const auto &point_apriltag : apriltag_points)
        {
            geometry_msgs::PointStamped point_map;
            tf_listener.transformPoint("map", point_apriltag, point_map);

            // save the transformed point in the global vector
            global_points.push_back(point_map.point);
        }

        ROS_INFO("Generated and transformed %d points to the map frame.", n);

    }
    catch (tf::TransformException &ex)
    {
        ROS_WARN("Could not transform points to map frame: %s", ex.what());
    }
}

void extendTorso()
{
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> torso_client("/torso_controller/follow_joint_trajectory", true);

    ROS_INFO("Waiting for torso controller action server...");
    torso_client.waitForServer();
    ROS_INFO("Torso controller action server available!");

    trajectory_msgs::JointTrajectory trajectory;
    trajectory.joint_names.push_back("torso_lift_joint");

    trajectory_msgs::JointTrajectoryPoint point;
    point.positions.push_back(0.35); // desired height of the torso
    point.time_from_start = ros::Duration(2.0);

    // add the point to the trajectory
    trajectory.points.push_back(point);

    // create the goal message
    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory = trajectory;

    ROS_INFO("Sending torso extension goal...");
    torso_client.sendGoal(goal);

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
    actionlib::SimpleActionClient<control_msgs::PointHeadAction> head_client("head_controller/point_head_action", true);

    ROS_INFO("Waiting for head_controller action server...");
    head_client.waitForServer();
    ROS_INFO("Head controller action server available!");

    // define a PointHead goal
    control_msgs::PointHeadGoal goal;
    goal.target.header.stamp = ros::Time(0);
    goal.target.header.frame_id = "xtion_rgb_optical_frame";
    goal.target.point.x = 0.0;
    goal.target.point.y = tan(pitch); // convert pitch angle to tangent
    goal.target.point.z = 1.0;
    goal.pointing_axis.z = 1.0;       // set the pointing axis
    goal.pointing_frame = "xtion_rgb_optical_frame";
    goal.min_duration = ros::Duration(1.0);
    goal.max_velocity = 0.6;

    ROS_INFO("Sending goal to head_controller...");
    ros::spinOnce();
    head_client.sendGoal(goal);

    ROS_INFO("Goal sent to head_controller, now waiting for result...");

    bool finished_before_timeout = head_client.waitForResult(ros::Duration(5.0));
    if (finished_before_timeout)
    {
        ROS_INFO("Robot is now looking with an inclined head.");
    }
    else
    {
        ROS_WARN("Failed to incline head within the timeout period.");
    }
}


void moveToPoses(const std::vector<geometry_msgs::Pose> &poses)
{
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);

    ROS_INFO("Waiting for move_base action server...");
    ac.waitForServer();
    ROS_INFO("move_base action server available!");

    for (size_t i = 0; i < poses.size(); ++i)
    {
        // define the goal pose
        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.pose = poses[i];

        ROS_INFO("Sending goal %lu: Position(%f, %f, %f), Orientation(%f, %f, %f, %f)",
                 i + 1,
                 poses[i].position.x, poses[i].position.y, poses[i].position.z,
                 poses[i].orientation.x, poses[i].orientation.y, poses[i].orientation.z, poses[i].orientation.w);
        ac.sendGoal(goal);

        bool success = ac.waitForResult(ros::Duration(30.0));
        if (success && ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ROS_INFO("Successfully reached goal %lu!", i + 1);
        }
        else
        {
            ROS_WARN("Failed to reach goal %lu. Aborting remaining goals.", i + 1);
            break;
        }
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "node_a");
    auto nh = std::make_shared<ros::NodeHandle>();
    NodeHandleShared nh_ptr(nh);
    
    debug_marker_pub = nh->advertise<visualization_msgs::Marker>("debug_points", 10);

    ros::ServiceClient client = nh->serviceClient<tiago_iaslab_simulation::Coeffs>("/straight_line_srv");

    tiago_iaslab_simulation::Coeffs srv;

    ROS_INFO("Waiting for /straight_line_srv service...");
    ros::service::waitForService("/straight_line_srv");
    srv.request.ready = true; 
    ROS_INFO("/straight_line_srv service available!");

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
    inclineHead(M_PI / 4);

    moveToPoses(target_poses);

    int n = 6;
    int num_goals = 3;

    generatePointsInAprilTagFrame(m, q, n, *nh);

    // publish the debug points
    publishDebugPoints(global_points, *nh);

    ir2425_group_08::PlaceService srv_place_goal;

    ros::ServiceClient client_place_goal = nh->serviceClient<ir2425_group_08::PlaceService>("/place_goal", true);
    ros::service::waitForService("/place_goal");
    srv_place_goal.request.target_points = global_points;
    srv_place_goal.request.num_goals = num_goals;
    
    if (!client_place_goal.call(srv_place_goal))
    {
        ROS_ERROR("Failed to call /place_goal service.");
        return 1;
    }

    ros::spin();
    return 0;
}
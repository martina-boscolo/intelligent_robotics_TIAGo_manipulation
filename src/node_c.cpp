#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf/transform_datatypes.h>
#include "ir2425_group_08/PickAndPlaceAction.h"
#include <gazebo_ros_link_attacher/Attach.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <actionlib/client/simple_action_client.h>

const float tiago_start_x = -6.580157;
const float tiago_start_y = 1.369999;

// TODO
//  class Manipulation
//  {
//  };

actionlib::SimpleActionServer<ir2425_group_08::PickAndPlaceAction> *as_ptr;

ros::ServiceClient attachService_;
ros::ServiceClient detachService_;

void pickAndPlaceCallback(const ir2425_group_08::PickAndPlaceGoalConstPtr &goal)
{
    ROS_INFO_STREAM("Got tag " << goal->id << " with pose " << goal->goal_pose);

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit::planning_interface::MoveGroupInterface move_group("arm_torso");
    moveit::planning_interface::MoveGroupInterface gripper_group("gripper");

    move_group.setPoseReferenceFrame("base_link");
    gripper_group.setPoseReferenceFrame("base_link");

    gripper_group.setPlanningTime(10.0);

    //  ROS_INFO_STREAM("MoveIt planning frame: " << move_group.getPlanningFrame());
    move_group.setPlanningTime(30.0);

    // Add the object to the collision environment
    moveit_msgs::CollisionObject obj;
    obj.id = "obj";
    obj.header.frame_id = "base_link";

    geometry_msgs::Pose obj_pose;
    obj_pose.position.x = goal->goal_pose.position.x;
    obj_pose.position.y = goal->goal_pose.position.y;
    obj_pose.position.z = goal->goal_pose.position.z;
    obj_pose.orientation = goal->goal_pose.orientation;

    shape_msgs::SolidPrimitive obj_shape;
    obj_shape.type = obj_shape.BOX;
    obj_shape.dimensions = {0.05, 0.05, 0.05};

    obj.primitives.push_back(obj_shape);
    obj.primitive_poses.push_back(obj_pose);
    obj.operation = obj.ADD;

    planning_scene_interface.applyCollisionObjects({obj});

    // Plan to pre-grasp pose
    geometry_msgs::Pose pre_grasp_pose = obj_pose;
    pre_grasp_pose.position.z += 0.3; // Offset above the object

    // tf2::Quaternion q_orig, q_rot, q_new;
    //     q_orig.setX(0.0);
    //     q_orig.setY(0.0);
    //     q_orig.setZ(0.0);
    //     q_orig.setW(1.0); // Identity quaternion (no rotation)
    //     double roll = M_PI / 2;            // No roll adjustment
    //     double pitch = M_PI / 2;    // 90 degrees
    //     double yaw = M_PI / 2;             // No yaw adjustment
    //     q_rot.setRPY(roll, pitch, yaw);
    //     // Combine the original orientation with the desired rotation
    //     q_new = q_rot * q_orig; // Order matters: q_rot * q_orig applies the rotation
    //     q_new.normalize();      // Ensure the quaternion is normalized
    //     pre_grasp_pose.orientation.x = q_new.x();
    //     pre_grasp_pose.orientation.y = q_new.y();
    //     pre_grasp_pose.orientation.z = q_new.z();
    //     pre_grasp_pose.orientation.w = q_new.w();

    //  orientation.setRPY(0, 1.57, 0); // Roll = 180°, Pitch = 0° blu, Yaw = 0°

    move_group.setPoseTarget(pre_grasp_pose);
    moveit::planning_interface::MoveGroupInterface::Plan pre_grasp_plan;
    if (move_group.plan(pre_grasp_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
    {
        ROS_INFO("Planning to pre-grasp pose successful. Executing...");
        move_group.execute(pre_grasp_plan);
    }
    else
    {
        ROS_ERROR("Planning to pre-grasp pose failed.");
        return;
    }

    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> gripper_client("/gripper_controller/follow_joint_trajectory", true);

    ROS_INFO("Waiting for the gripper action server...");
    gripper_client.waitForServer();
    ROS_INFO("Gripper action server connected.");

    // Define the goal for opening the gripper
    control_msgs::FollowJointTrajectoryGoal gripper_goal;
    gripper_goal.trajectory.joint_names.push_back("gripper_left_finger_joint");
    gripper_goal.trajectory.joint_names.push_back("gripper_right_finger_joint");

    trajectory_msgs::JointTrajectoryPoint open_point;
    open_point.positions.push_back(0.04);            // Open position for left finger (meters/radians)
    open_point.positions.push_back(0.04);            // Open position for right finger (meters/radians)
    open_point.time_from_start = ros::Duration(1.0); // 1-second movement duration
    gripper_goal.trajectory.points.push_back(open_point);

    gripper_goal.trajectory.header.stamp = ros::Time::now();
    gripper_client.sendGoal(gripper_goal);
    gripper_client.waitForResult();

    if (gripper_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("Gripper opened successfully.");
    }
    else
    {
        ROS_WARN("Failed to open the gripper.");
        return;
    }

    geometry_msgs::Pose grasp_pose = obj_pose;
    grasp_pose.position.z += 0.15;
    // se si cambia orientazione sopra va cambiata anche qui

    move_group.setPoseTarget(grasp_pose);
    moveit::planning_interface::MoveGroupInterface::Plan grasp_plan;
    if (move_group.plan(grasp_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
    {
        ROS_INFO("Planning to grasp pose successful. Executing...");
        move_group.execute(grasp_plan);
    }
    else
    {
        ROS_ERROR("Planning to grasp pose failed.");
        return;
    }
    gripper_goal.trajectory.points.clear(); // Clear the previous trajectory
    trajectory_msgs::JointTrajectoryPoint close_point;
    close_point.positions.push_back(0.0);             // Closed position for left finger
    close_point.positions.push_back(0.0);             // Closed position for right finger
    close_point.time_from_start = ros::Duration(1.0); // 1-second movement duration
    gripper_goal.trajectory.points.push_back(close_point);

    gripper_goal.trajectory.header.stamp = ros::Time::now();
    gripper_client.sendGoal(gripper_goal);
    gripper_client.waitForResult();

    if (gripper_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("Gripper closed successfully. Object grasped!");
    }
    else
    {
        ROS_WARN("Failed to close the gripper.");
        return;
    }

    gazebo_ros_link_attacher::Attach attach_srv_msg;
    attach_srv_msg.request.model_name_1 = "tiago";
    attach_srv_msg.request.link_name_1 = "arm_7_link";

    attach_srv_msg.request.model_name_2 = "cube_5";
    attach_srv_msg.request.link_name_2 = "cube_5_link";

    // Check if object is attached
    if (attachService_.call(attach_srv_msg))
    {
        ROS_INFO("Attached object");
    }

    // Raise the arm with the object
    move_group.setPoseTarget(pre_grasp_pose);
    move_group.move();

    if (detachService_.call(attach_srv_msg))
    {
        ROS_INFO("Detached object");
    }

    // Remove the object from the collision environment
    planning_scene_interface.removeCollisionObjects({obj.id});

    // Indicate success
    ir2425_group_08::PickAndPlaceResult result;
    result.success = true;
    as_ptr->setSucceeded(result);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "node_c");
    ros::NodeHandle nh;
    attachService_ = nh.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/attach");
    attachService_.waitForExistence();
    detachService_ = nh.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/detach");
    detachService_.waitForExistence();

    // Initialize MoveIt interfaces
    // moveit::planning_interface::MoveGroupInterface arm_group("arm_torso");
    // moveit::planning_interface::MoveGroupInterface gripper_group("gripper");
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // arm_group.setPlanningTime(10.0);
    // gripper_group.setPlanningTime(10.0);

    // Add the pick_table to the collision environment
    moveit_msgs::CollisionObject pick_table;
    pick_table.id = "pick_table";
    pick_table.header.frame_id = "map";

    geometry_msgs::Pose pick_table_pose;
    pick_table_pose.position.x = 1.332080 - tiago_start_x;
    pick_table_pose.position.y = -1.646500 - tiago_start_y;
    pick_table_pose.position.z = 0.4;
    pick_table_pose.orientation.w = 1.0;

    shape_msgs::SolidPrimitive pick_table_shape;
    pick_table_shape.type = pick_table_shape.BOX;
    pick_table_shape.dimensions = {0.9, 0.9, pick_table_pose.position.z * 2};

    pick_table.primitives.push_back(pick_table_shape);
    pick_table.primitive_poses.push_back(pick_table_pose);
    pick_table.operation = pick_table.ADD;

    // Add the place_table to the collision environment
    moveit_msgs::CollisionObject place_table;
    place_table.id = "place_table";
    place_table.header.frame_id = "map";

    geometry_msgs::Pose place_table_pose;
    place_table_pose.position.x = 1.315500 - tiago_start_x;
    place_table_pose.position.y = -0.553829 - tiago_start_y;
    place_table_pose.position.z = 0.4;
    place_table_pose.orientation.w = 1.0;

    shape_msgs::SolidPrimitive place_table_shape;
    place_table_shape.type = place_table_shape.BOX;
    place_table_shape.dimensions = {0.9, 0.9, place_table_pose.position.z * 2};

    place_table.primitives.push_back(place_table_shape);
    place_table.primitive_poses.push_back(place_table_pose);
    place_table.operation = place_table.ADD;

    planning_scene_interface.applyCollisionObjects({pick_table, place_table});

    actionlib::SimpleActionServer<ir2425_group_08::PickAndPlaceAction> as(nh, "/pick_and_place", pickAndPlaceCallback, false);
    as_ptr = &as;
    as.start();
    ROS_INFO("Server started!");

    // to remove objects
    // planning_scene_interface.removeCollisionObjects({pick_table , place_table});

    ros::spin();

    return 0;
}
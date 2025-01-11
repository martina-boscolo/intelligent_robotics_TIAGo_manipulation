#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
#include <tf/transform_datatypes.h>
#include "ir2425_group_08/PickAndPlaceAction.h"

const float tiago_start_x = -6.580157;
const float tiago_start_y = 1.369999;

// TODO
//  class Manipulation
//  {
//  };

actionlib::SimpleActionServer<ir2425_group_08::PickAndPlaceAction> *as_ptr;



void pickAndPlaceCallback(const ir2425_group_08::PickAndPlaceGoalConstPtr &goal)
{
    ROS_INFO_STREAM("Got tag " << goal->id << " with pose " << goal->goal_pose);

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit::planning_interface::MoveGroupInterface move_group("arm_torso");
    moveit::planning_interface::MoveGroupInterface gripper_group("gripper");
    move_group.setPlanningTime(10.0);
    gripper_group.setPlanningTime(10.0);

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
    pre_grasp_pose.position.z += 0.4; // Offset above the object

    // Adjust the orientation to make the gripper point downward
    // tf::Quaternion orientation;
    // orientation.setRPY(M_PI, 0, 0); // Roll = 180°, Pitch = 0°, Yaw = 0°
    // pre_grasp_pose.orientation.x = orientation.x();
    // pre_grasp_pose.orientation.y = orientation.y();
    // pre_grasp_pose.orientation.z = orientation.z();
    // pre_grasp_pose.orientation.w = orientation.w();

  
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
  

    // Indicate success
    ir2425_group_08::PickAndPlaceResult result;
    result.success = true;
    as_ptr->setSucceeded(result);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "node_c");
    ros::NodeHandle nh;

    // Initialize MoveIt interfaces
    moveit::planning_interface::MoveGroupInterface arm_group("arm_torso");
    moveit::planning_interface::MoveGroupInterface gripper_group("gripper");
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    arm_group.setPlanningTime(10.0);
    gripper_group.setPlanningTime(10.0);

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
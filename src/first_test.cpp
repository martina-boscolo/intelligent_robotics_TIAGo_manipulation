#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/CollisionObject.h>

#include <shape_msgs/SolidPrimitive.h>
#include <geometry_msgs/Pose.h>

const float tiago_start_x = -6.580157;
const float tiago_start_y = 1.369999;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "fisr_test");

    uint32_t thread_count = 0;
    ros::AsyncSpinner spinner(thread_count);
    spinner.start();
    
    // get the move_group interface for the arm
    moveit::planning_interface::MoveGroupInterface move_group("arm");

    // get the planning scene interface
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // save a pointer to the joint group for the arm 
    const moveit::core::JointModelGroup* joint_move_group = move_group.getCurrentState()->getJointModelGroup("arm");

    // ---------------------------
    // collision object experiment

    // get planning frame for the collision object
    /*
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = move_group.getPlanningFrame();
    ROS_INFO_STREAM("Planning frame: " + collision_object.header.frame_id);
    */
   moveit_msgs::CollisionObject collision_table_1;
   collision_table_1.header.frame_id = "map";

    // define two boxes
    collision_table_1.id = "table_1";
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = 1;
    primitive.dimensions[primitive.BOX_Y] = 1;
    primitive.dimensions[primitive.BOX_Z] = 0.9;

    // add a pose for the first table
    geometry_msgs::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = 1.315500 - tiago_start_x;
    box_pose.position.y = -0.553829 - tiago_start_y;
    box_pose.position.z = 0.45;

    // link the primitive to the collision object
    collision_table_1.primitives.push_back(primitive);

    // link the pose to the collision_object
    collision_table_1.primitive_poses.push_back(box_pose);

    collision_table_1.operation = collision_table_1.ADD;

    // add the collision object (encapsuled in a vector)
    planning_scene_interface.applyCollisionObjects({collision_table_1});

    //ros::spin();
    return 0;
}
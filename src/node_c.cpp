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
#include <iostream>
#include <string>
#include "ir2425_group_08/RouteHandler.h"

using NodeHandleShared = std::shared_ptr<ros::NodeHandle>;

const float tiago_start_x = -6.580157;
const float tiago_start_y = 1.369999;

// define name of models of links of Gazebo objects
const std::string model_types[9] = {"Hexagon", "Hexagon_2", "Hexagon_3", "cube", "cube_5",
	"cube_6", "Triangle", "Triangle_8", "Triangle_9"};

const std::string link_types[9] = {"Hexagon_link", "Hexagon_2_link", "Hexagon_3_link", "cube_link", "cube_5_link",
	"cube_6_link", "Triangle_link", "Triangle_8_link", "Triangle_9_link"};


// TODO
//  class Manipulation
//  {
//  };

actionlib::SimpleActionServer<ir2425_group_08::PickAndPlaceAction> *as_ptr;
ros::ServiceClient attachService_;
ros::ServiceClient detachService_;
ir2425_group_08::RouteHandler* rh_ptr;
ros::NodeHandle* nh_ptr;

void armInSafePosition()
{
    moveit::planning_interface::MoveGroupInterface move_group("arm_torso");
    move_group.setPlanningTime(15.0);

    std::map<std::string, double> target_joint_values;
    target_joint_values["torso_lift_joint"] = 0.272; // meters
    target_joint_values["arm_1_joint"] = 27.0 * M_PI / 180.0; // Convert degrees to radians
    target_joint_values["arm_2_joint"] = 33.0 * M_PI / 180.0;
    target_joint_values["arm_3_joint"] = -163.0 * M_PI / 180.0;
    target_joint_values["arm_4_joint"] = 83.0 * M_PI / 180.0;
    target_joint_values["arm_5_joint"] = -83.0 * M_PI / 180.0;
    target_joint_values["arm_6_joint"] = 14.0 * M_PI / 180.0;
    target_joint_values["arm_7_joint"] = 33.0 * M_PI / 180.0;

    move_group.setJointValueTarget(target_joint_values);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
    {
        ROS_INFO("Arm in safe position successful. Executing...");
        move_group.execute(plan);
    }
    else
    {
        ROS_ERROR("Arm in safe position failed.");
    }
}

void armInPregraspPosition()
{
    moveit::planning_interface::MoveGroupInterface move_group("arm_torso");
    move_group.setPlanningTime(15.0);

    std::map<std::string, double> target_joint_values;
    target_joint_values["torso_lift_joint"] = 0.272; // meters
    target_joint_values["arm_1_joint"] = 25.0 * M_PI / 180.0; // Convert degrees to radians
    target_joint_values["arm_2_joint"] = 58.0 * M_PI / 180.0;
    target_joint_values["arm_3_joint"] = -69.0 * M_PI / 180.0;
    target_joint_values["arm_4_joint"] = 83.0 * M_PI / 180.0;
    target_joint_values["arm_5_joint"] = -74.0 * M_PI / 180.0;
    target_joint_values["arm_6_joint"] = -7.0 * M_PI / 180.0;
    target_joint_values["arm_7_joint"] = 45.0 * M_PI / 180.0;

    move_group.setJointValueTarget(target_joint_values);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
    {
        ROS_INFO("Arm over the table position successful. Executing...");
        move_group.execute(plan);
    }
    else
    {
        ROS_ERROR("Arm over the table position failed.");
    }
}
void addCollisionObject(const int id, const geometry_msgs::Pose pose )
{
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    moveit_msgs::CollisionObject obj;
    obj.header.frame_id = "map";
    obj.id = std::to_string(id);

    geometry_msgs::Pose obj_pose;
    obj_pose.position.x = pose.position.x;
    obj_pose.position.y = pose.position.y;
    obj_pose.position.z = pose.position.z;
    obj_pose.orientation = pose.orientation;

    shape_msgs::SolidPrimitive obj_shape;

    if (id < 1 || id > 9){
        ROS_ERROR("ERROR  | Object with id %d not supported", id);
        return;
    } else if (id <= 3 && id >= 1) {
        obj_shape.type = obj_shape.CYLINDER;
        obj_shape.dimensions = {0.10, 0.04};
        obj_pose.position.z -=0.05;
        ROS_INFO("hexagon");
    } else if (id <= 6 && id >= 4) {
        obj_shape.type = obj_shape.BOX;
        obj_shape.dimensions = {0.05, 0.05, 0.05};
        obj_pose.position.z -=0.025;
        ROS_INFO("cube");
    } else if (id <= 9 && id >= 7) {
        obj_shape.type = obj_shape.BOX;
        obj_shape.dimensions = {0.05, 0.05, 0.05};
        obj_pose.position.z -=0.01; //approx
        ROS_INFO("triangle");
    }
    obj.primitives.push_back(obj_shape);
    obj.primitive_poses.push_back(obj_pose);
    obj.operation = obj.ADD;

    planning_scene_interface.applyCollisionObjects({obj});
}

void addCollisionObjects(const ir2425_group_08::PickAndPlaceGoalConstPtr &goal)
{
    for (const auto &o : goal->detectedObj)
    { // Cycle through all detected objects

        addCollisionObject(o.id, o.pose);
    }
}

void addGlobalCollisionObject( )
{
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    //Add the pick_table to the collision environment
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
}

geometry_msgs::Pose goToPreGrasp(const ir2425_group_08::PickAndPlaceGoalConstPtr &goal, bool& success )
{
    moveit::planning_interface::MoveGroupInterface move_group("arm_torso");
    move_group.setPoseReferenceFrame("map");
    move_group.setPlanningTime(30.0);

    geometry_msgs::Pose obj_pose = goal->goal_pose;
    geometry_msgs::Pose pre_grasp_pose = obj_pose;

    tf::Quaternion init_quat(obj_pose.orientation.x, obj_pose.orientation.y, obj_pose.orientation.z, obj_pose.orientation.w);
    double roll, pitch, yaw;
    tf::Quaternion final_quat(0,0,0,1);
    tf::Matrix3x3(init_quat).getRPY(roll, pitch, yaw);

    if (goal->id < 1 || goal->id > 9)
    {
        ROS_ERROR("ERROR  | Object with id %d not supported", goal->id);
    }
    else if (goal->id <= 3 && goal->id >= 1)
    {
        roll = 0;
        pitch = M_PI_2;
        pre_grasp_pose.position.z += 0.3; 
        ROS_INFO("hexagon");
    }
    else if (goal->id <= 6 && goal->id >= 4)
    {
        roll = 0;
        pitch = M_PI_2;
        pre_grasp_pose.position.z += 0.3;
        ROS_INFO("cube"); 
    }
    else if (goal->id <= 9 && goal->id >= 7)
    {
        roll = 0;
        pitch = M_PI_2;
        //yaw += M_PI_2;
        pre_grasp_pose.position.z += 0.3;
        ROS_INFO("triangle");
    }
    final_quat.setRPY(roll, pitch, yaw);
    pre_grasp_pose.orientation.w = final_quat.w();
    pre_grasp_pose.orientation.x = final_quat.x();
    pre_grasp_pose.orientation.y = final_quat.y();
    pre_grasp_pose.orientation.z = final_quat.z();

    move_group.setPoseTarget(pre_grasp_pose);

    moveit::planning_interface::MoveGroupInterface::Plan pre_grasp_plan;
    if (move_group.plan(pre_grasp_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
    {
        ROS_INFO("Planning to pre-grasp pose successful. Executing...");
        move_group.execute(pre_grasp_plan);
        success = true;
    }
    else
    {
        ROS_ERROR("Planning to pre-grasp pose failed.");
        success = false;  // Set failure
    }
    return pre_grasp_pose;
}


bool goToGrasp(const geometry_msgs::Pose pre_grasp_pose, const int goal){

    moveit::planning_interface::MoveGroupInterface move_group("arm_torso");
    move_group.setPoseReferenceFrame("map");
    move_group.setPlanningTime(30.0);
    geometry_msgs::Pose grasp_pose = pre_grasp_pose;
    if (goal < 1 || goal > 9)
    {
        ROS_ERROR("ERROR  | Object with id %d not supported", goal);
        return false;
    }
    else if (goal <= 3 && goal >= 1)
    {
        grasp_pose.position.z -= 0.076; //0.3-0.076
        ROS_INFO("hexagon");
    }
    else if (goal <= 6 && goal >= 4)
    {
        grasp_pose.position.z -= 0.08;
        ROS_INFO("cube"); 
    }
    else if (goal <= 9 && goal >= 7)
    {
        grasp_pose.position.z -= 0.063;
        ROS_INFO("triangle");
    }
    move_group.setPoseTarget(grasp_pose);
    moveit::planning_interface::MoveGroupInterface::Plan grasp_plan;
    if (move_group.plan(grasp_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
    {
        ROS_INFO("Planning to grasp pose successful. Executing...");
        move_group.execute(grasp_plan);
        return true;
    }
    else
    {
        ROS_ERROR("Planning to grasp pose failed.");
        return false;
    }
}

geometry_msgs::Pose goToPlace(const ir2425_group_08::PickAndPlaceGoalConstPtr &goal, bool &success, int &selected_index)
{
    moveit::planning_interface::MoveGroupInterface move_group("arm_torso");
    move_group.setPoseReferenceFrame("map");
    move_group.setPlanningTime(30.0);
    geometry_msgs::Pose pre_grasp_pose;
    int i;
    for ( i = 0; i < goal->target_points.size(); i++)
    {

        geometry_msgs::Point current_attempt = goal->target_points[i];

        ROS_INFO_STREAM("Target point " << current_attempt);

        // Define pre-grasp pose using the provided point
        

        // Set the position using the provided point
        pre_grasp_pose.position.x = current_attempt.x;
        pre_grasp_pose.position.y = current_attempt.y;
        pre_grasp_pose.position.z = current_attempt.z + 0.3; // Offset to ensure pre-grasp height

        // Set the orientation to ensure the gripper is pointing downward
        tf::Quaternion final_quat;
        double roll = 0.0;
        double pitch = M_PI_2; // Pointing down
        double yaw = 0.0;      // Yaw can be adjusted as needed
        final_quat.setRPY(roll, pitch, yaw);

        pre_grasp_pose.orientation.w = final_quat.w();
        pre_grasp_pose.orientation.x = final_quat.x();
        pre_grasp_pose.orientation.y = final_quat.y();
        pre_grasp_pose.orientation.z = final_quat.z();

        // Set the target pose in the MoveGroupInterface
        move_group.setPoseTarget(pre_grasp_pose);

        // Plan and execute the motion
        moveit::planning_interface::MoveGroupInterface::Plan pre_grasp_plan;
        if (move_group.plan(pre_grasp_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
        {
            ROS_INFO_STREAM("Positioned object on place table. Executing..." << i );
            move_group.execute(pre_grasp_plan);
            pre_grasp_pose.position.z -= 0.2;
            success = true;
            selected_index = i; 
            break; 
        }
        else
        {
            ROS_ERROR("Failed to position object on place table. %d index failed" , i);
            success = false; // Set failure
        }
    }

    return pre_grasp_pose;
}

bool controlGripper(actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> &gripper_client, const std::vector<double> &positions) {
    control_msgs::FollowJointTrajectoryGoal gripper_goal;
    gripper_goal.trajectory.joint_names = {"gripper_left_finger_joint", "gripper_right_finger_joint"};

    trajectory_msgs::JointTrajectoryPoint point;
    point.positions = positions;
    point.time_from_start = ros::Duration(3.0); // 1 second duration
    gripper_goal.trajectory.points.push_back(point);
    gripper_goal.trajectory.header.stamp = ros::Time::now();

    gripper_client.sendGoal(gripper_goal);
    gripper_client.waitForResult();

    if (gripper_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("Gripper action succeeded.");
        //return true;
    } else {
        ROS_WARN("Gripper action failed.");
        //return false;
    }
    return true;
}

void attachObjectToRobot(const ir2425_group_08::PickAndPlaceGoalConstPtr &goal) {
    gazebo_ros_link_attacher::Attach attach_srv_msg;
    attach_srv_msg.request.model_name_1 = "tiago";
    attach_srv_msg.request.link_name_1 = "arm_7_link";
    attach_srv_msg.request.model_name_2 = model_types[goal->id - 1];
    attach_srv_msg.request.link_name_2 = link_types[goal->id - 1];

    if (attachService_.call(attach_srv_msg)) {
        ROS_INFO("Attached object");
    }
}

void detachObjectFromRobot(const ir2425_group_08::PickAndPlaceGoalConstPtr &goal) {
    gazebo_ros_link_attacher::Attach detach_srv_msg;
    detach_srv_msg.request.model_name_1 = "tiago";
    detach_srv_msg.request.link_name_1 = "arm_7_link";
    detach_srv_msg.request.model_name_2 = model_types[goal->id - 1];
    detach_srv_msg.request.link_name_2 = link_types[goal->id - 1];

    if (detachService_.call(detach_srv_msg)) {
        ROS_INFO("Detached object");
    }
}

void setActionResSucceeded(int index) {
    ir2425_group_08::PickAndPlaceResult result;
    result.success = true;
    result.selected_target_index=index;
    result.new_current_waypoint = rh_ptr->getCurrentWaypointIndex();
    as_ptr->setSucceeded(result);
}

void setActionResAborted() {
    ROS_INFO("ABORTING GOAL");

    ir2425_group_08::PickAndPlaceResult result;
    result.success = false;
    result.selected_target_index=42;
    result.new_current_waypoint = rh_ptr->getCurrentWaypointIndex();
    as_ptr->setAborted(result);
}


void pickAndPlaceCallback(const ir2425_group_08::PickAndPlaceGoalConstPtr &goal)
{
    ROS_INFO_STREAM("Got tag " << goal->id << " with pose " << goal->goal_pose);
   

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit::planning_interface::MoveGroupInterface move_group("arm_torso");
    moveit::planning_interface::MoveGroupInterface gripper_group("gripper");
    move_group.setPoseReferenceFrame("map");
    move_group.setPlanningTime(30.0);
    gripper_group.setPoseReferenceFrame("map");
    gripper_group.setPlanningTime(10.0);
    rh_ptr->setCurrentWaypointIndex(goal->current_waypoint);

    addCollisionObjects(goal);
    armInSafePosition();
    ros::Duration(1.0).sleep();
    armInPregraspPosition();

    geometry_msgs::Pose obj_pose = goal->goal_pose;

    //deve diventare addCollisionObjects
    
    bool success;
    geometry_msgs::Pose pre_grasp_pose = goToPreGrasp(goal, success);
        if (!success) {  
        armInSafePosition();      
        setActionResAborted();
        return;
    }
  
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> gripper_client("/gripper_controller/follow_joint_trajectory", true);
    ROS_INFO("Waiting for the gripper action server...");
    gripper_client.waitForServer();
    ROS_INFO("Gripper action server connected.");

    controlGripper(gripper_client, {0.05, 0.05}); // Open gripper
    if (!goToGrasp(pre_grasp_pose, goal->id)){
        armInSafePosition();
        setActionResAborted();
        return;
    }
    planning_scene_interface.removeCollisionObjects({std::to_string(goal->id)});

    //gripper_goal.trajectory.points.clear(); // Clear the previous trajectory

    controlGripper(gripper_client, {0.00, 0.00}); // Open gripper

    attachObjectToRobot(goal);
    ros::Duration(0.5).sleep();
    goToPreGrasp(goal, success); 
        if (!success) {  
        armInSafePosition();      
        setActionResAborted();
        return;
    }
    armInSafePosition();
    
    //qua si deve muovere
    rh_ptr->goFrontPlace(0);
    ROS_INFO("Moving...");
    armInPregraspPosition();
    int selected_index;
    geometry_msgs::Pose placed = goToPlace(goal, success, selected_index);
        if (!success) { 
        controlGripper(gripper_client, {0.05, 0.05});// Open gripper
        detachObjectFromRobot(goal);  
        armInSafePosition();     
        setActionResAborted();
        return;
    }

    controlGripper(gripper_client, {0.05, 0.05});// Open gripper
    detachObjectFromRobot(goal);
    addCollisionObject(goal->id, placed );
    ros::Duration(2.0).sleep();
    armInPregraspPosition();
    armInSafePosition();

    setActionResSucceeded(selected_index);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "node_c");
    //ros::NodeHandle nh;
    auto nh = std::make_shared<ros::NodeHandle>();
    NodeHandleShared nh_ptr_shared(nh);
    nh_ptr = nh.get();
    attachService_ = nh->serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/attach");
    attachService_.waitForExistence();
    detachService_ = nh->serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/detach");
    detachService_.waitForExistence();

    ir2425_group_08::RouteHandler rh(nh_ptr_shared);
    rh_ptr = &rh;

    // Initialize MoveIt interfaces
    // moveit::planning_interface::MoveGroupInterface arm_group("arm_torso");
    // moveit::planning_interface::MoveGroupInterface gripper_group("gripper");
    // arm_group.setPlanningTime(10.0);
    // gripper_group.setPlanningTime(10.0);
    addGlobalCollisionObject();

    actionlib::SimpleActionServer<ir2425_group_08::PickAndPlaceAction> as( "/pick_and_place", pickAndPlaceCallback, false);
    as_ptr = &as;
    as.start();
    ROS_INFO("Server started!");

    ros::spin();

    return 0;
}
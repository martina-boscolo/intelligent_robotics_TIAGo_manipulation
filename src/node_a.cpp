#include "ros/ros.h"
#include <actionlib/client/simple_action_client.h>

#include <tiago_iaslab_simulation/Objs.h>
#include <ir2425_group_08/FindTagsAction.h>
#include <apriltag_ros/AprilTagDetection.h>

void feedbackCallback(const ir2425_group_08::FindTagsFeedbackConstPtr &msg)
{
    apriltag_ros::AprilTagDetection tag = msg->current_detection;

    ROS_INFO("Found tag with ID: %d at position (wrt map) - x: %.2f, y: %.2f, z: %.2f",
             tag.id[0],
             tag.pose.pose.pose.position.x,
             tag.pose.pose.pose.position.y,
             tag.pose.pose.pose.position.z);
    //ROS_INFO("Robot Status: %s", msg->robot_status.c_str());
    ROS_INFO("Up to now found %.1f tags.", msg->progress_status);

    //TO DO print also status message 
}

void resultCallback(const actionlib::SimpleClientGoalState &state,
                    const ir2425_group_08::FindTagsResultConstPtr &result)
{
    ROS_INFO("Action finished with state: %s", state.toString().c_str());
    ROS_INFO("Finished: %s", result->finished ? "true" : "false");
    ROS_INFO("Status Message: %s", result->status_message.c_str());
    ROS_INFO("Detected Tags:");

    for (const auto &tag : result->detected_tags)
    {
        ROS_INFO(" - ID: %d at position (wrt map) - x: %.2f, y: %.2f, z: %.2f",
                 tag.id[0],
                 tag.pose.pose.pose.position.x,
                 tag.pose.pose.pose.position.y,
                 tag.pose.pose.pose.position.z);
    }
}

int main(int argc, char **argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "node_a");
    ros::NodeHandle nh;

    // Create a service client to request Apriltag IDs
    ros::ServiceClient client = nh.serviceClient<tiago_iaslab_simulation::Objs>("/apriltag_ids_srv");

    // Create a service request and response object
    tiago_iaslab_simulation::Objs srv;

    // Wait for the service to become available
    ROS_INFO("Waiting for /apriltags_ids_srv service...");
    ros::service::waitForService("/apriltag_ids_srv");
    srv.request.ready = true; 
    ROS_INFO("/apriltags_ids_srv service available!");

    // Call the service to get target IDs
    if (!client.call(srv))
    {
        ROS_ERROR("Failed to call /apriltags_ids_srv service.");
        return 1;
    }

    // Print the received IDs
    ROS_INFO("Received %lu target IDs.", srv.response.ids.size());
    ROS_INFO("Apriltags IDs:");
    for (const auto &id : srv.response.ids)
    {
        ROS_INFO(" - ID: %d", id);
    }

    // Create an action client to interact with Node B
    actionlib::SimpleActionClient<ir2425_group_08::FindTagsAction> ac("find_tags", true);
    ROS_INFO("Waiting for find_tags server to start (launch node B!)");
    ac.waitForServer();
    ROS_INFO("find_tags server started!");

    //create and send a goal
    ir2425_group_08::FindTagsGoal goal;
    goal.target_ids = srv.response.ids;
    ROS_INFO("Sending goal to node B via find_tags...");
    ac.sendGoal(goal,  
                &resultCallback,
                actionlib::SimpleActionClient<ir2425_group_08::FindTagsAction>::SimpleActiveCallback(),
                &feedbackCallback);

    //Wait for the action to complete
    ac.waitForResult(); 


    // while (!ac.waitForResult(ros::Duration(10.0))) {
    //     ROS_INFO("Waiting for action to complete...");
    // }

    ROS_INFO("It's Over!");

   // ros::spin();
    return 0;
}

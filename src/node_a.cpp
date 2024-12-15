#include "ros/ros.h"
#include <actionlib/client/simple_action_client.h>

#include <tiago_iaslab_simulation/Objs.h>
#include <ir2425_group_08/FindTagsAction.h>
#include <apriltag_ros/AprilTagDetection.h>

void feedbackCallback(const ir2425_group_08::FindTagsFeedbackConstPtr& msg)
{
    for (apriltag_ros::AprilTagDetection tag : msg->alreadyFoundTags)
    {
        //TODO better display
        ROS_INFO("Found tag n %d", tag.id[0]);
    }
}

int main(int argc, char **argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "node_a");
    ros::NodeHandle nh;

    // Wait for the service to become available
    ROS_INFO("Waiting for /apriltags_ids_srv service...");
    ros::service::waitForService("/apriltag_ids_srv");

    // Create a service client
    ros::ServiceClient client = nh.serviceClient<tiago_iaslab_simulation::Objs>("/apriltag_ids_srv");

    // Create a service request and response object
    tiago_iaslab_simulation::Objs srv;

    srv.request.ready = true;

    // Call the service
    if (client.call(srv))
    {
        // Print the received IDs
        ROS_INFO("Received apriltags IDs:");
        for (const auto &id : srv.response.ids)
        {
            ROS_INFO(" - ID: %d", id);
        }

        actionlib::SimpleActionClient<ir2425_group_08::FindTagsAction> ac("find_tags", true);
        
        ROS_INFO("Waiting for find_tags server (launch node B...)");
        ac.waitForServer();

        ir2425_group_08::FindTagsGoal goal;
        goal.ids = srv.response.ids;

        ROS_INFO("Sending goal to node B via find_tags...");
        ac.sendGoal(goal, actionlib::SimpleActionClient<ir2425_group_08::FindTagsAction>::SimpleDoneCallback(),
                actionlib::SimpleActionClient<ir2425_group_08::FindTagsAction>::SimpleActiveCallback(),
                &feedbackCallback);

        ac.waitForResult();

        ROS_INFO("It's Over!");
    }
    else
    {
        ROS_ERROR("Failed to call service /apriltag_ids_srv");
    }

    ros::spin();
    return 0;
}

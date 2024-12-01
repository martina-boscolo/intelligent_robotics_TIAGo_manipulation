#include "ros/ros.h"
#include <tiago_iaslab_simulation/Objs.h>

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
    }
    else
    {
        ROS_ERROR("Failed to call service /apriltag_ids_srv");
    }

    return 0;
}

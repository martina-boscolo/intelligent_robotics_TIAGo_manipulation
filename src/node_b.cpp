#include <ros/ros.h>
#include <ros/topic.h>

#include "ir2425_group_08/PlaceGoal.h"
#include "ir2425_group_08/RouteHandler.h"

std::string NODE_A_TOPIC = "/";

int main(int argc, char** argv)
{
    ros::init(argc, argv, "node_b");
    ros::NodeHandle nh;

    const ir2425_group_08::PlaceGoalConstPtr msg = ros::topic::waitForMessage<ir2425_group_08::PlaceGoal>(NODE_A_TOPIC);
    ROS_INFO_STREAM("Recieved " << msg->num_goals << " goals for pick and place");

    // ciclo di msg->num_goals iterazioni
        // cerca un apriltag -> qua può essere utile spostarsi
        // richiedi al node_c di trasportarlo usando il service apposito

    return 0;
}
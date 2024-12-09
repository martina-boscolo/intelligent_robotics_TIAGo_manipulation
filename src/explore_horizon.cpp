#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <sensor_msgs/LaserScan.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>

#include <vector>
#include <random>

float processScan()
{
    ROS_INFO("Reading lidar...");

    const sensor_msgs::LaserScanConstPtr& msg = ros::topic::waitForMessage<sensor_msgs::LaserScan>("scan"); 

    std::vector<float> ranges = msg->ranges;

    for (int i = 0; i < ranges.size(); ++i)
    {
        if (ranges[i] > msg->range_max)
        {
            ranges[i] = msg->range_max;
        }
    }
    float max = *std::max_element(ranges.begin(), ranges.end());

    // find horizons

    std::vector<std::vector<int>> horizons;
    std::vector<int> tempHorizon;
    for (int i = 0; i < ranges.size(); ++i)
    {
        if (ranges[i] == max)
        {
            tempHorizon.push_back(i);
        }
        else if (!tempHorizon.empty())
        {
            horizons.push_back(tempHorizon);
            tempHorizon.clear();
        }
    }

    ROS_INFO("Lidar found %d horizons", static_cast<int>(horizons.size()));

    // pick one of the most wide horizons

    int max_length = 0;
    std::vector<int> horizonsIndexVector;
    for (int i = 0; i < horizons.size(); ++i)
    {
        if (horizons[i].size() == max_length)
        {
            horizonsIndexVector.push_back(i);
        }
        else if (horizons[i].size() > max_length)
        {
            horizonsIndexVector.clear();
            max_length = horizons[i].size();
            horizonsIndexVector.push_back(i);
        }
    }

    int chosenOne = 0;
    std::random_device dev;
    std::mt19937 rng(dev());
    std::uniform_int_distribution<std::mt19937::result_type> dist(0, horizonsIndexVector.size() - 1);
    chosenOne = dist(rng);

    // pick theta

    int horizonSize = horizons[chosenOne].size();
    float theta = msg->angle_min + (msg->angle_increment * horizons[chosenOne][horizonSize / 2]);

    return theta;
}

geometry_msgs::PoseStamped getMovment(float theta)
{
    tf::Quaternion rotation;
    rotation.setRPY(0, 0, theta);

    geometry_msgs::PoseStamped movment;
    movment.header.stamp = ros::Time(0);
    movment.header.frame_id = "base_link";
    movment.pose.position.x  = 1.0;
    tf::quaternionTFToMsg(rotation, movment.pose.orientation);

    return movment;
}

int main(int argc,  char** argv)
{
    ros::init(argc, argv, "explore_horizon");
    ros::NodeHandle nh;

    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);

    ROS_INFO("Waiting for action server to start");
    ac.waitForServer();
    ROS_INFO("Server online");

    // setting an upper bound to the movments for now
    int step = 0;
    float theta = 0;

    move_base_msgs::MoveBaseGoal goal;

    while (ros::ok() && step < 10)
    {
        theta = processScan();        

        // move the robot

        ROS_INFO("Sending goal, moving 1 meter in free space...");

        goal.target_pose = getMovment(theta);
        ac.sendGoal(goal);

        bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));
        if (finished_before_timeout)
        {
            actionlib::SimpleClientGoalState state = ac.getState();
            ROS_INFO("Action finished: %s",state.toString().c_str());
        }
        else
        {
            ROS_INFO("Action did not finish before the time out.");
        }

        step++;

        ros::spinOnce();
    }

    return 0;
}
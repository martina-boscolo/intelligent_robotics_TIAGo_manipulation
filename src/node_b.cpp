#include <ros/ros.h>
#include <ir2425_group_08/FindTags.h>

using NodeHandleShared = std::shared_ptr<ros::NodeHandle>;

void cameraCallback(const sensor_msgs::ImageConstPtr &msg)
{
    cv::imshow("camera", cv_bridge::toCvCopy(msg, msg->encoding)->image);
    cv::waitKey(1);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "node_b");
    auto nh = std::make_shared<ros::NodeHandle>();
    NodeHandleShared nh_ptr(nh);

    // // Debug section for camera
    // cv::namedWindow("camera");
    // ros::Subscriber sub = nh_ptr->subscribe("xtion/rgb/image_raw", 1, cameraCallback);

    ir2425_group_08::FindTags findTags(nh_ptr, "find_tags");

    ros::spin(); 
    return 0;
}
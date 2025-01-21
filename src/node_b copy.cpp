#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <tf/transform_listener.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <std_msgs/String.h>

#include "ir2425_group_08/RouteHandler.h"
#include "ir2425_group_08/waypoints_generator.h"
#include "ir2425_group_08/PlaceService.h"
#include "ir2425_group_08/PickAndPlaceAction.h"

#include "waypoints_generator.h"
#include <vector>
// tavolo pick: Setting goal: Frame:map, Position(8.907, -2.977, 0.000), Orientation(0.000, 0.000, 1.000, 0.009) = Angle: 3.124
  std::vector<geometry_msgs::Pose> waypoints = {};
  const float tiago_start_x = -6.580157;
const float tiago_start_y = 1.369999;

std::vector<geometry_msgs::Pose> target_poses = {
    // First pose
    []()
    {
        geometry_msgs::Pose pose;
        pose.position.x = 8.907;
        pose.position.y = -2.977;
        pose.position.z = 0.0;
        pose.orientation.x = 0.0;
        pose.orientation.y = 0.0;
        pose.orientation.z = 1.0;
        pose.orientation.w = 0.009;
        return pose;
    }()};

std::string NODE_A_SRV = "/place_goal";
std::vector<geometry_msgs::Point> PlaceServicePoints;
int numPlaceServicePoints;

cv::Mat current_image;
std::vector<geometry_msgs::Pose> foundTags;
std::vector<int> foundTagIds;
ir2425_group_08::RouteHandler *rh_ptr;
actionlib::SimpleActionClient<ir2425_group_08::PickAndPlaceAction> *ac_ptr;

// Color detection thresholds in HSV
cv::Scalar red_lower1(0, 50, 50), red_upper1(10, 255, 255);
cv::Scalar red_lower2(170, 50, 50), red_upper2(180, 255, 255);
cv::Scalar blue_lower(100, 150, 50), blue_upper(140, 255, 255);
cv::Scalar green_lower(40, 70, 50), green_upper(80, 255, 255);

// Callback for camera image
// void imageCallback(const sensor_msgs::ImageConstPtr &msg)
// {
//     try
//     {
//         current_image = cv_bridge::toCvShare(msg, "bgr8")->image;
//         // cv::imshow("current_image", current_image);
//     }
//     catch (cv_bridge::Exception &e)
//     {
//         ROS_ERROR("cv_bridge exception: %s", e.what());
//     }
// }

// Detect predominant color in an ROI
std::string detectColor(const cv::Mat &image)
{
    // Convert ROI to HSV
    //::Mat hsv_image;
    // cv::cvtColor(image, hsv_image, cv::COLOR_BGR2HSV);

    // Define HSV thresholds
    cv::imshow("image", image);
    cv::waitKey(1);

    cv::Scalar red_lower(200, 0, 0), red_upper(255, 15, 15);
    cv::Scalar blue_lower(90, 100, 100), blue_upper(130, 255, 255);
    cv::Scalar green_lower(35, 100, 100), green_upper(85, 255, 255);
    cv::Scalar orange_lower(240, 100, 0), orange_upper(255, 180, 10); // Table color

    // Create masks
    cv::Mat red_mask, blue_mask, green_mask, orange_mask;
    cv::inRange(image, red_lower, red_upper, red_mask);
    cv::inRange(image, blue_lower, blue_upper, blue_mask);
    cv::inRange(image, green_lower, green_upper, green_mask);
    cv::inRange(image, orange_lower, orange_upper, orange_mask);

    // Exclude orange (table pixels)
    red_mask.setTo(0, orange_mask);
    blue_mask.setTo(0, orange_mask);
    green_mask.setTo(0, orange_mask);

    // Count non-zero pixels for each mask
    int red_count = cv::countNonZero(red_mask);
    int blue_count = cv::countNonZero(blue_mask);
    int green_count = cv::countNonZero(green_mask);

    // cv::imshow("Red Mask", red_mask);
    // cv::imshow("Blue Mask", blue_mask);
    // cv::imshow("Green Mask", green_mask);
    // cv::waitKey(1);

    // Determine the dominant color
    if (red_count > blue_count && red_count > green_count)
    {
        return "red";
    }
    else if (blue_count > red_count && blue_count > green_count)
    {
        return "blue";
    }
    else if (green_count > red_count && green_count > blue_count)
    {
        return "green";
    }
    else
    {
        return "unknown";
    }
}

bool handlePlaceService(ir2425_group_08::PlaceService::Request &req, ir2425_group_08::PlaceService::Response &res)
{
    ROS_INFO_STREAM("Received " << req.num_goals << " goals for pick and place");

    PlaceServicePoints = req.target_points;
    numPlaceServicePoints = req.num_goals; // remove it, redoundant

    rh_ptr->followPoses(target_poses);

    res.success = true; // Assuming a response member named `success`.
    return true;        // Return true to indicate the service was processed successfully.
}

void tagsCallback(const apriltag_ros::AprilTagDetectionArrayConstPtr &msg)
{
    for (auto detection : msg->detections)
    {
        if (detection.id[0] != 10)
        {
            if (std::find(foundTagIds.begin(), foundTagIds.end(), detection.id[0]) == foundTagIds.end())
            {

                tf::TransformListener listener;
                tf::StampedTransform transform;
                std::string target_frame = "base_link";
                std::string source_frame = detection.pose.header.frame_id;

                while (!listener.canTransform(target_frame, source_frame, ros::Time(0)))
                    ros::Duration(0.5).sleep();

                // Transform available
                geometry_msgs::PoseStamped pos_in;
                geometry_msgs::PoseStamped pos_out;

                // if (msg->detections.at(i).pose.header.frame_id == "tag_10")
                // { detection.pose.pose.pose;
                // pos_in.header.frame_id = detection.pose.header.frame_id;
                // pos_in.pose.position.x = detection.pose.pose.pose.position.x;
                // pos_in.pose.position.y = detection.pose.pose.pose.position.y;
                // pos_in.pose.position.z = detection.pose.pose.pose.position.z;
                // pos_in.pose.orientation.x = detection.pose.pose.pose.orientation.x;
                // pos_in.pose.orientation.y = detection.pose.pose.pose.orientation.y;
                // pos_in.pose.orientation.z = detection.pose.pose.pose.orientation.z;
                // pos_in.pose.orientation.w = detection.pose.pose.pose.orientation.w;

                // piu' compatto ma da testare
                pos_in.header.frame_id = detection.pose.header.frame_id;
                pos_in.pose = detection.pose.pose.pose;

                listener.transformPose(target_frame, pos_in, pos_out);

                ROS_INFO_STREAM("Obj with ID: " << detection.id[0]);
                ROS_INFO_STREAM("Original pose\n"
                                << pos_in);
                ROS_INFO_STREAM("Transformed pose\n"
                                << pos_out);
                // tag10Processed = true;
                // }

                ROS_INFO_STREAM("Found apriltag " << detection.id[0] << " at pose " << pos_out.pose);
                foundTags.push_back(pos_out.pose);
                foundTagIds.push_back(detection.id[0]);

                // if (current_image.empty())
                // {
                //     ROS_WARN("No image data available for color detection.");
                //     return;
                // }
                // // cv::imshow("current_image", current_image);

                // const double tag_side_m = 0.026; // AprilTag side length in meters
                // const int margin_px = 50;        // Margin in pixels around the tag

                // // Extract ROI around AprilTag
                // int center_x = static_cast<int>(detection.pose.pose.pose.position.x);
                // int center_y = static_cast<int>(detection.pose.pose.pose.position.y);

                // int x_start = std::max(0, int(center_x - margin_px));
                // int y_start = std::max(0, int(center_y - margin_px));
                // int x_end = std::min(current_image.cols, int(center_x + margin_px));
                // int y_end = std::min(current_image.rows, int(center_y + margin_px));

                // // Extract ROI around AprilTag
                // cv::Rect roi(x_start, y_start, x_end - x_start, y_end - y_start);
                // cv::Mat roi_image = current_image(roi);
                // cv::imwrite("/home/local/boscmar80445/catkin_ws/roi_debug.png", roi_image);
                // cv::imwrite("/home/local/boscmar80445/catkin_ws/current.png", current_image);
                // // cv::rectangle(current_image, roi, cv::Scalar(255, 0, 0), 2);
                // // cv::imshow("ROI Debug", current_image);
                // // cv::waitKey(1);

                // if (roi_image.empty())
                // {
                //     ROS_WARN("ROI extraction failed. Skipping.");
                //     continue;
                // }

                // // Mask out the AprilTag itself (center of ROI)
                // int tag_x_start = std::max(0, int(center_x - tag_side_m / 2));
                // int tag_y_start = std::max(0, int(center_y - tag_side_m / 2));
                // int tag_x_end = std::min(current_image.cols, int(center_x + tag_side_m / 2));
                // int tag_y_end = std::min(current_image.rows, int(center_y + tag_side_m / 2));

                // cv::Mat mask = cv::Mat::ones(roi_image.size(), CV_8UC1) * 255; // Full white mask
                // cv::rectangle(mask, cv::Point(tag_x_start, tag_y_start), cv::Point(tag_x_end, tag_y_end), cv::Scalar(0), -1);

                // cv::Mat filtered_roi;
                // roi_image.copyTo(filtered_roi, mask);

                // Detect object color
                // std::string color = detectColor(roi_image);

                // ROS_INFO_STREAM("Detected color for AprilTag ID " << detection.id[0] << ": " << color);

                ir2425_group_08::PickAndPlaceGoal goal;
                goal.goal_pose = pos_out.pose;
                goal.id = detection.id[0];
                // goal.color = color; // Assuming goal includes a color field

                ac_ptr->sendGoal(goal);
            }
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "node_b");
    ros::NodeHandle nh;

    geometry_msgs::Point table_center;
    table_center.x = 1.332080 - tiago_start_x;
    table_center.y =-1.646500 - tiago_start_y;
    table_center.z = 0.0;

    double table_half_side = 0.55;
    bool include_edges = true;

    waypoints = generateSquareTableWaypoints(table_center, table_half_side, include_edges);

        ROS_INFO("Generated %zu waypoints:", waypoints.size());
    for (size_t i = 0; i < waypoints.size(); ++i) {
        const auto &wp = waypoints[i];
        ROS_INFO("Waypoint %zu: Position(%.2f, %.2f, %.2f)", i, wp.position.x, wp.position.y, wp.position.z);
    }

    image_transport::ImageTransport it(nh);
    //image_transport::Subscriber image_sub = it.subscribe("xtion/rgb/image_raw", 1, imageCallback);

    ros::ServiceServer server = nh.advertiseService(NODE_A_SRV, handlePlaceService);

    ROS_INFO("Node B is ready to handle place_goal service.");

    ir2425_group_08::RouteHandler rh;
    rh_ptr = &rh;

    actionlib::SimpleActionClient<ir2425_group_08::PickAndPlaceAction> ac("/pick_and_place", true);
    ac_ptr = &ac;

    ROS_INFO("Waiting for pick_and_place server to start (launch node C!)");
    ac.waitForServer();
    ROS_INFO("pick_and_place server started!");

    ros::Subscriber sub = nh.subscribe("/tag_detections", 10, tagsCallback);

    ros::spin();

    return 0;
}

// Is /clock being published?
// header:
//   seq: 0
//   stamp:
//     secs: 8474
//     nsecs: 133000000
//   frame_id: "base_footprint"
// point:
//   x: 0.06090688705444336
//   y: 0.13613003492355347
//   z: 0.2996702194213867
// ---
// header:
//   seq: 1
//   stamp:
//     secs: 8565
//     nsecs: 853000000
//   frame_id: "map"
// point:
//   x: 9.05079460144043
//   y: -3.8301162719726562
//   z: 0.00225067138671875
// ---
// header:
//   seq: 2
//   stamp:
//     secs: 8576
//     nsecs: 702000000
//   frame_id: "map"
// point:
//   x: 8.841548919677734
//   y: -3.015349864959717
//   z: 0.34714603424072266
// ---
// header:
//   seq: 3
//   stamp:
//     secs: 8584
//     nsecs: 192000000
//   frame_id: "map"
// point:
//   x: 8.997461318969727
//   y: -3.9414262771606445
//   z: 0.0023632049560546875
// ---
// header:
//   seq: 4
//   stamp:
//     secs: 8590
//     nsecs: 464000000
//   frame_id: "map"
// point:
//   x: 7.9129462242126465
//   y: -4.013263702392578
//   z: 0.0024499893188476562
// ---
// header:
//   seq: 5
//   stamp:
//     secs: 8595
//     nsecs: 393000000
//   frame_id: "map"
// point:
//   x: 6.969523906707764
//   y: -3.8619306087493896
//   z: 0.00231170654296875
// ---
// header:
//   seq: 6
//   stamp:
//     secs: 8600
//     nsecs: 162000000
//   frame_id: "map"
// point:
//   x: 6.975785255432129
//   y: -3.0438425540924072
//   z: 0.0014944076538085938
// ---
// header:
//   seq: 7
//   stamp:
//     secs: 8605
//     nsecs: 456000000
//   frame_id: "map"
// point:
//   x: 6.912846088409424
//   y: -2.439483404159546
//   z: 0.0008907318115234375
// ---

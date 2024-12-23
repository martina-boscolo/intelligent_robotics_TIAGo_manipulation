#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PointStamped.h"
#include "visualization_msgs/Marker.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf/transform_listener.h"
#include "move_base_msgs/MoveBaseAction.h"
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_datatypes.h>
#include <vector>
#include <algorithm>
#include <utility>

ros::Publisher centroid_pub;
ros::Publisher marker_pub;
tf::TransformListener* tf_listener_ptr;

std::vector<geometry_msgs::Point> visited_points;       
std::vector<geometry_msgs::Point> POI;              //this ideally should be a priority queue

const float SIMILARITY_THRESHOLD = 0.3;
const float SAFETY_THRESHOLD = 0.5;

geometry_msgs::Point transformPoint(const geometry_msgs::Point& point_in, const std::string& source_frame, const std::string& target_frame) {
    // Wrap the point in a PointStamped
    geometry_msgs::PointStamped point_stamped_in, point_stamped_out;
    point_stamped_in.header.frame_id = source_frame;
    point_stamped_in.header.stamp = ros::Time(0);
    point_stamped_in.point = point_in;

    // Transform the PointStamped
    try {
        tf_listener_ptr->waitForTransform(target_frame, source_frame, ros::Time(0), ros::Duration(1.0));
        tf_listener_ptr->transformPoint(target_frame, point_stamped_in, point_stamped_out);
    } catch (tf::TransformException& ex) {
        ROS_ERROR("Transform error: %s", ex.what());
        throw;
    }

    // Return the transformed point
    return point_stamped_out.point;
}

/**
 * Clustering function to find the small cubes containing Apriltags functioning described in the report
 */
std::vector<geometry_msgs::Point> clusterLidarData(const sensor_msgs::LaserScan::ConstPtr& scan) {
    std::vector<float> ranges = scan->ranges;
    std::vector<geometry_msgs::Point> centroids;

    float cluster_sum_x = 0.0, cluster_sum_y = 0.0;
    int cluster_count = 0;
    const float cluster_distance_threshold = 0.5; 
    const float angle_increment = scan->angle_increment;
    const float angle_min = scan->angle_min;

    const int min_points = 3;    
    const int max_points = 100; 
    const float min_cluster_width = 0.05; 
    const float max_cluster_width = 0.35;

    float cluster_min_x = std::numeric_limits<float>::max();
    float cluster_max_x = std::numeric_limits<float>::lowest();
    float cluster_min_y = std::numeric_limits<float>::max();
    float cluster_max_y = std::numeric_limits<float>::lowest();

    for (size_t i = 0; i < ranges.size(); ++i) {
        float range = ranges[i];
        float angle = angle_min + i * angle_increment;

        if (!std::isinf(range) && !std::isnan(range)) {
            float x = range * cos(angle);   // rho
            float y = range * sin(angle);   // theta

            // Check if this is not the first point basically
            if (cluster_count > 0) {
                // distance between current point and current cluster tmp centroid
                float dx = x - cluster_sum_x / cluster_count;
                float dy = y - cluster_sum_y / cluster_count;
                float distance = sqrt(dx * dx + dy * dy);

                if (distance > cluster_distance_threshold) {
                    // If distance is too large it means that this point does not belong to this cluster
                    // Compute centroid
                    float centroid_x = cluster_sum_x / cluster_count;
                    float centroid_y = cluster_sum_y / cluster_count;

                    // Compute cluster bounding box
                    float cluster_width_x = cluster_max_x - cluster_min_x;
                    float cluster_width_y = cluster_max_y - cluster_min_y;
                    float cluster_max_width = std::max(cluster_width_x, cluster_width_y);

                    // Check cluster size and bounding box
                    if (cluster_count >= min_points && cluster_count <= max_points &&
                        cluster_max_width >= min_cluster_width && cluster_max_width <= max_cluster_width) {
                        geometry_msgs::Point point;
                        point.x = centroid_x;
                        point.y = centroid_y;
                        centroids.push_back(point);
                    }

                    // Reset cluster
                    cluster_sum_x = 0.0;
                    cluster_sum_y = 0.0;
                    cluster_count = 0;
                    cluster_min_x = std::numeric_limits<float>::max();
                    cluster_max_x = std::numeric_limits<float>::lowest();
                    cluster_min_y = std::numeric_limits<float>::max();
                    cluster_max_y = std::numeric_limits<float>::lowest();
                }
            }

            // Add point to the cluster
            cluster_sum_x += x;
            cluster_sum_y += y;
            cluster_count++;
            cluster_min_x = std::min(cluster_min_x, x);
            cluster_max_x = std::max(cluster_max_x, x);
            cluster_min_y = std::min(cluster_min_y, y);
            cluster_max_y = std::max(cluster_max_y, y);
        }
    }

    // Computations for the final cluster
    if (cluster_count >= min_points && cluster_count <= max_points) {
        float centroid_x = cluster_sum_x / cluster_count;
        float centroid_y = cluster_sum_y / cluster_count;

        float cluster_width_x = cluster_max_x - cluster_min_x;
        float cluster_width_y = cluster_max_y - cluster_min_y;
        float cluster_max_width = std::max(cluster_width_x, cluster_width_y);

        if (cluster_max_width >= min_cluster_width && cluster_max_width <= max_cluster_width) {
            geometry_msgs::Point point;
            point.x = centroid_x;
            point.y = centroid_y;
            centroids.push_back(point);
        }
    }
    
    std::vector<geometry_msgs::Point> centroids_map;
    for(auto centroid : centroids){
        geometry_msgs::Point centroid_map;
        centroid_map = transformPoint(centroid, "base_laser_link", "map");
        centroids_map.push_back(centroid_map);
    }
    return centroids_map;
}

// Checks if centroid is similar to another point already in list
bool isCentroidSimilar(geometry_msgs::Point point, std::vector<geometry_msgs::Point> point_list) {
    for(auto target_point : point_list) {
        float dx = point.x - target_point.x;
        float dy = point.y - target_point.y;
        float distance = sqrt(dx * dx + dy * dy);
        if(distance <= SIMILARITY_THRESHOLD) {
            return true;
        }
    }
    return false;
}

// upadtes POI only if the new centroid is not near an already visited point or a point already in POI list
void updatePOI(std::vector<geometry_msgs::Point> centroids) {
    for(auto& centroid : centroids) {
        if(!isCentroidSimilar(centroid, visited_points) && !isCentroidSimilar(centroid, POI)) {
            POI.push_back(centroid);  
        }
    }
}

/**
 * This function works by trying to move three quarters of the way towards the first POI and then by rotate the robot to look at it. 
 * It would have been better to find a random feasible point near the first POI and use that as a goal
 */
void moveTowardsNextPOI(tf::TransformListener& tf_listener, actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>& ac) {
    geometry_msgs::Point baselink_point = transformPoint(POI[0], "map", "base_link");

    float distance = sqrt(baselink_point.x * baselink_point.x + baselink_point.y * baselink_point.y);

    // Calculate the yaw angle
    double yaw = atan2(baselink_point.y, baselink_point.x);

    tf::Quaternion tf_quat;
    tf_quat.setRPY(0, 0, yaw); // Roll and pitch are 0, only yaw is set

    geometry_msgs::Quaternion geom_quat;
    tf::quaternionTFToMsg(tf_quat, geom_quat);

    // Set the goal
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "base_link";
    goal.target_pose.header.stamp = ros::Time::now();

    if (distance / 4.0 > SAFETY_THRESHOLD) {
        // Move 3/4 of the way towards the POI
        goal.target_pose.pose.position.x = baselink_point.x * 3.0 / 4.0;
        goal.target_pose.pose.position.y = baselink_point.y * 3.0 / 4.0;
        goal.target_pose.pose.position.z = 0.0;
    } else {
        // Stay in place but orient towards the POI, not needed, just to clarify
        goal.target_pose.pose.position.x = 0.0;
        goal.target_pose.pose.position.y = 0.0;
        goal.target_pose.pose.position.z = 0.0;
    }

    goal.target_pose.pose.orientation = geom_quat;

    ROS_INFO("MOVING TO NEXT POINT, KILL ALL HUMANS!");
    ac.sendGoal(goal);

}


// LaserScan callback
void laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
    ROS_INFO("Processing LaserScan data...");
    auto centroids = clusterLidarData(scan);
    updatePOI(centroids);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "nodeB");
    ros::NodeHandle nh;
    tf::TransformListener tf_listener;
    tf_listener_ptr = &tf_listener;
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);
    ROS_INFO("Waiting for move_base server...");

    ac.waitForServer();

    ROS_INFO("move_base server: OK");


    ROS_INFO("Node B is running...");

    centroid_pub = nh.advertise<geometry_msgs::PointStamped>("centroids", 10);
    marker_pub = nh.advertise<visualization_msgs::Marker>("centroid_markers", 10);

    const sensor_msgs::LaserScan::ConstPtr& scan_msg = ros::topic::waitForMessage<sensor_msgs::LaserScan>("/scan", nh);

    if (scan_msg) {
        laserCallback(scan_msg);
    } else {
        ROS_WARN("No message received on /scan.");
    }

    //while all the wanted apriltags are yet to be found
    while(!POI.empty()) {
        moveTowardsNextPOI(tf_listener, ac);
        ac.waitForResult(ros::Duration(10.0)); //debug
        //while(goal not reached && goal == POI[0])
            //listen to Apriltag once and remove(POI[0]) if Apriltag found
            //rescan with lidar
        //stop this goal and go to the next
        POI.erase(POI.begin());
    }

    ros::spin();
    return 0;
}

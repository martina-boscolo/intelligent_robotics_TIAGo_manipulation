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
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/cost_values.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetPlan.h>
#include <vector>
#include <algorithm>
#include <utility>

ros::Publisher centroid_pub;
ros::Publisher marker_pub;
tf::TransformListener* tf_listener_ptr;
//costmap_2d::Costmap2D* global_costmap = nullptr;
nav_msgs::OccupancyGrid global_costmap;

std::vector<geometry_msgs::Point> visited_points;       
std::vector<geometry_msgs::Point> POI;              //coda dovrebbe avere priorità

const float SIMILARITY_THRESHOLD = 0.5; //DA FINE TUNINGARE
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

// Function to find clusters and their centroids
std::vector<geometry_msgs::Point> clusterLidarData(const sensor_msgs::LaserScan::ConstPtr& scan) {
    std::vector<float> ranges = scan->ranges;
    std::vector<geometry_msgs::Point> centroids;

    float cluster_sum_x = 0.0, cluster_sum_y = 0.0;
    int cluster_count = 0;
    const float cluster_distance_threshold = 0.5; // Increased flexibility for spread
    const float angle_increment = scan->angle_increment;
    const float angle_min = scan->angle_min;

    const int min_points = 3;    // Lowered minimum to detect smaller clusters
    const int max_points = 100; // Increased maximum to allow larger clusters
    const float min_cluster_width = 0.05; // Allow smaller clusters
    const float max_cluster_width = 0.35; // Allow larger clusters

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


void publishCentroids(const std::vector<geometry_msgs::Point>& centroids) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "centroids";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 1.0; // Fully opaque
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;

    for (const auto& centroid : centroids) {
        // Publish individual centroid
        geometry_msgs::PointStamped centroid_msg;
        centroid_msg.header.frame_id = "map";
        centroid_msg.header.stamp = ros::Time::now();
        centroid_msg.point = centroid;

        centroid_pub.publish(centroid_msg);

        // Add to marker for RViz visualization
        marker.points.push_back(centroid);
    }

    // Publish the marker
    marker_pub.publish(marker);
}


////////////////////////////////////////////////////////////////////
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

void updatePOI(std::vector<geometry_msgs::Point> centroids) {
    for(auto& centroid : centroids) {
        if(!isCentroidSimilar(centroid, visited_points) && !isCentroidSimilar(centroid, POI)) {
            POI.push_back(centroid);   // con la coda sarebbe push_back basato sulla priorità (distanza dal robot attuale)
        }
    }
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//CRAZY TESTS
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

// Callback to update the costmap
void costmapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    global_costmap = *msg;
}

geometry_msgs::PoseStamped getRobotPose() {
    geometry_msgs::PoseStamped pose;
    tf::StampedTransform transform;

    try {
        // Wait for the transform and get the robot's pose in the "map" frame
        tf_listener_ptr->waitForTransform("map", "base_link", ros::Time(0), ros::Duration(1.0));
        tf_listener_ptr->lookupTransform("map", "base_link", ros::Time(0), transform);

        pose.header.frame_id = "map";
        pose.header.stamp = ros::Time::now();
        pose.pose.position.x = transform.getOrigin().x();
        pose.pose.position.y = transform.getOrigin().y();
        pose.pose.position.z = transform.getOrigin().z();
        pose.pose.orientation.x = transform.getRotation().x();
        pose.pose.orientation.y = transform.getRotation().y();
        pose.pose.orientation.z = transform.getRotation().z();
        pose.pose.orientation.w = transform.getRotation().w();
    } catch (tf::TransformException& ex) {
        ROS_ERROR("Transform error: %s", ex.what());
    }

    return pose;
}

// bool isGoalFeasible(const move_base_msgs::MoveBaseGoal& goal) {
//     if (!global_costmap) {
//         ROS_ERROR("Global costmap is not initialized.");
//         return false;
//     }

//     unsigned int mx, my;

//     // Convert world coordinates to map coordinates
//     if (!global_costmap->worldToMap(goal.target_pose.pose.position.x, goal.target_pose.pose.position.y, mx, my)) {
//         ROS_WARN("Goal is out of bounds in the costmap.");
//         return false;
//     }

//     // Check the cost at the goal position
//     unsigned char cost = global_costmap->getCost(mx, my);

//     if (cost == costmap_2d::LETHAL_OBSTACLE) {
//         ROS_WARN("Goal is in a lethal obstacle.");
//         return false;
//     } else if (cost == costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
//         ROS_WARN("Goal is too close to an obstacle.");
//         return false;
//     } else if (cost == costmap_2d::NO_INFORMATION) {
//         ROS_WARN("Goal is in an unknown area.");
//         return false;
//     }

//     ROS_INFO("Goal is in a free space.");
//     return true;
// }

bool isGoalFeasible(const move_base_msgs::MoveBaseGoal& goal, const geometry_msgs::PoseStamped& start_pose) {
    ros::NodeHandle nh;
    ros::ServiceClient client = nh.serviceClient<nav_msgs::GetPlan>("/move_base/make_plan");

    // Create the service request and response
    nav_msgs::GetPlan srv;
    srv.request.start = start_pose; // The robot's current pose
    srv.request.goal = goal.target_pose; // The goal pose
    srv.request.tolerance = 0.5; // Tolerance in meters for the path

    if (client.call(srv)) {
        if (!srv.response.plan.poses.empty()) {
            ROS_INFO("A feasible path exists.");
            return true;
        } else {
            ROS_WARN("No feasible path found.");
            return false;
        }
    } else {
        ROS_ERROR("Failed to call service /move_base/make_plan.");
        return false;
    }
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//END OF CRAZY TESTS
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void moveTowardsNextPOI(tf::TransformListener& tf_listener, actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>& ac) {

    geometry_msgs::Point baselink_point = transformPoint(POI[0], "map", "base_link");

    float distance = sqrt(baselink_point.x * baselink_point.x + baselink_point.y * baselink_point.y);

    // Calculate the yaw angle
    double yaw = atan2(baselink_point.y, baselink_point.x);

    // Convert yaw to a quaternion
    tf::Quaternion tf_quat;
    tf_quat.setRPY(0, 0, yaw); // Roll and pitch are 0, only yaw is set

    // Convert tf::Quaternion to geometry_msgs::Quaternion
    geometry_msgs::Quaternion geom_quat;
    tf::quaternionTFToMsg(tf_quat, geom_quat);

    // Set the goal
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "base_link";
    goal.target_pose.header.stamp = ros::Time::now();

    if (distance / 4.0 > SAFETY_THRESHOLD) {
        // Move 3/4 of the way towards the POI
        //////////////////////////////
        //forse dovremmo fare muovere tiago fino a distance-SAFETY THRESHOLD, da vedere
        //////////////////////////////
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

    if(isGoalFeasible(goal, getRobotPose())) {
        ROS_INFO("MOVING TO NEXT POINT");
        ac.sendGoal(goal);
    }
    else
        ROS_INFO("SKIPPING THIS GOAL");

}
////////////////////////////////////////////////////////////////////

// LaserScan callback
void laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
    ROS_INFO("Processing LaserScan data...");
    auto centroids = clusterLidarData(scan);
    publishCentroids(centroids);
    updatePOI(centroids);
}

int main(int argc, char** argv) {


    ros::init(argc, argv, "nodeB");
    ros::NodeHandle nh;

    // // Create the Costmap2DROS object
    // tf2_ros::Buffer tfBuffer(ros::Duration(10));
    // tf2_ros::TransformListener tfListener(tfBuffer);
    // costmap_2d::Costmap2DROS global_costmap_ros("global_costmap", tfBuffer);

    // // Wait for the costmap to initialize
    // ros::Duration(2.0).sleep(); // Allow some time for initialization

    // // Retrieve the underlying Costmap2D instance
    // global_costmap = global_costmap_ros.getCostmap();

    // if (!global_costmap) {
    //     ROS_ERROR("Failed to initialize the global costmap.");
    //     return -1;
    // }

    // ROS_INFO("Global costmap initialized successfully.");

    // Subscribe to the costmap topic
    ros::Subscriber costmap_sub = nh.subscribe("/move_base/global_costmap/costmap", 1, costmapCallback);

    // Wait for costmap to be populated
    ROS_INFO("Waiting for costmap...");
    while (global_costmap.data.empty() && ros::ok()) {
        ros::spinOnce();
        ros::Duration(0.1).sleep();
    }
    ROS_INFO("Costmap received!");


    tf::TransformListener tf_listener;
    tf_listener_ptr = &tf_listener;
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);
    ROS_INFO("Waiting for move_base server...");

    ac.waitForServer();

    ROS_INFO("move_base server: OK");


    ROS_INFO("Node B is running...");

    centroid_pub = nh.advertise<geometry_msgs::PointStamped>("centroids", 10);
    marker_pub = nh.advertise<visualization_msgs::Marker>("centroid_markers", 10);

    //ros::Subscriber laser_sub = nh.subscribe("/scan", 10, laserCallback);
    const sensor_msgs::LaserScan::ConstPtr& scan_msg = ros::topic::waitForMessage<sensor_msgs::LaserScan>("/scan", nh);

    if (scan_msg) {
        laserCallback(scan_msg); // Pass the received message to your callback
    } else {
        ROS_WARN("No message received on /scan.");
    }

    //while che cerca gli apriltag che ci interessano e spostamenti intelligenti
    while(!POI.empty()) {
        //guarda POI[0] e punta in quella direzione
        moveTowardsNextPOI(tf_listener, ac);
        ac.waitForResult(ros::Duration(30.0)); //debug
        //while(goal non raggiunto && goal == POI[0])
            //ascolta apriltag una volta (poppa se trova)
            //ascolta lidar una volta
        //interrompi il goal di movimento se l'attuale goal è ancora attivo
        visited_points.push_back(POI[0]);
        POI.erase(POI.begin());
        if (scan_msg) {
        laserCallback(scan_msg); // Pass the received message to your callback
        } else {
            ROS_WARN("No message received on /scan.");
        }
    }

    ros::spin();
    return 0;
}



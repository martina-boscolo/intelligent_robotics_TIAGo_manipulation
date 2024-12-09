#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PointStamped.h"
#include "visualization_msgs/Marker.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <vector>
#include <utility>

ros::Publisher centroid_pub;
ros::Publisher marker_pub;

// Function to find clusters and their centroids
std::vector<std::pair<float, float>> clusterLidarData(const sensor_msgs::LaserScan::ConstPtr& scan) {
    std::vector<float> ranges = scan->ranges;
    std::vector<std::pair<float, float>> centroids;

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
                        // emplace_back is more efficent than push_back
                        centroids.emplace_back(centroid_x, centroid_y);
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
            centroids.emplace_back(centroid_x, centroid_y);
        }
    }

    return centroids;
}


// Function to transform and publish centroids
void publishTransformedCentroids(const std::vector<std::pair<float, float>>& centroids, const std_msgs::Header& header) {
    static tf2_ros::Buffer tfBuffer;
    static tf2_ros::TransformListener tfListener(tfBuffer);

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
        geometry_msgs::PointStamped lidar_point, map_point;
        lidar_point.header = header;
        lidar_point.point.x = centroid.first;
        lidar_point.point.y = centroid.second;
        lidar_point.point.z = 0.0;

        try {
            geometry_msgs::TransformStamped transformStamped = tfBuffer.lookupTransform("map", header.frame_id, ros::Time(0));
            tf2::doTransform(lidar_point, map_point, transformStamped);

            // Publish individual centroid
            centroid_pub.publish(map_point);

            // Add to marker for RViz
            geometry_msgs::Point point;
            point.x = map_point.point.x;
            point.y = map_point.point.y;
            point.z = map_point.point.z;
            marker.points.push_back(point);
        } catch (tf2::TransformException& ex) {
            ROS_WARN("Transform failed: %s", ex.what());
        }
    }

    // Publish the marker for RViz
    marker_pub.publish(marker);
}

// LaserScan callback
void laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
    ROS_INFO("Processing LaserScan data...");
    auto centroids = clusterLidarData(scan);
    publishTransformedCentroids(centroids, scan->header);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "nodeB");
    ros::NodeHandle nh;

    ROS_INFO("Node B is running...");

    centroid_pub = nh.advertise<geometry_msgs::PointStamped>("centroids", 10);
    marker_pub = nh.advertise<visualization_msgs::Marker>("centroid_markers", 10);

    ros::Subscriber laser_sub = nh.subscribe("/scan", 10, laserCallback);

    ros::spin();
    return 0;
}

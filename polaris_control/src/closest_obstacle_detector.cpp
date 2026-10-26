#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <Eigen/Dense>
#include <vector>
#include <cmath>

/*
Universidade Federal de Minas Gerais (UFMG) - 2025
Laboratório CORO
Instituto Tecnologico Vale (ITV)

Contact:
Enio Moreira Silva Júnior, <eniomoreirasilva@gmail.com>

Node: Closest Obstacle Detector
Description: Simplified detector for 2D LaserScan data
             Converts scan to points, clusters with DBSCAN, finds closest obstacle
Input: sensor_msgs/msg/LaserScan
Output: geometry_msgs/msg/Point (closest obstacle)
*/

using namespace std::chrono_literals;

class LaserScanObstacleDetector : public rclcpp::Node {
public:
    LaserScanObstacleDetector() : Node("laserscan_obstacle_detector") {
        loadParameters();
        setupROS();
        
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        
        RCLCPP_INFO(get_logger(), "LaserScan Obstacle Detector Node Started");
        RCLCPP_INFO(get_logger(), "  DBSCAN eps: %.3f m", eps_);
        RCLCPP_INFO(get_logger(), "  DBSCAN min_points: %d", min_points_);
        RCLCPP_INFO(get_logger(), "  Max range: %.2f m", max_range_);
    }

private:
    // Parameters
    std::string input_topic_;
    std::string output_topic_;
    std::string visualization_topic_;
    std::string laser_frame_;
    std::string world_frame_;
    double eps_;                    // DBSCAN neighborhood radius
    int min_points_;                // DBSCAN minimum points per cluster
    double max_range_;              // Maximum laser range to consider
    double min_range_;              // Minimum laser range to consider
    bool publish_markers_;
    
    // ROS interfaces
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr obstacle_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    void loadParameters() {
        input_topic_ = declare_parameter<std::string>("input_topic", "scan");
        output_topic_ = declare_parameter<std::string>("output_topic", "closest_obstacle");
        visualization_topic_ = declare_parameter<std::string>("visualization_topic", "obstacle_clusters");
        laser_frame_ = declare_parameter<std::string>("laser_frame", "base_scan");
        world_frame_ = declare_parameter<std::string>("world_frame", "odom");
        eps_ = declare_parameter<double>("eps", 0.3);
        min_points_ = declare_parameter<int>("min_points", 3);
        max_range_ = declare_parameter<double>("max_range", 10.0);
        min_range_ = declare_parameter<double>("min_range", 0.1);
        publish_markers_ = declare_parameter<bool>("publish_markers", true);
    }

    void setupROS() {
        scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            input_topic_, rclcpp::SensorDataQoS(), 
            std::bind(&LaserScanObstacleDetector::scanCallback, this, std::placeholders::_1));
        
        obstacle_pub_ = create_publisher<geometry_msgs::msg::Point>(output_topic_, 10);
        marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(visualization_topic_, 10);
    }

    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        // Convert LaserScan to 2D points
        std::vector<Eigen::Vector2d> points;
        laserScanToPoints(msg, points);
        
        if (points.empty()) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "No valid points in laser scan");
            return;
        }

        // Perform DBSCAN clustering
        std::vector<std::vector<Eigen::Vector2d>> clusters = dbscanClustering(points);
        
        if (clusters.empty()) {
            RCLCPP_DEBUG(get_logger(), "No clusters found");
            return;
        }

        // Find closest cluster centroid
        int closest_cluster_idx = -1;
        double min_distance = std::numeric_limits<double>::max();
        Eigen::Vector2d closest_point;

        for (size_t i = 0; i < clusters.size(); ++i) {
            // Compute cluster centroid
            Eigen::Vector2d centroid(0, 0);
            for (const auto& p : clusters[i]) {
                centroid += p;
            }
            centroid /= clusters[i].size();

            // Distance from origin (robot position in laser frame)
            double dist = centroid.norm();
            
            if (dist < min_distance) {
                min_distance = dist;
                closest_cluster_idx = i;
                closest_point = centroid;
            }
        }

        // Transform to world frame and publish
        if (closest_cluster_idx >= 0) {
            geometry_msgs::msg::Point obstacle_world;
            if (transformToWorldFrame(closest_point, msg->header, obstacle_world)) {
                obstacle_pub_->publish(obstacle_world);

                RCLCPP_DEBUG(get_logger(), "Closest obstacle at (%.2f, %.2f) local, distance: %.2f m",
                    closest_point.x(), closest_point.y(), min_distance);
            }
        }

        // Publish visualization markers
        if (publish_markers_) {
            publishClusterMarkers(clusters, closest_cluster_idx, msg->header);
        }
    }

    void laserScanToPoints(const sensor_msgs::msg::LaserScan::SharedPtr& scan,
                          std::vector<Eigen::Vector2d>& points) {
        float angle = scan->angle_min;
        
        for (size_t i = 0; i < scan->ranges.size(); ++i) {
            float range = scan->ranges[i];
            
            // Filter invalid ranges
            if (std::isfinite(range) && 
                range >= min_range_ && 
                range <= max_range_ &&
                range >= scan->range_min && 
                range <= scan->range_max) {
                
                // Convert polar to cartesian
                double x = range * std::cos(angle);
                double y = range * std::sin(angle);
                points.emplace_back(x, y);
            }
            
            angle += scan->angle_increment;
        }
    }

    bool transformToWorldFrame(const Eigen::Vector2d& point_local,
                              const std_msgs::msg::Header& header,
                              geometry_msgs::msg::Point& point_world) {
        try {
            // Create point in laser frame
            geometry_msgs::msg::PointStamped point_laser;
            point_laser.header = header;
            point_laser.point.x = point_local.x();
            point_laser.point.y = point_local.y();
            point_laser.point.z = 0.0;

            // Transform to world frame
            auto point_transformed = tf_buffer_->transform(
                point_laser, world_frame_, tf2::durationFromSec(0.1));
            
            point_world = point_transformed.point;
            return true;
            
        } catch (tf2::TransformException& ex) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                "Could not transform from %s to %s: %s", 
                header.frame_id.c_str(), world_frame_.c_str(), ex.what());
            return false;
        }
    }

    std::vector<std::vector<Eigen::Vector2d>> dbscanClustering(
        const std::vector<Eigen::Vector2d>& points) {
        
        std::vector<int> labels(points.size(), -1);  // -1 = unclassified
        int cluster_id = 0;

        for (size_t i = 0; i < points.size(); ++i) {
            if (labels[i] != -1) continue;

            std::vector<size_t> neighbors = regionQuery(points, i);

            if (static_cast<int>(neighbors.size()) < min_points_) {
                labels[i] = -2;  // Mark as noise
                continue;
            }

            labels[i] = cluster_id;
            expandCluster(points, labels, neighbors, cluster_id);
            cluster_id++;
        }

        // Group points by cluster
        std::vector<std::vector<Eigen::Vector2d>> clusters(cluster_id);
        for (size_t i = 0; i < points.size(); ++i) {
            if (labels[i] >= 0) {
                clusters[labels[i]].push_back(points[i]);
            }
        }

        RCLCPP_DEBUG(get_logger(), "DBSCAN found %d clusters from %zu points", 
            cluster_id, points.size());
        return clusters;
    }

    std::vector<size_t> regionQuery(const std::vector<Eigen::Vector2d>& points, size_t idx) {
        std::vector<size_t> neighbors;
        const auto& point = points[idx];

        for (size_t i = 0; i < points.size(); ++i) {
            if ((points[i] - point).norm() <= eps_) {
                neighbors.push_back(i);
            }
        }

        return neighbors;
    }

    void expandCluster(const std::vector<Eigen::Vector2d>& points,
                      std::vector<int>& labels,
                      std::vector<size_t>& neighbors,
                      int cluster_id) {
        
        size_t i = 0;
        while (i < neighbors.size()) {
            size_t neighbor_idx = neighbors[i];

            if (labels[neighbor_idx] == -2) {
                labels[neighbor_idx] = cluster_id;
            }

            if (labels[neighbor_idx] != -1) {
                i++;
                continue;
            }

            labels[neighbor_idx] = cluster_id;

            std::vector<size_t> neighbor_neighbors = regionQuery(points, neighbor_idx);

            if (static_cast<int>(neighbor_neighbors.size()) >= min_points_) {
                neighbors.insert(neighbors.end(), 
                    neighbor_neighbors.begin(), neighbor_neighbors.end());
            }

            i++;
        }
    }

    void publishClusterMarkers(const std::vector<std::vector<Eigen::Vector2d>>& clusters,
                              int closest_cluster_idx,
                              const std_msgs::msg::Header& header) {
        visualization_msgs::msg::MarkerArray marker_array;

        // Delete old markers
        visualization_msgs::msg::Marker delete_marker;
        delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
        marker_array.markers.push_back(delete_marker);

        // Create markers for each cluster
        for (size_t i = 0; i < clusters.size(); ++i) {
            visualization_msgs::msg::Marker marker;
            marker.header = header;
            marker.ns = "laser_clusters";
            marker.id = i;
            marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
            marker.action = visualization_msgs::msg::Marker::ADD;

            for (const auto& p : clusters[i]) {
                geometry_msgs::msg::Point pt;
                pt.x = p.x();
                pt.y = p.y();
                pt.z = 0.0;
                marker.points.push_back(pt);
            }

            marker.scale.x = 0.1;
            marker.scale.y = 0.1;
            marker.scale.z = 0.1;

            // Highlight closest cluster
            if (static_cast<int>(i) == closest_cluster_idx) {
                marker.color.r = 1.0;
                marker.color.g = 0.0;
                marker.color.b = 0.0;
                marker.color.a = 1.0;
            } else {
                marker.color.r = 0.0;
                marker.color.g = 1.0;
                marker.color.b = 1.0;
                marker.color.a = 0.6;
            }

            marker_array.markers.push_back(marker);
        }

        marker_pub_->publish(marker_array);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LaserScanObstacleDetector>());
    rclcpp::shutdown();
    return 0;
}

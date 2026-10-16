#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rate.hpp>
#include <fstream>
#include <vector>
#include <string>
#include <memory>
#include <tuple>

/*
Universidade Federal de Minas Gerais (UFMG) - 2025
Laboratório CORO
Instituto Tecnologico Vale (ITV)
Contact:
Thales Andrade Soares, <thalesasoares02@gmail.com>
*/

using namespace std::chrono_literals;

// ----------  ----------  ----------  ----------  ----------
// Class to read a path from a file and publish it as a ROS Path message
class PathFromFile : public rclcpp::Node {
public:
    PathFromFile() : Node("planner"), rate_(0.5) {
        // Declare parameters (default)
        this->declare_parameter<std::string>("pkg_path", "/home/baiao/ros2_ws/src/polaris_planning");
        this->declare_parameter<int>("path_number", 1);
        this->declare_parameter<double>("marker_scale", 0.03);
        this->declare_parameter<std::string>("frame_id", "map");
        this->declare_parameter<std::string>("path_topic_name", "espeleo/path");
        this->declare_parameter<std::string>("visualization_topic_name", "visual_path");

        // Get parameters
        pkg_path_ = this->get_parameter("pkg_path").as_string();
        curve_number_ = this->get_parameter("path_number").as_int();
        marker_scale_ = this->get_parameter("marker_scale").as_double();
        frame_id_ = this->get_parameter("frame_id").as_string();
        path_topic_name = this->get_parameter("path_topic_name").as_string();
        visualization_topic_name = this->get_parameter("visualization_topic_name").as_string();

        // Check if curve number is valid
        if (curve_number_ <= 0 || curve_number_ > 4) {
            RCLCPP_ERROR(this->get_logger(), "Invalid curve_number! Must be between 1 and 4.");
            return;
        }

        // Initialize publishers
        pub_path_ = this->create_publisher<nav_msgs::msg::Path>(path_topic_name, 1);
        pub_rviz_curve_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            visualization_topic_name, 1);

        rate_.sleep();
        // Load and publish path
        loadAndPublishPath();
    }

// ----------  ----------  ----------  ----------  ----------
private:
    std::string pkg_path_;
    int curve_number_;
    double marker_scale_;
    std::string frame_id_;
    std::string path_topic_name;
    std::string visualization_topic_name;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_rviz_curve_;

    rclcpp::Rate rate_;

    // ----------  ----------  ----------  ----------  ----------
    // Loads the path from a file and publishes it
    void loadAndPublishPath()
    {
        auto [path, number_of_samples] = readPath(curve_number_);
        if (number_of_samples == 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load path.");
            return;
        }

        // Publish path
        auto path_msg = createPathMsg(path);
        pub_path_->publish(path_msg);

        // Visualize in RViz
        sendCurveRviz(path);

        RCLCPP_INFO(this->get_logger(), "----------------------------");
        RCLCPP_INFO(this->get_logger(), "Curve read and published");
        RCLCPP_INFO(this->get_logger(), "Curve type: %d", curve_number_);
        RCLCPP_INFO(this->get_logger(), "Sampled samples: %zu", number_of_samples);
        RCLCPP_INFO(this->get_logger(), "----------------------------");
    }

    // ----------  ----------  ----------  ----------  ----------
    // Reads the path from a file and returns it as a tuple of vectors
    std::tuple<std::vector<std::vector<double>>, size_t> readPath(int id)
    {
        std::string file_path = pkg_path_ + "/path_txt/path_" + std::to_string(id) + ".txt";
        std::vector<std::vector<double>> path(3);
        size_t n = 0;

        std::ifstream file(file_path);
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open path file: %s", file_path.c_str());
            return {path, 0};
        }

        std::string line;
        size_t i = 1;
        while (std::getline(file, line)) {
            std::istringstream iss(line);
            if (i == 1) {
                iss >> n; 
            } else {
                double x, y, z;
                iss >> x >> y >> z;
                path[0].push_back(x);
                path[1].push_back(y);
                path[2].push_back(z);
            }
            i++;
        }

        file.close();
        return {path, n};
    }

    // ----------  ----------  ----------  ----------  ----------
    // Creates a Path message from the path data
    nav_msgs::msg::Path createPathMsg(const std::vector<std::vector<double>>& path)
    {
        nav_msgs::msg::Path path_msg;
        path_msg.header.stamp = this->now();
        path_msg.header.frame_id = frame_id_;

        for (size_t k = 0; k < path[0].size(); ++k) {
            geometry_msgs::msg::PoseStamped pose_msg;
            pose_msg.header.stamp = this->now();
            pose_msg.header.frame_id = frame_id_;

            pose_msg.pose.position.x = path[0][k];
            pose_msg.pose.position.y = path[1][k];
            pose_msg.pose.position.z = path[2][k];

            pose_msg.pose.orientation.x = 0.0;
            pose_msg.pose.orientation.y = 0.0;
            pose_msg.pose.orientation.z = 0.0;
            pose_msg.pose.orientation.w = 1.0;

            path_msg.poses.push_back(pose_msg);
        }

        return path_msg;
    }

    // ----------  ----------  ----------  ----------  ----------
    // Publishes the path as a series of markers in RViz
    void sendCurveRviz(const std::vector<std::vector<double>>& path)
    {
        auto points_marker = std::make_shared<visualization_msgs::msg::MarkerArray>();

        for (size_t k = 0; k < path[0].size(); ++k) {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = frame_id_;
            marker.header.stamp = this->now();
            marker.id = k;
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            
            marker.scale.x = marker_scale_;
            marker.scale.y = marker_scale_;
            marker.scale.z = marker_scale_;
            
            marker.color.a = 1.0;
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            
            marker.pose.orientation.w = 1.0;
            marker.pose.position.x = path[0][k];
            marker.pose.position.y = path[1][k];
            marker.pose.position.z = path[2][k];
            
            points_marker->markers.push_back(marker);
        }

        pub_rviz_curve_->publish(*points_marker);
    }
};





// ----------  ----------  ----------  ----------  ----------
// Main function
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PathFromFile>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rate.hpp>
#include "std_srvs/srv/trigger.hpp" 
#include <cmath>
#include <vector>
#include <memory>
#include <array>
#include <unordered_map>
#include <functional>

/*
Universidade Federal de Minas Gerais (UFMG) - 2025
Laboratório CORO
Instituto Tecnologico Vale (ITV)
Contact:
Thales Andrade Soares, <thalesasoares02@gmail.com>
*/

using namespace std::chrono_literals;

namespace PathGenerator {
// Ellipse
std::array<std::vector<double>, 2> generateEllipse(int N, double a, double b, double phi, double cx, double cy) {
    const double dp = 2 * M_PI / N;
    std::array<std::vector<double>, 2> path;
    path[0].reserve(N);
    path[1].reserve(N);

    for(int k = 0; k < N; ++k) {
        const double p = k * dp;
        const double x_ref0 = a * cos(p);
        const double y_ref0 = b * sin(p);
        path[0].push_back(cos(phi) * x_ref0 - sin(phi) * y_ref0 + cx);
        path[1].push_back(sin(phi) * x_ref0 + cos(phi) * y_ref0 + cy);
    }
    return path;
}

    // ---------- ---------- ---------- ---------- ----------
    // Figure 8
    std::array<std::vector<double>, 2> generateFigure8(int N, double a, double b, double phi, double cx, double cy) {
        const double dp = 2 * M_PI / N;
        std::array<std::vector<double>, 2> path;
        path[0].reserve(N);
        path[1].reserve(N);


        for(int k = 0; k < N; ++k) {

        const double p = k * dp;
        const double x_ref0 = a * sin(p);
        const double y_ref0 = b * sin(2.0 * p);
        path[0].push_back(cos(phi) * x_ref0 - sin(phi) * y_ref0 + cx);
        path[1].push_back(sin(phi) * x_ref0 + cos(phi) * y_ref0 + cy);

        }

        return path;
    }


    // ---------- ---------- ---------- ---------- ----------

    // Rectangle

    std::array<std::vector<double>, 2> generateRectangle(int N, double a, double b, double phi, double cx, double cy) {

        const double dp = 2 * M_PI / N;
        std::array<std::vector<double>, 2> path;
        path[0].reserve(N);
        path[1].reserve(N);


        for(int k = 0; k < N; ++k) {

            const double p = k * dp;
            const double r = pow(pow(cos(p), 4) + pow(sin(p), 4), -0.25);
            const double x_ref0 = a * r * cos(p);
            const double y_ref0 = b * r * sin(p);

            path[0].push_back(cos(phi) * x_ref0 - sin(phi) * y_ref0 + cx);
            path[1].push_back(sin(phi) * x_ref0 + cos(phi) * y_ref0 + cy);

        }

        return path;

    }


    // ---------- ---------- ---------- ---------- ----------

    // Sinusoidal

    std::array<std::vector<double>, 2> generateSinusoidal(int N, double a, double b, double phi, double cx, double cy) {

        const double dp = 2 * M_PI / N;
        std::array<std::vector<double>, 2> path;
        path[0].reserve(N);
        path[1].reserve(N);


        for(int k = 0; k < N; ++k) {

            const double p = k * dp;
            const double x_ref0 = p - M_PI;
            const double y_ref0 = sin(x_ref0);

            path[0].push_back(cos(phi) * x_ref0 - sin(phi) * y_ref0 + cx);
            path[1].push_back(sin(phi) * x_ref0 + cos(phi) * y_ref0 + cy);

        }

        return path;

    }


    // ---------- ---------- ---------- ---------- ----------

    // Lemniscate

    std::array<std::vector<double>, 2> generateLemniscate(int N, double a, double b, double phi, double cx, double cy) {

        const double dp = 2 * M_PI / N;
        std::array<std::vector<double>, 2> path;
        path[0].reserve(N);
        path[1].reserve(N);


        for(int k = 0; k < N; ++k) {

            const double p = k * dp;
            const double denom = 1 + pow(sin(p), 2);
            const double x_ref0 = a * cos(p) / denom;
            const double y_ref0 = b * 2.0 * sqrt(2.0) * cos(p) * sin(p) / denom;

            path[0].push_back(cos(phi) * x_ref0 - sin(phi) * y_ref0 + cx);
            path[1].push_back(sin(phi) * x_ref0 + cos(phi) * y_ref0 + cy);

        }

        return path;

    }

    }


// ----------  ----------  ----------  ----------  ----------
// Class for generating and publishing paths
class PathFromEquation : public rclcpp::Node {
public:
    PathFromEquation() : Node("planner"), rate_(0.5) {
        declareParameters();
        loadParameters();
        logParameters();
        initializeInterfaces(); // Renamed for clarity
        generateAndPublishPath();
    }

private:
    // Keep track of all node parameters
    struct Parameters {
        int path_number;
        int number_of_samples;
        double a, b, phi, cx, cy;
        bool closed_path_flag;
        int insert_n_points;
        int filter_path_n_average;
        double marker_scale;
        double marker_z_offset;
        std::string frame_id;
        std::string publish_path_topic_name;
        std::string visual_path_topic_name;
        std::string is_path_closed_service_name;
        rclcpp::TimerBase::SharedPtr publication_timer_;
        nav_msgs::msg::Path path_to_publish_;          
    } params_;

    rclcpp::Rate rate_;

    // ROS Interfaces
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr is_path_closed_service_; // <-- NEW

    rclcpp::TimerBase::SharedPtr publication_timer_;
    nav_msgs::msg::Path path_to_publish_;

    // ----------  ----------  ----------  ----------  ----------
    // Default values
    void declareParameters() {
        this->declare_parameter<int>("path_number", 2);
        this->declare_parameter<int>("number_of_samples", 100);
        this->declare_parameter<double>("a", 1.3);
        this->declare_parameter<double>("b", 0.6);
        this->declare_parameter<double>("phi", 0.0);
        this->declare_parameter<double>("cx", 0.0);
        this->declare_parameter<double>("cy", 0.0);
        this->declare_parameter<bool>("closed_path_flag", true);
        this->declare_parameter<int>("insert_n_points", 0);
        this->declare_parameter<int>("filter_path_n_average", 0);
        this->declare_parameter<double>("marker_scale", 0.15);
        this->declare_parameter<double>("marker_z_offset", -0.1);
        this->declare_parameter<std::string>("frame_id", "map");
        this->declare_parameter<std::string>("publish_path_topic_name", "ref_path");
        this->declare_parameter<std::string>("visual_path_topic_name", "visual_path");
        this->declare_parameter<std::string>("is_path_closed_service_name", "is_path_closed"); // <-- NEW
    }

    // ----------  ----------  ----------  ----------  ----------
    // Try to get parameters from config file
    void loadParameters() {
        // (loading for existing parameters is unchanged)
        params_.path_number = this->get_parameter("path_number").as_int();
        params_.number_of_samples = this->get_parameter("number_of_samples").as_int();
        params_.a = this->get_parameter("a").as_double();
        params_.b = this->get_parameter("b").as_double();
        params_.phi = this->get_parameter("phi").as_double() * (M_PI / 180.0);
        params_.cx = this->get_parameter("cx").as_double();
        params_.cy = this->get_parameter("cy").as_double();
        params_.closed_path_flag = this->get_parameter("closed_path_flag").as_bool();
        params_.insert_n_points = this->get_parameter("insert_n_points").as_int();
        params_.filter_path_n_average = this->get_parameter("filter_path_n_average").as_int();
        params_.marker_scale = this->get_parameter("marker_scale").as_double();
        params_.marker_z_offset = this->get_parameter("marker_z_offset").as_double();
        params_.frame_id = this->get_parameter("frame_id").as_string();
        params_.publish_path_topic_name = this->get_parameter("publish_path_topic_name").as_string();
        params_.visual_path_topic_name = this->get_parameter("visual_path_topic_name").as_string();
        params_.is_path_closed_service_name = this->get_parameter("is_path_closed_service_name").as_string(); // <-- NEW
    }

    // ----------  ----------  ----------  ----------  ----------
    void logParameters() const {
        // (unchanged)
        RCLCPP_INFO(this->get_logger(), "Path Parameters:");
        RCLCPP_INFO(this->get_logger(), "  Path number: %d", params_.path_number);
        RCLCPP_INFO(this->get_logger(), "  Number of points: %d", params_.number_of_samples);
        RCLCPP_INFO(this->get_logger(), "  a: %.2f, b: %.2f", params_.a, params_.b);
        RCLCPP_INFO(this->get_logger(), "  Rotation angle: %.2f rad", params_.phi);
        RCLCPP_INFO(this->get_logger(), "  Center: (%.2f, %.2f)", params_.cx, params_.cy);
    }
    
    // ----------  ----------  ----------  ----------  ----------
    // Setup publishers and services
    void initializeInterfaces() {
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>(params_.publish_path_topic_name, 10);
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            params_.visual_path_topic_name, 1);
        
        // Create the service server
        is_path_closed_service_ = this->create_service<std_srvs::srv::Trigger>(
            params_.is_path_closed_service_name,
            std::bind(&PathFromEquation::handleIsPathClosedRequest, this, std::placeholders::_1, std::placeholders::_2)
        );
        RCLCPP_INFO(this->get_logger(), "Service '%s' is ready.", params_.is_path_closed_service_name.c_str());
    }

    // ----------  ----------  ----------  ----------  ----------
    // NEW: Callback for the service
    void handleIsPathClosedRequest(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        (void)request; // Unused parameter

        // This service will always report that the path is closed
        response->success = true;
        response->message = "Path is considered closed by the planner node.";
        
        RCLCPP_INFO(this->get_logger(), "Service '%s' called, responding with 'true'.", params_.is_path_closed_service_name.c_str());
    }

    // ----------  ----------  ----------  ----------  ----------
    // (generateAndPublishPath, publishPath, and visualizePath methods are unchanged)
    void generateAndPublishPath() {
        const std::unordered_map<int, std::function<std::array<std::vector<double>, 2>(int)>> generators = {
            {1, [this](int N) { return PathGenerator::generateEllipse(N, params_.a, params_.b, params_.phi, params_.cx, params_.cy); }},
            {2, [this](int N) { return PathGenerator::generateFigure8(N, params_.a, params_.b, params_.phi, params_.cx, params_.cy); }},
            {3, [this](int N) { return PathGenerator::generateRectangle(N, params_.a, params_.b, params_.phi, params_.cx, params_.cy); }},
            {4, [this](int N) { return PathGenerator::generateSinusoidal(N, params_.a, params_.b, params_.phi, params_.cx, params_.cy); }},
            {5, [this](int N) { return PathGenerator::generateLemniscate(N, params_.a, params_.b, params_.phi, params_.cx, params_.cy); }}
        };

        auto it = generators.find(params_.path_number);
        if (it == generators.end()) {
            RCLCPP_ERROR(this->get_logger(), "Invalid path number: %d", params_.path_number);
            return;
        }

        const auto path = it->second(params_.number_of_samples);

        path_to_publish_.header.stamp = this->now();
        path_to_publish_.header.frame_id = params_.frame_id;

        for (size_t k = 0; k < path[0].size(); ++k) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header.stamp = this->now();
            pose.header.frame_id = params_.frame_id;
            pose.pose.position.x = path[0][k];
            pose.pose.position.y = path[1][k];
            pose.pose.position.z = 0.0;
            pose.pose.orientation.w = 1.0;
            path_to_publish_.poses.push_back(pose);
        }


        RCLCPP_INFO(this->get_logger(), "Caminho gerado. Agendando publicação para daqui a 2 segundos...");
        publication_timer_ = this->create_wall_timer(
            2s, std::bind(&PathFromEquation::publishPath, this));

        visualizePath(path);

        RCLCPP_INFO(this->get_logger(), "Path published successfully");
    }

    void publishPath() {
        path_pub_->publish(path_to_publish_);
        RCLCPP_INFO(this->get_logger(), "Caminho publicado após o atraso.");
        
        // Cancela o timer para que ele não dispare novamente
        publication_timer_->cancel();
    }

    void visualizePath(const std::array<std::vector<double>, 2>& path) {
        auto marker_array = std::make_shared<visualization_msgs::msg::MarkerArray>();

        for (size_t k = 0; k < path[0].size(); ++k) {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = params_.frame_id;
            marker.header.stamp = this->now();
            marker.id = k;
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.scale.x = params_.marker_scale;
            marker.scale.y = params_.marker_scale;
            marker.scale.z = params_.marker_scale;
            marker.color.a = 0.7;
            marker.color.r = 1.0;
            marker.color.g = 1.0;
            marker.color.b = 1.0;
            marker.pose.position.x = path[0][k];
            marker.pose.position.y = path[1][k];
            marker.pose.position.z = params_.marker_z_offset;
            marker.pose.orientation.w = 1.0;
            marker_array->markers.push_back(marker);
        }

        rate_.sleep();
        marker_pub_->publish(*marker_array);
    }
};

// ----------  ----------  ----------  ----------  ----------
// Main function
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PathFromEquation>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <Eigen/Dense> 
#include "tf2/exceptions.h"
#include "tf2/time.h"
#include "tf2_ros/transform_listener.h"
#include <tf2_ros/buffer.h>
#include "tf2_msgs/msg/tf_message.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

#include <vector>
#include <memory>

/*
Universidade Federal de Minas Gerais (UFMG) - 2025
Laboratório CORO
Instituto Tecnologico Vale (ITV)
Contact:
Thales Andrade Soares, <thalesasoares02@gmail.com>
*/

using namespace std::chrono_literals;

// ----------  ----------  ----------  ----------  ----------
// Class to interpolate a path from a set of waypoints and publish it as a ROS Path message
class PathFromPoints : public rclcpp::Node
{
public:
    PathFromPoints() : Node("planner")
    {
        // Declare parameters with defaults
        this->declare_parameter<int>("N_points", 5);
        this->declare_parameter<int>("flag_interpolation_method", 2);
        this->declare_parameter<double>("max_point_distance", 0.2);
        this->declare_parameter<std::string>("path_topic_name", "ref_path");
        this->declare_parameter<std::string>("close_path_service_name", "close_path");
        this->declare_parameter<std::string>("is_path_closed_service_name", "is_path_closed");
        this->declare_parameter<std::string>("visualization_topic_name", "visual_path");
        this->declare_parameter<std::string>("clicked_point_topic_name", "goal_pose");
        this->declare_parameter<std::string>("pose_topic_type", "TFMessage"); // TFMessage | Odometry | PoseWithCovarience
        this->declare_parameter<std::string>("pose_topic_name", "/tf");
        this->declare_parameter<std::string>("tf_reference_frame", "odom");
        this->declare_parameter<std::string>("tf_robot_pose", "body");
        this->declare_parameter<std::string>("start_service_name", "start_planner");
        this->declare_parameter<std::string>("clear_service_name", "clear_planner");
        this->declare_parameter<std::string>("remove_last_point_service_name", "remove_last_point");

        // Get parameters
        N_interpolation_ = this->get_parameter("N_points").as_int();
        max_point_distance = this->get_parameter("max_point_distance").as_double();
        flag_interpolation_method_ = this->get_parameter("flag_interpolation_method").as_int();
        path_topic_name_ = this->get_parameter("path_topic_name").as_string();
        close_path_service_name_ = this->get_parameter("close_path_service_name").as_string();
        is_path_closed_service_name_ = this->get_parameter("is_path_closed_service_name").as_string();
        visualization_topic_name_ = this->get_parameter("visualization_topic_name").as_string();
        clicked_point_topic_name_ = this->get_parameter("clicked_point_topic_name").as_string();
        pose_topic_type_ = this->get_parameter("pose_topic_type").as_string();
        pose_topic_name_ = this->get_parameter("pose_topic_name").as_string();
        tf_reference_frame_ = this->get_parameter("tf_reference_frame").as_string();
        tf_robot_pose_ = this->get_parameter("tf_robot_pose").as_string();
        start_service_name_ = this->get_parameter("start_service_name").as_string();
        clear_service_name_ = this->get_parameter("clear_service_name").as_string();
        remove_last_point_service_name = this->get_parameter("remove_last_point_service_name").as_string();

        RCLCPP_INFO(this->get_logger(), "Parameters loaded:");
        RCLCPP_INFO(this->get_logger(), "N_interpolation_: %d", N_interpolation_);
        RCLCPP_INFO(this->get_logger(), "flag_interpolation_method_: %d", flag_interpolation_method_);
        RCLCPP_INFO(this->get_logger(), "path_topic_name: %s", path_topic_name_.c_str());
        RCLCPP_INFO(this->get_logger(), "visualization_topic_name: %s", visualization_topic_name_.c_str());
        RCLCPP_INFO(this->get_logger(), "clicked_point_topic_name: %s", clicked_point_topic_name_.c_str());
        RCLCPP_INFO(this->get_logger(), "pose_topic_type: %s", pose_topic_type_.c_str());
        RCLCPP_INFO(this->get_logger(), "pose_topic_name: %s", pose_topic_name_.c_str());
        RCLCPP_INFO(this->get_logger(), "TF reference frame: %s", tf_reference_frame_.c_str());
        RCLCPP_INFO(this->get_logger(), "TF robot frame: %s", tf_robot_pose_.c_str());
        RCLCPP_INFO(this->get_logger(), "start_service_name: %s", start_service_name_.c_str());
        RCLCPP_INFO(this->get_logger(), "clear_service_name: %s", clear_service_name_.c_str());
        RCLCPP_INFO(this->get_logger(), "remove_last_point_service_name: %s", remove_last_point_service_name.c_str());

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // ref_path is a one-shot message per plan: the fleet manager (or
        // rviz) publishes a new goal, this node computes the path once and
        // publishes it once. TRANSIENT_LOCAL caches the last plan so a
        // vector_field_control.cpp reader that (re)matches after a discovery
        // race or a brief robot-network drop still receives it, instead of
        // being stuck following a stale/empty path.
        rclcpp::QoS latched_qos(1);
        latched_qos.reliable();
        latched_qos.transient_local();

        // Initialize publishers
        pub_path_ = this->create_publisher<nav_msgs::msg::Path>(path_topic_name_, latched_qos);
        pub_marker_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(visualization_topic_name_, 10);
        pub_waypoints_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(visualization_topic_name_, 10);

        // Initialize subscribers
        // Matches the TRANSIENT_LOCAL QoS the fleet manager now uses for
        // target_publisher (op1319_manager2.py), so this node also picks up
        // the last goal pose if it was published before we finished matching.
        clicked_point_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            clicked_point_topic_name_, latched_qos,
            std::bind(&PathFromPoints::callback_new_point, this, std::placeholders::_1));
            
        if (pose_topic_type_ == "TFMessage") {
            pose_sub_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
                pose_topic_name_, 10,
                std::bind(&PathFromPoints::callbackTF, this, std::placeholders::_1));
        } else if (pose_topic_type_ == "Odometry") {
            pose_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
                pose_topic_name_, 10,
                std::bind(&PathFromPoints::callback_odom, this, std::placeholders::_1));
        } else if (pose_topic_type_ == "PoseWithCovarience" || pose_topic_type_ == "PoseWithCovarianceStamped") {
            pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
                pose_topic_name_, 10,
                std::bind(&PathFromPoints::callbackAmclPose, this, std::placeholders::_1));
        } else {
            RCLCPP_WARN(
                this->get_logger(),
                "Invalid pose_topic_type '%s'. Falling back to TFMessage.",
                pose_topic_type_.c_str());
            pose_sub_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
                pose_topic_name_, 10,
                std::bind(&PathFromPoints::callbackTF, this, std::placeholders::_1));
        }
  

        // Initialize services
        start_service_ = create_service<std_srvs::srv::Trigger>(
            start_service_name_,
            std::bind(&PathFromPoints::handle_start_service, this,
                     std::placeholders::_1, std::placeholders::_2));
            
        clear_service_ = create_service<std_srvs::srv::Trigger>(
            clear_service_name_,
            std::bind(&PathFromPoints::handle_clear_service, this,
                     std::placeholders::_1, std::placeholders::_2));
            
        remove_last_service_ = create_service<std_srvs::srv::Trigger>(
            remove_last_point_service_name,
            std::bind(&PathFromPoints::handle_remove_last_service, this,
                     std::placeholders::_1, std::placeholders::_2));

        close_path_service_ = create_service<std_srvs::srv::Trigger>(
            close_path_service_name_,
            std::bind(&PathFromPoints::handle_close_path_service, this,
                     std::placeholders::_1, std::placeholders::_2));

        is_path_closed_service_ = create_service<std_srvs::srv::Trigger>(
            is_path_closed_service_name_,
            std::bind(&PathFromPoints::handle_is_path_closed_service, this,
                    std::placeholders::_1, std::placeholders::_2));

        // Initialize robot position
        robot_position_ = {0.0, 0.0, 0.0};
        has_robot_pose_ = false;
    }

// ----------  ----------  ----------  ----------  ----------
private:
    // Member variables
    int N_interpolation_;
    int flag_interpolation_method_;
    double max_point_distance; 
    std::string path_topic_name_;
    std::string close_path_service_name_;
    std::string is_path_closed_service_name_;
    std::string visualization_topic_name_;
    std::string clicked_point_topic_name_;
    std::string pose_topic_type_;
    std::string pose_topic_name_;
    std::string tf_reference_frame_;
    std::string tf_robot_pose_;
    std::string start_service_name_;
    std::string clear_service_name_;
    std::string remove_last_point_service_name;
    bool closed_path_ = false;
    bool has_robot_pose_ = false;
    
    std::vector<std::vector<double>> points_;
    std::array<double, 3> robot_position_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    
    // Publishers
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_marker_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_waypoints_;
    
    // Subscribers
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr clicked_point_subscription_;
    rclcpp::SubscriptionBase::SharedPtr pose_sub_;

    // Services
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr remove_last_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr close_path_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr is_path_closed_service_;
    
     std::vector<std::vector<double>> plannerFunction() {
        // Include the current robot's position in the planning
        std::vector<std::vector<double>> points;
        points.push_back({robot_position_[0], robot_position_[1], robot_position_[2]});
        points.insert(points.end(), points_.begin(), points_.end());

        std::vector<std::vector<double>> path;
        
        // Select interpolation method based on flag
        switch(flag_interpolation_method_) {
            case 1:
                path = linearInterpolation(points);
                break;
            case 2:
                path = quadraticInterpolation(points);
                break;
            case 3:
                path = cubicSplineInterpolation(points);
                break;
            case 4:
                path = hermiteInterpolation(points);
                break;
            default:
                // Default to linear 
                path = cubicSplineInterpolation(points);
                break;
        }

        // Publish the computed path to be visualized on rviz
        send_marker_array_to_rviz(path);
        
        return path;
    }

    // Implementation of linear interpolation
    std::vector<std::vector<double>> linearInterpolation(const std::vector<std::vector<double>>& points) {
        std::vector<std::vector<double>> path(3); // x, y, z components
        
        if (points.empty()) {
            return path;
        }

        // Number of segments between points
        size_t num_segments = points.size() - 1;
        if (num_segments == 0) {
            // Only one point, return it
            path[0].push_back(points[0][0]);
            path[1].push_back(points[0][1]);
            path[2].push_back(points[0][2]);
            return path;
        }

        // Interpolate each segment
        for (size_t i = 0; i < num_segments; ++i) {
            const auto& p0 = points[i];
            const auto& p1 = points[i+1];
            
            // Calculate segment length
            double dx = p1[0] - p0[0];
            double dy = p1[1] - p0[1];
            double dz = p1[2] - p0[2];
            double segment_length = std::sqrt(dx*dx + dy*dy + dz*dz);
            
            // Calculate number of points needed for this segment
            int points_in_segment = std::max(2, static_cast<int>(std::ceil(segment_length / max_point_distance)));
            
            // Generate the interpolated points
            for (int j = 0; j < points_in_segment; ++j) {
                double t = static_cast<double>(j) / (points_in_segment - 1);
                
                double x = p0[0] * (1.0 - t) + p1[0] * t;
                double y = p0[1] * (1.0 - t) + p1[1] * t;
                double z = p0[2] * (1.0 - t) + p1[2] * t;
                
                path[0].push_back(x);
                path[1].push_back(y);
                path[2].push_back(z);
            }
        }

        return path;
    }

    // Implementation of quadratic interpolation
    std::vector<std::vector<double>> quadraticInterpolation(const std::vector<std::vector<double>>& points) {
        // ... (código da interpolação quadrática sem alterações)
        std::vector<std::vector<double>> path(3);
        if (points.empty()) { return path; }
        size_t n = points.size();
        if (n == 1) {
            path[0].push_back(points[0][0]);
            path[1].push_back(points[0][1]);
            path[2].push_back(points[0][2]);
            return path;
        }
        Eigen::Matrix4d M, Minv;
        M << 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 1, 0, 3, 2, 1, 0;
        Minv << 2, -2, 1, 1, -3, 3, -2, -1, 0, 0, 1, 0, 1, 0, 0, 0;
        std::vector<std::vector<double>> T;
        for (size_t k = 0; k < n; ++k) {
            std::vector<double> v(3);
            if (k == 0) {
                v = {points[k+1][0]-points[k][0], points[k+1][1]-points[k][1], points[k+1][2]-points[k][2]};
            } else if (k == n-1) {
                v = {points[k][0]-points[k-1][0], points[k][1]-points[k-1][1], points[k][2]-points[k-1][2]};
            } else {
                v = {(points[k+1][0]-points[k-1][0])/2.0, (points[k+1][1]-points[k-1][1])/2.0, (points[k+1][2]-points[k-1][2])/2.0};
            }
            T.push_back(v);
        }
        for (size_t k = 0; k < n-1; ++k) {
            Eigen::MatrixXd A(4, 3);
            A << points[k][0], points[k][1], points[k][2],
                points[k+1][0], points[k+1][1], points[k+1][2],
                T[k][0], T[k][1], T[k][2],
                T[k+1][0], T[k+1][1], T[k+1][2];
            Eigen::MatrixXd ck = Minv * A;
            double segment_length = 0.0;
            const int num_samples = 10;
            Eigen::Vector3d prev_point = A.row(0);
            for (int i = 1; i <= num_samples; ++i) {
                double s = i / static_cast<double>(num_samples);
                Eigen::Vector3d point = ck.row(0)*pow(s,3) + ck.row(1)*pow(s,2) + ck.row(2)*s + ck.row(3);
                segment_length += (point - prev_point).norm();
                prev_point = point;
            }
            int points_in_segment = std::max(2, static_cast<int>(std::ceil(segment_length / max_point_distance)));
            for (int j = 0; j < points_in_segment; ++j) {
                double s = static_cast<double>(j) / (points_in_segment - 1);
                double x = ck(0,0)*pow(s,3) + ck(1,0)*pow(s,2) + ck(2,0)*s + ck(3,0);
                double y = ck(0,1)*pow(s,3) + ck(1,1)*pow(s,2) + ck(2,1)*s + ck(3,1);
                double z = points[k][2]*(1.0-s) + points[k+1][2]*s;
                path[0].push_back(x); path[1].push_back(y); path[2].push_back(z);
            }
        }
        if (!points.empty()) {
            path[0].push_back(points.back()[0]);
            path[1].push_back(points.back()[1]);
            path[2].push_back(points.back()[2]);
        }
        return path;
    }

    // Implementation of cubic spline interpolation
    std::vector<std::vector<double>> cubicSplineInterpolation(const std::vector<std::vector<double>>& points) {
        // ... (código da interpolação cúbica sem alterações)
        std::vector<std::vector<double>> path(3);
        if (points.empty()) { return path; }
        size_t n = points.size();
        if (n == 1) {
            path[0].push_back(points[0][0]); path[1].push_back(points[0][1]); path[2].push_back(points[0][2]);
            return path;
        }
        std::vector<double> D1 = {0.0, 0.0, 0.0};
        for (size_t k = 0; k < n - 2; ++k) {
            Eigen::Matrix4d A;
            A << pow(k,3), pow(k,2), k, 1, pow(k+1,3), pow(k+1,2), k+1, 1, pow(k+2,3), pow(k+2,2), k+2, 1, 3*pow(k,2), 2*k, 1, 0;
            Eigen::MatrixXd b(4, 3);
            b << points[k][0], points[k][1], points[k][2], points[k+1][0], points[k+1][1], points[k+1][2], points[k+2][0], points[k+2][1], points[k+2][2], D1[0], D1[1], D1[2];
            Eigen::MatrixXd ck = A.inverse() * b;
            D1[0] = 3*ck(0,0)*pow(k+1,2) + 2*ck(1,0)*(k+1) + ck(2,0);
            D1[1] = 3*ck(0,1)*pow(k+1,2) + 2*ck(1,1)*(k+1) + ck(2,1);
            D1[2] = 3*ck(0,2)*pow(k+1,2) + 2*ck(1,2)*(k+1) + ck(2,2);
            double segment_length = 0.0;
            const int num_samples = 10;
            Eigen::Vector3d prev_point(points[k][0], points[k][1], points[k][2]);
            for (int i = 1; i <= num_samples; ++i) {
                double u = k + (i / static_cast<double>(num_samples));
                Eigen::Vector3d point(ck(0,0)*pow(u,3) + ck(1,0)*pow(u,2) + ck(2,0)*u + ck(3,0), ck(0,1)*pow(u,3) + ck(1,1)*pow(u,2) + ck(2,1)*u + ck(3,1), ck(0,2)*pow(u,3) + ck(1,2)*pow(u,2) + ck(2,2)*u + ck(3,2));
                segment_length += (point - prev_point).norm();
                prev_point = point;
            }
            int points_in_segment = std::max(2, static_cast<int>(std::ceil(segment_length / max_point_distance)));
            for (int j = 0; j < points_in_segment; ++j) {
                double u = k + (static_cast<double>(j) / (points_in_segment - 1));
                path[0].push_back(ck(0,0)*pow(u,3) + ck(1,0)*pow(u,2) + ck(2,0)*u + ck(3,0));
                path[1].push_back(ck(0,1)*pow(u,3) + ck(1,1)*pow(u,2) + ck(2,1)*u + ck(3,1));
                path[2].push_back(ck(0,2)*pow(u,3) + ck(1,2)*pow(u,2) + ck(2,2)*u + ck(3,2));
            }
        }
        if (!points.empty()) {
            path[0].push_back(points.back()[0]); path[1].push_back(points.back()[1]); path[2].push_back(points.back()[2]);
        }
        return path;
    }

    // Implementation of hermite interpolation
    std::vector<std::vector<double>> hermiteInterpolation(const std::vector<std::vector<double>>& points) {
        // ... (código da interpolação de Hermite sem alterações)
        std::vector<std::vector<double>> path(3);
        if (points.empty()) { return path; }
        if (points.size() == 1) {
            path[0].push_back(points[0][0]); path[1].push_back(points[0][1]); path[2].push_back(points[0][2]);
            return path;
        }
        std::vector<std::vector<double>> tangents(points.size(), std::vector<double>(3, 0.0));
        for (size_t i = 0; i < points.size(); ++i) {
            if (i == 0) {
                tangents[i][0] = points[i+1][0] - points[i][0]; tangents[i][1] = points[i+1][1] - points[i][1]; tangents[i][2] = points[i+1][2] - points[i][2];
            } else if (i == points.size() - 1) {
                tangents[i][0] = points[i][0] - points[i-1][0]; tangents[i][1] = points[i][1] - points[i-1][1]; tangents[i][2] = points[i][2] - points[i-1][2];
            } else {
                tangents[i][0] = 0.5 * (points[i+1][0] - points[i-1][0]); tangents[i][1] = 0.5 * (points[i+1][1] - points[i-1][1]); tangents[i][2] = 0.5 * (points[i+1][2] - points[i-1][2]);
            }
        }
        auto h00 = [](double t) { return 2*t*t*t - 3*t*t + 1; };
        auto h10 = [](double t) { return t*t*t - 2*t*t + t; };
        auto h01 = [](double t) { return -2*t*t*t + 3*t*t; };
        auto h11 = [](double t) { return t*t*t - t*t; };
        for (size_t i = 0; i < points.size() - 1; ++i) {
            const auto& p0 = points[i]; const auto& p1 = points[i+1]; const auto& m0 = tangents[i]; const auto& m1 = tangents[i+1];
            double segment_length = 0.0; const int num_samples = 10;
            std::vector<double> prev_point = {h00(0)*p0[0] + h10(0)*m0[0] + h01(0)*p1[0] + h11(0)*m1[0], h00(0)*p0[1] + h10(0)*m0[1] + h01(0)*p1[1] + h11(0)*m1[1], h00(0)*p0[2] + h10(0)*m0[2] + h01(0)*p1[2] + h11(0)*m1[2]};
            for (int j = 1; j <= num_samples; ++j) {
                double t = static_cast<double>(j) / num_samples;
                std::vector<double> point = {h00(t)*p0[0] + h10(t)*m0[0] + h01(t)*p1[0] + h11(t)*m1[0], h00(t)*p0[1] + h10(t)*m0[1] + h01(t)*p1[1] + h11(t)*m1[1], h00(t)*p0[2] + h10(t)*m0[2] + h01(t)*p1[2] + h11(t)*m1[2]};
                double dx = point[0] - prev_point[0]; double dy = point[1] - prev_point[1]; double dz = point[2] - prev_point[2];
                segment_length += std::sqrt(dx*dx + dy*dy + dz*dz);
                prev_point = point;
            }
            int points_in_segment = std::max(2, static_cast<int>(std::ceil(segment_length / max_point_distance)));
            for (int j = 0; j < points_in_segment; ++j) {
                double t = static_cast<double>(j) / (points_in_segment - 1);
                double x = h00(t)*p0[0] + h10(t)*m0[0] + h01(t)*p1[0] + h11(t)*m1[0];
                double y = h00(t)*p0[1] + h10(t)*m0[1] + h01(t)*p1[1] + h11(t)*m1[1];
                double z = h00(t)*p0[2] + h10(t)*m0[2] + h01(t)*p1[2] + h11(t)*m1[2];
                path[0].push_back(x); path[1].push_back(y); path[2].push_back(z);
            }
        }
        path[0].push_back(points.back()[0]); path[1].push_back(points.back()[1]); path[2].push_back(points.back()[2]);
        return path;
    }

    // ----------  ----------  ----------  ----------  ----------
    void send_marker_array_to_rviz(const std::vector<std::vector<double>>& path)
    {
        auto points_marker = std::make_shared<visualization_msgs::msg::MarkerArray>();
        
        for (size_t i = 0; i < path[0].size(); ++i) {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = tf_reference_frame_;
            marker.header.stamp = this->now();
            marker.id = i;
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.scale.x = 0.15;
            marker.scale.y = 0.15;
            marker.scale.z = 0.15;
            marker.color.a = 0.7;
            marker.color.r = 1.0;
            marker.color.g = 1.0;
            marker.color.b = 1.0;
            marker.pose.orientation.w = 1.0;
            marker.pose.position.x = path[0][i];
            marker.pose.position.y = path[1][i];
            marker.pose.position.z = path[2][i];

            points_marker->markers.push_back(marker);
        }

        pub_marker_->publish(*points_marker);
    }

    void visualize_waypoints()
    {
        auto waypoints_marker = std::make_shared<visualization_msgs::msg::MarkerArray>();
        
        visualization_msgs::msg::Marker clear_marker;
        clear_marker.header.frame_id = tf_reference_frame_;
        clear_marker.header.stamp = this->now();
        clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
        waypoints_marker->markers.push_back(clear_marker);

        for (size_t i = 0; i < points_.size(); ++i) {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = tf_reference_frame_;
            marker.header.stamp = this->now();
            marker.id = i;
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.scale.x = 0.2;
            marker.scale.y = 0.2;
            marker.scale.z = 0.2;
            marker.color.a = 1.0;
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            marker.pose.orientation.w = 1.0;
            marker.pose.position.x = points_[i][0];
            marker.pose.position.y = points_[i][1];
            marker.pose.position.z = points_[i][2];

            waypoints_marker->markers.push_back(marker);
        }

        pub_waypoints_->publish(*waypoints_marker);
    }


    // ----------  ----------  ----------  ----------  ----------
    // Service callbacks

    // ADICIONADO: Nova função que centraliza a lógica de planejamento para ser reusada.
    bool plan_and_publish_path()
    {
        refreshRobotPoseFromTf();

        if (!has_robot_pose_) {
            RCLCPP_WARN(get_logger(), "Cannot plan yet: waiting for a valid robot pose.");
            return false;
        }

        if (points_.size() < static_cast<size_t>(flag_interpolation_method_)-1) {
            RCLCPP_WARN(get_logger(), "Não há pontos suficientes para o método de interpolação. Planejamento ignorado.");
            return false;
        }
        
        // Corrigido: a condição original era impossível de ser verdadeira (x < 1 && x > 4)
        if (flag_interpolation_method_ < 1 || flag_interpolation_method_ > 4) {
            RCLCPP_WARN(get_logger(), "Método de interpolação inválido. Use 1-4. Planejamento ignorado.");
            return false;
        }
        
        std::vector<std::vector<double>> path_planned = plannerFunction();

        auto poly_msg = std::make_shared<nav_msgs::msg::Path>();
        poly_msg->header.stamp = this->now();
        poly_msg->header.frame_id = tf_reference_frame_;

        for (size_t i = 0; i < path_planned[0].size(); ++i) {
            geometry_msgs::msg::PoseStamped pose_msg;
            pose_msg.header.stamp = this->now();
            pose_msg.header.frame_id = tf_reference_frame_;

            pose_msg.pose.position.x = path_planned[0][i];
            pose_msg.pose.position.y = path_planned[1][i];
            pose_msg.pose.position.z = path_planned[2][i];

            pose_msg.pose.orientation.w = 1.0;

            poly_msg->poses.push_back(pose_msg);
        }

        pub_path_->publish(*poly_msg);
        
        RCLCPP_INFO(get_logger(), "Caminho planejado e publicado com sucesso.");
        return true;
    }

    // MODIFICADO: O callback do serviço agora chama a função centralizada.
    void handle_start_service(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        (void)request;  // Unused parameter
        RCLCPP_INFO(get_logger(), "Serviço 'start_planner' chamado.");

        response->success = plan_and_publish_path();
        response->message = response->success
            ? "Comando de planejamento executado."
            : "Planejamento ignorado: pose válida ou pontos insuficientes.";
    }

    void handle_clear_service(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        (void)request;  // Unused parameter
        
        points_.clear();
        closed_path_ = false;
        visualize_waypoints();

        auto poly_msg = std::make_shared<nav_msgs::msg::Path>();
        poly_msg->header.stamp = this->now();
        poly_msg->header.frame_id = tf_reference_frame_;
        pub_path_->publish(*poly_msg);
        
        response->success = true;
        response->message = "All points and ref_path successfully cleared";
        RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
    }

    void handle_remove_last_service(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        (void)request; 
        
        if (!points_.empty()) {
            points_.pop_back();
            visualize_waypoints();  
            response->success = true;
            response->message = "Last point successfully cleared";
            RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
        } else {
            response->success = false;
            response->message = "No points to remove";
            RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
        }
    }

    void handle_close_path_service(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        (void)request;  // Unused parameter
        
        if (points_.size() < 2) {
            response->success = false;
            response->message = "Not enough points to close path. At least 2 points are needed.";
            RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
            return;
        }

        if (!points_.empty()) {
            points_.push_back({robot_position_[0], robot_position_[1], robot_position_[2]});
            visualize_waypoints();
        }

        closed_path_ = true;
        response->success = true;
        response->message = "Path successfully closed by adding first point again";
        RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
    }

    void handle_is_path_closed_service(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        (void)request;  // Unused parameter
        
        response->success = closed_path_;
        
        if (response->success) {
            response->message = "Path is closed";
        } else {
            response->message = "Path is not closed";
        }
    }

    void updateRobotPose(double x, double y, double z, const geometry_msgs::msg::Quaternion& q) {
        (void)q;
        robot_position_[0] = x;
        robot_position_[1] = y;
        robot_position_[2] = z;
        has_robot_pose_ = true;
    }

    bool refreshRobotPoseFromTf()
    {
        if (!tf_buffer_) {
            return false;
        }

        try {
            auto transform = tf_buffer_->lookupTransform(
                tf_reference_frame_,
                tf_robot_pose_,
                tf2::TimePointZero,
                tf2::durationFromSec(0.05));

            updateRobotPose(
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z,
                transform.transform.rotation);

            return true;
        } catch (const tf2::TransformException & ex) {
            RCLCPP_DEBUG(
                get_logger(),
                "TF pose lookup unavailable (%s <- %s): %s",
                tf_reference_frame_.c_str(),
                tf_robot_pose_.c_str(),
                ex.what());
            return false;
        }
    }

    // ----------  ----------  ----------  ----------  ----------
    // Subscribers callbacks
    void callback_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        robot_position_ = {
            msg->pose.pose.position.x,
            msg->pose.pose.position.y,
            msg->pose.pose.position.z - 0.4
        };
        has_robot_pose_ = true;
    }

    void callbackTF(const tf2_msgs::msg::TFMessage::SharedPtr msg) {
        
        for (const auto& transform : msg->transforms) {
            if (transform.header.frame_id == tf_reference_frame_ &&
                transform.child_frame_id == tf_robot_pose_) {
                updateRobotPose(
                    transform.transform.translation.x,
                    transform.transform.translation.y,
                    transform.transform.translation.z,
                    transform.transform.rotation);
                    // std::cout << "Recebeu pose" << std::endl;
            }
        }
    }

    void callbackAmclPose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        // Acessa a posição e a orientação diretamente da mensagem
        // O caminho é: msg->pose.pose.position e msg->pose.pose.orientation
        updateRobotPose(
            msg->pose.pose.position.x,
            msg->pose.pose.position.y,
            msg->pose.pose.position.z,
            msg->pose.pose.orientation
        );
        // std::cout << "Pose recebida do /amcl_pose" << std::endl;
    }

    // MODIFICADO: O callback de novo ponto agora também chama a função de planejamento.
    void callback_new_point(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        points_.clear(); 
        points_.push_back({
            msg->pose.position.x,
            msg->pose.position.y,
            msg->pose.position.z
        });
        RCLCPP_INFO(this->get_logger(), "Novo ponto recebido. Total: %zu", points_.size());
        
        visualize_waypoints();

        // ADICIONADO: Chama a lógica de planejamento automaticamente após adicionar o ponto.
        plan_and_publish_path();
    }
};

// ----------  ----------  ----------  ----------  ----------
// Main function
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PathFromPoints>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

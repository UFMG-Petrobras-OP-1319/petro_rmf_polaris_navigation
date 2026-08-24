#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <nav_msgs/msg/path.hpp>
#include "tf2/exceptions.h"
#include "tf2/time.h"
#include "tf2_ros/transform_listener.h"
#include <tf2_ros/buffer.h>
#include "tf2_msgs/msg/tf_message.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"


#include <memory>
#include <vector>
#include <cmath>
#include <algorithm>

/*
Universidade Federal de Minas Gerais (UFMG) - 2025
Laboratório CORO
Instituto Tecnologico Vale (ITV)
Contact: Thales Andrade Soares, <thalesasoares02@gmail.com>
*/

using namespace std::chrono_literals;

class VectorFieldController : public rclcpp::Node {
public:
    VectorFieldController() : Node("vector_field_controller") {
        loadParameters();
        initializeVariables();
        setupROS();
 
        timer_ = create_wall_timer(
            50ms, std::bind(&VectorFieldController::controlLoop, this));
    }

private:
    // Parameters
    double speed_ref_;
    double convergence_gain_;
    double switch_dist_;         // Corresponds to D_in in the paper
    double switch_dist_outer_;   // Corresponds to D_in^0 in the paper
    double lambda_;              // Desired circumnavigation distance
    double goal_tolerance_;
    double goal_hysteresis_;
    double slowdown_radius_;
    double min_tracking_speed_;
    bool flag_follow_obstacle_;
    bool closed_path_flag_;
    
    // Topic names
    std::string pose_topic_name_;
    std::string pose_topic_type_;
    std::string path_topic_name_;
    std::string cmd_vel_topic_name_;
    std::string closest_obstacle_topic_name_;
    std::string is_path_closed_service_name_;
    std::string tf_robot_pose_;
    std::string tf_reference_frame_;
    
    // Path data
    std::vector<std::vector<double>> path_;
    int current_closest_index_;
    int search_radius_;
    bool has_path_flag_;
    size_t path_size_;
    
    // Robot and Obstacle state
    std::vector<double> robot_pos_;
    std::vector<double> robot_euler_angles_;
    std::vector<double> obstacle_pos_;
    bool has_obstacle_flag_;
    bool has_pose_flag_;
    bool goal_reached_flag_;

    // ROS interfaces
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr rviz_command_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr rviz_path_pub_;

    rclcpp::SubscriptionBase::SharedPtr pose_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr obstacle_sub_;

    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr is_path_closed_client_;

    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    rclcpp::TimerBase::SharedPtr timer_;

    void loadParameters() {
        speed_ref_ = declare_parameter<double>("speed_ref", 0.2);
        convergence_gain_ = declare_parameter<double>("convergence_gain", 5.0);
        
        pose_topic_name_ = declare_parameter<std::string>("pose_topic_name", "amcl_pose");
        pose_topic_type_ = declare_parameter<std::string>("pose_topic_type", "Odometry");
        path_topic_name_ = declare_parameter<std::string>("path_topic_name", "ref_path");
        cmd_vel_topic_name_ = declare_parameter<std::string>("cmd_vel_topic_name", "vec_to_follow");
        closest_obstacle_topic_name_ = declare_parameter<std::string>("closest_obstacle_topic_name", "closest_obstacle"); // Corrected typo
        is_path_closed_service_name_ = declare_parameter<std::string>("is_path_closed_service_name", "is_path_closed");
        tf_robot_pose_ = declare_parameter<std::string>("tf_robot_pose", "robot");
        tf_reference_frame_ = declare_parameter<std::string>("tf_reference_frame", "odom");

        // Obstacle avoidance parameters from the paper
        flag_follow_obstacle_ = declare_parameter<bool>("flag_follow_obstacle", true); // IMPORTANT: Must be set to true to enable avoidance
        lambda_ = declare_parameter<double>("lambda", 0.7); 
        switch_dist_ = declare_parameter<double>("switch_dist", 0.8); 
        switch_dist_outer_ = declare_parameter<double>("switch_dist_outer", 1.1);
        goal_tolerance_ = declare_parameter<double>("goal_tolerance", 0.15);
        goal_hysteresis_ = declare_parameter<double>("goal_hysteresis", 0.22);
        slowdown_radius_ = declare_parameter<double>("slowdown_radius", 0.6);
        min_tracking_speed_ = declare_parameter<double>("min_tracking_speed", 0.08);
        
        closed_path_flag_ = false;
        logParameters();
    }

    void logParameters() {
        RCLCPP_INFO(get_logger(), "--- Vector Field Controller Parameters ---");
        RCLCPP_INFO(get_logger(), "  speed_ref: %.2f, convergence_gain: %.2f", speed_ref_, convergence_gain_);
        RCLCPP_INFO(get_logger(), "  flag_follow_obstacle: %s", flag_follow_obstacle_ ? "true" : "false");
        RCLCPP_INFO(get_logger(), "  lambda (avoid dist): %.2f", lambda_);
        RCLCPP_INFO(get_logger(), "  switch_dist (D_in): %.2f", switch_dist_);
        RCLCPP_INFO(get_logger(), "  switch_dist_outer (D_in^0): %.2f", switch_dist_outer_);
        RCLCPP_INFO(get_logger(), "  goal_tolerance: %.2f, slowdown_radius: %.2f, min_tracking_speed: %.2f",
            goal_tolerance_, slowdown_radius_, min_tracking_speed_);
        RCLCPP_INFO(get_logger(), "  TF reference frame: %s, robot frame: %s", tf_reference_frame_.c_str(), tf_robot_pose_.c_str());
        RCLCPP_INFO(get_logger(), "-----------------------------------------");
    }

    void initializeVariables() {
        robot_pos_ = {0.0, 0.0, 0.0};
        robot_euler_angles_ = {0.0, 0.0, 0.0};
        obstacle_pos_ = {0.0, 0.0, 0.0};
        
        current_closest_index_ = 0;
        search_radius_ = 10;
        path_size_ = 0;
        has_path_flag_ = false;
        has_obstacle_flag_ = false;
        has_pose_flag_ = false;
        goal_reached_flag_ = false;
    }

    void setupROS() {
        cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Vector3>(cmd_vel_topic_name_, 10);
        rviz_command_pub_ = create_publisher<visualization_msgs::msg::Marker>("visualization_command", 10);
        rviz_path_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("visualization_path", 10);

        if (pose_topic_type_ == "TFMessage") {
            pose_sub_ = create_subscription<tf2_msgs::msg::TFMessage>(pose_topic_name_, 10, std::bind(&VectorFieldController::callbackTF, this, std::placeholders::_1));
        } else if (pose_topic_type_ == "Odometry") {
            //pose_sub_ = create_subscription<nav_msgs::msg::Odometry>(pose_topic_name_, 10, std::bind(&VectorFieldController::callbackOdometry, this, std::placeholders::_1));
            pose_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            pose_topic_name_, // É melhor usar a variável do parâmetro
            10, 
            std::bind(&VectorFieldController::callbackOdometry, this, std::placeholders::_1) 
            );
        } else if (pose_topic_type_ == "PoseWithCovarience" || pose_topic_type_ == "PoseWithCovarianceStamped") {
            //pose_sub_ = create_subscription<nav_msgs::msg::Odometry>(pose_topic_name_, 10, std::bind(&VectorFieldController::callbackOdometry, this, std::placeholders::_1));
            pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            pose_topic_name_, // É melhor usar a variável do parâmetro
            10, 
            std::bind(&VectorFieldController::callbackAmclPose, this, std::placeholders::_1) 
            );
        }
        
        
        path_sub_ = create_subscription<nav_msgs::msg::Path>(path_topic_name_, 10, std::bind(&VectorFieldController::callbackPath, this, std::placeholders::_1));
        
        obstacle_sub_ = create_subscription<geometry_msgs::msg::Point>(
            closest_obstacle_topic_name_, 10, std::bind(&VectorFieldController::callbackObstacle, this, std::placeholders::_1));

        is_path_closed_client_ = create_client<std_srvs::srv::Trigger>(is_path_closed_service_name_);

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }

    // --- Main Loop ---
    void controlLoop() {
        refreshRobotPoseFromTf();

        if (!has_pose_flag_) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Waiting for a valid robot pose...");
            publishStopCommand();
            return;
        }

        if (!has_path_flag_ || path_size_ == 0) {
            RCLCPP_INFO_ONCE(get_logger(), "Waiting for a path to be published...");
            publishStopCommand();
            return;
        }
        
        auto vel = computeVelocityCommand();
        cmd_vel_pub_->publish(vel);
    }

    // --- Main Velocity Computation with Obstacle Avoidance ---
    geometry_msgs::msg::Vector3 computeVelocityCommand() {
        if (isGoalReached()) {
            visualizeCommand(0.0, 0.0);
            return makeCommand(0.0, 0.0);
        }

        // 1. Calculate the primary path-following field (Φ in the paper)
        auto field_path = computePathFollowingField();

        // 2. Check if obstacle avoidance is disabled or if we haven't seen an obstacle yet.
        //    If so, just follow the path.
        if (!flag_follow_obstacle_ || !has_obstacle_flag_) {
            field_path = applyGoalSpeedSchedule(field_path);
            visualizeCommand(field_path.first, field_path.second);
            return makeCommand(field_path.first, field_path.second);
        }

        // 3. Implement the switching logic from Eq. 18 of the paper
        double dx_obs = robot_pos_[0] - obstacle_pos_[0];
        double dy_obs = robot_pos_[1] - obstacle_pos_[1];
        double dist_to_obstacle = std::sqrt(dx_obs * dx_obs + dy_obs * dy_obs);

        // Vector D_o from obstacle to robot
        std::pair<double, double> Do_vec = {dx_obs, dy_obs};

        // Dot product D_o^T * Φ
        double dot_product = Do_vec.first * field_path.first + Do_vec.second * field_path.second;
        
        std::pair<double, double> final_field;

        // Condition 1: Follow path (moving away from obstacle OR far enough)
        if (dot_product >= 0 || dist_to_obstacle > switch_dist_outer_) {
            final_field = field_path;
        } 
        // Condition 2: Circumnavigate (moving towards obstacle AND very close)
        else if (dot_product < 0 && dist_to_obstacle < switch_dist_) {
            final_field = computeObstacleAvoidanceField(field_path, Do_vec, dist_to_obstacle);
        }
        // Condition 3: Transition (blending zone between D_in and D_in^0)
        else {
            auto field_obstacle = computeObstacleAvoidanceField(field_path, Do_vec, dist_to_obstacle);
            
            // Calculate blending factor theta (Eq. 17)
            double theta = (dist_to_obstacle - switch_dist_) / (switch_dist_outer_ - switch_dist_);
            theta = std::clamp(theta, 0.0, 1.0);

            // F = θ*Φ + (1-θ)*Ψ
            final_field.first = theta * field_path.first + (1.0 - theta) * field_obstacle.first;
            final_field.second = theta * field_path.second + (1.0 - theta) * field_obstacle.second;
        }
        
        final_field = applyGoalSpeedSchedule(final_field);
        visualizeCommand(final_field.first, final_field.second);
        return makeCommand(final_field.first, final_field.second);
    }

    // --- Field Calculation Functions ---

    // Calculates the path-following field (Φ)
    std::pair<double, double> computePathFollowingField() {
        int closest_index = findClosestPathPoint(robot_pos_[0], robot_pos_[1]);
        
        double dx = robot_pos_[0] - path_[0][closest_index];
        double dy = robot_pos_[1] - path_[1][closest_index];
        double distance = std::sqrt(dx*dx + dy*dy);
        
        std::pair<double, double> grad_D = {dx / (distance + 1e-6), dy / (distance + 1e-6)};
        std::pair<double, double> T = computeTangentVector(closest_index);
        
        double P = 0.5 * distance * distance;
        double G = -(2.0/M_PI) * atan(convergence_gain_ * sqrt(P));
        double H = std::sqrt(1.0 - G * G);
        
        double Vx = speed_ref_ * (G * grad_D.first + H * T.first);
        double Vy = speed_ref_ * (G * grad_D.second + H * T.second);
        
        return {Vx, Vy};
    }

    geometry_msgs::msg::Vector3 makeCommand(double vx, double vy) {
        geometry_msgs::msg::Vector3 vel_msg;
        vel_msg.x = vx;
        vel_msg.y = vy;
        vel_msg.z = 0.0;
        return vel_msg;
    }

    void publishStopCommand() {
        cmd_vel_pub_->publish(makeCommand(0.0, 0.0));
    }

    double distanceToGoal() const {
        if (path_size_ == 0) {
            return std::numeric_limits<double>::infinity();
        }

        const double dx = robot_pos_[0] - path_[0].back();
        const double dy = robot_pos_[1] - path_[1].back();
        return std::sqrt(dx * dx + dy * dy);
    }

    bool isGoalReached() {
        if (closed_path_flag_ || !has_path_flag_ || path_size_ == 0) {
            return false;
        }

        const double distance = distanceToGoal();
        const double hysteresis = std::max(goal_hysteresis_, goal_tolerance_);

        if (goal_reached_flag_) {
            goal_reached_flag_ = distance < hysteresis;
            return goal_reached_flag_;
        }

        if (distance <= goal_tolerance_) {
            goal_reached_flag_ = true;
            RCLCPP_INFO(get_logger(), "Goal reached within %.3f m.", distance);
            return true;
        }

        return false;
    }

    std::pair<double, double> applyGoalSpeedSchedule(std::pair<double, double> field) {
        if (closed_path_flag_ || !has_path_flag_ || path_size_ == 0) {
            return field;
        }

        const double distance = distanceToGoal();
        if (distance >= slowdown_radius_ || distance <= goal_tolerance_) {
            return field;
        }

        const double speed = std::sqrt(field.first * field.first + field.second * field.second);
        if (speed < 1e-6 || speed_ref_ < 1e-6) {
            return field;
        }

        const double span = std::max(slowdown_radius_ - goal_tolerance_, 1e-6);
        const double t = std::clamp((distance - goal_tolerance_) / span, 0.0, 1.0);
        const double smooth = t * t * (3.0 - 2.0 * t);
        const double floor_speed = std::clamp(min_tracking_speed_, 0.0, speed_ref_);
        const double target_speed = floor_speed + (speed_ref_ - floor_speed) * smooth;
        const double scale = target_speed / speed;

        return {field.first * scale, field.second * scale};
    }

    // Calculates the obstacle avoidance field (Ψ)
    std::pair<double, double> computeObstacleAvoidanceField(
        const std::pair<double, double>& auxiliary_field,
        const std::pair<double, double>& Do_vec,
        double dist_to_obstacle)
    {
        // Calculate D_lambda vector (Eq. 11)
        double Do_norm = dist_to_obstacle + 1e-6;
        std::pair<double, double> D_lambda_vec = {
            Do_vec.first - lambda_ * (Do_vec.first / Do_norm),
            Do_vec.second - lambda_ * (Do_vec.second / Do_norm)
        };
        double D_lambda_norm = std::sqrt(D_lambda_vec.first * D_lambda_vec.first + D_lambda_vec.second * D_lambda_vec.second);
        
        // Gradient of D_lambda
        std::pair<double, double> grad_D_lambda = {
            D_lambda_vec.first / (D_lambda_norm + 1e-6),
            D_lambda_vec.second / (D_lambda_norm + 1e-6)
        };

        // Calculate tangent T_lambda using the path field as auxiliary (Eq. 13)
        double dot_aux_grad = auxiliary_field.first * grad_D_lambda.first + auxiliary_field.second * grad_D_lambda.second;
        std::pair<double, double> T_lambda_unnormalized = {
            auxiliary_field.first - dot_aux_grad * grad_D_lambda.first,
            auxiliary_field.second - dot_aux_grad * grad_D_lambda.second
        };
        double T_lambda_norm = std::sqrt(T_lambda_unnormalized.first * T_lambda_unnormalized.first + T_lambda_unnormalized.second * T_lambda_unnormalized.second);
        
        std::pair<double, double> T_lambda = {
            T_lambda_unnormalized.first / (T_lambda_norm + 1e-6),
            T_lambda_unnormalized.second / (T_lambda_norm + 1e-6)
        };

        // Use the same gain functions, but with D_lambda_norm
        double P = 0.5 * D_lambda_norm * D_lambda_norm;
        double G = -(2.0/M_PI) * atan(convergence_gain_ * sqrt(P));
        double H = std::sqrt(1.0 - G * G);

        // Final obstacle field components (Eq. 14)
        double Vx = speed_ref_ * (G * grad_D_lambda.first + H * T_lambda.first);
        double Vy = speed_ref_ * (G * grad_D_lambda.second + H * T_lambda.second);

        return {Vx, Vy};
    }

    // --- Helper and Callback Functions ---

    int findClosestPathPoint(double x, double y) {
        std::vector<int> neighbor_indices;
        neighbor_indices.reserve(2 * search_radius_ + 1);
        
        for (int i = -search_radius_; i <= search_radius_; i++) {
            int idx = current_closest_index_ + i;
            
            if (closed_path_flag_) {
                if (idx < 0) idx += path_size_;
                if (idx >= static_cast<int>(path_size_)) idx -= path_size_;
            } else {
                idx = std::clamp(idx, 0, static_cast<int>(path_size_) - 1);
            }
            
            neighbor_indices.push_back(idx);
        }
        
        double min_distance = std::numeric_limits<double>::max();
        int closest_index = current_closest_index_;
        
        for (int idx : neighbor_indices) {
            double dx = x - path_[0][idx];
            double dy = y - path_[1][idx];
            double distance = dx*dx + dy*dy;
            
            if (distance < min_distance) {
                closest_index = idx;
                min_distance = distance;
            }
        }
        
        current_closest_index_ = closest_index;
        return closest_index;
    }

    std::pair<double, double> computeTangentVector(int closest_index) {
        int prev_index = closest_index - 1;
        int next_index = closest_index + 1;
        
        if (closed_path_flag_) {
            if (prev_index < 0) prev_index = path_size_ - 1;
            if (next_index >= static_cast<int>(path_size_)) next_index = 0;
        } else {
            prev_index = std::max(prev_index, 0);
            next_index = std::min(next_index, static_cast<int>(path_size_) - 1);
        }

        double Tx = path_[0][next_index] - path_[0][prev_index];
        double Ty = path_[1][next_index] - path_[1][prev_index];
        
        double norm = std::sqrt(Tx*Tx + Ty*Ty);
        return {Tx/(norm + 1e-6), Ty/(norm + 1e-6)};
    }

    void visualizeCommand(double Vx, double Vy) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = now();
        marker.ns = "control_vector";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::ARROW;
        marker.action = visualization_msgs::msg::Marker::ADD;
        
        double speed = std::sqrt(Vx*Vx + Vy*Vy);
        marker.scale.x = 1.0 * speed; // Arrow length
        marker.scale.y = 0.05;       // Arrow width
        marker.scale.z = 0.05;       // Arrow height
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        
        marker.pose.position.x = robot_pos_[0];
        marker.pose.position.y = robot_pos_[1];
        marker.pose.position.z = robot_pos_[2];
        
        tf2::Quaternion quat;
        quat.setRPY(0, 0, atan2(Vy, Vx));
        marker.pose.orientation = tf2::toMsg(quat);
        
        rviz_command_pub_->publish(marker);
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

    void callbackOdometry(const nav_msgs::msg::Odometry::SharedPtr msg) {
        updateRobotPose(
            msg->pose.pose.position.x,
            msg->pose.pose.position.y,
            msg->pose.pose.position.z,
            msg->pose.pose.orientation);
    }

    void updateRobotPose(double x, double y, double z, const geometry_msgs::msg::Quaternion& q) {
        robot_pos_[0] = x;
        robot_pos_[1] = y;
        robot_pos_[2] = z;
        has_pose_flag_ = true;
        
        tf2::Quaternion quat(q.x, q.y, q.z, q.w);
        tf2::Matrix3x3 mat(quat);
        mat.getRPY(robot_euler_angles_[0], robot_euler_angles_[1], robot_euler_angles_[2]);
    }

    bool refreshRobotPoseFromTf() {
        if (!tf_buffer_) {
            return false;
        }

        try {
            auto transform = tf_buffer_->lookupTransform(
                tf_reference_frame_,
                tf_robot_pose_,
                tf2::TimePointZero,
                tf2::durationFromSec(0.02));

            updateRobotPose(
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z,
                transform.transform.rotation);

            return true;
        } catch (const tf2::TransformException& ex) {
            RCLCPP_DEBUG_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "TF pose lookup unavailable (%s <- %s): %s",
                tf_reference_frame_.c_str(),
                tf_robot_pose_.c_str(),
                ex.what());
            return false;
        }
    }

    void callbackPath(const nav_msgs::msg::Path::SharedPtr msg) {
        path_.clear();
        path_.resize(2);
        for (const auto& pose : msg->poses) {
            path_[0].push_back(pose.pose.position.x);
            path_[1].push_back(pose.pose.position.y);
        }

        path_size_ = path_[0].size();
        if (path_size_ == 0) {
            has_path_flag_ = false;
            closed_path_flag_ = false;
            goal_reached_flag_ = false;
            current_closest_index_ = 0;
            RCLCPP_INFO(get_logger(), "Path cleared; waiting for a new ref_path");
            return;
        }

        current_closest_index_ = 0;
        double min_distance_sq = std::numeric_limits<double>::max();
        for (size_t i = 0; i < path_size_; i++) {
            double dx = robot_pos_[0] - path_[0][i];
            double dy = robot_pos_[1] - path_[1][i];
            double distance_sq = dx*dx + dy*dy;
            
            if (distance_sq < min_distance_sq) {
                current_closest_index_ = i;
                min_distance_sq = distance_sq;
            }
        }

        has_path_flag_ = true;
        goal_reached_flag_ = false;
        closed_path_flag_ = false;

        if (!is_path_closed_client_->wait_for_service(1s)) {
            RCLCPP_WARN(get_logger(), "Service '%s' not available, assuming path is open.", is_path_closed_service_name_.c_str());
            RCLCPP_INFO(get_logger(), "Received new path with %zu points (assumed open)", path_size_);
            return;
        }

        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        auto future_result = is_path_closed_client_->async_send_request(request,
            [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
                auto result = future.get();
                this->closed_path_flag_ = result->success;
                RCLCPP_INFO(get_logger(), "Path status updated: %s", this->closed_path_flag_ ? "CLOSED" : "OPEN");
            });

        RCLCPP_INFO(get_logger(), "Received new path with %zu points. Requesting path status...", path_size_);
    }

    void callbackObstacle(const geometry_msgs::msg::Point::SharedPtr msg) {
        obstacle_pos_[0] = msg->x;
        obstacle_pos_[1] = msg->y;
        obstacle_pos_[2] = msg->z;

        if (!has_obstacle_flag_) {
            has_obstacle_flag_ = true;
            RCLCPP_INFO(get_logger(), "First obstacle position received at (%.2f, %.2f, %.2f)",
                obstacle_pos_[0], obstacle_pos_[1], obstacle_pos_[2]);
        }
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VectorFieldController>());
    rclcpp::shutdown();
    return 0;
}

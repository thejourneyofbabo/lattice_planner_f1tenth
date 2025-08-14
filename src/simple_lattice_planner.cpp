#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <vector>
#include <cmath>
#include <algorithm>
#include <fstream>
#include <mutex>

struct Point2D {
    double x, y;
    Point2D(double x=0, double y=0) : x(x), y(y) {}
};

struct PathPoint {
    double x, y, yaw, curvature, velocity;
    PathPoint(double x=0, double y=0, double yaw=0, double curvature=0, double velocity=0)
        : x(x), y(y), yaw(yaw), curvature(curvature), velocity(velocity) {}
};

class SimpleLatticeController : public rclcpp::Node {
public:
    SimpleLatticeController() : Node("simple_lattice_controller") {
        // Parameters
        this->declare_parameter("reference_path_file", std::string(""));
        this->declare_parameter("max_lateral_offset", 2.0);
        this->declare_parameter("lateral_samples", 7);
        this->declare_parameter("lookahead_distance", 2.0);
        this->declare_parameter("wheelbase", 0.33);
        this->declare_parameter("target_speed", 3.0);
        this->declare_parameter("max_speed", 5.0);
        this->declare_parameter("min_speed", 1.0);
        
        // Load parameters
        reference_path_file_ = this->get_parameter("reference_path_file").as_string();
        max_lateral_offset_ = this->get_parameter("max_lateral_offset").as_double();
        lateral_samples_ = this->get_parameter("lateral_samples").as_int();
        lookahead_distance_ = this->get_parameter("lookahead_distance").as_double();
        wheelbase_ = this->get_parameter("wheelbase").as_double();
        target_speed_ = this->get_parameter("target_speed").as_double();
        max_speed_ = this->get_parameter("max_speed").as_double();
        min_speed_ = this->get_parameter("min_speed").as_double();
        
        // Load reference path
        if (!load_reference_path()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load reference path");
            return;
        }
        
        // Publishers
        drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 10);
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/planned_path", 10);
        
        // Subscribers
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/ego_racecar/odom", 10,
            std::bind(&SimpleLatticeController::odom_callback, this, std::placeholders::_1));
            
        laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&SimpleLatticeController::laser_callback, this, std::placeholders::_1));
        
        // Timer for control loop
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&SimpleLatticeController::control_callback, this));
        
        RCLCPP_INFO(this->get_logger(), "Simple Lattice Controller initialized");
    }

private:
    // Parameters
    std::string reference_path_file_;
    double max_lateral_offset_;
    int lateral_samples_;
    double lookahead_distance_;
    double wheelbase_;
    double target_speed_, max_speed_, min_speed_;
    
    // State
    std::vector<PathPoint> reference_path_;
    Point2D vehicle_pos_;
    double vehicle_yaw_ = 0.0;
    double vehicle_vel_ = 0.0;
    bool odom_received_ = false;
    
    std::vector<Point2D> obstacles_;
    std::mutex state_mutex_;
    
    // ROS
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    
    bool load_reference_path() {
        if (reference_path_file_.empty()) {
            // Create simple test path
            reference_path_.clear();
            for (int i = 0; i < 100; ++i) {
                double x = i * 0.5;
                double y = 2.0 * std::sin(i * 0.1);
                double yaw = std::atan2(2.0 * 0.1 * std::cos(i * 0.1), 1.0);
                reference_path_.emplace_back(x, y, yaw, 0.0, target_speed_);
            }
            return true;
        }
        
        std::ifstream file(reference_path_file_);
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Cannot open file: %s", reference_path_file_.c_str());
            return false;
        }
        
        reference_path_.clear();
        std::string line;
        while (std::getline(file, line)) {
            std::stringstream ss(line);
            std::string item;
            std::vector<double> values;
            
            while (std::getline(ss, item, ',')) {
                try {
                    values.push_back(std::stod(item));
                } catch (const std::exception&) {
                    continue; // Skip invalid values
                }
            }
            
            if (values.size() >= 2) {
                double x = values[0];
                double y = values[1];
                double yaw = (values.size() >= 3) ? values[2] : 0.0;
                double vel = (values.size() >= 4) ? values[3] : target_speed_;
                reference_path_.emplace_back(x, y, yaw, 0.0, vel);
            }
        }
        
        RCLCPP_INFO(this->get_logger(), "Loaded %zu reference points", reference_path_.size());
        return !reference_path_.empty();
    }
    
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        
        vehicle_pos_.x = msg->pose.pose.position.x;
        vehicle_pos_.y = msg->pose.pose.position.y;
        
        tf2::Quaternion q;
        tf2::fromMsg(msg->pose.pose.orientation, q);
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        vehicle_yaw_ = yaw;
        
        double vx = msg->twist.twist.linear.x;
        double vy = msg->twist.twist.linear.y;
        vehicle_vel_ = std::sqrt(vx*vx + vy*vy);
        
        odom_received_ = true;
    }
    
    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        
        obstacles_.clear();
        for (size_t i = 0; i < msg->ranges.size(); ++i) {
            double range = msg->ranges[i];
            if (range < msg->range_min || range > msg->range_max || 
                std::isnan(range) || std::isinf(range) || range > 5.0) {
                continue;
            }
            
            double angle = msg->angle_min + i * msg->angle_increment;
            double global_angle = vehicle_yaw_ + angle;
            
            Point2D obstacle;
            obstacle.x = vehicle_pos_.x + range * std::cos(global_angle);
            obstacle.y = vehicle_pos_.y + range * std::sin(global_angle);
            obstacles_.push_back(obstacle);
        }
    }
    
    void control_callback() {
        if (!odom_received_ || reference_path_.empty()) {
            return;
        }
        
        std::lock_guard<std::mutex> lock(state_mutex_);
        
        // Find closest point on reference path
        int closest_idx = find_closest_point();
        if (closest_idx < 0) return;
        
        // Generate lattice candidates
        std::vector<std::vector<PathPoint>> candidates = generate_lattice_paths(closest_idx);
        
        // Evaluate and select best path
        int best_idx = select_best_path(candidates);
        
        if (best_idx >= 0 && best_idx < candidates.size()) {
            // Publish selected path
            publish_path(candidates[best_idx]);
            
            // Calculate control commands using pure pursuit
            auto [steering, speed] = pure_pursuit_control(candidates[best_idx]);
            
            // Publish drive command
            auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
            drive_msg.header.stamp = this->get_clock()->now();
            drive_msg.drive.steering_angle = steering;
            drive_msg.drive.speed = speed;
            drive_pub_->publish(drive_msg);
        }
    }
    
    int find_closest_point() {
        double min_dist = std::numeric_limits<double>::max();
        int closest_idx = -1;
        
        for (size_t i = 0; i < reference_path_.size(); ++i) {
            double dx = reference_path_[i].x - vehicle_pos_.x;
            double dy = reference_path_[i].y - vehicle_pos_.y;
            double dist = std::sqrt(dx*dx + dy*dy);
            
            if (dist < min_dist) {
                min_dist = dist;
                closest_idx = i;
            }
        }
        
        return closest_idx;
    }
    
    std::vector<std::vector<PathPoint>> generate_lattice_paths(int start_idx) {
        std::vector<std::vector<PathPoint>> candidates;
        
        // Generate lateral offsets
        for (int i = 0; i < lateral_samples_; ++i) {
            double lateral_offset = -max_lateral_offset_ + 
                                   (2.0 * max_lateral_offset_ * i) / (lateral_samples_ - 1);
            
            std::vector<PathPoint> path = generate_offset_path(start_idx, lateral_offset);
            if (!path.empty()) {
                candidates.push_back(path);
            }
        }
        
        return candidates;
    }
    
    std::vector<PathPoint> generate_offset_path(int start_idx, double lateral_offset) {
        std::vector<PathPoint> path;
        
        int horizon_points = std::min(25, static_cast<int>(reference_path_.size() - start_idx));
        
        for (int i = 0; i < horizon_points; ++i) {
            int ref_idx = start_idx + i;
            if (ref_idx >= reference_path_.size()) break;
            
            const auto& ref_point = reference_path_[ref_idx];
            
            // Apply lateral offset perpendicular to reference path direction
            double offset_x = ref_point.x + lateral_offset * std::cos(ref_point.yaw + M_PI/2);
            double offset_y = ref_point.y + lateral_offset * std::sin(ref_point.yaw + M_PI/2);
            
            path.emplace_back(offset_x, offset_y, ref_point.yaw, 0.0, target_speed_);
        }
        
        return path;
    }
    
    int select_best_path(const std::vector<std::vector<PathPoint>>& candidates) {
        if (candidates.empty()) return -1;
        
        double best_cost = std::numeric_limits<double>::max();
        int best_idx = -1;
        
        for (size_t i = 0; i < candidates.size(); ++i) {
            double cost = evaluate_path_cost(candidates[i]);
            
            if (cost < best_cost) {
                best_cost = cost;
                best_idx = i;
            }
        }
        
        return best_idx;
    }
    
    double evaluate_path_cost(const std::vector<PathPoint>& path) {
        if (path.empty()) return 1e6;
        
        double cost = 0.0;
        
        // Distance from reference line cost (prefer center)
        double lateral_cost = 0.0;
        for (const auto& point : path) {
            // Find closest reference point
            double min_dist = 1e6;
            for (const auto& ref : reference_path_) {
                double dx = point.x - ref.x;
                double dy = point.y - ref.y;
                double dist = std::sqrt(dx*dx + dy*dy);
                min_dist = std::min(min_dist, dist);
            }
            lateral_cost += min_dist * min_dist;
        }
        cost += lateral_cost * 1.0; // Weight
        
        // Obstacle cost
        double obstacle_cost = 0.0;
        for (const auto& point : path) {
            for (const auto& obs : obstacles_) {
                double dx = point.x - obs.x;
                double dy = point.y - obs.y;
                double dist = std::sqrt(dx*dx + dy*dy);
                
                if (dist < 0.5) {
                    obstacle_cost += 100.0 / (dist + 0.1); // High cost for close obstacles
                }
            }
        }
        cost += obstacle_cost;
        
        return cost;
    }
    
    std::pair<double, double> pure_pursuit_control(const std::vector<PathPoint>& path) {
        if (path.empty()) return {0.0, 0.0};
        
        // Find lookahead point
        double lookahead = std::max(lookahead_distance_, vehicle_vel_ * 0.3);
        Point2D target = find_lookahead_point(path, lookahead);
        
        // Calculate steering angle using pure pursuit
        double dx = target.x - vehicle_pos_.x;
        double dy = target.y - vehicle_pos_.y;
        
        // Transform to vehicle frame
        double cos_yaw = std::cos(-vehicle_yaw_);
        double sin_yaw = std::sin(-vehicle_yaw_);
        double local_x = dx * cos_yaw - dy * sin_yaw;
        double local_y = dx * sin_yaw + dy * cos_yaw;
        
        double lookahead_dist = std::sqrt(local_x*local_x + local_y*local_y);
        if (lookahead_dist < 0.1) return {0.0, target_speed_};
        
        double curvature = 2.0 * local_y / (lookahead_dist * lookahead_dist);
        double steering_angle = std::atan(wheelbase_ * curvature);
        
        // Limit steering
        steering_angle = std::clamp(steering_angle, -0.4, 0.4);
        
        // Adjust speed based on steering
        double speed = target_speed_;
        if (std::abs(steering_angle) > 0.2) {
            speed *= 0.7; // Slow down in turns
        }
        speed = std::clamp(speed, min_speed_, max_speed_);
        
        return {steering_angle, speed};
    }
    
    Point2D find_lookahead_point(const std::vector<PathPoint>& path, double lookahead) {
        if (path.empty()) return vehicle_pos_;
        
        for (const auto& point : path) {
            double dx = point.x - vehicle_pos_.x;
            double dy = point.y - vehicle_pos_.y;
            double dist = std::sqrt(dx*dx + dy*dy);
            
            if (dist >= lookahead) {
                return Point2D(point.x, point.y);
            }
        }
        
        // Return last point if no lookahead point found
        return Point2D(path.back().x, path.back().y);
    }
    
    void publish_path(const std::vector<PathPoint>& path) {
        auto path_msg = nav_msgs::msg::Path();
        path_msg.header.frame_id = "map";
        path_msg.header.stamp = this->get_clock()->now();
        
        for (const auto& point : path) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header = path_msg.header;
            pose.pose.position.x = point.x;
            pose.pose.position.y = point.y;
            pose.pose.position.z = 0.0;
            
            tf2::Quaternion q;
            q.setRPY(0, 0, point.yaw);
            tf2::convert(q, pose.pose.orientation);
            
            path_msg.poses.push_back(pose);
        }
        
        path_pub_->publish(path_msg);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimpleLatticeController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
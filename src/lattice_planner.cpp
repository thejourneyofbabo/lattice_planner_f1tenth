#include "lattice_planner_pkg/lattice_planner.hpp"
#include "lattice_planner_pkg/msg/path_point_array.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace lattice_planner_pkg {

LatticePlanner::LatticePlanner() 
    : Node("lattice_planner"),
      vehicle_yaw_(0.0),
      vehicle_velocity_(0.0),
      odom_received_(false) {
    
    if (!initialize()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize lattice planner");
        return;
    }
    
    RCLCPP_INFO(this->get_logger(), "Lattice Planner initialized successfully");
}

LatticePlanner::~LatticePlanner() {
}

bool LatticePlanner::initialize() {
    // Initialize TF2
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    
    // Load configuration parameters
    this->declare_parameter("reference_path_file", std::string(""));
    this->declare_parameter("path_resolution", 0.1);
    this->declare_parameter("lateral_step", 0.5);
    this->declare_parameter("max_lateral_offset", 2.0);
    this->declare_parameter("planning_horizon", 3.0);
    this->declare_parameter("dt", 0.1);
    this->declare_parameter("max_velocity", 10.0);
    this->declare_parameter("planning_frequency", 10.0);
    
    config_.reference_path_file = this->get_parameter("reference_path_file").as_string();
    config_.path_resolution = this->get_parameter("path_resolution").as_double();
    config_.lateral_step = this->get_parameter("lateral_step").as_double();
    config_.max_lateral_offset = this->get_parameter("max_lateral_offset").as_double();
    config_.planning_horizon = this->get_parameter("planning_horizon").as_double();
    config_.dt = this->get_parameter("dt").as_double();
    config_.max_velocity = this->get_parameter("max_velocity").as_double();
    
    double planning_frequency = this->get_parameter("planning_frequency").as_double();
    
    // Load reference path
    if (!load_reference_path()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load reference path");
        return false;
    }
    
    // Initialize modules
    frenet_coord_ = std::make_shared<FrenetCoordinate>();
    if (!frenet_coord_->initialize(reference_path_)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize Frenet coordinate system");
        return false;
    }
    
    path_generator_ = std::make_shared<PathGenerator>(config_);
    path_generator_->set_frenet_coordinate(frenet_coord_);
    
    obstacle_detector_ = std::make_shared<ObstacleDetector>(config_);
    
    // Initialize publishers
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
        "/planned_path", 10);
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/path_candidates", 10);
    ref_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
        "/reference_path", 10);
    
    // Initialize subscribers
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/ego_racecar/odom", 10, 
        std::bind(&LatticePlanner::odom_callback, this, std::placeholders::_1));
    
    laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10,
        std::bind(&LatticePlanner::laser_callback, this, std::placeholders::_1));
    
    grid_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", 10,
        std::bind(&LatticePlanner::grid_callback, this, std::placeholders::_1));
    
    // Initialize timer
    auto timer_period = std::chrono::milliseconds(static_cast<int>(1000.0 / planning_frequency));
    planning_timer_ = this->create_wall_timer(
        timer_period, std::bind(&LatticePlanner::planning_timer_callback, this));
    
    // Publish reference path
    publish_reference_path();
    
    return true;
}

bool LatticePlanner::load_reference_path() {
    if (config_.reference_path_file.empty()) {
        // Create a simple test path if no file specified
        RCLCPP_WARN(this->get_logger(), "No reference path file specified, creating test path");
        
        reference_path_.clear();
        for (int i = 0; i < 100; ++i) {
            RefPoint point;
            point.x = i * 0.5;
            point.y = 2.0 * std::sin(i * 0.1);
            point.velocity = 5.0;
            reference_path_.push_back(point);
        }
        
        SplineUtils::calculate_arc_length(reference_path_);
        SplineUtils::calculate_heading(reference_path_);
        SplineUtils::calculate_curvature(reference_path_);
    } else {
        reference_path_ = SplineUtils::load_reference_path_from_csv(
            config_.reference_path_file, config_.path_resolution);
    }
    
    if (reference_path_.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Reference path is empty");
        return false;
    }
    
    RCLCPP_INFO(this->get_logger(), "Loaded reference path with %zu points", reference_path_.size());
    return true;
}

void LatticePlanner::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(vehicle_state_mutex_);
    
    vehicle_position_.x = msg->pose.pose.position.x;
    vehicle_position_.y = msg->pose.pose.position.y;
    
    // Convert quaternion to yaw
    tf2::Quaternion q;
    tf2::fromMsg(msg->pose.pose.orientation, q);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    vehicle_yaw_ = yaw;
    
    // Calculate velocity from twist
    double vx = msg->twist.twist.linear.x;
    double vy = msg->twist.twist.linear.y;
    vehicle_velocity_ = std::sqrt(vx*vx + vy*vy);
    
    odom_received_ = true;
}

void LatticePlanner::laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    if (!odom_received_) return;
    
    std::lock_guard<std::mutex> lock(obstacles_mutex_);
    
    Point2D vehicle_pos;
    double vehicle_yaw;
    {
        std::lock_guard<std::mutex> state_lock(vehicle_state_mutex_);
        vehicle_pos = vehicle_position_;
        vehicle_yaw = vehicle_yaw_;
    }
    
    current_obstacles_ = obstacle_detector_->detect_from_laser_scan(
        msg, vehicle_pos, vehicle_yaw);
}

void LatticePlanner::grid_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    if (!odom_received_) return;
    
    std::lock_guard<std::mutex> lock(obstacles_mutex_);
    
    Point2D vehicle_pos;
    {
        std::lock_guard<std::mutex> state_lock(vehicle_state_mutex_);
        vehicle_pos = vehicle_position_;
    }
    
    auto grid_obstacles = obstacle_detector_->detect_from_occupancy_grid(msg, vehicle_pos);
    
    // Merge with laser obstacles (simple approach - replace for now)
    // In practice, you might want to fuse multiple obstacle sources
    if (current_obstacles_.empty()) {
        current_obstacles_ = grid_obstacles;
    }
}

void LatticePlanner::planning_timer_callback() {
    if (!odom_received_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 
                             1000, "Waiting for odometry data");
        return;
    }
    
    plan_paths();
}

void LatticePlanner::plan_paths() {
    Point2D vehicle_pos;
    double vehicle_yaw;
    double vehicle_vel;
    std::vector<Obstacle> obstacles;
    
    {
        std::lock_guard<std::mutex> state_lock(vehicle_state_mutex_);
        vehicle_pos = vehicle_position_;
        vehicle_yaw = vehicle_yaw_;
        vehicle_vel = vehicle_velocity_;
    }
    
    {
        std::lock_guard<std::mutex> obs_lock(obstacles_mutex_);
        obstacles = current_obstacles_;
    }
    
    // Generate path candidates
    std::vector<PathCandidate> candidates = path_generator_->generate_paths(
        vehicle_pos, vehicle_yaw, vehicle_vel, obstacles);
    
    if (candidates.empty()) {
        RCLCPP_WARN(this->get_logger(), "No valid path candidates generated");
        return;
    }
    
    // Select best path
    PathCandidate selected_path = select_best_path(candidates);
    
    // Publish selected path
    publish_selected_path(selected_path);
    
    // Publish visualization
    publish_path_visualization(candidates, selected_path);
}

PathCandidate LatticePlanner::select_best_path(const std::vector<PathCandidate>& candidates) {
    if (candidates.empty()) {
        return PathCandidate();
    }
    
    // Find path with minimum cost among safe paths
    double min_cost = std::numeric_limits<double>::max();
    size_t best_idx = 0;
    
    for (size_t i = 0; i < candidates.size(); ++i) {
        if (candidates[i].is_safe && candidates[i].cost < min_cost) {
            min_cost = candidates[i].cost;
            best_idx = i;
        }
    }
    
    // If no safe path found, select least dangerous one
    if (min_cost == std::numeric_limits<double>::max()) {
        RCLCPP_WARN(this->get_logger(), "No safe path found, selecting least dangerous");
        min_cost = std::numeric_limits<double>::max();
        for (size_t i = 0; i < candidates.size(); ++i) {
            if (candidates[i].cost < min_cost) {
                min_cost = candidates[i].cost;
                best_idx = i;
            }
        }
    }
    
    return candidates[best_idx];
}

void LatticePlanner::publish_selected_path(const PathCandidate& path) {
    if (path.points.empty()) return;
    
    // Publish path as nav_msgs::Path
    auto nav_path = convert_to_nav_path(path);
    path_pub_->publish(nav_path);
}

void LatticePlanner::publish_path_visualization(
    const std::vector<PathCandidate>& candidates,
    const PathCandidate& selected) {
    
    auto markers = create_path_markers(candidates, selected);
    marker_pub_->publish(markers);
}

void LatticePlanner::publish_reference_path() {
    nav_msgs::msg::Path path_msg;
    path_msg.header.frame_id = "map";
    path_msg.header.stamp = this->get_clock()->now();
    
    for (const auto& point : reference_path_) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = path_msg.header;
        pose.pose.position.x = point.x;
        pose.pose.position.y = point.y;
        pose.pose.position.z = 0.0;
        
        tf2::Quaternion q;
        q.setRPY(0, 0, point.heading);
        tf2::convert(q, pose.pose.orientation);
        
        path_msg.poses.push_back(pose);
    }
    
    ref_path_pub_->publish(path_msg);
}


nav_msgs::msg::Path LatticePlanner::convert_to_nav_path(const PathCandidate& path) {
    nav_msgs::msg::Path nav_path;
    nav_path.header.frame_id = "map";
    nav_path.header.stamp = this->get_clock()->now();
    
    for (const auto& point : path.points) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = nav_path.header;
        pose.pose.position.x = point.x;
        pose.pose.position.y = point.y;
        pose.pose.position.z = 0.0;
        
        tf2::Quaternion q;
        q.setRPY(0, 0, point.yaw);
        tf2::convert(q, pose.pose.orientation);
        
        nav_path.poses.push_back(pose);
    }
    
    return nav_path;
}

visualization_msgs::msg::MarkerArray LatticePlanner::create_path_markers(
    const std::vector<PathCandidate>& candidates,
    const PathCandidate& selected) {
    
    visualization_msgs::msg::MarkerArray markers;
    
    // Clear previous markers
    visualization_msgs::msg::Marker clear_marker;
    clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(clear_marker);
    
    // Create markers for candidates
    for (size_t i = 0; i < candidates.size(); ++i) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "path_candidates";
        marker.id = static_cast<int>(i);
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.action = visualization_msgs::msg::Marker::ADD;
        
        marker.scale.x = 0.05;  // Line width
        
        // Color coding: green for safe, red for unsafe
        if (candidates[i].is_safe) {
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            marker.color.a = 0.5;
        } else {
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            marker.color.a = 0.3;
        }
        
        for (const auto& point : candidates[i].points) {
            geometry_msgs::msg::Point p;
            p.x = point.x;
            p.y = point.y;
            p.z = 0.0;
            marker.points.push_back(p);
        }
        
        markers.markers.push_back(marker);
    }
    
    // Highlight selected path
    if (!selected.points.empty()) {
        visualization_msgs::msg::Marker selected_marker;
        selected_marker.header.frame_id = "map";
        selected_marker.header.stamp = this->get_clock()->now();
        selected_marker.ns = "selected_path";
        selected_marker.id = 0;
        selected_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        selected_marker.action = visualization_msgs::msg::Marker::ADD;
        
        selected_marker.scale.x = 0.1;  // Thicker line
        selected_marker.color.r = 0.0;
        selected_marker.color.g = 0.0;
        selected_marker.color.b = 1.0;  // Blue
        selected_marker.color.a = 1.0;
        
        for (const auto& point : selected.points) {
            geometry_msgs::msg::Point p;
            p.x = point.x;
            p.y = point.y;
            p.z = 0.1;  // Slightly elevated
            selected_marker.points.push_back(p);
        }
        
        markers.markers.push_back(selected_marker);
    }
    
    return markers;
}

} // namespace lattice_planner_pkg

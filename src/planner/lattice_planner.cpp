#include "lattice_planner_pkg/planner/lattice_planner.hpp"
#include "lattice_planner_pkg/msg/path_point_array.hpp"
#include "lattice_planner_pkg/obstacle_detector.hpp"
#include "lattice_planner_pkg/path_selector.hpp"
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
    this->declare_parameter("max_curvature", 1.0);
    this->declare_parameter("lateral_cost_weight", 1.0);
    this->declare_parameter("curvature_cost_weight", 1.0);
    this->declare_parameter("longitudinal_cost_weight", 0.1);
    this->declare_parameter("obstacle_cost_weight", 10.0);
    
    config_.reference_path_file = this->get_parameter("reference_path_file").as_string();
    config_.path_resolution = this->get_parameter("path_resolution").as_double();
    config_.lateral_step = this->get_parameter("lateral_step").as_double();
    config_.max_lateral_offset = this->get_parameter("max_lateral_offset").as_double();
    config_.planning_horizon = this->get_parameter("planning_horizon").as_double();
    config_.dt = this->get_parameter("dt").as_double();
    config_.max_velocity = this->get_parameter("max_velocity").as_double();
    config_.max_curvature = this->get_parameter("max_curvature").as_double();
    config_.lateral_cost_weight = this->get_parameter("lateral_cost_weight").as_double();
    config_.curvature_cost_weight = this->get_parameter("curvature_cost_weight").as_double();
    config_.longitudinal_cost_weight = this->get_parameter("longitudinal_cost_weight").as_double();
    config_.obstacle_cost_weight = this->get_parameter("obstacle_cost_weight").as_double();
    
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
    
    // Initialize new obstacle detector and path selector
    advanced::ObstacleDetectionConfig obs_config;
    obs_config.max_detection_range = 8.0;
    obs_config.forward_distance_max = 6.0;
    obs_config.lateral_distance_max = 3.0;
    advanced_obstacle_detector_ = std::make_unique<advanced::ObstacleDetector>(obs_config);
    
    advanced::PathSelectionConfig sel_config;
    sel_config.commit_min_progress = 1.0;
    sel_config.commit_min_time_sec = 0.8;
    path_selector_ = std::make_unique<advanced::PathSelector>(sel_config);
    
    // Keep old detector for compatibility
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
        "/pf/pose/odom", 10, 
        std::bind(&LatticePlanner::odom_callback, this, std::placeholders::_1));
    
    laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10,
        std::bind(&LatticePlanner::laser_callback, this, std::placeholders::_1));
    
    grid_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", 10,
        std::bind(&LatticePlanner::grid_callback, this, std::placeholders::_1));
    
    // Initialize planning timer
    auto timer_period = std::chrono::milliseconds(static_cast<int>(1000.0 / planning_frequency));
    planning_timer_ = this->create_wall_timer(
        timer_period, std::bind(&LatticePlanner::planning_timer_callback, this));
    
    // Initialize reference path publishing timer (2 Hz)
    auto ref_path_timer_period = std::chrono::milliseconds(500); // 2 Hz = 500ms
    ref_path_timer_ = this->create_wall_timer(
        ref_path_timer_period, std::bind(&LatticePlanner::ref_path_timer_callback, this));
    
    // Publish reference path immediately
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
        // Resolve reference path file - similar to f1tenth_gym_ros approach
        std::string resolved_path;
        if (config_.reference_path_file.find('/') != std::string::npos) {
            // If full path is provided, use it directly
            resolved_path = config_.reference_path_file;
        } else {
            // Use package share directory for proper path resolution
            std::string pkg_share_dir = ament_index_cpp::get_package_share_directory("lattice_planner_pkg");
            resolved_path = pkg_share_dir + "/config/reference_paths/" + config_.reference_path_file + ".csv";
        }
        
        reference_path_ = SplineUtils::load_reference_path_from_csv(
            resolved_path, config_.path_resolution);
    }
    
    if (reference_path_.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Reference path is empty");
        return false;
    }
    
    RCLCPP_INFO(this->get_logger(), "Reference path loaded successfully");
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
    
    // Use new advanced obstacle detector
    advanced_obstacle_detector_->detectObstaclesFromScan(
        msg, vehicle_pos.x, vehicle_pos.y, vehicle_yaw, this->get_clock());
    
    // Keep old detector for compatibility
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
    
    // Update advanced obstacle detector with occupancy grid
    advanced_obstacle_detector_->updateOccupancyGrid(msg);
    
    // Keep old detector for compatibility
    auto grid_obstacles = obstacle_detector_->detect_from_occupancy_grid(msg, vehicle_pos);
    
    // Merge with laser obstacles (simple approach - replace for now)
    // In practice, you might want to fuse multiple obstacle sources
    if (current_obstacles_.empty()) {
        current_obstacles_ = grid_obstacles;
    }
}

void LatticePlanner::planning_timer_callback() {
    if (!odom_received_) {
        return; // Skip planning until odometry is available
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
        RCLCPP_WARN(this->get_logger(), "[DEBUG] No path candidates to select from");
        return PathCandidate();
    }
    
    RCLCPP_INFO(this->get_logger(), "[DEBUG] Starting path selection with %zu candidates", candidates.size());
    
    // Convert PathCandidate to CandidateResult for advanced path selection
    std::vector<advanced::CandidateResult> advanced_candidates;
    
    Point2D vehicle_pos;
    {
        std::lock_guard<std::mutex> state_lock(vehicle_state_mutex_);
        vehicle_pos = vehicle_position_;
    }
    
    for (size_t i = 0; i < candidates.size(); ++i) {
        const auto& candidate = candidates[i];
        advanced::CandidateResult result;
        result.cost = candidate.cost;
        result.collided = !candidate.is_safe;
        
        // Check if path is out of track (assuming F1TENTH track width ~3.5m)
        bool out_of_track = std::abs(candidate.lateral_offset) > 1.75; // Half track width
        result.out_of_track = out_of_track;
        result.d_offset = candidate.lateral_offset; // Set the actual lateral offset
        
        // Convert PathPoint to geometry_msgs::Point
        for (const auto& point : candidate.points) {
            geometry_msgs::msg::Point p;
            p.x = point.x;
            p.y = point.y;
            p.z = 0.0;
            result.path_points.push_back(p);
        }
        
        double original_cost = result.cost;
        
        // Enhanced cost calculation with obstacle detection
        if (!result.path_points.empty()) {
            double obstacle_cost = advanced_obstacle_detector_->calculateOccupancyCost(result.path_points);
            bool path_collides = advanced_obstacle_detector_->pathCollides(result.path_points);
            
            result.cost += obstacle_cost;
            result.collided = result.collided || path_collides;
            
            RCLCPP_INFO(this->get_logger(), "[DEBUG] Candidate %zu: original_cost=%.3f, obstacle_cost=%.3f, total_cost=%.3f, safe=%s, collides=%s, lateral_offset=%.3f", 
                       i, original_cost, obstacle_cost, result.cost, 
                       candidate.is_safe ? "true" : "false",
                       path_collides ? "true" : "false",
                       candidate.lateral_offset);
        }
        
        advanced_candidates.push_back(result);
    }
    
    // ENHANCED RACELINE PREFERENCE: Force raceline selection with relaxed collision check
    size_t raceline_candidate = std::numeric_limits<size_t>::max();
    bool found_safe_raceline = false;
    
    for (size_t i = 0; i < candidates.size(); ++i) {
        if (std::abs(candidates[i].lateral_offset) < 0.05) { // raceline candidate
            // Additional check: ensure raceline path has reasonable curvature for corners
            double max_curvature = 0.0;
            for (const auto& point : candidates[i].points) {
                max_curvature = std::max(max_curvature, std::abs(point.curvature));
            }
            
            // First priority: Safe raceline with reasonable curvature
            if (!advanced_candidates[i].collided && max_curvature < 0.8) {
                raceline_candidate = i;
                found_safe_raceline = true;
                RCLCPP_INFO(this->get_logger(), "[RACELINE FORCE] Safe raceline found: candidate %zu", raceline_candidate);
                break;
            }
            // Second priority: Raceline with collision but reasonable curvature (for tight corners)
            else if (!found_safe_raceline && max_curvature < 1.2) {
                raceline_candidate = i;
                RCLCPP_WARN(this->get_logger(), "[RACELINE FORCE] Collision raceline selected: candidate %zu (curvature=%.3f)", i, max_curvature);
            }
        }
    }
    
    // Force raceline selection even with collision in extreme cases
    if (raceline_candidate != std::numeric_limits<size_t>::max()) {
        if (found_safe_raceline) {
            RCLCPP_INFO(this->get_logger(), "[RACELINE FORCE] Using safe raceline: candidate %zu", raceline_candidate);
        } else {
            RCLCPP_WARN(this->get_logger(), "[RACELINE FORCE] Using collision raceline to prevent cutting: candidate %zu", raceline_candidate);
        }
        return candidates[raceline_candidate];
    }
    
    // Use advanced path selector
    bool has_obstacles = advanced_obstacle_detector_->hasObstacles(vehicle_pos.x, vehicle_pos.y);
    double current_s = 0.0; // TODO: calculate current arc length position
    
    RCLCPP_INFO(this->get_logger(), "[DEBUG] has_obstacles=%s, vehicle_pos=(%.2f, %.2f)", 
               has_obstacles ? "true" : "false", vehicle_pos.x, vehicle_pos.y);
    
    auto* selected = path_selector_->selectOptimalPath(
        advanced_candidates, has_obstacles, current_s, this->get_clock());
    
    if (selected) {
        path_selector_->updateCommitState(selected, current_s, this->get_clock());
        
        // Find corresponding original candidate
        for (size_t i = 0; i < candidates.size(); ++i) {
            if (&advanced_candidates[i] == selected) {
                RCLCPP_INFO(this->get_logger(), "[DEBUG] Selected advanced candidate %zu with cost=%.3f, collided=%s", 
                           i, selected->cost, selected->collided ? "true" : "false");
                return candidates[i];
            }
        }
    } else {
        RCLCPP_WARN(this->get_logger(), "[DEBUG] Advanced path selector returned null, falling back to simple selection");
    }
    
    // Fallback to simple cost-based selection
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

void LatticePlanner::ref_path_timer_callback() {
    // Continuously publish reference path at 2 Hz
    publish_reference_path();
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

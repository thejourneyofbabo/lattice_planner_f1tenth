#pragma once

#include "types.hpp"
#include "frenet_coordinate.hpp"
#include "path_generator.hpp"
#include "obstacle_detector.hpp"
#include "spline_utils.hpp"

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>


#include <memory>
#include <vector>
#include <mutex>

namespace lattice_planner_pkg {

class LatticePlanner : public rclcpp::Node {
public:
    LatticePlanner();
    ~LatticePlanner();

private:
    // ROS2 Publishers and Subscribers
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr ref_path_pub_;
    
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_sub_;
    
    rclcpp::TimerBase::SharedPtr planning_timer_;
    
    // Core modules
    PlannerConfig config_;
    std::shared_ptr<FrenetCoordinate> frenet_coord_;
    std::shared_ptr<PathGenerator> path_generator_;
    std::shared_ptr<ObstacleDetector> obstacle_detector_;
    
    // Vehicle state
    Point2D vehicle_position_;
    double vehicle_yaw_;
    double vehicle_velocity_;
    bool odom_received_;
    std::mutex vehicle_state_mutex_;
    
    // Obstacles
    std::vector<Obstacle> current_obstacles_;
    std::mutex obstacles_mutex_;
    
    // Reference path
    std::vector<RefPoint> reference_path_;
    
    // TF2
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
    
    // Callbacks
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void grid_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void planning_timer_callback();
    
    // Core functions
    bool initialize();
    bool load_reference_path();
    void plan_paths();
    PathCandidate select_best_path(const std::vector<PathCandidate>& candidates);
    
    // Publishing functions
    void publish_selected_path(const PathCandidate& path);
    void publish_path_visualization(const std::vector<PathCandidate>& candidates, 
                                   const PathCandidate& selected);
    void publish_reference_path();
    
    // Utility functions
    nav_msgs::msg::Path convert_to_nav_path(const PathCandidate& path);
    visualization_msgs::msg::MarkerArray create_path_markers(
        const std::vector<PathCandidate>& candidates,
        const PathCandidate& selected
    );
};

} // namespace lattice_planner_pkg
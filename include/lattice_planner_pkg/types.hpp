#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <vector>
#include <string>

namespace lattice_planner_pkg {

// Note: Advanced obstacle detection and path selection are in 
// lattice_planner_pkg::advanced namespace to avoid conflicts

// 설정 구조체
struct Config {
    // 파일 경로
    std::string csv_file_path;
    
    // 경로 생성 파라미터
    double ds;
    double lookahead_dist;
    double transition_length;
    
    // 차선 설정
    std::vector<double> d_offsets;
    int default_lane;
    std::vector<std::vector<float>> lane_colors;
    
    // 시각화 설정
    double line_width;
    float alpha;
    int lifetime_ms;
    bool smooth_transition;
    bool cubic_interpolation;
    
    // 속도 설정
    double base_velocity;
    
    // 발행 설정
    int timer_ms;
    int queue_size;
    
    // ROS 토픽 이름
    std::string odom_input;
    std::string marker_output;
    std::string path_prefix;
    std::string velocity_prefix;
    
    // 디버그 설정
    int log_throttle_ms;
    int warn_no_odom_ms;
    int info_ego_position_ms;
    
    // Spline 설정
    int min_points;
    double numerical_stability;
    bool natural_boundary;
    
    // 장애물 회피 설정
    std::string laser_scan_input;
    double obstacle_detection_range;
    double obstacle_avoidance_distance;
    double obstacle_slow_down_distance;
    double obstacle_cost_weight;
    double curvature_velocity_factor;
    double min_obstacle_velocity;
};

} // namespace lattice_planner_pkg
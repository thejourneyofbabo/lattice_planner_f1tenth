#pragma once

#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <string>

namespace lattice_planner_pkg {

// Frenet coordinate types
struct Point2D {
    double x, y;
    Point2D() : x(0.0), y(0.0) {}
    Point2D(double x_, double y_) : x(x_), y(y_) {}
};

struct FrenetPoint {
    double s;        // longitudinal position
    double d;        // lateral position  
    double s_dot;    // longitudinal velocity
    double d_dot;    // lateral velocity
    double s_ddot;   // longitudinal acceleration
    double d_ddot;   // lateral acceleration
    
    FrenetPoint() : s(0.0), d(0.0), s_dot(0.0), d_dot(0.0), s_ddot(0.0), d_ddot(0.0) {}
};

struct CartesianPoint {
    double x, y;     // position
    double yaw;      // heading angle
    double velocity; // speed
    double curvature;// path curvature
    double time;     // time from start
    
    CartesianPoint() : x(0.0), y(0.0), yaw(0.0), velocity(0.0), curvature(0.0), time(0.0) {}
};

// Reference path point
struct RefPoint {
    double x, y;
    double s;           // arc length
    double curvature;
    double heading;
    double velocity;    // reference velocity
    
    RefPoint() : x(0.0), y(0.0), s(0.0), curvature(0.0), heading(0.0), velocity(0.0) {}
};

// Obstacle information
struct Obstacle {
    double x, y;           // position
    double distance;       // distance from vehicle
    double angle;          // angle from vehicle
    rclcpp::Time timestamp; // detection time
    
    Obstacle() : x(0.0), y(0.0), distance(0.0), angle(0.0) {}
};

// Path candidate
struct PathCandidate {
    std::vector<CartesianPoint> points;
    double cost;
    double lateral_offset;
    bool is_safe;
    
    PathCandidate() : cost(0.0), lateral_offset(0.0), is_safe(true) {}
};

// Configuration structure
struct PlannerConfig {
    // Reference path
    std::string reference_path_file;
    double path_resolution;         // ds
    
    // Lattice sampling
    double lateral_step;           // d step size
    double max_lateral_offset;     // max |d|
    double planning_horizon;       // T planning time
    double dt;                     // time step
    
    // Vehicle parameters
    double wheelbase;
    double max_velocity;
    double max_acceleration;
    double max_curvature;
    
    // Costs
    double lateral_cost_weight;
    double longitudinal_cost_weight;
    double obstacle_cost_weight;
    double curvature_cost_weight;
    
    // Safety
    double safety_margin;
    double collision_radius;
    double obstacle_detection_range;
    
    PlannerConfig() {
        path_resolution = 0.1;
        lateral_step = 0.5;
        max_lateral_offset = 2.0;
        planning_horizon = 3.0;
        dt = 0.1;
        wheelbase = 0.33;
        max_velocity = 10.0;
        max_acceleration = 2.0;
        max_curvature = 1.0;
        lateral_cost_weight = 1.0;
        longitudinal_cost_weight = 1.0;
        obstacle_cost_weight = 10.0;
        curvature_cost_weight = 1.0;
        safety_margin = 0.3;
        collision_radius = 0.5;
        obstacle_detection_range = 10.0;
    }
};

} // namespace lattice_planner_pkg
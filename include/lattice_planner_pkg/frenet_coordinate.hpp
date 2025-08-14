#pragma once

#include "types.hpp"
#include <vector>
#include <memory>

namespace lattice_planner_pkg {

class FrenetCoordinate {
public:
    FrenetCoordinate();
    ~FrenetCoordinate();
    
    // Initialize with reference path
    bool initialize(const std::vector<RefPoint>& reference_path);
    
    // Convert Cartesian to Frenet
    FrenetPoint cartesian_to_frenet(const Point2D& cartesian_point) const;
    
    // Convert Frenet to Cartesian
    CartesianPoint frenet_to_cartesian(const FrenetPoint& frenet_point) const;
    
    // Get reference point at arc length s
    RefPoint get_reference_point(double s) const;
    
    // Get closest reference point to cartesian point
    int find_closest_reference_index(const Point2D& point) const;
    
    // Get total path length
    double get_total_path_length() const { return total_length_; }
    
    // Check if s is within valid range
    bool is_valid_s(double s) const;

private:
    std::vector<RefPoint> reference_path_;
    double total_length_;
    bool is_initialized_;
    
    // Helper functions
    double calculate_distance(const Point2D& p1, const Point2D& p2) const;
    double normalize_angle(double angle) const;
    RefPoint interpolate_reference_point(double s) const;
};

} // namespace lattice_planner_pkg
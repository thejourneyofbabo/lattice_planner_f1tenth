#pragma once

#include "types.hpp"
#include <vector>

namespace lattice_planner_pkg {

class SplineUtils {
public:
    SplineUtils();
    ~SplineUtils();
    
    // Load reference path from CSV file
    static std::vector<RefPoint> load_reference_path_from_csv(
        const std::string& file_path,
        double resolution = 0.1
    );
    
    // Calculate arc length for path points
    static void calculate_arc_length(std::vector<RefPoint>& path);
    
    // Calculate curvature for path points
    static void calculate_curvature(std::vector<RefPoint>& path);
    
    // Calculate heading for path points
    static void calculate_heading(std::vector<RefPoint>& path);
    
    // Smooth path using spline interpolation
    static std::vector<RefPoint> smooth_path(
        const std::vector<RefPoint>& raw_path,
        double resolution = 0.1
    );
    
    // Interpolate point at specific arc length
    static RefPoint interpolate_at_s(
        const std::vector<RefPoint>& path,
        double s
    );

private:
    // Helper functions
    static double calculate_distance(const Point2D& p1, const Point2D& p2);
    static double normalize_angle(double angle);
    static double calculate_curvature_at_point(
        const RefPoint& prev, const RefPoint& curr, const RefPoint& next
    );
};

} // namespace lattice_planner_pkg
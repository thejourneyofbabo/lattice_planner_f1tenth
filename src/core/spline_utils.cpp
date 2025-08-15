#include "lattice_planner_pkg/core/spline_utils.hpp"
#include <fstream>
#include <sstream>
#include <iostream>
#include <algorithm>
#include <cmath>

namespace lattice_planner_pkg {

SplineUtils::SplineUtils() {
}

SplineUtils::~SplineUtils() {
}

std::vector<RefPoint> SplineUtils::load_reference_path_from_csv(
    const std::string& file_path, double resolution) {
    
    std::vector<RefPoint> path;
    std::ifstream file(file_path);
    
    if (!file.is_open()) {
        std::cerr << "Failed to open CSV file: " << file_path << std::endl;
        return path;
    }
    
    std::string line;
    bool first_line = true;
    
    while (std::getline(file, line)) {
        // Skip header line
        if (first_line) {
            first_line = false;
            continue;
        }
        
        std::stringstream ss(line);
        std::string cell;
        std::vector<std::string> row;
        
        while (std::getline(ss, cell, ',')) {
            row.push_back(cell);
        }
        
        if (row.size() >= 3) {  // At least x, y, velocity
            RefPoint point;
            try {
                point.x = std::stod(row[0]);
                point.y = std::stod(row[1]);
                point.velocity = row.size() > 2 ? std::stod(row[2]) : 5.0;  // Default velocity
            } catch (const std::exception& e) {
                std::cerr << "Error parsing CSV line: " << line << std::endl;
                continue;
            }
            
            path.push_back(point);
        }
    }
    
    file.close();
    
    if (path.empty()) {
        std::cerr << "No valid points loaded from CSV file" << std::endl;
        return path;
    }
    
    // Calculate derived properties
    calculate_arc_length(path);
    calculate_heading(path);
    calculate_curvature(path);
    
    // Resample path if needed
    if (resolution > 0.0) {
        path = smooth_path(path, resolution);
    }
    
    std::cout << "Loaded " << path.size() << " reference points from " << file_path << std::endl;
    
    return path;
}

void SplineUtils::calculate_arc_length(std::vector<RefPoint>& path) {
    if (path.empty()) return;
    
    path[0].s = 0.0;
    
    for (size_t i = 1; i < path.size(); ++i) {
        double dx = path[i].x - path[i-1].x;
        double dy = path[i].y - path[i-1].y;
        double ds = std::sqrt(dx*dx + dy*dy);
        
        path[i].s = path[i-1].s + ds;
    }
}

void SplineUtils::calculate_heading(std::vector<RefPoint>& path) {
    if (path.size() < 2) return;
    
    // Calculate heading for each point
    for (size_t i = 0; i < path.size(); ++i) {
        if (i == 0) {
            // First point: use direction to next point
            double dx = path[i+1].x - path[i].x;
            double dy = path[i+1].y - path[i].y;
            path[i].heading = std::atan2(dy, dx);
        } else if (i == path.size() - 1) {
            // Last point: use direction from previous point
            double dx = path[i].x - path[i-1].x;
            double dy = path[i].y - path[i-1].y;
            path[i].heading = std::atan2(dy, dx);
        } else {
            // Middle points: average of directions
            double dx1 = path[i].x - path[i-1].x;
            double dy1 = path[i].y - path[i-1].y;
            double heading1 = std::atan2(dy1, dx1);
            
            double dx2 = path[i+1].x - path[i].x;
            double dy2 = path[i+1].y - path[i].y;
            double heading2 = std::atan2(dy2, dx2);
            
            // Handle angle wrapping
            double diff = heading2 - heading1;
            while (diff > M_PI) diff -= 2.0 * M_PI;
            while (diff < -M_PI) diff += 2.0 * M_PI;
            
            path[i].heading = normalize_angle(heading1 + diff * 0.5);
        }
    }
}

void SplineUtils::calculate_curvature(std::vector<RefPoint>& path) {
    if (path.size() < 3) return;
    
    for (size_t i = 1; i < path.size() - 1; ++i) {
        path[i].curvature = calculate_curvature_at_point(path[i-1], path[i], path[i+1]);
    }
    
    // Handle boundary points
    if (path.size() >= 3) {
        path[0].curvature = path[1].curvature;
        path[path.size()-1].curvature = path[path.size()-2].curvature;
    }
}

std::vector<RefPoint> SplineUtils::smooth_path(
    const std::vector<RefPoint>& raw_path, double resolution) {
    
    if (raw_path.size() < 2) {
        return raw_path;
    }
    
    std::vector<RefPoint> smoothed_path;
    double total_length = raw_path.back().s;
    
    // Resample at uniform intervals
    for (double s = 0.0; s <= total_length; s += resolution) {
        RefPoint interpolated = interpolate_at_s(raw_path, s);
        smoothed_path.push_back(interpolated);
    }
    
    // Ensure we include the last point
    if (!smoothed_path.empty() && 
        std::abs(smoothed_path.back().s - total_length) > resolution * 0.5) {
        smoothed_path.push_back(raw_path.back());
    }
    
    return smoothed_path;
}

RefPoint SplineUtils::interpolate_at_s(const std::vector<RefPoint>& path, double s) {
    if (path.empty()) {
        return RefPoint();
    }
    
    if (s <= path[0].s) {
        return path[0];
    }
    
    if (s >= path.back().s) {
        return path.back();
    }
    
    // Find surrounding points
    size_t i = 0;
    for (i = 0; i < path.size() - 1; ++i) {
        if (path[i + 1].s > s) {
            break;
        }
    }
    
    if (i >= path.size() - 1) {
        return path.back();
    }
    
    // Linear interpolation
    const RefPoint& p1 = path[i];
    const RefPoint& p2 = path[i + 1];
    
    double ratio = (s - p1.s) / (p2.s - p1.s);
    ratio = std::max(0.0, std::min(1.0, ratio));
    
    RefPoint interpolated;
    interpolated.x = p1.x + ratio * (p2.x - p1.x);
    interpolated.y = p1.y + ratio * (p2.y - p1.y);
    interpolated.s = s;
    interpolated.velocity = p1.velocity + ratio * (p2.velocity - p1.velocity);
    interpolated.heading = p1.heading + ratio * normalize_angle(p2.heading - p1.heading);
    interpolated.curvature = p1.curvature + ratio * (p2.curvature - p1.curvature);
    
    return interpolated;
}

double SplineUtils::calculate_distance(const Point2D& p1, const Point2D& p2) {
    double dx = p1.x - p2.x;
    double dy = p1.y - p2.y;
    return std::sqrt(dx * dx + dy * dy);
}

double SplineUtils::normalize_angle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

double SplineUtils::calculate_curvature_at_point(
    const RefPoint& prev, const RefPoint& curr, const RefPoint& next) {
    
    // Use finite differences to approximate curvature
    double dx1 = curr.x - prev.x;
    double dy1 = curr.y - prev.y;
    double dx2 = next.x - curr.x;
    double dy2 = next.y - curr.y;
    
    double cross_product = dx1 * dy2 - dy1 * dx2;
    double ds1 = std::sqrt(dx1*dx1 + dy1*dy1);
    double ds2 = std::sqrt(dx2*dx2 + dy2*dy2);
    
    if (ds1 < 1e-6 || ds2 < 1e-6) {
        return 0.0;
    }
    
    // Approximate curvature
    double curvature = 2.0 * cross_product / (ds1 * ds2 * (ds1 + ds2));
    
    return curvature;
}

} // namespace lattice_planner_pkg
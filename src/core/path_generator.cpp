#include "lattice_planner_pkg/core/path_generator.hpp"
#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <algorithm>

namespace lattice_planner_pkg {

PathGenerator::PathGenerator(const PlannerConfig& config) 
    : config_(config) {
}

PathGenerator::~PathGenerator() {
}

void PathGenerator::set_frenet_coordinate(std::shared_ptr<FrenetCoordinate> frenet_coord) {
    frenet_coord_ = frenet_coord;
}

std::vector<PathCandidate> PathGenerator::generate_paths(
    const Point2D& vehicle_position,
    double vehicle_yaw,
    double vehicle_velocity,
    const std::vector<Obstacle>& obstacles) {
    
    std::vector<PathCandidate> candidates;
    
    if (!frenet_coord_) {
        return candidates;
    }
    
    // Convert vehicle position to Frenet coordinates
    FrenetPoint start_frenet = frenet_coord_->cartesian_to_frenet(vehicle_position);
    
    // If vehicle is too far from reference path, find closest point and project onto it
    int closest_ref_idx = frenet_coord_->find_closest_reference_index(vehicle_position);
    if (closest_ref_idx >= 0) {
        RefPoint closest_ref = frenet_coord_->get_reference_point(start_frenet.s);
        double distance_to_ref = std::sqrt(
            std::pow(vehicle_position.x - closest_ref.x, 2) + 
            std::pow(vehicle_position.y - closest_ref.y, 2)
        );
        
        // If too far (>5m), project vehicle onto reference path
        if (distance_to_ref > 5.0) {
            start_frenet.s = closest_ref.s;
            start_frenet.d = 0.0;  // Start from reference line
            RCLCPP_WARN_ONCE(rclcpp::get_logger("path_generator"), 
                "Vehicle too far from reference path (%.1fm), projecting to reference line", 
                distance_to_ref);
        }
    }
    
    // Generate lateral offset samples
    std::vector<double> lateral_samples = generate_lateral_samples();
    
    // Generate velocity samples  
    std::vector<double> velocity_samples = generate_velocity_samples(vehicle_velocity);
    
    // Generate paths for each combination
    for (double lateral_offset : lateral_samples) {
        for (double target_velocity : velocity_samples) {
            PathCandidate candidate = generate_single_path(
                start_frenet, lateral_offset, target_velocity);
                
            if (!candidate.points.empty()) {
                // Calculate cost and check collision
                candidate.cost = calculate_path_cost(candidate, obstacles);
                candidate.is_safe = !check_collision(candidate, obstacles);
                candidate.lateral_offset = lateral_offset;
                
                candidates.push_back(candidate);
            }
        }
    }
    
    return candidates;
}

PathCandidate PathGenerator::generate_single_path(
    const FrenetPoint& start_frenet,
    double target_lateral_offset,
    double target_velocity) {
    
    PathCandidate candidate;
    
    if (!frenet_coord_) {
        return candidate;
    }
    
    // Generate time samples
    std::vector<double> time_samples = generate_time_samples();
    
    for (size_t i = 0; i < time_samples.size(); ++i) {
        double t = time_samples[i];
        
        // Simple linear interpolation for lateral motion
        double d = start_frenet.d + (target_lateral_offset - start_frenet.d) * (t / config_.planning_horizon);
        
        // Constant velocity longitudinal motion
        double s = start_frenet.s + target_velocity * t;
        
        // Create Frenet point
        FrenetPoint frenet_point;
        frenet_point.s = s;
        frenet_point.d = d;
        frenet_point.s_dot = target_velocity;
        frenet_point.d_dot = 0.0;
        frenet_point.s_ddot = 0.0;
        frenet_point.d_ddot = 0.0;
        
        // Convert to Cartesian
        CartesianPoint cartesian_point = frenet_coord_->frenet_to_cartesian(frenet_point);
        cartesian_point.time = t;
        cartesian_point.velocity = target_velocity;
        
        // Calculate curvature (simplified)
        if (i > 0) {
            const CartesianPoint& prev = candidate.points[i-1];
            double dx = cartesian_point.x - prev.x;
            double dy = cartesian_point.y - prev.y;
            double ds = std::sqrt(dx*dx + dy*dy);
            
            if (ds > 1e-6) {
                double dyaw = cartesian_point.yaw - prev.yaw;
                // Normalize angle difference
                while (dyaw > M_PI) dyaw -= 2.0 * M_PI;
                while (dyaw < -M_PI) dyaw += 2.0 * M_PI;
                
                cartesian_point.curvature = std::abs(dyaw / ds);
            }
        }
        
        candidate.points.push_back(cartesian_point);
    }
    
    return candidate;
}

std::vector<double> PathGenerator::generate_time_samples() const {
    std::vector<double> samples;
    
    int num_samples = static_cast<int>(config_.planning_horizon / config_.dt) + 1;
    
    for (int i = 0; i < num_samples; ++i) {
        samples.push_back(i * config_.dt);
    }
    
    return samples;
}

std::vector<double> PathGenerator::generate_lateral_samples() const {
    std::vector<double> samples;
    
    int num_steps = static_cast<int>(2.0 * config_.max_lateral_offset / config_.lateral_step) + 1;
    
    for (int i = 0; i < num_steps; ++i) {
        double offset = -config_.max_lateral_offset + i * config_.lateral_step;
        samples.push_back(offset);
    }
    
    return samples;
}

std::vector<double> PathGenerator::generate_velocity_samples(double current_velocity) const {
    std::vector<double> samples;
    
    // Ensure minimum velocity for planning
    double min_planning_vel = 2.0;  // Minimum 2 m/s for reasonable path length
    double effective_velocity = std::max(min_planning_vel, current_velocity);
    
    // Generate velocity samples around effective velocity
    double min_vel = std::max(min_planning_vel, effective_velocity - 1.0);
    double max_vel = std::min(config_.max_velocity, effective_velocity + 3.0);
    
    int num_samples = 3;  // Reduce samples for performance
    if (num_samples == 1) {
        samples.push_back((min_vel + max_vel) / 2.0);
    } else {
        double vel_step = (max_vel - min_vel) / (num_samples - 1);
        for (int i = 0; i < num_samples; ++i) {
            samples.push_back(min_vel + i * vel_step);
        }
    }
    
    return samples;
}

double PathGenerator::calculate_path_cost(
    const PathCandidate& path,
    const std::vector<Obstacle>& obstacles) {
    
    if (path.points.empty()) {
        return std::numeric_limits<double>::max();
    }
    
    double cost = 0.0;
    
    // Lateral deviation cost
    double lateral_cost = std::abs(path.lateral_offset) * config_.lateral_cost_weight;
    cost += lateral_cost;
    
    // Curvature cost
    double curvature_cost = 0.0;
    for (const auto& point : path.points) {
        curvature_cost += point.curvature * point.curvature;
    }
    curvature_cost *= config_.curvature_cost_weight;
    cost += curvature_cost;
    
    // Obstacle cost - only if obstacles exist
    double obstacle_cost = 0.0;
    if (!obstacles.empty()) {
        for (const auto& point : path.points) {
            for (const auto& obstacle : obstacles) {
                double dx = point.x - obstacle.x;
                double dy = point.y - obstacle.y;
                double distance = std::sqrt(dx*dx + dy*dy);
                
                if (distance < config_.obstacle_detection_range) {
                    obstacle_cost += config_.obstacle_cost_weight / (distance + 0.1);
                }
            }
        }
    }
    cost += obstacle_cost;
    
    // Velocity cost (prefer maintaining speed)
    double velocity_cost = 0.0;
    for (const auto& point : path.points) {
        double vel_diff = point.velocity - config_.max_velocity * 0.8;  // Prefer 80% of max velocity
        velocity_cost += vel_diff * vel_diff;
    }
    velocity_cost *= config_.longitudinal_cost_weight * 0.01;
    cost += velocity_cost;
    
    return cost;
}

bool PathGenerator::check_collision(
    const PathCandidate& path,
    const std::vector<Obstacle>& obstacles) {
    
    // If no obstacles, path is safe
    if (obstacles.empty()) {
        return false;
    }
    
    for (const auto& point : path.points) {
        for (const auto& obstacle : obstacles) {
            double dx = point.x - obstacle.x;
            double dy = point.y - obstacle.y;
            double distance = std::sqrt(dx*dx + dy*dy);
            
            if (distance < config_.collision_radius) {
                return true;  // Collision detected
            }
        }
    }
    
    return false;  // No collision
}

std::vector<double> PathGenerator::solve_quintic_polynomial(
    double start_pos, double start_vel, double start_acc,
    double end_pos, double end_vel, double end_acc,
    double time_duration) {
    
    // Simplified quintic polynomial coefficients
    // This is a placeholder - in practice you'd solve the full system
    std::vector<double> coeffs(6, 0.0);
    
    double T = time_duration;
    double T2 = T * T;
    double T3 = T2 * T;
    double T4 = T3 * T;
    double T5 = T4 * T;
    
    coeffs[0] = start_pos;
    coeffs[1] = start_vel;
    coeffs[2] = start_acc / 2.0;
    
    // Solve for remaining coefficients (simplified)
    double delta_pos = end_pos - start_pos - start_vel * T - start_acc * T2 / 2.0;
    coeffs[3] = (20.0 * delta_pos - (8.0 * end_vel + 12.0 * start_vel) * T - 
                (3.0 * start_acc - end_acc) * T2) / (2.0 * T3);
    coeffs[4] = (-30.0 * delta_pos + (14.0 * end_vel + 16.0 * start_vel) * T + 
                (3.0 * start_acc - 2.0 * end_acc) * T2) / (2.0 * T4);
    coeffs[5] = (12.0 * delta_pos - 6.0 * (end_vel + start_vel) * T - 
                (start_acc - end_acc) * T2) / (2.0 * T5);
    
    return coeffs;
}

std::vector<double> PathGenerator::solve_quartic_polynomial(
    double start_pos, double start_vel, double start_acc,
    double end_vel, double time_duration) {
    
    // Simplified quartic polynomial - placeholder
    std::vector<double> coeffs(5, 0.0);
    
    coeffs[0] = start_pos;
    coeffs[1] = start_vel;
    coeffs[2] = start_acc / 2.0;
    
    // Simple linear velocity change
    double T = time_duration;
    coeffs[3] = (end_vel - start_vel - start_acc * T) / (T * T);
    
    return coeffs;
}

} // namespace lattice_planner_pkg
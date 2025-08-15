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
    
    // Get reference point at current s position to align with vehicle heading
    RefPoint ref_point = frenet_coord_->get_reference_point(start_frenet.s);
    double ref_heading = ref_point.heading;
    
    // Calculate heading difference between vehicle and reference path
    double heading_diff = vehicle_yaw - ref_heading;
    while (heading_diff > M_PI) heading_diff -= 2.0 * M_PI;
    while (heading_diff < -M_PI) heading_diff += 2.0 * M_PI;
    
    // If vehicle heading is significantly different from reference path, 
    // adjust the starting frenet coordinates to ensure smooth transition
    if (std::abs(heading_diff) > M_PI/6) {  // More than 30 degrees difference
        // Project vehicle's forward direction onto frenet frame
        double forward_s_component = vehicle_velocity * std::cos(heading_diff);
        double forward_d_component = vehicle_velocity * std::sin(heading_diff);
        
        // Initialize starting derivatives based on vehicle's actual motion
        start_frenet.s_dot = std::max(2.0, std::abs(forward_s_component));  // Ensure forward motion
        start_frenet.d_dot = forward_d_component * 0.3;  // Smooth lateral transition
        
        RCLCPP_INFO_ONCE(rclcpp::get_logger("path_generator"), 
            "Vehicle heading differs from reference by %.1f degrees, adjusting start derivatives", 
            heading_diff * 180.0 / M_PI);
    } else {
        // Vehicle is well aligned with reference path
        start_frenet.s_dot = std::max(2.0, vehicle_velocity);  // Ensure minimum forward speed
        start_frenet.d_dot = 0.0;  // No lateral motion if well aligned
    }
    
    // If vehicle is too far from reference path, gradually guide it back
    int closest_ref_idx = frenet_coord_->find_closest_reference_index(vehicle_position);
    if (closest_ref_idx >= 0) {
        RefPoint closest_ref = frenet_coord_->get_reference_point(start_frenet.s);
        double distance_to_ref = std::sqrt(
            std::pow(vehicle_position.x - closest_ref.x, 2) + 
            std::pow(vehicle_position.y - closest_ref.y, 2)
        );
        
        // If too far (>3m), bias the path generation towards reference line
        if (distance_to_ref > 3.0) {
            RCLCPP_WARN(rclcpp::get_logger("path_generator"), 
                "Vehicle far from reference path (%.1fm), biasing towards center", 
                distance_to_ref);
        }
    }
    
    // Generate lateral offset samples centered around vehicle's current position
    std::vector<double> lateral_samples = generate_lateral_samples(start_frenet.d);
    
    // Generate velocity samples  
    std::vector<double> velocity_samples = generate_velocity_samples(vehicle_velocity);
    
    // Generate paths for each combination
    for (double lateral_offset : lateral_samples) {
        for (double target_velocity : velocity_samples) {
            PathCandidate candidate = generate_single_path(
                start_frenet, lateral_offset, target_velocity);
                
            if (!candidate.points.empty()) {
                // Set lateral offset BEFORE cost calculation
                candidate.lateral_offset = lateral_offset;
                
                // Calculate cost and check collision
                candidate.cost = calculate_path_cost(candidate, obstacles);
                candidate.is_safe = !check_collision(candidate, obstacles);
                
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
    
    // Debug: Log path generation parameters
    static int path_count = 0;
    if (path_count < 10) {  // Only log first 10 paths to avoid spam
        RCLCPP_INFO(rclcpp::get_logger("path_generator"), 
            "[PATH %d] start_d=%.3f -> target_d=%.3f, start_s=%.3f, vel=%.3f", 
            path_count++, start_frenet.d, target_lateral_offset, start_frenet.s, target_velocity);
    }
    
    // Generate time samples
    std::vector<double> time_samples = generate_time_samples();
    
    // Use quintic polynomial for smooth lateral motion
    double T = config_.planning_horizon;
    
    // Check reference path curvature ahead for adaptive planning
    RefPoint current_ref = frenet_coord_->get_reference_point(start_frenet.s);
    RefPoint ahead_ref = frenet_coord_->get_reference_point(start_frenet.s + T * start_frenet.s_dot);
    double avg_curvature = (std::abs(current_ref.curvature) + std::abs(ahead_ref.curvature)) / 2.0;
    
    // Adaptive target adjustment for corners
    double conservative_factor = 1.0;
    if (avg_curvature > 0.3) { // High curvature corner
        conservative_factor = 0.6; // More conservative - stick closer to raceline
        static rclcpp::Clock clock;
        RCLCPP_INFO_THROTTLE(rclcpp::get_logger("path_generator"), clock, 1000,
            "[CORNER MODE] High curvature %.3f detected, applying conservative factor %.1f", 
            avg_curvature, conservative_factor);
    } else if (avg_curvature > 0.15) { // Medium curvature
        conservative_factor = 0.8;
    }
    
    // Apply conservative factor to lateral offset
    double adjusted_target_lateral_offset = target_lateral_offset * conservative_factor;
    
    // Lateral motion parameters
    double d_start = start_frenet.d;
    double d_dot_start = start_frenet.d_dot;
    double d_ddot_start = 0.0;  // Assume zero initial lateral acceleration
    
    double d_end = adjusted_target_lateral_offset;
    double d_dot_end = 0.0;     // Target zero lateral velocity at end
    double d_ddot_end = 0.0;    // Target zero lateral acceleration at end
    
    // Solve quintic polynomial for lateral motion
    std::vector<double> lateral_coeffs = solve_quintic_polynomial(
        d_start, d_dot_start, d_ddot_start,
        d_end, d_dot_end, d_ddot_end, T);
    
    // Longitudinal motion parameters
    double s_start = start_frenet.s;
    double s_dot_start = start_frenet.s_dot;
    double s_ddot_start = 0.0;
    
    // Simple constant velocity longitudinal motion (can be improved with quartic polynomial)
    double target_s_dot = target_velocity;
    
    for (size_t i = 0; i < time_samples.size(); ++i) {
        double t = time_samples[i];
        
        // Calculate lateral position using quintic polynomial
        double d = 0.0;
        double d_dot = 0.0;
        double d_ddot = 0.0;
        
        if (lateral_coeffs.size() >= 6) {
            d = lateral_coeffs[0] + lateral_coeffs[1]*t + lateral_coeffs[2]*t*t + 
                lateral_coeffs[3]*t*t*t + lateral_coeffs[4]*t*t*t*t + lateral_coeffs[5]*t*t*t*t*t;
            d_dot = lateral_coeffs[1] + 2*lateral_coeffs[2]*t + 3*lateral_coeffs[3]*t*t + 
                    4*lateral_coeffs[4]*t*t*t + 5*lateral_coeffs[5]*t*t*t*t;
            d_ddot = 2*lateral_coeffs[2] + 6*lateral_coeffs[3]*t + 
                     12*lateral_coeffs[4]*t*t + 20*lateral_coeffs[5]*t*t*t;
        } else {
            // Fallback to linear interpolation if polynomial solving fails
            d = d_start + (d_end - d_start) * (t / T);
            d_dot = (d_end - d_start) / T;
            d_ddot = 0.0;
        }
        
        // Longitudinal motion with smooth acceleration
        double s = s_start + s_dot_start * t + 0.5 * (target_s_dot - s_dot_start) / T * t * t;
        double s_dot = s_dot_start + (target_s_dot - s_dot_start) * (t / T);
        double s_ddot = (target_s_dot - s_dot_start) / T;
        
        // Create Frenet point
        FrenetPoint frenet_point;
        frenet_point.s = s;
        frenet_point.d = d;
        frenet_point.s_dot = s_dot;
        frenet_point.d_dot = d_dot;
        frenet_point.s_ddot = s_ddot;
        frenet_point.d_ddot = d_ddot;
        
        // Convert to Cartesian
        CartesianPoint cartesian_point = frenet_coord_->frenet_to_cartesian(frenet_point);
        cartesian_point.time = t;
        cartesian_point.velocity = s_dot;  // Use longitudinal velocity
        
        // Calculate curvature more accurately
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
                
                // Limit maximum curvature for vehicle dynamics
                if (cartesian_point.curvature > config_.max_curvature) {
                    cartesian_point.curvature = config_.max_curvature;
                }
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

std::vector<double> PathGenerator::generate_lateral_samples(double current_d) const {
    std::vector<double> samples;
    
    // Calculate the range of lateral offsets centered around current position
    // The center lattice should target the raceline (d=0), others should be parallel
    
    // CORNER OPTIMIZATION: Reduce lattice range to prevent track departure
    double effective_max_offset = config_.max_lateral_offset;
    
    // Check current curvature to detect corners
    double current_s = 0.0; // This will be set from vehicle position
    if (frenet_coord_) {
        // Get current curvature from reference path
        RefPoint ref_point = frenet_coord_->get_reference_point(current_s);
        double current_curvature = std::abs(ref_point.curvature);
        
        // In high curvature (corners), limit lateral range to stay within track
        if (current_curvature > 0.3) {  // Corner detected
            effective_max_offset = std::min(config_.max_lateral_offset, 1.2);  // Limit to ±1.2m - less restrictive
            RCLCPP_INFO_ONCE(rclcpp::get_logger("path_generator"), 
                "[CORNER LATTICE] Limiting lateral range to ±%.1fm for high curvature %.3f", 
                effective_max_offset, current_curvature);
        }
    }
    
    int num_steps = static_cast<int>(2.0 * effective_max_offset / config_.lateral_step) + 1;
    
    // Create lattice paths that transition from current position to target positions
    // Center lattice targets raceline (d=0), others are offset from raceline
    double half_range = effective_max_offset;
    
    for (int i = 0; i < num_steps; ++i) {
        // Target lateral positions relative to raceline
        double target_offset = -half_range + i * config_.lateral_step;
        samples.push_back(target_offset);
    }
    
    // Sort by distance from raceline to prioritize center paths
    std::sort(samples.begin(), samples.end(), [](double a, double b) {
        return std::abs(a) < std::abs(b);
    });
    
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
    
    // CORNER OPTIMIZATION: Reduce velocity samples in corners
    int num_samples = 3;  // Default samples
    
    // Check current curvature to detect corners
    double current_s = 0.0; // This will be set from vehicle position
    if (frenet_coord_) {
        RefPoint ref_point = frenet_coord_->get_reference_point(current_s);
        double current_curvature = std::abs(ref_point.curvature);
        
        // In high curvature (corners), reduce velocity samples for performance
        if (current_curvature > 0.3) {  // Corner detected
            num_samples = 2;  // Reduce to 2 samples in corners
            RCLCPP_INFO_ONCE(rclcpp::get_logger("path_generator"), 
                "[CORNER LATTICE] Reducing velocity samples to %d for high curvature %.3f", 
                num_samples, current_curvature);
        }
    }
    
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
    
    // Strong raceline preference but not extreme
    if (std::abs(path.lateral_offset) < 0.05) {
        // Almost raceline - give good bonus
        cost = 0.0; // Start with zero cost for raceline
    } else {
        // Lateral deviation cost - moderate exponential penalty
        double lateral_cost = std::pow(path.lateral_offset, 2) * config_.lateral_cost_weight * 5.0;
        cost += lateral_cost;
    }
    
    // Curvature cost
    double curvature_cost = 0.0;
    for (const auto& point : path.points) {
        curvature_cost += point.curvature * point.curvature;
    }
    curvature_cost *= config_.curvature_cost_weight;
    cost += curvature_cost;
    
    // Debug: Log cost breakdown for all paths with enhanced detail
    static int cost_log_count = 0;
    if (cost_log_count < 10 && (std::abs(path.lateral_offset) < 0.1 || std::abs(path.lateral_offset) > 0.3)) {
        double lateral_cost = (std::abs(path.lateral_offset) < 0.05) ? 0.0 : std::pow(path.lateral_offset, 4) * config_.lateral_cost_weight;
        RCLCPP_INFO(rclcpp::get_logger("path_generator"), 
            "[COST DEBUG] lateral_offset=%.3f: lateral_cost=%.1f (%.3f^4 * %.1f), curvature_cost=%.1f, total=%.1f %s", 
            path.lateral_offset, lateral_cost, path.lateral_offset, config_.lateral_cost_weight, curvature_cost, cost,
            (std::abs(path.lateral_offset) < 0.05) ? "*** RACELINE ***" : "");
        cost_log_count++;
    }
    
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
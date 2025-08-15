#pragma once

#include "lattice_planner_pkg/core/types.hpp"
#include "lattice_planner_pkg/core/frenet_coordinate.hpp"
#include <vector>
#include <memory>

namespace lattice_planner_pkg {

class PathGenerator {
public:
    PathGenerator(const PlannerConfig& config);
    ~PathGenerator();
    
    // Set frenet coordinate system
    void set_frenet_coordinate(std::shared_ptr<FrenetCoordinate> frenet_coord);
    
    // Generate lattice paths from current vehicle state
    std::vector<PathCandidate> generate_paths(
        const Point2D& vehicle_position,
        double vehicle_yaw,
        double vehicle_velocity,
        const std::vector<Obstacle>& obstacles
    );
    
    // Generate single path with specific lateral offset
    PathCandidate generate_single_path(
        const FrenetPoint& start_frenet,
        double target_lateral_offset,
        double target_velocity
    );

private:
    PlannerConfig config_;
    std::shared_ptr<FrenetCoordinate> frenet_coord_;
    
    // Quintic polynomial for lateral planning
    std::vector<double> solve_quintic_polynomial(
        double start_pos, double start_vel, double start_acc,
        double end_pos, double end_vel, double end_acc,
        double time_duration
    );
    
    // Quartic polynomial for longitudinal planning
    std::vector<double> solve_quartic_polynomial(
        double start_pos, double start_vel, double start_acc,
        double end_vel, double time_duration
    );
    
    // Calculate path cost
    double calculate_path_cost(
        const PathCandidate& path,
        const std::vector<Obstacle>& obstacles
    );
    
    // Check path collision with obstacles
    bool check_collision(
        const PathCandidate& path,
        const std::vector<Obstacle>& obstacles
    );
    
    // Generate time samples
    std::vector<double> generate_time_samples() const;
    
    // Generate lateral offset samples relative to current position
    std::vector<double> generate_lateral_samples(double current_d) const;
    
    // Generate velocity samples
    std::vector<double> generate_velocity_samples(double current_velocity) const;
};

} // namespace lattice_planner_pkg
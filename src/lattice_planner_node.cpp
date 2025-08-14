#include "lattice_planner_pkg/lattice_planner.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    try {
        auto node = std::make_shared<lattice_planner_pkg::LatticePlanner>();
        RCLCPP_INFO(node->get_logger(), "Lattice Planner Node Started");
        
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("lattice_planner"), "Exception: %s", e.what());
    }
    
    rclcpp::shutdown();
    return 0;
}
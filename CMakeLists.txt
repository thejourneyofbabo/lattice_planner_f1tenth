cmake_minimum_required(VERSION 3.8)
project(lattice_planner_pkg)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Generate custom messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PathPoint.msg"
  "msg/PathPointArray.msg"
  DEPENDENCIES builtin_interfaces std_msgs nav_msgs
)

# Lattice Planner Node (reorganized structure)
add_executable(lattice_planner_node 
  src/planner/lattice_planner_node.cpp
  src/planner/lattice_planner.cpp
  src/core/frenet_coordinate.cpp
  src/core/path_generator.cpp
  src/core/obstacle_detector.cpp
  src/core/spline_utils.cpp
)

ament_target_dependencies(lattice_planner_node
  rclcpp 
  nav_msgs 
  geometry_msgs 
  std_msgs
  visualization_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# Link custom messages
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(lattice_planner_node ${cpp_typesupport_target})

# Add dependencies to ensure messages are built before the executable
add_dependencies(lattice_planner_node ${PROJECT_NAME})

# Include directories for organized structure
target_include_directories(lattice_planner_node
  PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Install targets
install(TARGETS 
        lattice_planner_node
        DESTINATION lib/${PROJECT_NAME})

# Install header files
install(DIRECTORY include/
        DESTINATION include/)

# Install config files
install(DIRECTORY config/
        DESTINATION share/${PROJECT_NAME}/config)

# Install launch files
install(DIRECTORY launch/
        DESTINATION share/${PROJECT_NAME}/launch)

# Export runtime dependencies
ament_export_dependencies(rosidl_default_runtime)

ament_package()
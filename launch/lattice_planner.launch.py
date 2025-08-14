from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('lattice_planner_pkg')
    
    # Config file path
    config_file = os.path.join(pkg_dir, 'config', 'planner_config.yaml')
    
    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )
    
    # Lattice planner node
    lattice_planner_node = Node(
        package='lattice_planner_pkg',
        executable='lattice_planner_node',
        name='lattice_planner',
        output='screen',
        parameters=[
            config_file,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        remappings=[
            # Remap topics as needed
            ('/odom', '/odom'),
            ('/scan', '/scan'),
            ('/map', '/map'),
        ]
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        lattice_planner_node,
    ])
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Get package share directory
    pkg_share = FindPackageShare('lattice_planner_pkg')
    
    # Path to config file
    config_file = PathJoinSubstitution([
        pkg_share, 
        'config', 
        'simple_config.yaml'
    ])
    
    # Simple lattice controller node
    simple_lattice_node = Node(
        package='lattice_planner_pkg',
        executable='simple_lattice_planner',
        name='simple_lattice_controller',
        parameters=[config_file],
        output='screen',
        emulate_tty=True
    )
    
    return LaunchDescription([
        simple_lattice_node
    ])
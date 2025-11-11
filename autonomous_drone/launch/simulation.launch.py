"""Launch file for autonomous drone simulation"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os


def generate_launch_description():
    """Generate launch description for full simulation"""
    
    # Declare arguments
    vehicle_arg = DeclareLaunchArgument(
        'vehicle',
        default_value='iris',
        description='Vehicle type to simulate'
    )
    
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='empty',
        description='Gazebo world to load'
    )
    
    # PX4 SITL
    px4_sitl = ExecuteProcess(
        cmd=[
            'bash', '-c',
            'cd /root/PX4-Autopilot && '
            'HEADLESS=1 make px4_sitl gazebo-classic'
        ],
        output='screen',
        name='px4_sitl'
    )
    
    # MAVROS node
    mavros_node = Node(
        package='mavros',
        executable='mavros_node',
        name='mavros',
        output='screen',
        parameters=[{
            'fcu_url': 'udp://:14540@127.0.0.1:14557',
            'gcs_url': '',
            'target_system_id': 1,
            'target_component_id': 1,
            'fcu_protocol': 'v2.0',
            'system_id': 255,
            'component_id': 240,
        }],
        respawn=True
    )
    
    # Waypoint navigator node
    waypoint_navigator = Node(
        package='autonomous_drone',
        executable='waypoint_navigator.py',
        name='waypoint_navigator',
        output='screen'
    )
    
    # RViz for visualization
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('autonomous_drone'),
        'config',
        'drone_view.rviz'
    ])
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )
    
    return LaunchDescription([
        vehicle_arg,
        world_arg,
        px4_sitl,
        mavros_node,
        waypoint_navigator,
        rviz_node,
    ])

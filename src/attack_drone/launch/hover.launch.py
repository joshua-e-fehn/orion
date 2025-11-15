#!/usr/bin/env python3

"""
Launch file for Orion autonomous hover flight with RViz visualization

This launch file starts:
1. The hover control node
2. RViz2 with pre-configured visualization
3. Static TF publishers for coordinate frames
4. Optional: micro-ros-agent for PX4 communication

Usage:
    ros2 launch orion_flight hover.launch.py
    
Optional parameters:
    flight_height:=5.0       # Flight height in meters (positive value)
    trail_length:=10         # Trail visualization length
    use_micro_ros_agent:=false  # Start micro-ros-agent
    
Author: Orion ARM Team - Joshua Fehn
Date: November 15, 2025
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    
    # Get package directories
    pkg_orion_flight = get_package_share_directory('orion_flight')
    
    # Path to RViz config file
    rviz_config_file = os.path.join(pkg_orion_flight, 'config', 'hover.rviz')
    
    # Declare launch arguments
    flight_height_arg = DeclareLaunchArgument(
        'flight_height',
        default_value='5.0',
        description='Flight height above ground in meters (positive value, will be converted to NED)'
    )
    
    trail_length_arg = DeclareLaunchArgument(
        'trail_length',
        default_value='10',
        description='Number of position markers to display in trail'
    )
    
    use_micro_ros_agent_arg = DeclareLaunchArgument(
        'use_micro_ros_agent',
        default_value='false',
        description='Launch micro-ros-agent for PX4 communication'
    )
    
    # Micro-ROS Agent (optional, for PX4 communication)
    # Note: PX4 v1.14+ has built-in uXRCE-DDS, so this is usually not needed
    micro_ros_agent = ExecuteProcess(
        condition=IfCondition(LaunchConfiguration('use_micro_ros_agent')),
        cmd=['micro-ros-agent', 'udp4', '--port', '8888', '-v'],
        shell=False,
        output='screen'
    )
    
    # Circle Trajectory Control Node
    hover_node = Node(
        package='attack_drone',
        executable='hover_node',
        name='hover_node',
        output='screen',
        parameters=[{
            'flight_height': ['-', LaunchConfiguration('flight_height')],  # Convert to negative for NED
            'trail_length': LaunchConfiguration('trail_length'),
        }],
        emulate_tty=True,
    )
    

    # RViz2 Node with custom configuration
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen',
        emulate_tty=True,
    )
    
    # Static transform publisher for map -> odom frame
    # This creates the base coordinate frame for visualization
    static_tf_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_publisher_map_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        output='screen'
    )
    
    return LaunchDescription([
        # Launch arguments
        flight_height_arg,
        trail_length_arg,
        use_micro_ros_agent_arg,
        
        # Nodes
        micro_ros_agent,
        hover_node,
        rviz_node,
        static_tf_publisher,
    ])

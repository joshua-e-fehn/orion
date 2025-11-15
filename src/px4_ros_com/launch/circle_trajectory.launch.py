#!/usr/bin/env python3

"""
Launch file for autonomous circular trajectory flight with RViz visualization

This launch file starts:
1. The circular trajectory control node
2. RViz2 with pre-configured visualization
3. Optional: micro-ros-agent for PX4 communication

Usage:
    ros2 launch px4_ros_com circle_trajectory.launch.py
    
Optional parameters:
    circle_radius:=4.0       # Circle radius in meters
    flight_height:=5.0       # Flight height in meters (positive value)
    angular_velocity:=0.3    # Angular velocity in rad/s
    num_circles:=2           # Number of circles to complete
    use_micro_ros_agent:=false  # Start micro-ros-agent
    
Author: Orion ARM Team
Date: November 13, 2025
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    
    # Get package directories
    pkg_px4_ros_com = get_package_share_directory('px4_ros_com')
    
    # Path to RViz config file
    rviz_config_file = os.path.join(pkg_px4_ros_com, 'config', 'circle_trajectory.rviz')
    
    # Declare launch arguments
    circle_radius_arg = DeclareLaunchArgument(
        'circle_radius',
        default_value='4.0',
        description='Radius of the circular trajectory in meters'
    )
    
    flight_height_arg = DeclareLaunchArgument(
        'flight_height',
        default_value='5.0',
        description='Flight height above ground in meters (positive value, will be converted to NED)'
    )
    
    angular_velocity_arg = DeclareLaunchArgument(
        'angular_velocity',
        default_value='0.3',
        description='Angular velocity for circular motion in rad/s'
    )
    
    num_circles_arg = DeclareLaunchArgument(
        'num_circles',
        default_value='2',
        description='Number of complete circles to fly'
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
    # Note: Adjust port and settings based on your PX4 configuration
    micro_ros_agent = ExecuteProcess(
        condition=IfCondition(LaunchConfiguration('use_micro_ros_agent')),
        cmd=['micro-ros-agent', 'udp4', '--port', '8888', '-v'],
        shell=False,
        output='screen'
    )
    
    # Circle Trajectory Control Node
    circle_trajectory_node = Node(
        package='px4_ros_com',
        executable='circle_trajectory_node.py',
        name='circle_trajectory_node',
        output='screen',
        parameters=[{
            'circle_radius': LaunchConfiguration('circle_radius'),
            'flight_height': ['-', LaunchConfiguration('flight_height')],  # Convert to negative for NED
            'angular_velocity': LaunchConfiguration('angular_velocity'),
            'num_circles': LaunchConfiguration('num_circles'),
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
    
    # Static transform publisher for map frame
    # This creates a static tf from 'map' to 'base_link' or 'odom'
    # Adjust as needed based on your robot's TF tree
    static_tf_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_publisher_map',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        output='screen'
    )
    
    return LaunchDescription([
        # Launch arguments
        circle_radius_arg,
        flight_height_arg,
        angular_velocity_arg,
        num_circles_arg,
        trail_length_arg,
        use_micro_ros_agent_arg,
        
        # Nodes
        micro_ros_agent,
        circle_trajectory_node,
        rviz_node,
        static_tf_publisher,
    ])

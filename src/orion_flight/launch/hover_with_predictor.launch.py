#!/usr/bin/env python3

"""
Launch file for hover demo with target prediction visualization

This launch file combines:
1. Target drone hover flight (straight line motion)
2. CV or CA predictor for target motion prediction
3. RViz2 with visualization for both target and predictions
4. Static TF publishers for coordinate frames

Usage:
    ros2 launch orion_flight hover_with_predictor.launch.py
    
Optional parameters:
    predictor_type:=cv           # Predictor type: 'cv' or 'ca'
    target_namespace:=px4_2      # Target drone namespace
    flight_height:=5.0           # Flight height in meters
    trail_length:=10             # Trail visualization length
    prediction_horizons:="[0.5, 1.0, 2.0, 3.0]"  # Prediction horizons in seconds
    
Author: Orion ARM Team
Date: November 15, 2025
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
import launch.conditions
import launch.substitutions
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    
    # Get package directories
    pkg_orion_flight = get_package_share_directory('orion_flight')
    
    # Path to RViz config file
    rviz_config_file = os.path.join(pkg_orion_flight, 'config', 'hover_with_predictor.rviz')
    # Fallback to hover.rviz if the specific config doesn't exist
    if not os.path.exists(rviz_config_file):
        rviz_config_file = os.path.join(pkg_orion_flight, 'config', 'hover.rviz')
    
    # Declare launch arguments
    predictor_type_arg = DeclareLaunchArgument(
        'predictor_type',
        default_value='cv',
        description='Type of predictor: cv or ca'
    )
    
    target_namespace_arg = DeclareLaunchArgument(
        'target_namespace',
        default_value='px4_2',
        description='ROS namespace for target drone (must match hover node)'
    )
    
    flight_height_arg = DeclareLaunchArgument(
        'flight_height',
        default_value='5.0',
        description='Flight height above ground in meters'
    )
    
    trail_length_arg = DeclareLaunchArgument(
        'trail_length',
        default_value='10',
        description='Number of position markers to display in trail'
    )
    
    prediction_update_rate_arg = DeclareLaunchArgument(
        'prediction_update_rate',
        default_value='20.0',
        description='Prediction update rate in Hz'
    )
    
    prediction_horizons_arg = DeclareLaunchArgument(
        'prediction_horizons',
        default_value='[0.5, 1.0, 2.0, 3.0, 5.0]',
        description='Prediction horizons in seconds'
    )
    
    use_micro_ros_agent_arg = DeclareLaunchArgument(
        'use_micro_ros_agent',
        default_value='false',
        description='Launch micro-ros-agent for PX4 communication'
    )
    
    # Get launch configurations
    predictor_type = LaunchConfiguration('predictor_type')
    target_namespace = LaunchConfiguration('target_namespace')
    
    # Micro-ROS Agent (optional)
    micro_ros_agent = ExecuteProcess(
        condition=IfCondition(LaunchConfiguration('use_micro_ros_agent')),
        cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
        shell=False,
        output='screen'
    )
    
    # Hover Trajectory Control Node for target drone
    hover_node = Node(
        package='attack_drone',
        executable='hover_node',
        name='hover_node',
        namespace=target_namespace,
        output='screen',
        parameters=[{
            'flight_height': LaunchConfiguration('flight_height'),
            'trail_length': LaunchConfiguration('trail_length'),
        }],
        emulate_tty=True,
    )
    
    # CV Predictor Node
    cv_predictor_node = Node(
        package='orion_flight',
        executable='cv_predictor_node',
        name='cv_predictor_node',
        output='screen',
        parameters=[{
            'target_namespace': target_namespace,
            'update_rate': LaunchConfiguration('prediction_update_rate'),
            'prediction_horizons': LaunchConfiguration('prediction_horizons'),
            'process_noise.position': 0.1,
            'process_noise.velocity': 0.5,
            'measurement_noise.position': 0.05,
            'measurement_noise.velocity': 0.1,
            'publish_markers': True,
            'marker_scale': 1.0,
        }],
        condition=launch.conditions.IfCondition(
            launch.substitutions.EqualsSubstitution(predictor_type, 'cv')
        )
    )
    
    # CA Predictor Node
    ca_predictor_node = Node(
        package='orion_flight',
        executable='ca_predictor_node',
        name='ca_predictor_node',
        output='screen',
        parameters=[{
            'target_namespace': target_namespace,
            'update_rate': LaunchConfiguration('prediction_update_rate'),
            'prediction_horizons': LaunchConfiguration('prediction_horizons'),
            'process_noise.position': 0.1,
            'process_noise.velocity': 0.5,
            'process_noise.acceleration': 1.0,
            'measurement_noise.position': 0.05,
            'measurement_noise.velocity': 0.1,
            'measurement_noise.acceleration': 0.5,
            'use_acceleration_from_msg': False,
            'publish_markers': True,
            'marker_scale': 1.0,
        }],
        condition=launch.conditions.IfCondition(
            launch.substitutions.EqualsSubstitution(predictor_type, 'ca')
        )
    )
    
    # RViz2 Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen',
        emulate_tty=True,
    )
    
    # Static transform publishers
    static_tf_map_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_publisher_map_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        output='screen'
    )
    
    return LaunchDescription([
        # Launch arguments
        predictor_type_arg,
        target_namespace_arg,
        flight_height_arg,
        trail_length_arg,
        prediction_update_rate_arg,
        prediction_horizons_arg,
        use_micro_ros_agent_arg,
        
        # Nodes
        micro_ros_agent,
        hover_node,
        cv_predictor_node,
        ca_predictor_node,
        rviz_node,
        static_tf_map_odom,
    ])

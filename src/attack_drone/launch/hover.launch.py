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

    # Expose trajectory pattern and key parameters
    pattern_arg = DeclareLaunchArgument(
        'pattern',
        default_value='sine',
        description='Trajectory pattern: straight|sine|circle|figure8|lissajous|lawnmower|helix|spiral|random_walk|waypoints'
    )
    speed_arg = DeclareLaunchArgument(
        'speed',
        default_value='1.0',
        description='Forward speed for straight/lawnmower/waypoints (m/s)'
    )
    radius_arg = DeclareLaunchArgument(
        'radius',
        default_value='3.0',
        description='Radius for circle/figure8/helix (m)'
    )
    amp_y_arg = DeclareLaunchArgument(
        'amplitude_y',
        default_value='2.0',
        description='Lateral amplitude for sine/lissajous (m)'
    )
    amp_z_arg = DeclareLaunchArgument(
        'amplitude_z',
        default_value='0.5',
        description='Vertical amplitude for oscillatory patterns (m)'
    )
    freq_y_arg = DeclareLaunchArgument(
        'freq_y',
        default_value='0.1',
        description='Lateral/circular frequency (Hz)'
    )
    freq_z_arg = DeclareLaunchArgument(
        'freq_z',
        default_value='0.05',
        description='Vertical frequency (Hz)'
    )
    liss_ratio_arg = DeclareLaunchArgument(
        'lissajous_ratio',
        default_value='2.0',
        description='Frequency ratio for lissajous pattern'
    )
    area_length_arg = DeclareLaunchArgument(
        'area_length',
        default_value='30.0',
        description='Lawnmower area length (x-axis, m)'
    )
    area_width_arg = DeclareLaunchArgument(
        'area_width',
        default_value='20.0',
        description='Lawnmower area width (y-axis, m)'
    )
    sweep_spacing_arg = DeclareLaunchArgument(
        'sweep_spacing',
        default_value='3.0',
        description='Spacing between lawnmower sweeps (m)'
    )
    helix_pitch_arg = DeclareLaunchArgument(
        'helix_pitch',
        default_value='1.0',
        description='Vertical meters per revolution for helix'
    )
    spiral_growth_arg = DeclareLaunchArgument(
        'spiral_growth',
        default_value='0.3',
        description='Meters per radian growth for spiral'
    )
    yaw_align_arg = DeclareLaunchArgument(
        'yaw_align',
        default_value='true',
        description='Align yaw to motion'
    )
    loop_path_arg = DeclareLaunchArgument(
        'loop_path',
        default_value='true',
        description='Loop waypoint/lawnmower paths'
    )
    waypoints_arg = DeclareLaunchArgument(
        'waypoints',
        default_value='',
        description='Flattened [x,y,z,...] list for waypoints (relative to start)'
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
    
    # Hover Trajectory Control Node
    hover_node = Node(
        package='attack_drone',
        executable='hover_node',
        name='hover_node',
        output='screen',
        parameters=[{
            'flight_height': ['-', LaunchConfiguration('flight_height')],  # Convert to negative for NED
            'trail_length': LaunchConfiguration('trail_length'),
            'pattern': LaunchConfiguration('pattern'),
            'speed': LaunchConfiguration('speed'),
            'radius': LaunchConfiguration('radius'),
            'amplitude_y': LaunchConfiguration('amplitude_y'),
            'amplitude_z': LaunchConfiguration('amplitude_z'),
            'freq_y': LaunchConfiguration('freq_y'),
            'freq_z': LaunchConfiguration('freq_z'),
            'lissajous_ratio': LaunchConfiguration('lissajous_ratio'),
            'area_length': LaunchConfiguration('area_length'),
            'area_width': LaunchConfiguration('area_width'),
            'sweep_spacing': LaunchConfiguration('sweep_spacing'),
            'helix_pitch': LaunchConfiguration('helix_pitch'),
            'spiral_growth': LaunchConfiguration('spiral_growth'),
            'yaw_align': LaunchConfiguration('yaw_align'),
            'loop_path': LaunchConfiguration('loop_path'),
            # Waypoints only if provided (string to list parse happens inside node if supported)
            # Launch passes string; node expects list. Keeping default empty if not provided.
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
        pattern_arg,
        speed_arg,
        radius_arg,
        amp_y_arg,
        amp_z_arg,
        freq_y_arg,
        freq_z_arg,
        liss_ratio_arg,
        area_length_arg,
        area_width_arg,
        sweep_spacing_arg,
        helix_pitch_arg,
        spiral_growth_arg,
        yaw_align_arg,
        loop_path_arg,
        waypoints_arg,
        use_micro_ros_agent_arg,
        
        # Nodes
        micro_ros_agent,
        hover_node,
        rviz_node,
        static_tf_publisher,
    ])

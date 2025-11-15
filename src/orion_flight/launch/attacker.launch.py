#!/usr/bin/env python3

"""
Attacker Drone Behavior Launch File
====================================

Controls the behavior of the attacker drone (px4_1).

Available modes:
- hover: Straight-line hover flight
- circle: Circular trajectory flight
- waypoint: Follow waypoint sequence (future)
- evasive: Evasive maneuvers (future)

Usage:
    ros2 launch orion_flight attacker.launch.py mode:=hover
    ros2 launch orion_flight attacker.launch.py mode:=circle radius:=10.0
    
Parameters:
    mode:=hover              # Behavior mode: hover, circle, waypoint, evasive
    namespace:=px4_1        # Drone namespace
    
    # Hover mode parameters
    flight_height:=5.0      # Flight altitude (meters, positive)
    hover_duration:=30.0    # How long to hover (seconds)
    
    # Circle mode parameters  
    radius:=5.0             # Circle radius (meters)
    altitude:=5.0           # Flight altitude (meters, positive)
    angular_velocity:=0.5   # Angular velocity (rad/s)
    
    # Common parameters
    trail_length:=50        # Visualization trail length
    rviz:=true             # Show RViz visualization
    
Author: Orion ARM Team
Date: November 15, 2025
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    """Setup function to access launch configurations."""
    
    # Get launch configurations
    mode = LaunchConfiguration('mode').perform(context)
    namespace = LaunchConfiguration('namespace').perform(context)
    rviz = LaunchConfiguration('rviz').perform(context)
    
    # Get package directory
    pkg_orion_flight = get_package_share_directory('orion_flight')
    pkg_attack_drone = get_package_share_directory('attack_drone')
    
    actions = []
    
    print(f"\n{'='*70}")
    print(f"  Attacker Drone Behavior: {mode.upper()}")
    print(f"{'='*70}\n")
    print(f"  Namespace: {namespace}")
    print(f"  Mode: {mode}\n")
    
    # =========================================================================
    # HOVER MODE - Straight line hover flight
    # =========================================================================
    
    if mode == 'hover':
        flight_height = LaunchConfiguration('flight_height').perform(context)
        hover_duration = LaunchConfiguration('hover_duration').perform(context)
        trail_length = LaunchConfiguration('trail_length').perform(context)
        
        print(f"  Hover Parameters:")
        print(f"    - Flight Height: {flight_height} m")
        print(f"    - Duration: {hover_duration} s")
        print(f"    - Trail Length: {trail_length}\n")
        
        # RViz config for hover
        rviz_config = os.path.join(pkg_orion_flight, 'config', 'hover.rviz')
        
        hover_node = Node(
            package='attack_drone',
            executable='hover_node',
            name='hover_node',
            namespace=namespace,
            output='screen',
            parameters=[{
                'namespace': namespace,
                'flight_height': float(flight_height),
                'hover_duration': float(hover_duration),
                'trail_length': int(trail_length),
            }],
            emulate_tty=True,
        )
        actions.append(hover_node)
        
        if rviz.lower() == 'true':
            if os.path.exists(rviz_config):
                rviz_args = ['-d', rviz_config]
            else:
                print(f"  Warning: RViz config not found, using defaults\n")
                rviz_args = []
            
            rviz_node = Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=rviz_args,
                output='screen',
            )
            actions.append(rviz_node)
    
    # =========================================================================
    # CIRCLE MODE - Circular trajectory flight
    # =========================================================================
    
    elif mode == 'circle':
        radius = LaunchConfiguration('radius').perform(context)
        altitude = LaunchConfiguration('altitude').perform(context)
        angular_velocity = LaunchConfiguration('angular_velocity').perform(context)
        trail_length = LaunchConfiguration('trail_length').perform(context)
        
        print(f"  Circle Parameters:")
        print(f"    - Radius: {radius} m")
        print(f"    - Altitude: {altitude} m")
        print(f"    - Angular Velocity: {angular_velocity} rad/s")
        print(f"    - Trail Length: {trail_length}\n")
        
        # RViz config for circle
        rviz_config = os.path.join(pkg_orion_flight, 'config', 'circle.rviz')
        
        circle_node = Node(
            package='attack_drone',
            executable='circle_node',
            name='circle_node',
            namespace=namespace,
            output='screen',
            parameters=[{
                'namespace': namespace,
                'radius': float(radius),
                'altitude': float(altitude),
                'angular_velocity': float(angular_velocity),
                'trail_length': int(trail_length),
            }],
            emulate_tty=True,
        )
        actions.append(circle_node)
        
        if rviz.lower() == 'true':
            if os.path.exists(rviz_config):
                rviz_args = ['-d', rviz_config]
            else:
                print(f"  Warning: RViz config not found, using defaults\n")
                rviz_args = []
            
            rviz_node = Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=rviz_args,
                output='screen',
            )
            actions.append(rviz_node)
    
    # =========================================================================
    # WAYPOINT MODE - Follow waypoint sequence (future implementation)
    # =========================================================================
    
    elif mode == 'waypoint':
        print(f"  ERROR: Waypoint mode not yet implemented!")
        print(f"  Available modes: hover, circle\n")
        raise NotImplementedError("Waypoint mode coming soon!")
    
    # =========================================================================
    # EVASIVE MODE - Evasive maneuvers (future implementation)
    # =========================================================================
    
    elif mode == 'evasive':
        print(f"  ERROR: Evasive mode not yet implemented!")
        print(f"  Available modes: hover, circle\n")
        raise NotImplementedError("Evasive mode coming soon!")
    
    # =========================================================================
    # Invalid mode
    # =========================================================================
    
    else:
        print(f"  ERROR: Unknown mode '{mode}'!")
        print(f"  Available modes: hover, circle, waypoint (future), evasive (future)\n")
        raise ValueError(f"Unknown attacker mode: {mode}")
    
    # =========================================================================
    # Static TF publishers
    # =========================================================================
    
    static_tf_map_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_map_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
    )
    actions.append(static_tf_map_odom)
    
    static_tf_odom_baselink = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name=f'static_tf_odom_{namespace}',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', f'{namespace}_base_link'],
    )
    actions.append(static_tf_odom_baselink)
    
    print(f"{'='*70}\n")
    print(f"  Attacker behavior node launching...")
    print(f"  Use Ctrl+C to stop\n")
    print(f"{'='*70}\n")
    
    return actions


def generate_launch_description():
    """Generate launch description for attacker behavior."""
    
    return LaunchDescription([
        # =====================================================================
        # Common arguments
        # =====================================================================
        
        DeclareLaunchArgument(
            'mode',
            default_value='hover',
            description='Attacker behavior mode: hover, circle, waypoint, evasive'
        ),
        DeclareLaunchArgument(
            'namespace',
            default_value='px4_1',
            description='Drone namespace (px4_1 for attacker)'
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Launch RViz for visualization'
        ),
        DeclareLaunchArgument(
            'trail_length',
            default_value='50',
            description='Number of trail markers to display'
        ),
        
        # =====================================================================
        # Hover mode parameters
        # =====================================================================
        
        DeclareLaunchArgument(
            'flight_height',
            default_value='5.0',
            description='[HOVER] Flight altitude in meters (positive)'
        ),
        DeclareLaunchArgument(
            'hover_duration',
            default_value='30.0',
            description='[HOVER] How long to hover in seconds'
        ),
        
        # =====================================================================
        # Circle mode parameters
        # =====================================================================
        
        DeclareLaunchArgument(
            'radius',
            default_value='5.0',
            description='[CIRCLE] Circle radius in meters'
        ),
        DeclareLaunchArgument(
            'altitude',
            default_value='5.0',
            description='[CIRCLE] Flight altitude in meters (positive)'
        ),
        DeclareLaunchArgument(
            'angular_velocity',
            default_value='0.5',
            description='[CIRCLE] Angular velocity in rad/s'
        ),
        
        # Launch setup
        OpaqueFunction(function=launch_setup),
    ])

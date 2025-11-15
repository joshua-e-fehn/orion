#!/usr/bin/env python3

"""
Interceptor Drone Behavior Launch File
=======================================

Controls the interceptor drone (px4_2) with prediction and planning capabilities.

Operation Modes:
- prediction_only: Run predictor to track attacker, no planning
- planning_only: Run planner with ground truth, no prediction
- full: Run both predictor and planner (complete system)

Available Predictors:
- cv: Constant Velocity predictor (6-state Kalman filter)
- ca: Constant Acceleration predictor (9-state Kalman filter)
- imm: Interacting Multiple Model (future)

Available Planners:
- pp: Pure Pursuit planner
- apf: Artificial Potential Field (future)
- mpc: Model Predictive Control (future)

Usage:
    # Prediction only with CV predictor
    ros2 launch orion_flight interceptor.launch.py mode:=prediction_only predictor:=cv
    
    # Full system with CA predictor and Pure Pursuit planner
    ros2 launch orion_flight interceptor.launch.py mode:=full predictor:=ca planner:=pp
    
    # Planning only (uses ground truth from attacker)
    ros2 launch orion_flight interceptor.launch.py mode:=planning_only planner:=pp
    
Parameters:
    mode:=full              # Operation mode: prediction_only, planning_only, full
    namespace:=px4_2       # Interceptor drone namespace
    target_namespace:=px4_1 # Attacker drone namespace
    
    # Predictor settings
    predictor:=cv          # Predictor type: cv, ca, imm
    prediction_horizon:=2.0 # Prediction horizon in seconds
    
    # Planner settings
    planner:=pp            # Planner type: pp, apf, mpc
    lookahead_distance:=3.0 # Pure Pursuit lookahead distance
    max_speed:=5.0         # Maximum interceptor speed (m/s)
    
    # Visualization
    rviz:=true             # Show RViz visualization
    show_prediction:=true  # Show prediction markers
    show_path:=true        # Show planned path
    trail_length:=50       # Visualization trail length
    
Author: Orion ARM Team
Date: November 15, 2025
"""

import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    """Setup function to access launch configurations."""
    
    # Get launch configurations
    mode = LaunchConfiguration('mode').perform(context)
    namespace = LaunchConfiguration('namespace').perform(context)
    target_namespace = LaunchConfiguration('target_namespace').perform(context)
    predictor = LaunchConfiguration('predictor').perform(context)
    planner = LaunchConfiguration('planner').perform(context)
    rviz = LaunchConfiguration('rviz').perform(context)
    
    # Get package directories
    pkg_orion_flight = get_package_share_directory('orion_flight')
    pkg_interceptor = get_package_share_directory('interceptor')
    
    actions = []
    
    print(f"\n{'='*70}")
    print(f"  Interceptor Drone System: {mode.upper()}")
    print(f"{'='*70}\n")
    print(f"  Interceptor: {namespace}")
    print(f"  Target: {target_namespace}")
    print(f"  Mode: {mode}\n")
    
    # Validate mode
    valid_modes = ['prediction_only', 'planning_only', 'full']
    if mode not in valid_modes:
        print(f"  ERROR: Invalid mode '{mode}'!")
        print(f"  Valid modes: {', '.join(valid_modes)}\n")
        raise ValueError(f"Invalid mode: {mode}")
    
    # =========================================================================
    # PREDICTOR SETUP (for prediction_only and full modes)
    # =========================================================================
    
    if mode in ['prediction_only', 'full']:
        print(f"  Predictor Configuration:")
        print(f"    - Type: {predictor.upper()}")
        
        # Validate predictor type
        valid_predictors = ['cv', 'ca', 'imm']
        if predictor not in valid_predictors:
            print(f"    ERROR: Invalid predictor '{predictor}'!")
            print(f"    Valid predictors: {', '.join(valid_predictors)}\n")
            raise ValueError(f"Invalid predictor: {predictor}")
        
        # Get predictor-specific config file
        if predictor == 'cv':
            config_file = 'cv_predictor.yaml'
            executable = 'cv_predictor_node'
        elif predictor == 'ca':
            config_file = 'ca_predictor.yaml'
            executable = 'ca_predictor_node'
        elif predictor == 'imm':
            print(f"    ERROR: IMM predictor not yet implemented!\n")
            raise NotImplementedError("IMM predictor coming soon!")
        
        config_path = os.path.join(pkg_orion_flight, 'config', config_file)
        
        # Check if config exists
        if not os.path.exists(config_path):
            print(f"    Warning: Config not found at {config_path}")
            print(f"    Using default parameters\n")
            predictor_params = [{
                'target_namespace': target_namespace,
                'prediction_horizon': LaunchConfiguration('prediction_horizon'),
            }]
        else:
            predictor_params = [
                config_path,
                {
                    'target_namespace': target_namespace,
                    'prediction_horizon': LaunchConfiguration('prediction_horizon'),
                }
            ]
        
        print(f"    - Config: {config_file}")
        print(f"    - Target: {target_namespace}")
        print(f"    - Horizon: {LaunchConfiguration('prediction_horizon').perform(context)}s\n")
        
        # Launch predictor node
        predictor_node = Node(
            package='orion_flight',
            executable=executable,
            name=f'{predictor}_predictor_node',
            namespace=namespace,
            output='screen',
            parameters=predictor_params,
            emulate_tty=True,
        )
        actions.append(predictor_node)
    
    # =========================================================================
    # PLANNER SETUP (for planning_only and full modes)
    # =========================================================================
    
    if mode in ['planning_only', 'full']:
        print(f"  Planner Configuration:")
        print(f"    - Type: {planner.upper()}")
        
        # Validate planner type
        valid_planners = ['pp', 'apf', 'mpc']
        if planner not in valid_planners:
            print(f"    ERROR: Invalid planner '{planner}'!")
            print(f"    Valid planners: {', '.join(valid_planners)}\n")
            raise ValueError(f"Invalid planner: {planner}")
        
        # Get planner-specific parameters
        if planner == 'pp':
            planner_executable = 'pp_planner_node'
            planner_config = os.path.join(pkg_orion_flight, 'config', 'pp_planner.yaml')
            
            lookahead = LaunchConfiguration('lookahead_distance').perform(context)
            max_speed = LaunchConfiguration('max_speed').perform(context)
            
            print(f"    - Lookahead Distance: {lookahead} m")
            print(f"    - Max Speed: {max_speed} m/s")
            
            if os.path.exists(planner_config):
                planner_params = [
                    planner_config,
                    {
                        'namespace': namespace,
                        'target_namespace': target_namespace,
                        'lookahead_distance': float(lookahead),
                        'max_speed': float(max_speed),
                        'use_prediction': mode == 'full',  # Use prediction if in full mode
                    }
                ]
            else:
                planner_params = [{
                    'namespace': namespace,
                    'target_namespace': target_namespace,
                    'lookahead_distance': float(lookahead),
                    'max_speed': float(max_speed),
                    'use_prediction': mode == 'full',
                }]
        
        elif planner == 'apf':
            print(f"    ERROR: APF planner not yet implemented!\n")
            raise NotImplementedError("APF planner coming soon!")
        
        elif planner == 'mpc':
            print(f"    ERROR: MPC planner not yet implemented!\n")
            raise NotImplementedError("MPC planner coming soon!")
        
        print()
        
        # Launch planner node
        planner_node = Node(
            package='orion_flight',
            executable=planner_executable,
            name=f'{planner}_planner_node',
            namespace=namespace,
            output='screen',
            parameters=planner_params,
            emulate_tty=True,
        )
        actions.append(planner_node)
    
    # =========================================================================
    # VISUALIZATION (RViz)
    # =========================================================================
    
    if rviz.lower() == 'true':
        # Select RViz config based on mode
        if mode == 'prediction_only':
            rviz_config = os.path.join(pkg_orion_flight, 'config', 'predictor_viz.rviz')
        elif mode == 'planning_only':
            rviz_config = os.path.join(pkg_orion_flight, 'config', 'planner_viz.rviz')
        else:  # full mode
            rviz_config = os.path.join(pkg_orion_flight, 'config', 'interceptor_full.rviz')
        
        # Check if config exists
        if os.path.exists(rviz_config):
            rviz_args = ['-d', rviz_config]
            print(f"  RViz Config: {os.path.basename(rviz_config)}\n")
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
    # Static TF Publishers
    # =========================================================================
    
    static_tf_map_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_map_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
    )
    actions.append(static_tf_map_odom)
    
    static_tf_interceptor = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name=f'static_tf_{namespace}',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', f'{namespace}_base_link'],
    )
    actions.append(static_tf_interceptor)
    
    static_tf_target = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name=f'static_tf_{target_namespace}',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', f'{target_namespace}_base_link'],
    )
    actions.append(static_tf_target)
    
    # =========================================================================
    # Summary
    # =========================================================================
    
    print(f"{'='*70}\n")
    print(f"  System Components:")
    if mode in ['prediction_only', 'full']:
        print(f"    ✓ {predictor.upper()} Predictor")
    if mode in ['planning_only', 'full']:
        print(f"    ✓ {planner.upper()} Planner")
    if rviz.lower() == 'true':
        print(f"    ✓ RViz Visualization")
    print(f"\n  Interceptor system launching...")
    print(f"  Use Ctrl+C to stop\n")
    print(f"{'='*70}\n")
    
    return actions


def generate_launch_description():
    """Generate launch description for interceptor behavior."""
    
    return LaunchDescription([
        # =====================================================================
        # Common arguments
        # =====================================================================
        
        DeclareLaunchArgument(
            'mode',
            default_value='full',
            description='Operation mode: prediction_only, planning_only, full'
        ),
        DeclareLaunchArgument(
            'namespace',
            default_value='px4_2',
            description='Interceptor drone namespace'
        ),
        DeclareLaunchArgument(
            'target_namespace',
            default_value='px4_1',
            description='Target (attacker) drone namespace'
        ),
        
        # =====================================================================
        # Predictor arguments
        # =====================================================================
        
        DeclareLaunchArgument(
            'predictor',
            default_value='cv',
            description='Predictor type: cv, ca, imm'
        ),
        DeclareLaunchArgument(
            'prediction_horizon',
            default_value='2.0',
            description='Prediction horizon in seconds'
        ),
        
        # =====================================================================
        # Planner arguments
        # =====================================================================
        
        DeclareLaunchArgument(
            'planner',
            default_value='pp',
            description='Planner type: pp, apf, mpc'
        ),
        DeclareLaunchArgument(
            'lookahead_distance',
            default_value='3.0',
            description='[Pure Pursuit] Lookahead distance in meters'
        ),
        DeclareLaunchArgument(
            'max_speed',
            default_value='5.0',
            description='Maximum interceptor speed in m/s'
        ),
        
        # =====================================================================
        # Visualization arguments
        # =====================================================================
        
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Launch RViz for visualization'
        ),
        DeclareLaunchArgument(
            'show_prediction',
            default_value='true',
            description='Show prediction markers in RViz'
        ),
        DeclareLaunchArgument(
            'show_path',
            default_value='true',
            description='Show planned path in RViz'
        ),
        DeclareLaunchArgument(
            'trail_length',
            default_value='50',
            description='Number of trail markers to display'
        ),
        
        # Launch setup
        OpaqueFunction(function=launch_setup),
    ])

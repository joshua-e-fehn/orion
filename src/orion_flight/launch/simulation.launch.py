#!/usr/bin/env python3

"""
Complete Simulation Launch File
================================

This launch file starts the entire simulation environment with:
- 2 PX4 drones in Gazebo
- MicroXRCE-DDS agents for both drones
- Safety features disabled
- RViz for visualization

Usage:
    ros2 launch orion_flight simulation.launch.py
    
Optional parameters:
    world:=default           # Gazebo world name
    gui:=true               # Show Gazebo GUI
    headless:=false         # Run headless (no GUI)
    rviz:=true              # Launch RViz
    px4_dir:=${HOME}/PX4-Autopilot  # Path to PX4-Autopilot directory
    
Author: Orion ARM Team
Date: November 15, 2025
"""

import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    RegisterEventHandler,
    TimerAction,
    OpaqueFunction,
    Shutdown,
)
from launch.event_handlers import OnProcessStart, OnProcessExit, OnShutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import signal
import subprocess
import time


def get_px4_directory(context):
    """Get PX4 directory from launch configuration or default."""
    px4_dir_config = LaunchConfiguration('px4_dir').perform(context)
    
    if px4_dir_config:
        px4_path = Path(px4_dir_config).expanduser()
    else:
        px4_path = Path.home() / 'PX4-Autopilot'
    
    if not px4_path.exists():
        raise RuntimeError(f"PX4-Autopilot not found at {px4_path}. "
                         f"Please install PX4 or specify px4_dir parameter.")
    
    return str(px4_path)


def launch_setup(context, *args, **kwargs):
    """Setup function to access launch configurations."""
    
    # Get launch configurations
    world = LaunchConfiguration('world').perform(context)
    gui = LaunchConfiguration('gui').perform(context)
    headless = LaunchConfiguration('headless').perform(context)
    rviz = LaunchConfiguration('rviz').perform(context)
    px4_dir = get_px4_directory(context)
    
    # Get package directories
    pkg_orion_flight = get_package_share_directory('orion_flight')
    rviz_config_file = os.path.join(pkg_orion_flight, 'config', 'simulation.rviz')
    
    actions = []
    
    # =========================================================================
    # PRE-LAUNCH CLEANUP - Kill any existing simulation processes
    # =========================================================================
    
    print(f"\n{'='*70}")
    print(f"  Pre-launch Cleanup")
    print(f"{'='*70}\n")
    print("  Terminating any existing simulation processes...")
    
    # Kill existing processes
    subprocess.run(['pkill', '-9', '-f', 'px4.*-i'], stderr=subprocess.DEVNULL)
    subprocess.run(['pkill', '-9', '-f', 'MicroXRCEAgent'], stderr=subprocess.DEVNULL)
    subprocess.run(['pkill', '-9', '-f', 'gz sim'], stderr=subprocess.DEVNULL)
    subprocess.run(['pkill', '-9', '-f', 'gzserver'], stderr=subprocess.DEVNULL)
    subprocess.run(['pkill', '-9', '-f', 'gzclient'], stderr=subprocess.DEVNULL)
    subprocess.run(['pkill', '-9', '-f', 'ruby.*gz'], stderr=subprocess.DEVNULL)
    
    # Wait a moment for processes to fully terminate
    time.sleep(2)
    
    print("  ✓ Cleanup complete\n")
    
    # Add cleanup handler for shutdown
    cleanup_script = os.path.join(pkg_orion_flight, 'scripts', 'cleanup_terminals.sh')
    
    # Create cleanup script if it doesn't exist
    cleanup_script_content = """#!/bin/bash
# Kill all PX4 and MicroXRCEAgent processes
echo "Cleaning up simulation processes..."
pkill -f "px4.*-i 1"
pkill -f "px4.*-i 2"  
pkill -f "MicroXRCEAgent udp4 -p 8888"
pkill -f "MicroXRCEAgent udp4 -p 8889"
pkill -f "gz sim"
pkill -f "gzserver"
pkill -f "ruby.*gz.*sim"
sleep 1
echo "✓ Cleaned up all simulation processes"
"""
    
    # Write cleanup script
    os.makedirs(os.path.dirname(cleanup_script), exist_ok=True)
    with open(cleanup_script, 'w') as f:
        f.write(cleanup_script_content)
    os.chmod(cleanup_script, 0o755)
    
    # Register cleanup on shutdown
    cleanup_handler = RegisterEventHandler(
        OnShutdown(
            on_shutdown=[
                ExecuteProcess(
                    cmd=['bash', cleanup_script],
                    output='screen',
                )
            ]
        )
    )
    actions.append(cleanup_handler)
    
    # =========================================================================
    # Launch Gazebo with first drone (px4_1)
    # =========================================================================
    
    print(f"\n{'='*70}")
    print(f"  Starting Gazebo Simulation with 2 Drones")
    print(f"{'='*70}\n")
    print(f"  PX4 Directory: {px4_dir}")
    print(f"  World: {world}")
    print(f"  GUI: {gui}")
    print(f"  Headless: {headless}\n")
    
    # Drone 1 environment - This one starts Gazebo!
    # Note: NO PX4_GZ_STANDALONE for the first drone - it launches Gazebo
    
    # Start PX4 SITL for drone 1 (launches Gazebo)
    # Run directly without terminal wrapper to ensure Gazebo GUI appears
    px4_drone1 = ExecuteProcess(
        cmd=[
            'bash', '-c',
            f'cd {px4_dir} && '
            f'PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=gz_x500 '
            f'./build/px4_sitl_default/bin/px4 -i 1'
        ],
        output='screen',
        name='px4_drone1',
        shell=False,
    )
    actions.append(px4_drone1)
    
    # =========================================================================
    # Launch second drone (px4_2) after a delay
    # =========================================================================
    
    # Drone 2 connects to existing Gazebo instance
    # Note: PX4_GZ_STANDALONE=1 means "connect to existing Gazebo"
    
    # Start PX4 SITL for drone 2 (delayed to avoid conflicts)
    px4_drone2 = TimerAction(
        period=10.0,  # Wait 10 seconds for Gazebo to fully start
        actions=[
            ExecuteProcess(
                cmd=[
                    'bash', '-c',
                    f'cd {px4_dir} && '
                    f'PX4_GZ_STANDALONE=1 PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE="0,1" PX4_SIM_MODEL=gz_x500 '
                    f'./build/px4_sitl_default/bin/px4 -i 2'
                ],
                output='screen',
                name='px4_drone2',
                shell=False,
            )
        ]
    )
    actions.append(px4_drone2)
    
    # =========================================================================
    # MicroXRCE-DDS Agents for both drones
    # =========================================================================
    
    # Agent for drone 1 (port 8888)
    agent1 = TimerAction(
        period=5.0,  # Wait for PX4 drone 1 to start
        actions=[
            ExecuteProcess(
                cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
                output='screen',
                name='agent_drone1',
                shell=False,
            )
        ]
    )
    actions.append(agent1)
    
    # Disable safety for drone 1 (after agent starts)
    safety_drone1 = TimerAction(
        period=8.0,  # Wait for agent to connect
        actions=[
            ExecuteProcess(
                cmd=[
                    'python3',
                    os.path.join(pkg_orion_flight, 'scripts', 'disable_safety_for_instance.py'),
                    '1'
                ],
                output='screen',
                name='safety_drone1',
            )
        ]
    )
    actions.append(safety_drone1)
    
    # Agent for drone 2 (port 8889)
    agent2 = TimerAction(
        period=15.0,  # Wait for drone 2 to start (10s + 5s buffer)
        actions=[
            ExecuteProcess(
                cmd=['MicroXRCEAgent', 'udp4', '-p', '8889'],
                output='screen',
                name='agent_drone2',
                shell=False,
            )
        ]
    )
    actions.append(agent2)
    
    # Disable safety for drone 2 (after agent starts)
    safety_drone2 = TimerAction(
        period=18.0,  # Wait for agent 2 to connect
        actions=[
            ExecuteProcess(
                cmd=[
                    'python3',
                    os.path.join(pkg_orion_flight, 'scripts', 'disable_safety_for_instance.py'),
                    '2'
                ],
                output='screen',
                name='safety_drone2',
            )
        ]
    )
    actions.append(safety_drone2)
    
    # =========================================================================
    # RViz Visualization (optional)
    # =========================================================================
    
    if rviz.lower() == 'true':
        # Check if config file exists, create minimal one if not
        if not os.path.exists(rviz_config_file):
            print(f"  Warning: RViz config not found at {rviz_config_file}")
            print(f"  RViz will start with default configuration\n")
            rviz_args = []
        else:
            rviz_args = ['-d', rviz_config_file]
        
        rviz_node = TimerAction(
            period=15.0,  # Wait for simulation to stabilize
            actions=[
                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    arguments=rviz_args,
                    output='screen',
                )
            ]
        )
        actions.append(rviz_node)
    
    # =========================================================================
    # Static TF publishers for coordinate frames
    # =========================================================================
    
    # Map to odom frame
    static_tf_map_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_map_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
    )
    actions.append(static_tf_map_odom)
    
    # Odom to base_link for drone 1
    static_tf_odom_drone1 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_odom_drone1',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'drone1_base_link'],
    )
    actions.append(static_tf_odom_drone1)
    
    # Odom to base_link for drone 2
    static_tf_odom_drone2 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_odom_drone2',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'drone2_base_link'],
    )
    actions.append(static_tf_odom_drone2)
    
    print(f"\n{'='*70}")
    print(f"  Simulation Starting...")
    print(f"{'='*70}\n")
    print(f"  Drone 1: px4_1 at (0, 0)   → Agent on port 8888 → Launches Gazebo")
    print(f"  Drone 2: px4_2 at (0, 1)   → Agent on port 8889 → Joins Gazebo")
    print(f"\n  Timeline:")
    print(f"    t=0s   : Drone 1 starts + Gazebo launches")
    print(f"    t=5s   : Agent 1 starts")
    print(f"    t=8s   : Safety disabled for Drone 1")
    print(f"    t=10s  : Drone 2 starts (connects to Gazebo)")
    print(f"    t=15s  : Agent 2 + RViz start")
    print(f"    t=18s  : Safety disabled for Drone 2")
    print(f"\n  Both drones will have safety features automatically disabled!")
    print(f"  Press Ctrl+C to stop everything (all terminals will close)")
    print(f"{'='*70}\n")
    
    return actions


def generate_launch_description():
    """Generate launch description for complete simulation."""
    
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'world',
            default_value='default',
            description='Gazebo world to load'
        ),
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='Start Gazebo GUI'
        ),
        DeclareLaunchArgument(
            'headless',
            default_value='false',
            description='Run simulation headless (no GUI)'
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Launch RViz for visualization'
        ),
        DeclareLaunchArgument(
            'px4_dir',
            default_value=str(Path.home() / 'PX4-Autopilot'),
            description='Path to PX4-Autopilot directory'
        ),
        
        # Launch setup with access to configurations
        OpaqueFunction(function=launch_setup),
    ])

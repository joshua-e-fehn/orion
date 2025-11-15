"""Launch file for Proportional Navigation planner."""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Generate launch description for PN planner."""
    
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'interceptor_namespace',
            default_value='px4_1',
            description='Namespace for interceptor drone'
        ),
        
        DeclareLaunchArgument(
            'target_namespace',
            default_value='px4_2',
            description='Namespace for target drone'
        ),
        
        DeclareLaunchArgument(
            'N',
            default_value='3.0',
            description='Navigation constant (PN gain)'
        ),
        
        DeclareLaunchArgument(
            'amax_x',
            default_value='4.0',
            description='Maximum acceleration in x (North) direction (m/s²)'
        ),
        
        DeclareLaunchArgument(
            'amax_y',
            default_value='4.0',
            description='Maximum acceleration in y (East) direction (m/s²)'
        ),
        
        DeclareLaunchArgument(
            'amax_z',
            default_value='2.0',
            description='Maximum acceleration in z (Down) direction (m/s²)'
        ),
        
        DeclareLaunchArgument(
            'control_rate',
            default_value='20.0',
            description='Control loop frequency (Hz)'
        ),
        
        DeclareLaunchArgument(
            'use_predictor',
            default_value='true',
            description='Use predicted target state from predictor node'
        ),
        
        DeclareLaunchArgument(
            'convergence_distance',
            default_value='1.0',
            description='Distance threshold for successful intercept (meters)'
        ),
        
        DeclareLaunchArgument(
            'min_tgo',
            default_value='0.05',
            description='Minimum time-to-go safeguard (seconds)'
        ),
        
        DeclareLaunchArgument(
            'v_eps',
            default_value='0.1',
            description='Velocity epsilon for denominator safeguard (m/s)'
        ),
        
        # PN Planner Node
        Node(
            package='orion_flight',
            executable='pn_planner',
            name='pn_planner_node',
            output='screen',
            parameters=[{
                'interceptor_namespace': LaunchConfiguration('interceptor_namespace'),
                'target_namespace': LaunchConfiguration('target_namespace'),
                'N': LaunchConfiguration('N'),
                'amax': [
                    LaunchConfiguration('amax_x'),
                    LaunchConfiguration('amax_y'),
                    LaunchConfiguration('amax_z')
                ],
                'control_rate': LaunchConfiguration('control_rate'),
                'use_predictor': LaunchConfiguration('use_predictor'),
                'convergence_distance': LaunchConfiguration('convergence_distance'),
                'min_tgo': LaunchConfiguration('min_tgo'),
                'v_eps': LaunchConfiguration('v_eps'),
            }],
            emulate_tty=True,
        )
    ])

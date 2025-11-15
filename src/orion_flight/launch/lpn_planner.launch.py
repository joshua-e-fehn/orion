"""Launch file for Linearized Proportional Navigation (LPN) planner."""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Generate launch description for LPN planner."""
    
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
            'G_lpn',
            default_value='20.0',
            description='LPN gain (recommended: 20.0)'
        ),
        
        DeclareLaunchArgument(
            'min_tgo',
            default_value='0.05',
            description='Minimum time-to-go safeguard (seconds, recommended: 0.05-0.1)'
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
        
        # LPN Planner Node
        Node(
            package='orion_flight',
            executable='lpn_planner',
            name='lpn_planner_node',
            output='screen',
            parameters=[{
                'interceptor_namespace': LaunchConfiguration('interceptor_namespace'),
                'target_namespace': LaunchConfiguration('target_namespace'),
                'G_lpn': LaunchConfiguration('G_lpn'),
                'min_tgo': LaunchConfiguration('min_tgo'),
                'amax': [
                    LaunchConfiguration('amax_x'),
                    LaunchConfiguration('amax_y'),
                    LaunchConfiguration('amax_z')
                ],
                'control_rate': LaunchConfiguration('control_rate'),
                'use_predictor': LaunchConfiguration('use_predictor'),
                'convergence_distance': LaunchConfiguration('convergence_distance'),
            }],
            emulate_tty=True,
        )
    ])

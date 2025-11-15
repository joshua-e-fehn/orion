"""Launch file for Pure Pursuit planner."""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Generate launch description for PP planner."""
    
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
            'G_pp',
            default_value='2.0',
            description='Pure Pursuit proportional gain'
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
        
        # PP Planner Node
        Node(
            package='orion_flight',
            executable='pp_planner',
            name='pp_planner_node',
            output='screen',
            parameters=[{
                'interceptor_namespace': LaunchConfiguration('interceptor_namespace'),
                'target_namespace': LaunchConfiguration('target_namespace'),
                'G_pp': LaunchConfiguration('G_pp'),
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

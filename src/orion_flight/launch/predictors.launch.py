"""Launch file for target predictors."""

import launch
import launch.conditions
import launch.substitutions
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for predictor nodes."""
    
    # Declare launch arguments
    predictor_type_arg = DeclareLaunchArgument(
        'predictor_type',
        default_value='cv',
        description='Type of predictor: cv, ca, or imm'
    )
    
    target_namespace_arg = DeclareLaunchArgument(
        'target_namespace',
        default_value='px4_2',
        description='ROS namespace for target drone'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value='cv_predictor.yaml',
        description='Configuration file name (in config/ directory)'
    )
    
    # Get launch configurations
    predictor_type = LaunchConfiguration('predictor_type')
    config_file = LaunchConfiguration('config_file')
    
    # Build path to config file
    config_path = PathJoinSubstitution([
        FindPackageShare('orion_flight'),
        'config',
        config_file
    ])
    
    # CV Predictor Node
    cv_predictor_node = Node(
        package='orion_flight',
        executable='cv_predictor_node',
        name='cv_predictor_node',
        output='screen',
        parameters=[config_path],
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
        parameters=[config_path],
        condition=launch.conditions.IfCondition(
            launch.substitutions.EqualsSubstitution(predictor_type, 'ca')
        )
    )
    
    # IMM Predictor Node (placeholder - to be implemented)
    # imm_predictor_node = Node(
    #     package='orion_flight',
    #     executable='imm_predictor_node',
    #     name='imm_predictor_node',
    #     output='screen',
    #     parameters=[config_path],
    #     condition=launch.conditions.IfCondition(
    #         launch.substitutions.EqualsSubstitution(predictor_type, 'imm')
    #     )
    # )
    
    return LaunchDescription([
        predictor_type_arg,
        target_namespace_arg,
        config_file_arg,
        cv_predictor_node,
        ca_predictor_node,
        # imm_predictor_node,  # Uncomment when implemented
    ])

from launch import LaunchDescription
from launch.actions import TimerAction, ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    # start first PX4 instance
    px4_1 = ExecuteProcess(
        cmd=['bash','-lc','cd /root/PX4-Autopilot && HEADLESS=1 make px4_sitl gazebo-classic'],
        output='screen',
        name='px4_1'
    )

    # start second PX4 instance (staggered)
    px4_2 = TimerAction(
        period=3.0,
        actions=[ExecuteProcess(
            cmd=['bash','-lc','cd /root/PX4-Autopilot && HEADLESS=1 make px4_sitl gazebo-classic EXTRA_SIM_ARGS="--instance 1"'],
            output='screen',
            name='px4_2'
        )]
    )

    # spawn two iris models after Gazebo is likely ready
    spawn_iris1 = TimerAction(
        period=10.0,
        actions=[Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-file', '/root/PX4-Autopilot/Tools/sitl_gazebo/models/iris/iris.sdf',
                '-entity', 'iris1', '-x', '0', '-y', '0', '-z', '0.1'
            ],
            output='screen'
        )]
    )

    spawn_iris2 = TimerAction(
        period=11.0,
        actions=[Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-file', '/root/PX4-Autopilot/Tools/sitl_gazebo/models/iris/iris.sdf',
                '-entity', 'iris2', '-x', '8', '-y', '0', '-z', '0.1'
            ],
            output='screen'
        )]
    )

    # MAVROS nodes (namespaced, each connects to a different UDP port)
    mavros1 = TimerAction(
        period=12.0,
        actions=[Node(
            package='mavros',
            executable='mavros_node',
            namespace='drone1',
            name='mavros',
            parameters=[{'fcu_url': 'udp://127.0.0.1:14540'}],
            output='screen'
        )]
    )

    mavros2 = TimerAction(
        period=12.5,
        actions=[Node(
            package='mavros',
            executable='mavros_node',
            namespace='drone2',
            name='mavros',
            parameters=[{'fcu_url': 'udp://127.0.0.1:14541'}],
            output='screen'
        )]
    )

    # chaser node: drone1 chases drone2
    chaser = TimerAction(
        period=13.0,
        actions=[Node(
            package='autonomous_drone',
            executable='chaser.py',
            name='chaser',
            parameters=[{
                'namespace': 'drone1',
                'target_namespace': 'drone2',
                'max_speed': 1.0,
                'kp': 0.8
            }],
            output='screen'
        )]
    )

    return LaunchDescription([
        px4_1,
        px4_2,
        spawn_iris1,
        spawn_iris2,
        mavros1,
        mavros2,
        chaser,
    ])
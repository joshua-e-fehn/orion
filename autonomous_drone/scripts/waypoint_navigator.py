#!/usr/bin/env python3
"""
Autonomous Waypoint Navigator for PX4 with MAVROS
Flies the drone to a predefined waypoint and holds position
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
import time


class WaypointNavigator(Node):
    """
    ROS2 Node for autonomous waypoint navigation using MAVROS
    """

    def __init__(self):
        super().__init__('waypoint_navigator')
        
        # QoS profile for MAVROS topics
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # State variables
        self.current_state = State()
        self.target_pose = PoseStamped()
        
        # Publishers
        self.local_pos_pub = self.create_publisher(
            PoseStamped,
            '/mavros/setpoint_position/local',
            10
        )
        
        # Subscribers
        self.state_sub = self.create_subscription(
            State,
            '/mavros/state',
            self.state_callback,
            qos_profile
        )
        
        # Service clients
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        
        # Wait for services
        self.get_logger().info('Waiting for MAVROS services...')
        while not self.arming_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Arming service not available, waiting...')
        while not self.set_mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Set mode service not available, waiting...')
        
        self.get_logger().info('MAVROS services available')
        
        # Define target waypoint (5 meters forward, 3 meters up from home)
        self.target_pose.pose.position.x = 5.0
        self.target_pose.pose.position.y = 0.0
        self.target_pose.pose.position.z = 3.0
        
        # Timer for main control loop (20 Hz)
        self.timer = self.create_timer(0.05, self.control_loop)
        
        # Mission state
        self.mission_started = False
        self.offboard_set = False
        self.armed = False
        self.waypoint_reached = False
        self.loop_count = 0
        
        self.get_logger().info('Waypoint Navigator initialized')
        self.get_logger().info(f'Target waypoint: x={self.target_pose.pose.position.x}, '
                              f'y={self.target_pose.pose.position.y}, '
                              f'z={self.target_pose.pose.position.z}')

    def state_callback(self, msg):
        """Callback for MAVROS state messages"""
        self.current_state = msg

    def control_loop(self):
        """Main control loop for autonomous navigation"""
        self.loop_count += 1
        
        # Update timestamp
        self.target_pose.header.stamp = self.get_clock().now().to_msg()
        self.target_pose.header.frame_id = "map"
        
        # Publish setpoint continuously (required for offboard mode)
        self.local_pos_pub.publish(self.target_pose)
        
        # Wait for FCU connection
        if not self.current_state.connected:
            if self.loop_count % 40 == 0:  # Log every 2 seconds
                self.get_logger().info('Waiting for FCU connection...')
            return
        
        # Start mission sequence
        if not self.mission_started:
            self.get_logger().info('FCU connected! Starting mission sequence...')
            self.mission_started = True
        
        # Send some setpoints before entering OFFBOARD mode (PX4 requirement)
        if not self.offboard_set and self.loop_count < 100:
            if self.loop_count % 20 == 0:
                self.get_logger().info(f'Sending initial setpoints... ({self.loop_count}/100)')
            return
        
        # Set OFFBOARD mode
        if not self.offboard_set:
            if self.current_state.mode != "OFFBOARD":
                self.get_logger().info('Setting OFFBOARD mode...')
                self.set_mode("OFFBOARD")
            else:
                self.get_logger().info('OFFBOARD mode set')
                self.offboard_set = True
        
        # Arm the vehicle
        if self.offboard_set and not self.armed:
            if not self.current_state.armed:
                self.get_logger().info('Arming vehicle...')
                self.arm_vehicle(True)
            else:
                self.get_logger().info('Vehicle armed!')
                self.armed = True
        
        # Check if waypoint is reached
        if self.armed and not self.waypoint_reached:
            if self.loop_count % 40 == 0:  # Log every 2 seconds
                self.get_logger().info('Flying to waypoint...')
            
            # In a real implementation, you would check the current position
            # and compare it to the target. For this demo, we'll assume
            # the waypoint is reached after a certain time
            if self.loop_count > 500:  # ~25 seconds
                self.waypoint_reached = True
                self.get_logger().info('Waypoint reached! Holding position.')
        
        # Hold position at waypoint
        if self.waypoint_reached:
            if self.loop_count % 100 == 0:  # Log every 5 seconds
                self.get_logger().info('Holding position at waypoint')

    def set_mode(self, mode):
        """Set flight mode"""
        request = SetMode.Request()
        request.custom_mode = mode
        
        future = self.set_mode_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
        
        if future.result() is not None:
            if future.result().mode_sent:
                self.get_logger().info(f'Mode {mode} sent successfully')
            else:
                self.get_logger().warn(f'Failed to set mode {mode}')
        else:
            self.get_logger().error('Service call failed')

    def arm_vehicle(self, arm):
        """Arm or disarm the vehicle"""
        request = CommandBool.Request()
        request.value = arm
        
        future = self.arming_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
        
        if future.result() is not None:
            if future.result().success:
                self.get_logger().info('Arming command sent successfully')
            else:
                self.get_logger().warn('Failed to send arming command')
        else:
            self.get_logger().error('Service call failed')


def main(args=None):
    rclpy.init(args=args)
    
    navigator = WaypointNavigator()
    
    try:
        rclpy.spin(navigator)
    except KeyboardInterrupt:
        navigator.get_logger().info('Keyboard interrupt, shutting down...')
    finally:
        navigator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

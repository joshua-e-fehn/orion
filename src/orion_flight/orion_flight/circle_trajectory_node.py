#!/usr/bin/env python3

"""
Autonomous Circular Trajectory Flight Node with RViz Visualization

This node controls a drone to:
1. Takeoff to a specified height
2. Fly in a circular pattern
3. Visualize the last 10 positions in RViz
4. Land after completing circles

Author: Orion ARM Team
Date: November 13, 2025
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus, VehicleCommandAck
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from collections import deque
import math
import numpy as np


class CircleTrajectoryNode(Node):
    """Node for autonomous circular trajectory flight with visualization."""

    def __init__(self) -> None:
        super().__init__('circle_trajectory_node')

        # Declare parameters
        self.declare_parameter('circle_radius', 4.0)  # meters
        self.declare_parameter('flight_height', -5.0)  # meters (NED frame, negative is up)
        self.declare_parameter('angular_velocity', 0.3)  # rad/s
        self.declare_parameter('num_circles', 2)  # number of circles to complete
        self.declare_parameter('trail_length', 10)  # number of positions to show in trail

        # Get parameters
        self.circle_radius = self.get_parameter('circle_radius').value
        self.flight_height = self.get_parameter('flight_height').value
        self.angular_velocity = self.get_parameter('angular_velocity').value
        self.num_circles = self.get_parameter('num_circles').value
        self.trail_length = self.get_parameter('trail_length').value

        # Configure QoS profile for publishing (match PX4's VOLATILE durability)
        qos_profile_pub = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Configure QoS profile for subscribing (use TRANSIENT_LOCAL for receiving)
        qos_profile_sub = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Create publishers
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile_pub)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile_pub)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile_pub)
        
        # Publisher for visualization
        self.marker_publisher = self.create_publisher(
            MarkerArray, '/trajectory_markers', 10)

        # Create subscribers
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1', 
            self.vehicle_local_position_callback, qos_profile_sub)
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status_v1', 
            self.vehicle_status_callback, qos_profile_sub)
        self.vehicle_command_ack_subscriber = self.create_subscription(
            VehicleCommandAck, '/fmu/out/vehicle_command_ack',
            self.vehicle_command_ack_callback, qos_profile_sub)

        # Initialize variables
        self.offboard_setpoint_counter = 0
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.position_history = deque(maxlen=self.trail_length)
        
        # Flight state machine
        self.flight_phase = "INIT"  # INIT -> TAKEOFF -> CIRCLE -> LAND -> COMPLETE
        self.circle_start_time = None
        self.total_angle_traveled = 0.0
        self.last_update_time = self.get_clock().now()
        
        # Arm retry mechanism
        self.arm_retry_counter = 0
        self.max_arm_retries = 10
        self.is_armed = False
        self.is_offboard = False

        # Create a timer to publish control commands at 10Hz
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.get_logger().info('Circle Trajectory Node Started!')
        self.get_logger().info(f'Parameters: radius={self.circle_radius}m, '
                              f'height={-self.flight_height}m, '
                              f'angular_vel={self.angular_velocity}rad/s, '
                              f'circles={self.num_circles}')

    def vehicle_local_position_callback(self, vehicle_local_position):
        """Callback function for vehicle_local_position topic subscriber."""
        self.vehicle_local_position = vehicle_local_position
        if self.offboard_setpoint_counter % 20 == 0:  # Log every 2 seconds
            self.get_logger().info(f'[{self.flight_phase}] Position: x={vehicle_local_position.x:.2f}, y={vehicle_local_position.y:.2f}, z={vehicle_local_position.z:.2f}')
        
        # Add position to history for visualization (convert NED to visualization frame)
        if self.flight_phase in ["CIRCLE", "LAND"]:
            self.position_history.append({
                'x': vehicle_local_position.x,
                'y': vehicle_local_position.y,
                'z': vehicle_local_position.z
            })

    def vehicle_status_callback(self, vehicle_status):
        """Callback function for vehicle_status topic subscriber."""
        self.vehicle_status = vehicle_status
        
        # Update status flags
        # Arming states: 1=INIT, 2=STANDBY, 3=ARMED, 4=STANDBY_ERROR, 5=SHUTDOWN, 6=IN_AIR_RESTORE
        self.is_armed = (vehicle_status.arming_state >= 2)  # STANDBY or ARMED
        # Accept OFFBOARD or AUTO_TAKEOFF (PX4 may refuse OFFBOARD from AUTO_TAKEOFF)
        self.is_offboard = (vehicle_status.nav_state == 7 or vehicle_status.nav_state == 14)  # OFFBOARD=7, AUTO_TAKEOFF=14
        
        if self.offboard_setpoint_counter % 20 == 0:  # Log every 2 seconds
            nav_state_names = {
                0: "MANUAL", 1: "ALTCTL", 2: "POSCTL", 3: "AUTO_MISSION",
                4: "AUTO_LOITER", 5: "AUTO_RTL", 6: "ACRO", 7: "OFFBOARD",
                8: "STAB", 14: "AUTO_TAKEOFF", 15: "AUTO_LAND", 17: "AUTO_PRECLAND"
            }
            arming_state_names = {1: "INIT", 2: "STANDBY", 3: "ARMED", 4: "STANDBY_ERROR", 5: "SHUTDOWN", 6: "IN_AIR_RESTORE"}
            nav_name = nav_state_names.get(vehicle_status.nav_state, f"UNKNOWN({vehicle_status.nav_state})")
            arm_name = arming_state_names.get(vehicle_status.arming_state, f"UNKNOWN({vehicle_status.arming_state})")
            self.get_logger().info(f'[{self.flight_phase}] Status: nav_state={nav_name}, arming_state={arm_name}')

    def vehicle_command_ack_callback(self, ack):
        """Callback function for vehicle_command_ack topic subscriber."""
        command_names = {
            400: "ARM_DISARM",
            176: "DO_SET_MODE",
            21: "NAV_LAND",
            22: "NAV_TAKEOFF"
        }
        result_names = {
            0: "ACCEPTED",
            1: "TEMPORARILY_REJECTED",
            2: "DENIED",
            3: "UNSUPPORTED",
            4: "FAILED",
            5: "IN_PROGRESS",
            6: "CANCELLED"
        }
        cmd_name = command_names.get(ack.command, f"CMD_{ack.command}")
        result_name = result_names.get(ack.result, f"RESULT_{ack.result}")
        
        if ack.result == 0:  # ACCEPTED
            self.get_logger().info(f'✓ Command {cmd_name} {result_name}')
        elif ack.command == 400:  # ARM_DISARM - always log rejections
            self.get_logger().warn(f'✗ Command {cmd_name} {result_name} (result_param2={ack.result_param2})')
        else:
            self.get_logger().warn(f'✗ Command {cmd_name} {result_name}')

    def arm(self):
        """Send an arm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0, param2=21196.0)
        self.get_logger().info('Arm command sent (force arm)')

    def disarm(self):
        """Send a disarm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info('Disarm command sent')

    def engage_manual_mode(self):
        """Switch to manual mode (required before offboard from AUTO modes)."""
        # MAV_MODE_FLAG_CUSTOM_MODE_ENABLED (1) + main_mode (1 = MANUAL)
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=1.0)
        self.get_logger().info("Switching to manual mode")

    def engage_offboard_mode(self):
        """Switch to offboard mode."""
        # For offboard: base_mode=MAV_MODE_FLAG_CUSTOM_MODE_ENABLED (1)
        # main_mode=1 (PX4_CUSTOM_MAIN_MODE_MANUAL), sub_mode=6 (OFFBOARD)
        # Actually use: param1=1 (base mode), param2=6 (PX4 custom mode for offboard)
        # Correct: 1 | (1 << 16) for base, 6 for custom sub mode
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0, param3=0.0)
        self.get_logger().info("Switching to offboard mode")

    def land(self):
        """Switch to land mode."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info("Switching to land mode")

    def publish_offboard_control_heartbeat_signal(self):
        """Publish the offboard control mode."""
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)
        
        # Debug: Log occasionally
        if self.offboard_setpoint_counter % 50 == 0:
            self.get_logger().debug('Published offboard control mode (position control)')

    def publish_position_setpoint(self, x: float, y: float, z: float, yaw: float = 0.0):
        """Publish the trajectory setpoint."""
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = yaw
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)
        
        # Debug: Log occasionally
        if self.offboard_setpoint_counter % 50 == 0:
            self.get_logger().info(f'[{self.flight_phase}] Publishing setpoint: x={x:.2f}, y={y:.2f}, z={z:.2f}, yaw={yaw:.2f}')

    def publish_vehicle_command(self, command, **params) -> None:
        """Publish a vehicle command."""
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

    def calculate_circle_position(self, angle: float):
        """Calculate position on circle given an angle."""
        x = self.circle_radius * math.cos(angle)
        y = self.circle_radius * math.sin(angle)
        z = self.flight_height
        yaw = angle + math.pi / 2  # Tangent to circle
        return x, y, z, yaw

    def publish_trajectory_markers(self):
        """Publish MarkerArray for visualization in RViz."""
        marker_array = MarkerArray()
        
        # Create line strip for trajectory trail
        trail_marker = Marker()
        trail_marker.header.frame_id = "map"
        trail_marker.header.stamp = self.get_clock().now().to_msg()
        trail_marker.ns = "trajectory_trail"
        trail_marker.id = 0
        trail_marker.type = Marker.LINE_STRIP
        trail_marker.action = Marker.ADD
        trail_marker.pose.orientation.w = 1.0
        trail_marker.scale.x = 0.1  # Line width
        trail_marker.color.r = 0.0
        trail_marker.color.g = 1.0
        trail_marker.color.b = 1.0
        trail_marker.color.a = 1.0

        for pos in self.position_history:
            point = Point()
            point.x = pos['x']
            point.y = pos['y']
            point.z = -pos['z']  # Convert NED to visualization (Z-up)
            trail_marker.points.append(point)

        marker_array.markers.append(trail_marker)

        # Create sphere markers for each position
        for i, pos in enumerate(self.position_history):
            sphere_marker = Marker()
            sphere_marker.header.frame_id = "map"
            sphere_marker.header.stamp = self.get_clock().now().to_msg()
            sphere_marker.ns = "trajectory_points"
            sphere_marker.id = i + 1
            sphere_marker.type = Marker.SPHERE
            sphere_marker.action = Marker.ADD
            sphere_marker.pose.position.x = pos['x']
            sphere_marker.pose.position.y = pos['y']
            sphere_marker.pose.position.z = -pos['z']  # Convert NED to visualization
            sphere_marker.pose.orientation.w = 1.0
            sphere_marker.scale.x = 0.3
            sphere_marker.scale.y = 0.3
            sphere_marker.scale.z = 0.3
            
            # Color gradient from red (old) to green (new)
            ratio = i / max(len(self.position_history) - 1, 1)
            sphere_marker.color.r = 1.0 - ratio
            sphere_marker.color.g = ratio
            sphere_marker.color.b = 0.0
            sphere_marker.color.a = 0.8
            
            marker_array.markers.append(sphere_marker)

        # Create current position marker (larger and blue)
        if len(self.position_history) > 0:
            current_marker = Marker()
            current_marker.header.frame_id = "map"
            current_marker.header.stamp = self.get_clock().now().to_msg()
            current_marker.ns = "current_position"
            current_marker.id = 100
            current_marker.type = Marker.SPHERE
            current_marker.action = Marker.ADD
            pos = self.position_history[-1]
            current_marker.pose.position.x = pos['x']
            current_marker.pose.position.y = pos['y']
            current_marker.pose.position.z = -pos['z']
            current_marker.pose.orientation.w = 1.0
            current_marker.scale.x = 0.5
            current_marker.scale.y = 0.5
            current_marker.scale.z = 0.5
            current_marker.color.r = 0.0
            current_marker.color.g = 0.0
            current_marker.color.b = 1.0
            current_marker.color.a = 1.0
            
            marker_array.markers.append(current_marker)

        self.marker_publisher.publish(marker_array)

    def timer_callback(self) -> None:
        """Main control loop callback."""
        # Always publish offboard control mode heartbeat
        self.publish_offboard_control_heartbeat_signal()

        # State machine logic
        if self.flight_phase == "INIT":
            # Send setpoints before engaging offboard mode
            if self.offboard_setpoint_counter == 5:
                # First switch to manual mode (required transition from AUTO_TAKEOFF)
                self.engage_manual_mode()
                self.get_logger().info("✓ Switching from AUTO to MANUAL mode")
            
            if self.offboard_setpoint_counter == 10:
                # Now switch to offboard mode
                self.engage_offboard_mode()
                self.arm()
                self.flight_phase = "TAKEOFF"
                self.get_logger().info("✓ Transitioning to TAKEOFF phase")

            self.publish_position_setpoint(0.0, 0.0, self.flight_height)
            
            if self.offboard_setpoint_counter < 11:
                self.offboard_setpoint_counter += 1
                if self.offboard_setpoint_counter % 5 == 0:
                    self.get_logger().info(f'[INIT] Sending pre-offboard setpoints ({self.offboard_setpoint_counter}/10)')

        elif self.flight_phase == "TAKEOFF":
            # Publish takeoff position
            self.publish_position_setpoint(0.0, 0.0, self.flight_height)
            
            # Increment counter for logging
            self.offboard_setpoint_counter += 1
            
            # Retry arming if not armed yet
            if not self.is_armed and self.arm_retry_counter < self.max_arm_retries:
                if self.offboard_setpoint_counter % 10 == 0:  # Retry every second
                    self.arm()
                    self.arm_retry_counter += 1
                    self.get_logger().warn(f'Retrying arm command ({self.arm_retry_counter}/{self.max_arm_retries})')
            
            # Also retry offboard mode if not in offboard
            if not self.is_offboard:
                if self.offboard_setpoint_counter % 10 == 0:  # Retry every second
                    self.engage_offboard_mode()
                    self.get_logger().warn('Retrying offboard mode command')
            
            # Check if reached target height (with some tolerance)
            # In NED frame, z is DOWN, so target height of -5m means z should be around -5
            if self.vehicle_local_position is not None:
                current_z = self.vehicle_local_position.z
                target_z = self.flight_height
                altitude_error = abs(current_z - target_z)
                
                # Debug output every 2 seconds
                if self.offboard_setpoint_counter % 20 == 0:
                    armed_str = "✓ ARMED" if self.is_armed else "✗ NOT ARMED"
                    offboard_str = "✓ OFFBOARD" if self.is_offboard else "✗ NOT OFFBOARD"
                    self.get_logger().info(
                        f'[TAKEOFF] {armed_str}, {offboard_str} | '
                        f'Current z={current_z:.2f}m, Target z={target_z:.2f}m, '
                        f'Error={altitude_error:.2f}m'
                    )
                
                # Transition to CIRCLE when at target altitude (altitude_error already calculated above)
                if self.is_armed and self.is_offboard and altitude_error < 0.2:  # Within 20cm of target
                    self.flight_phase = "CIRCLE"
                    self.circle_start_time = self.get_clock().now()
                    self.total_angle_traveled = 0.0
                    self.get_logger().info(f"✓ Reached target height! Current z={current_z:.2f}m, Target z={target_z:.2f}m. Starting circular trajectory.")
            else:
                if self.offboard_setpoint_counter % 20 == 0:
                    self.get_logger().warn("[TAKEOFF] Waiting for vehicle position data...")

        elif self.flight_phase == "CIRCLE":
            # Calculate current angle based on time
            current_time = self.get_clock().now()
            dt = (current_time - self.last_update_time).nanoseconds / 1e9
            self.last_update_time = current_time
            
            angle_increment = self.angular_velocity * dt
            self.total_angle_traveled += angle_increment
            
            # Calculate and publish circular trajectory setpoint
            x, y, z, yaw = self.calculate_circle_position(self.total_angle_traveled)
            self.publish_position_setpoint(x, y, z, yaw)
            
            # Check if completed required number of circles
            total_circles_completed = self.total_angle_traveled / (2 * math.pi)
            if total_circles_completed >= self.num_circles:
                self.flight_phase = "LAND"
                self.get_logger().info(f"Completed {self.num_circles} circles, initiating landing")
                self.land()

        elif self.flight_phase == "LAND":
            # Keep publishing last position until landed
            # Landing is handled by PX4 land mode
            if self.vehicle_local_position.z > -0.5:  # Close to ground
                self.flight_phase = "COMPLETE"
                self.get_logger().info("Landing complete!")

        elif self.flight_phase == "COMPLETE":
            # Mission complete, just maintain state
            pass

        # Publish visualization markers
        if len(self.position_history) > 0:
            self.publish_trajectory_markers()


def main(args=None) -> None:
    print('Starting Circle Trajectory Node...')
    rclpy.init(args=args)
    circle_trajectory_node = CircleTrajectoryNode()
    rclpy.spin(circle_trajectory_node)
    circle_trajectory_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(f"Error: {e}")

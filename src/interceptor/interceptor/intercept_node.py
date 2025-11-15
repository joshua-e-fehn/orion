#!/usr/bin/env python3
"""
Autonomous Hover Flight Node with RViz Visualization

This node controls a drone to:
1. Takeoff to a specified height
2. Hover at that height
3. Visualize the positions in RViz
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus, VehicleCommandAck
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from collections import deque

class InterceptNode(Node):
    """Node for autonomous hover flight with visualization."""

    def __init__(self):
        super().__init__('intercept_node')

        # Parameters
        self.declare_parameter('flight_height', -5.0)  # NED frame (negative = up)
        self.declare_parameter('trail_length', 10)
        self.flight_height = self.get_parameter('flight_height').value
        self.trail_length = self.get_parameter('trail_length').value

        # QoS
        qos_pub = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                             durability=DurabilityPolicy.VOLATILE,
                             history=HistoryPolicy.KEEP_LAST,
                             depth=1)
        qos_sub = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST,
                             depth=1)

        # Publishers
        self.offboard_control_mode_pub = self.create_publisher(OffboardControlMode, '/px4_2/fmu/in/offboard_control_mode', qos_pub)
        self.trajectory_setpoint_pub = self.create_publisher(TrajectorySetpoint, '/px4_2/fmu/in/trajectory_setpoint', qos_pub)
        self.vehicle_command_pub = self.create_publisher(VehicleCommand, '/px4_2/fmu/in/vehicle_command', qos_pub)
        self.marker_pub = self.create_publisher(MarkerArray, '/trajectory_markers', 10)

        # Subscribers
        self.vehicle_pos_sub = self.create_subscription(VehicleLocalPosition, '/px4_2/fmu/out/vehicle_local_position_v1', self.vehicle_local_position_callback, qos_sub)
        self.vehicle_status_sub = self.create_subscription(VehicleStatus, '/px4_2/fmu/out/vehicle_status_v1', self.vehicle_status_callback, qos_sub)
        self.vehicle_ack_sub = self.create_subscription(VehicleCommandAck, '/px4_2/fmu/out/vehicle_command_ack', self.vehicle_command_ack_callback, qos_sub)

        # State
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.position_history = deque(maxlen=self.trail_length)
        self.flight_phase = "INIT"
        self.offboard_setpoint_counter = 0
        self.last_update_time = self.get_clock().now()
        self.is_armed = False
        self.is_offboard = False
        self.arm_retry_counter = 0
        self.max_arm_retries = 10

        # Timer
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info("HoverNode started!")

    def vehicle_local_position_callback(self, msg):
        self.vehicle_local_position = msg
        if self.flight_phase in ["HOVER"]:
            self.position_history.append({'x': msg.x, 'y': msg.y, 'z': msg.z})

    def vehicle_status_callback(self, msg):
        self.vehicle_status = msg
        self.is_armed = (msg.arming_state >= 2)
        self.is_offboard = (msg.nav_state == 7 or msg.nav_state == 14)

    def vehicle_command_ack_callback(self, ack):
        if ack.result == 0:
            self.get_logger().info(f"✓ Command {ack.command} ACCEPTED")
        else:
            self.get_logger().warn(f"✗ Command {ack.command} REJECTED (result_param2={ack.result_param2})")

    def arm(self):
        msg = VehicleCommand()
        msg.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        msg.param1 = 1.0
        msg.param2 = 21196.0  # force arm
        msg.target_system = 0
        msg.target_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_pub.publish(msg)
        self.get_logger().info("Arm command sent")

    def engage_offboard_mode(self):
        msg = VehicleCommand()
        msg.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
        msg.param1 = 1.0  # base mode
        msg.param2 = 6.0  # OFFBOARD
        msg.target_system = 0
        msg.target_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_pub.publish(msg)
        self.get_logger().info("Switching to OFFBOARD mode")

    def publish_offboard_control_heartbeat(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_pub.publish(msg)

    def publish_position_setpoint(self, x=0.0, y=0.0, z=None):
        z = self.flight_height if z is None else z
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_pub.publish(msg)

    def publish_markers(self):
        marker_array = MarkerArray()
        for i, pos in enumerate(self.position_history):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "trajectory_points"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = pos['x']
            marker.pose.position.y = pos['y']
            marker.pose.position.z = -pos['z']
            marker.scale.x = marker.scale.y = marker.scale.z = 0.3
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 1.0
            marker.color.a = 0.8
            marker_array.markers.append(marker)
        self.marker_pub.publish(marker_array)

    def timer_callback(self):
        self.publish_offboard_control_heartbeat()
        self.publish_position_setpoint()
        self.publish_markers()

        # INIT phase
        if self.flight_phase == "INIT":
            self.engage_offboard_mode()
            self.arm()
            self.flight_phase = "HOVER"
            self.get_logger().info("✓ Takeoff completed, hovering at target height")

def main(args=None):
    rclpy.init(args=args)
    intercept_node = InterceptNode()
    rclpy.spin(intercept_node)
    intercept_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""
Autonomous Straight-Line Flight Node with RViz Visualization

This node controls a drone to:
1. Takeoff to a specified height
2. Fly in a straight line along the X-axis
3. Visualize the positions in RViz
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus, VehicleCommandAck
from visualization_msgs.msg import Marker, MarkerArray
from collections import deque
import numpy as np
import math

class StraightLineNode(Node):
    """Node for autonomous straight-line flight with visualization."""

    def __init__(self):
        super().__init__('straight_line_node')
        # Parameters (basic)
        self.declare_parameter('flight_height', -5.0)  # NED frame (negative = up)
        self.declare_parameter('trail_length', 20)
        self.declare_parameter('speed', 1.0)  # forward speed (m/s)

        # Trajectory pattern parameters
        # pattern options: straight, sine, circle, figure8, lissajous, lawnmower, helix, spiral, random_walk, waypoints
        self.declare_parameter('pattern', 'sine')
        self.declare_parameter('amplitude_y', 2.0)     # lateral amplitude (m)
        self.declare_parameter('amplitude_z', 0.5)     # vertical amplitude (m, around flight_height)
        self.declare_parameter('freq_y', 0.1)          # Hz for lateral oscillation
        self.declare_parameter('freq_z', 0.05)         # Hz for vertical oscillation
        self.declare_parameter('radius', 3.0)          # radius for circle / figure8 (m)
        self.declare_parameter('lissajous_ratio', 2.0) # frequency ratio for lissajous pattern

        # Non-oscillatory patterns configuration
        self.declare_parameter('area_length', 30.0)    # lawnmower length along x (m)
        self.declare_parameter('area_width', 20.0)     # lawnmower total width along y (m)
        self.declare_parameter('sweep_spacing', 3.0)   # spacing between sweeps (m)
        self.declare_parameter('helix_pitch', 1.0)     # vertical meters per revolution
        self.declare_parameter('spiral_growth', 0.3)   # meters per rad (Archimedean spiral)
        self.declare_parameter('rw_speed', 1.0)        # random walk speed (m/s)
        self.declare_parameter('rw_turn_rate', 0.6)    # random walk max turn rate (rad/s)
        self.declare_parameter('yaw_align', True)      # align yaw to motion
        self.declare_parameter('loop_path', True)      # loop waypoint/path patterns
        # Waypoints as flattened list [x1,y1,z1,x2,y2,z2,...] relative to start
        self.declare_parameter('waypoints', [])

        # Load parameters
        self.flight_height = float(self.get_parameter('flight_height').value)
        self.trail_length = int(self.get_parameter('trail_length').value)
        self.speed = float(self.get_parameter('speed').value)
        self.pattern = self.get_parameter('pattern').value
        self.amp_y = float(self.get_parameter('amplitude_y').value)
        self.amp_z = float(self.get_parameter('amplitude_z').value)
        self.freq_y = float(self.get_parameter('freq_y').value)
        self.freq_z = float(self.get_parameter('freq_z').value)
        self.radius = float(self.get_parameter('radius').value)
        self.liss_ratio = float(self.get_parameter('lissajous_ratio').value)
        self.area_length = float(self.get_parameter('area_length').value)
        self.area_width = float(self.get_parameter('area_width').value)
        self.sweep_spacing = float(self.get_parameter('sweep_spacing').value)
        self.helix_pitch = float(self.get_parameter('helix_pitch').value)
        self.spiral_growth = float(self.get_parameter('spiral_growth').value)
        self.rw_speed = float(self.get_parameter('rw_speed').value)
        self.rw_turn_rate = float(self.get_parameter('rw_turn_rate').value)
        self.yaw_align = bool(self.get_parameter('yaw_align').value)
        self.loop_path = bool(self.get_parameter('loop_path').value)
        # Parse waypoints list
        self.waypoints = []
        _wps = self.get_parameter('waypoints').value
        if isinstance(_wps, (list, tuple)) and len(_wps) >= 3 and len(_wps) % 3 == 0:
            it = iter([float(v) for v in _wps])
            self.waypoints = [(x, y, z) for x, y, z in zip(it, it, it)]

        # QoS profiles for PX4 communication
        # Publisher QoS: Best effort, volatile, depth 1
        qos_pub = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Subscriber QoS: Best effort, volatile (matching PX4 publishers)
        # Large depth to handle all message buffering and prevent DDS errors
        qos_sub = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=100  # Large depth to prevent payload size errors
        )

        # Publishers
        self.offboard_control_mode_pub = self.create_publisher(OffboardControlMode, '/px4_1/fmu/in/offboard_control_mode', qos_pub)
        self.trajectory_setpoint_pub = self.create_publisher(TrajectorySetpoint, '/px4_1/fmu/in/trajectory_setpoint', qos_pub)
        self.vehicle_command_pub = self.create_publisher(VehicleCommand, '/px4_1/fmu/in/vehicle_command', qos_pub)
        self.marker_pub = self.create_publisher(MarkerArray, '/trajectory_markers', 10)

        # Subscribers
        self.vehicle_pos_sub = self.create_subscription(VehicleLocalPosition, '/px4_1/fmu/out/vehicle_local_position_v1', self.vehicle_local_position_callback, qos_sub)
        self.vehicle_status_sub = self.create_subscription(VehicleStatus, '/px4_1/fmu/out/vehicle_status_v1', self.vehicle_status_callback, qos_sub)
        self.vehicle_ack_sub = self.create_subscription(VehicleCommandAck, '/px4_1/fmu/out/vehicle_command_ack', self.vehicle_command_ack_callback, qos_sub)

        # State
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.position_history = deque(maxlen=self.trail_length)
        self.flight_phase = "INIT"
        self.offboard_setpoint_counter = 0
        self.last_update_time = self.get_clock().now()
        self.is_armed = False
        self.is_offboard = False
        self.init_position_received = False

        # Flight path origin (set after first valid position)
        self.start_x = 0.0
        self.start_y = 0.0
        self.start_z = self.flight_height
        self.current_x = 0.0
        self.trajectory_time = 0.0  # seconds since start of active pattern

        # Path follower state (for lawnmower/waypoints)
        self.path_points = []  # list of (x,y,z) in world frame
        self.path_index = 0
        self.path_progress = 0.0  # progress along current segment [0,1]

        # Random walk state
        self.rw_heading = 0.0
        self.rw_pos = np.array([0.0, 0.0])  # relative to start

        # Previous setpoint for velocity estimation
        self.prev_sp = None  # (x, y, z)

        # Precompute path if applicable
        self._init_pattern_paths()

        # Timer (10 Hz for control loop)
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info("StraightLineNode started!")

    def _init_pattern_paths(self):
        # Initialize paths for patterns that need precomputed waypoints
        if self.pattern in ['lawnmower', 'square', 'waypoints']:
            self.path_points.clear()
            self.path_index = 0
            self.path_progress = 0.0
            # Ensure start set later when initial position is known

    def _maybe_build_path(self):
        # Build path after initial position known
        if not self.init_position_received:
            return
        if self.pattern == 'lawnmower' and len(self.path_points) == 0:
            # Generate boustrophedon path within rectangle centered on start
            L = self.area_length
            W = self.area_width
            dx = self.sweep_spacing
            x0 = self.start_x - L/2.0
            y0 = self.start_y - W/2.0
            rows = max(1, int(max(1, math.floor(W / max(0.1, dx)))))
            y_vals = [y0 + i * (W / rows) for i in range(rows + 1)]
            # Build alternating sweeps along x
            pts = []
            for i, y in enumerate(y_vals):
                if i % 2 == 0:
                    pts.append((x0, y, self.flight_height))
                    pts.append((x0 + L, y, self.flight_height))
                else:
                    pts.append((x0 + L, y, self.flight_height))
                    pts.append((x0, y, self.flight_height))
            self.path_points = pts
        elif self.pattern == 'square' and len(self.path_points) == 0:
            L = self.area_length
            W = self.area_width
            x0 = self.start_x - L/2.0
            y0 = self.start_y - W/2.0
            self.path_points = [
                (x0, y0, self.flight_height),
                (x0 + L, y0, self.flight_height),
                (x0 + L, y0 + W, self.flight_height),
                (x0, y0 + W, self.flight_height),
            ]
        elif self.pattern == 'waypoints' and len(self.path_points) == 0 and len(self.waypoints) > 0:
            # Interpret waypoints relative to start
            self.path_points = [
                (self.start_x + dx, self.start_y + dy, self.flight_height + dz)
                for (dx, dy, dz) in self.waypoints
            ]

    def vehicle_local_position_callback(self, msg):
        self.vehicle_local_position = msg
        
        # Record initial position
        if not self.init_position_received and msg.xy_valid:
            self.start_x = msg.x
            self.start_y = msg.y
            self.current_x = msg.x
            self.init_position_received = True
            self.get_logger().info(f"Initial position: x={msg.x:.2f}, y={msg.y:.2f}, z={msg.z:.2f}")
        
        if self.flight_phase == "STRAIGHT_LINE":
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
        msg.param2 = 21196.0
        msg.target_system = 0
        msg.target_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_pub.publish(msg)
        self.get_logger().info("Arm command sent")

    def engage_offboard_mode(self):
        msg = VehicleCommand()
        msg.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
        msg.param1 = 1.0
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
        msg.velocity = True  # helps stabilize in some setups
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_pub.publish(msg)

    def publish_position_setpoint(self):
        """Publish trajectory setpoint based on selected pattern."""
        msg = TrajectorySetpoint()
        now = self.get_clock().now()
        dt = (now - self.last_update_time).nanoseconds / 1e9
        self.last_update_time = now

        if self.flight_phase == "INIT":
            # Hold initial position while initializing
            x = self.start_x
            y = self.start_y
            z = self.flight_height
        else:
            # Advance trajectory time
            self.trajectory_time += max(dt, 0.0)
            t = self.trajectory_time

            if self.pattern == 'straight':
                x = self.start_x + self.speed * t
                y = self.start_y
                z = self.flight_height
            elif self.pattern == 'sine':
                x = self.start_x + self.speed * t
                y = self.start_y + self.amp_y * np.sin(2.0 * np.pi * self.freq_y * t)
                z = self.flight_height + self.amp_z * np.sin(2.0 * np.pi * self.freq_z * t)
            elif self.pattern == 'circle':
                omega = 2.0 * np.pi * self.freq_y
                x = self.start_x + self.radius * np.cos(omega * t)
                y = self.start_y + self.radius * np.sin(omega * t)
                z = self.flight_height + self.amp_z * np.sin(2.0 * np.pi * self.freq_z * t)
            elif self.pattern == 'figure8':
                omega = 2.0 * np.pi * self.freq_y
                x = self.start_x + self.radius * np.sin(omega * t)
                y = self.start_y + self.radius * np.sin(omega * t) * np.cos(omega * t) * 2.0
                z = self.flight_height + self.amp_z * np.sin(2.0 * np.pi * self.freq_z * t)
            elif self.pattern == 'lissajous':
                omega = 2.0 * np.pi * self.freq_y
                x = self.start_x + self.radius * np.sin(omega * t)
                y = self.start_y + self.radius * np.sin(omega * self.liss_ratio * t + np.pi/2.0)
                z = self.flight_height + self.amp_z * np.sin(2.0 * np.pi * self.freq_z * t)
            elif self.pattern == 'helix':
                omega = 2.0 * np.pi * self.freq_y
                revs = omega * t / (2.0 * np.pi)
                x = self.start_x + self.radius * np.cos(omega * t)
                y = self.start_y + self.radius * np.sin(omega * t)
                z = self.flight_height + self.helix_pitch * revs + self.amp_z * np.sin(2.0 * np.pi * self.freq_z * t)
            elif self.pattern == 'spiral':
                omega = 2.0 * np.pi * self.freq_y
                theta = omega * t
                r = self.spiral_growth * theta
                x = self.start_x + r * np.cos(theta)
                y = self.start_y + r * np.sin(theta)
                z = self.flight_height + self.amp_z * np.sin(2.0 * np.pi * self.freq_z * t)
            elif self.pattern in ['lawnmower', 'square', 'waypoints']:
                self._maybe_build_path()
                if len(self.path_points) >= 2:
                    # Progress along current segment at constant speed
                    p0 = np.array(self.path_points[self.path_index % len(self.path_points)])
                    p1 = np.array(self.path_points[(self.path_index + 1) % len(self.path_points)])
                    seg = p1 - p0
                    seg_len = max(1e-3, float(np.linalg.norm(seg[:2])))
                    # Advance along segment based on planar distance
                    advance = (self.speed * dt) / seg_len
                    self.path_progress += advance
                    while self.path_progress >= 1.0:
                        self.path_progress -= 1.0
                        self.path_index += 1
                        if not self.loop_path and self.path_index >= len(self.path_points) - 1:
                            self.path_index = len(self.path_points) - 2
                            self.path_progress = 1.0  # stop at end
                            break
                        p0 = np.array(self.path_points[self.path_index % len(self.path_points)])
                        p1 = np.array(self.path_points[(self.path_index + 1) % len(self.path_points)])
                        seg = p1 - p0
                        seg_len = max(1e-3, float(np.linalg.norm(seg[:2])))
                    pos = p0 + self.path_progress * seg
                    x, y = float(pos[0]), float(pos[1])
                    # Add vertical motion for path patterns using existing amplitude_z/freq_z
                    base_z = float(pos[2])
                    z = base_z + (self.amp_z * np.sin(2.0 * np.pi * self.freq_z * t) if self.amp_z != 0.0 and self.freq_z != 0.0 else 0.0)
                else:
                    x = self.start_x
                    y = self.start_y
                    z = self.flight_height
            elif self.pattern == 'random_walk':
                # Smooth heading changes, integrate position
                d_heading = np.clip(np.random.randn() * self.rw_turn_rate, -self.rw_turn_rate, self.rw_turn_rate)
                self.rw_heading += d_heading * dt
                v = self.rw_speed
                self.rw_pos += np.array([v * math.cos(self.rw_heading), v * math.sin(self.rw_heading)]) * dt
                x = self.start_x + float(self.rw_pos[0])
                y = self.start_y + float(self.rw_pos[1])
                z = self.flight_height + self.amp_z * np.sin(2.0 * np.pi * self.freq_z * t)
            else:
                # Fallback to straight
                x = self.start_x + self.speed * t
                y = self.start_y
                z = self.flight_height

        # Ensure altitude remains negative (NED)
        z = min(z, -0.1)

        # Estimate velocity from previous setpoint
        if self.prev_sp is not None and dt > 1e-3:
            vx = (x - self.prev_sp[0]) / dt
            vy = (y - self.prev_sp[1]) / dt
            vz = (z - self.prev_sp[2]) / dt
            msg.velocity = [float(vx), float(vy), float(vz)]
        # Yaw alignment
        if self.yaw_align and self.prev_sp is not None:
            dx = x - self.prev_sp[0]
            dy = y - self.prev_sp[1]
            if abs(dx) + abs(dy) > 1e-4:
                msg.yaw = float(math.atan2(dy, dx))

        msg.position = [float(x), float(y), float(z)]
        msg.timestamp = int(now.nanoseconds / 1000)
        self.trajectory_setpoint_pub.publish(msg)
        self.prev_sp = (x, y, z)

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
        # Always send offboard heartbeat and setpoints
        self.publish_offboard_control_heartbeat()
        self.publish_position_setpoint()
        
        if self.flight_phase == "INIT":
            # Wait for initial position before proceeding
            if not self.init_position_received:
                return
            
            # Send setpoints for at least 20 iterations (~2 seconds) before engaging OFFBOARD
            self.offboard_setpoint_counter += 1
            
            if self.offboard_setpoint_counter == 10:
                self.get_logger().info("Sending initial setpoints before OFFBOARD mode...")
            elif self.offboard_setpoint_counter == 20:
                self.get_logger().info("Engaging OFFBOARD mode and arming...")
                self.engage_offboard_mode()
            elif self.offboard_setpoint_counter == 25:
                self.arm()
            elif self.offboard_setpoint_counter == 30:
                # Wait for drone to be armed and in OFFBOARD mode
                if self.is_armed and self.is_offboard:
                    self.flight_phase = "STRAIGHT_LINE"
                    self.last_update_time = self.get_clock().now()
                    self.get_logger().info("✓ Armed and in OFFBOARD mode, starting straight-line flight")
                else:
                    armed_status = "ARMED" if self.is_armed else "DISARMED"
                    mode_status = "OFFBOARD" if self.is_offboard else f"MODE {self.vehicle_status.nav_state}"
                    self.get_logger().warn(f"Waiting for ready state: {armed_status}, {mode_status}")
        
        elif self.flight_phase == "STRAIGHT_LINE":
            self.publish_markers()
            # Optional periodic debug
            if int(self.trajectory_time) % 10 == 0 and abs((self.trajectory_time % 10) - 0.0) < 0.05:
                self.get_logger().info(
                    f"Pattern={self.pattern} t={self.trajectory_time:.1f}s pos=({self.vehicle_local_position.x:.1f}, {self.vehicle_local_position.y:.1f}, {self.vehicle_local_position.z:.1f})"
                )

def main(args=None):
    rclpy.init(args=args)
    node = StraightLineNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

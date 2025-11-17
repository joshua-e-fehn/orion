#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import numpy as np
import math
import time

class TrajectoryNode(Node):
    def __init__(self):
        super().__init__('trajectory_node')

        # -------------------------------------------------------------
        # PARAMETERS
        # -------------------------------------------------------------
        self.declare_parameter('pattern', 'random_curves')  # DEFAULT
        self.declare_parameter('publish_rate', 30.0)
        self.declare_parameter('speed', 2.0)

        # Area settings
        self.declare_parameter('center_x', 0.0)
        self.declare_parameter('center_y', 0.0)
        self.declare_parameter('flight_height', 2.0)

        # Vertical modulation params
        self.declare_parameter('amp_z', 0.3)
        self.declare_parameter('freq_z', 0.2)

        # Random curves params
        self.declare_parameter('rc_min_turn_deg', 90.0)
        self.declare_parameter('rc_max_turn_deg', 180.0)
        self.declare_parameter('rc_segment_time', 3.0)
        self.declare_parameter('rc_curve_strength', 0.8)

        # Load parameters
        self.pattern = self.get_parameter('pattern').value
        self.publish_rate = float(self.get_parameter('publish_rate').value)
        self.speed = float(self.get_parameter('speed').value)

        self.center_x = float(self.get_parameter('center_x').value)
        self.center_y = float(self.get_parameter('center_y').value)
        self.flight_height = float(self.get_parameter('flight_height').value)
        self.amp_z = float(self.get_parameter('amp_z').value)
        self.freq_z = float(self.get_parameter('freq_z').value)

        # Convert turn angles
        self.rc_min_turn = math.radians(float(self.get_parameter('rc_min_turn_deg').value))
        self.rc_max_turn = math.radians(float(self.get_parameter('rc_max_turn_deg').value))
        self.rc_segment_time = float(self.get_parameter('rc_segment_time').value)
        self.rc_curve_strength = float(self.get_parameter('rc_curve_strength').value)

        # -------------------------------------------------------------
        # STATE VARIABLES
        # -------------------------------------------------------------
        self.start_time = time.time()
        self.prev_time = self.start_time

        # Random walk position
        self.rw_pos = np.array([0.0, 0.0], dtype=float)

        # Random curves state
        self.rc_heading = np.random.uniform(0, 2*math.pi)  # Random initial heading
        self.rc_segment_timer = 0.0
        self.rc_target_heading = self.rc_heading
        self.rc_curve_mode = "STRAIGHT"
        
        # Crazy pattern state
        self.angular_velocity = 0.0  # Current rotation rate
        self.target_angular_velocity = 0.0
        self.maneuver_timer = 0.0
        self.maneuver_duration = 0.0

        # -------------------------------------------------------------
        # ROS SETUP
        # -------------------------------------------------------------
        self.publisher = self.create_publisher(PoseStamped, '/setpoint_pose', 10)
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_position_setpoint)

        self.get_logger().info(f"Trajectory node started with pattern: {self.pattern}")

    # -------------------------------------------------------------
    # MAIN LOOP
    # -------------------------------------------------------------
    def publish_position_setpoint(self):
        now = time.time()
        dt = now - self.prev_time
        self.prev_time = now
        
        # Ensure dt is reasonable
        if dt <= 0 or dt > 0.1:
            dt = 1.0 / self.publish_rate

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()

        # -------------------------------------------------------------
        # CRAZY RANDOM CURVES PATTERN
        # -------------------------------------------------------------
        if self.pattern == 'random_curves':

            self.maneuver_timer += dt

            # Pick new crazy maneuver
            if self.maneuver_timer >= self.maneuver_duration:
                self.maneuver_timer = 0.0
                
                # VERY short, chaotic maneuver durations (0.2 to 1.2 seconds)
                self.maneuver_duration = np.random.uniform(0.2, 1.2)
                
                # Pick random maneuver type - weighted toward aggressive moves
                maneuver = np.random.choice(
                    ['sharp_turn', 'spiral', 'zigzag', 'loop', 'reversal', 'chaos', 'straight'],
                    p=[0.25, 0.20, 0.20, 0.15, 0.10, 0.05, 0.05]  # heavily favor aggressive maneuvers
                )
                
                if maneuver == 'sharp_turn':
                    # EXTREME instant turns: 120-270 degrees!
                    turn = np.random.uniform(2.0*math.pi/3, 1.5*math.pi) * np.random.choice([-1, 1])
                    self.rc_heading += turn
                    self.target_angular_velocity = 0.0
                    
                elif maneuver == 'spiral':
                    # INTENSE angular velocity for very tight spirals
                    self.target_angular_velocity = np.random.uniform(4.0, 7.0) * np.random.choice([-1, 1])
                    
                elif maneuver == 'zigzag':
                    # RAPID alternating turns
                    self.target_angular_velocity = np.random.uniform(5.0, 8.0) * np.random.choice([-1, 1])
                    self.maneuver_duration = np.random.uniform(0.2, 0.5)
                    
                elif maneuver == 'loop':
                    # FAST 360+ degree loops (can do multiple rotations)
                    rotations = np.random.uniform(1.0, 2.5)
                    self.target_angular_velocity = np.random.uniform(3.0, 6.0) * np.random.choice([-1, 1])
                    self.maneuver_duration = np.random.uniform(1.0, 1.8)
                    
                elif maneuver == 'reversal':
                    # Sudden 180 degree reversal
                    self.rc_heading += math.pi
                    self.target_angular_velocity = 0.0
                    self.maneuver_duration = np.random.uniform(0.3, 0.8)
                    
                elif maneuver == 'chaos':
                    # Random jittery motion
                    self.target_angular_velocity = np.random.uniform(-10.0, 10.0)
                    self.maneuver_duration = np.random.uniform(0.2, 0.4)
                    
                else:  # straight (rare!)
                    self.target_angular_velocity = 0.0
                    self.maneuver_duration = np.random.uniform(0.5, 1.0)

            # FASTER transition to target angular velocity for snappier response
            self.angular_velocity += (self.target_angular_velocity - self.angular_velocity) * 0.25
            
            # Update heading based on angular velocity
            self.rc_heading += self.angular_velocity * dt

            # Vary speed for more chaos (0.5x to 1.5x base speed)
            speed_multiplier = 0.8 + 0.4 * math.sin(now * 2.0)
            current_speed = self.speed * speed_multiplier

            # Move forward in current heading
            vx = current_speed * math.cos(self.rc_heading)
            vy = current_speed * math.sin(self.rc_heading)
            self.rw_pos += np.array([vx, vy]) * dt

            x = self.center_x + float(self.rw_pos[0])
            y = self.center_y + float(self.rw_pos[1])
            
            # EXTREME vertical motion - multiple chaotic frequency components + random bursts
            z_base = self.flight_height
            z_oscillation = (self.amp_z * 1.5 * math.sin(2.0 * math.pi * self.freq_z * now) +
                            0.4 * math.sin(2.0 * math.pi * 0.7 * now) +
                            0.3 * math.cos(2.0 * math.pi * 0.4 * now) +
                            0.2 * math.sin(2.0 * math.pi * 1.3 * now))
            # Random altitude bursts
            z_random = 0.3 * (np.random.random() - 0.5) if np.random.random() < 0.1 else 0.0
            z = z_base + z_oscillation + z_random

        # -------------------------------------------------------------
        # DEFAULT FAILSAFE: stay still if pattern unknown
        # -------------------------------------------------------------
        else:
            x = self.center_x
            y = self.center_y
            z = self.flight_height

        # -------------------------------------------------------------
        # PUBLISH
        # -------------------------------------------------------------
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)

        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

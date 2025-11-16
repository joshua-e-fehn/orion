#!/usr/bin/env python3
"""
RViz Camera Follower Node

Automatically updates RViz camera to follow the target drone.
Subscribes to target position and publishes camera position commands.
"""

import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleLocalPosition
from geometry_msgs.msg import PointStamped
import numpy as np


class RVizCameraFollower(Node):
    """Node that makes RViz camera follow the target drone."""
    
    def __init__(self):
        super().__init__('rviz_camera_follower')
        
        # Parameters
        self.declare_parameter('target_namespace', 'px4_2')
        self.declare_parameter('camera_distance', 20.0)  # Distance from target
        self.declare_parameter('camera_height_offset', 10.0)  # Height above target
        self.declare_parameter('update_rate', 10.0)  # Hz
        self.declare_parameter('smooth_factor', 0.2)  # Lower = smoother, 1.0 = instant
        
        self.target_namespace = self.get_parameter('target_namespace').value
        self.camera_distance = self.get_parameter('camera_distance').value
        self.camera_height = self.get_parameter('camera_height_offset').value
        self.update_rate = self.get_parameter('update_rate').value
        self.smooth_factor = self.get_parameter('smooth_factor').value
        
        # State
        self.target_position = None
        self.camera_focal_point = np.array([0.0, 0.0, -5.0])
        
        # Subscribe to target position
        target_topic = f'/{self.target_namespace}/fmu/out/vehicle_local_position'
        self.target_sub = self.create_subscription(
            VehicleLocalPosition,
            target_topic,
            self.target_callback,
            10
        )
        
        # Publisher for RViz focal point
        self.focal_point_pub = self.create_publisher(
            PointStamped,
            '/rviz/camera_placement',
            10
        )
        
        # Timer for smooth camera updates
        self.timer = self.create_timer(
            1.0 / self.update_rate,
            self.update_camera
        )
        
        self.get_logger().info(
            f'RViz Camera Follower started:\n'
            f'  Target: {self.target_namespace}\n'
            f'  Camera distance: {self.camera_distance}m\n'
            f'  Camera height offset: {self.camera_height}m\n'
            f'  Smooth factor: {self.smooth_factor}'
        )
    
    def target_callback(self, msg: VehicleLocalPosition):
        """Update target position from PX4 message."""
        if msg.xy_valid and msg.z_valid:
            # Store target position in NED
            self.target_position = np.array([msg.x, msg.y, msg.z])
    
    def update_camera(self):
        """Update camera focal point to follow target smoothly."""
        if self.target_position is None:
            return
        
        # Convert NED to ENU for visualization
        target_enu = self.ned_to_enu(self.target_position)
        
        # Smooth camera movement using exponential smoothing
        desired_focal = target_enu
        self.camera_focal_point = (
            self.smooth_factor * desired_focal +
            (1.0 - self.smooth_factor) * self.camera_focal_point
        )
        
        # Note: RViz2 doesn't have a direct API to set camera position programmatically
        # This is a placeholder - you'll need to manually set the focal point in RViz
        # or use the RViz Python API if available
        
        # Log the desired focal point
        self.get_logger().debug(
            f'Target focal point (ENU): [{self.camera_focal_point[0]:.2f}, '
            f'{self.camera_focal_point[1]:.2f}, {self.camera_focal_point[2]:.2f}]',
            throttle_duration_sec=2.0
        )
    
    @staticmethod
    def ned_to_enu(ned_coords):
        """Convert NED coordinates to ENU for RViz."""
        return np.array([ned_coords[1], ned_coords[0], -ned_coords[2]])


def main(args=None):
    rclpy.init(args=args)
    node = RVizCameraFollower()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

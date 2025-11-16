#!/usr/bin/env python3
"""
Publish TF frame for target drone position.

This creates a TF frame at the target's position that RViz can follow.
"""

import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleLocalPosition
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import numpy as np


class TargetTFPublisher(Node):
    """Publish TF frame for target drone."""
    
    def __init__(self):
        super().__init__('target_tf_publisher')
        
        # Parameters
        self.declare_parameter('target_namespace', 'px4_2')
        self.declare_parameter('target_frame_id', 'target_drone')
        self.declare_parameter('parent_frame_id', 'map')
        
        self.target_namespace = self.get_parameter('target_namespace').value
        self.target_frame = self.get_parameter('target_frame_id').value
        self.parent_frame = self.get_parameter('parent_frame_id').value
        
        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Subscribe to target position
        target_topic = f'/{self.target_namespace}/fmu/out/vehicle_local_position'
        self.target_sub = self.create_subscription(
            VehicleLocalPosition,
            target_topic,
            self.target_callback,
            10
        )
        
        self.get_logger().info(
            f'Target TF Publisher started:\n'
            f'  Target: {self.target_namespace}\n'
            f'  Frame: {self.parent_frame} -> {self.target_frame}'
        )
    
    def target_callback(self, msg: VehicleLocalPosition):
        """Publish TF frame for target position."""
        if not (msg.xy_valid and msg.z_valid):
            return
        
        # Convert NED to ENU for RViz
        ned = np.array([msg.x, msg.y, msg.z])
        enu = self.ned_to_enu(ned)
        
        # Create transform
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.parent_frame
        t.child_frame_id = self.target_frame
        
        t.transform.translation.x = float(enu[0])
        t.transform.translation.y = float(enu[1])
        t.transform.translation.z = float(enu[2])
        
        # No rotation (identity quaternion)
        t.transform.rotation.w = 1.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        
        # Broadcast
        self.tf_broadcaster.sendTransform(t)
    
    @staticmethod
    def ned_to_enu(ned_coords):
        """Convert NED coordinates to ENU for RViz."""
        return np.array([ned_coords[1], ned_coords[0], -ned_coords[2]])


def main(args=None):
    rclpy.init(args=args)
    node = TargetTFPublisher()
    
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

#!/usr/bin/env python3
"""
Constant Velocity (CV) Target Predictor Node.

This node subscribes to target position/velocity and publishes predictions
assuming the target maintains constant velocity.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import VehicleLocalPosition
from visualization_msgs.msg import MarkerArray
import numpy as np

from .cv_model import CVModel
from ..common.types import PredictorInput, PredictorOutput
from ..common.visualization import create_prediction_markers


class CVPredictorNode(Node):
    """ROS2 node for Constant Velocity target prediction."""
    
    def __init__(self):
        super().__init__('cv_predictor_node')
        
        # Declare and get parameters
        self._declare_parameters()
        self._get_parameters()
        
        # Initialize CV model
        process_noise = {
            'position': self.param_q_pos,
            'velocity': self.param_q_vel
        }
        measurement_noise = {
            'position': self.param_r_pos,
            'velocity': self.param_r_vel
        }
        
        self.cv_model = CVModel(process_noise, measurement_noise)
         # QoS profile for PX4 topics (BEST_EFFORT reliability)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        

        # Create subscribers
        target_topic = f'/{self.target_namespace}/fmu/out/vehicle_local_position_v1'
        self.target_sub = self.create_subscription(
            VehicleLocalPosition,
            target_topic,
            self.target_callback,
            qos_profile
        )
        
        # Create publishers
        self.prediction_pub = self.create_publisher(
            VehicleLocalPosition,
            '/target/predicted_state',
            10
        )
        
        if self.publish_markers:
            self.marker_pub = self.create_publisher(
                MarkerArray,
                '/target/prediction_markers',
                10
            )
            # Publisher for current target position marker
            self.target_marker_pub = self.create_publisher(
                MarkerArray,
                '/target/current_position_marker',
                10
            )
        
        # Create timer for regular prediction updates
        self.prediction_timer = self.create_timer(
            1.0 / self.update_rate,
            self.prediction_callback
        )
        
        # State
        self.last_measurement_time = None
        self.last_update_time = None
        
        self.get_logger().info(
            f'CV Predictor initialized:\n'
            f'  Target namespace: {self.target_namespace}\n'
            f'  Update rate: {self.update_rate} Hz\n'
            f'  Prediction horizons: {self.prediction_horizons}\n'
            f'  Process noise (pos, vel): ({self.param_q_pos}, {self.param_q_vel})\n'
            f'  Measurement noise (pos, vel): ({self.param_r_pos}, {self.param_r_vel})'
        )
        
        print("\n" + "="*70)
        print("  CV PREDICTOR NODE - DEBUG MODE")
        print("="*70)
        print(f"  Subscribing to: /{self.target_namespace}/fmu/out/vehicle_local_position_v1")
        print(f"  Publishing predictions to: /target/predicted_state")
        print(f"  Publishing markers to: /target/prediction_markers")
        print(f"  Waiting for first measurement from target drone...")
        print("="*70 + "\n")
    
    def _declare_parameters(self):
        """Declare ROS2 parameters."""
        self.declare_parameter('target_namespace', 'px4_2')
        self.declare_parameter('update_rate', 10.0)
        self.declare_parameter('prediction_horizons', [0.5, 1.0, 2.0, 3.0])
        self.declare_parameter('process_noise.position', 0.1)
        self.declare_parameter('process_noise.velocity', 0.5)
        self.declare_parameter('measurement_noise.position', 0.05)
        self.declare_parameter('measurement_noise.velocity', 0.1)
        self.declare_parameter('publish_markers', True)
        self.declare_parameter('marker_scale', 1.0)
    
    def _get_parameters(self):
        """Get parameter values."""
        self.target_namespace = self.get_parameter('target_namespace').value
        self.update_rate = self.get_parameter('update_rate').value
        self.prediction_horizons = self.get_parameter('prediction_horizons').value
        self.param_q_pos = self.get_parameter('process_noise.position').value
        self.param_q_vel = self.get_parameter('process_noise.velocity').value
        self.param_r_pos = self.get_parameter('measurement_noise.position').value
        self.param_r_vel = self.get_parameter('measurement_noise.velocity').value
        self.publish_markers = self.get_parameter('publish_markers').value
        self.marker_scale = self.get_parameter('marker_scale').value
    
    def target_callback(self, msg: VehicleLocalPosition):
        """
        Callback for target position measurements.
        
        Updates the CV model with new measurement.
        """
        current_time = self.get_clock().now().nanoseconds / 1e9
        
        # Check validity flags
        if not (msg.xy_valid and msg.z_valid and msg.v_xy_valid and msg.v_z_valid):
            self.get_logger().warn('Received invalid target measurement', throttle_duration_sec=1.0)
            print(f"[CV Node] ✗ Invalid measurement: xy_valid={msg.xy_valid}, z_valid={msg.z_valid}, "
                  f"v_xy_valid={msg.v_xy_valid}, v_z_valid={msg.v_z_valid}")
            return
        
        # Debug: First valid measurement
        if not self.cv_model.initialized:
            print(f"\n[CV Node] ✓ First valid measurement received!")
            print(f"  Position: [{msg.x:.3f}, {msg.y:.3f}, {msg.z:.3f}]")
            print(f"  Velocity: [{msg.vx:.3f}, {msg.vy:.3f}, {msg.vz:.3f}]")
        
        # Extract position and velocity (already in NED from PX4)
        position = np.array([msg.x, msg.y, msg.z])
        velocity = np.array([msg.vx, msg.vy, msg.vz])
        
        # Calculate dt
        if self.last_measurement_time is not None:
            dt = current_time - self.last_measurement_time
            if dt < 0:
                self.get_logger().error('Negative dt - time went backwards!')
                return
        else:
            dt = 0.1  # Default dt for first measurement
        
        # Update model
        self.cv_model.update(position, velocity, dt)
        
        self.last_measurement_time = current_time
        
        # Log diagnostics (throttled)
        if self.cv_model.initialized:
            innovation = self.cv_model.get_innovation()
            innovation_norm = np.linalg.norm(innovation[0:3])  # Position innovation
            
            # Debug output every 20 updates (~2 seconds at 10 Hz)
            if not hasattr(self, '_update_counter'):
                self._update_counter = 0
            self._update_counter += 1
            
            if self._update_counter % 20 == 0:
                print(f"\n[CV Node] Measurement Update #{self._update_counter}:")
                print(f"  dt: {dt:.3f}s")
                print(f"  Innovation (residual): {innovation_norm:.3f}m")
                print(f"  Current position: [{position[0]:.2f}, {position[1]:.2f}, {position[2]:.2f}]")
                print(f"  Current velocity: [{velocity[0]:.2f}, {velocity[1]:.2f}, {velocity[2]:.2f}]")
            
            self.get_logger().debug(
                f'CV Update: dt={dt:.3f}s, innovation_norm={innovation_norm:.3f}m',
                throttle_duration_sec=1.0
            )
        
        # Publish current target position marker
        if self.publish_markers:
            self._publish_target_marker(position, velocity)
    
    def _publish_target_marker(self, position: np.ndarray, velocity: np.ndarray):
        """
        Publish a marker showing the current target drone position.
        
        Args:
            position: Current position [x, y, z] in NED
            velocity: Current velocity [vx, vy, vz] in NED
        """
        from ..common.types import ned_to_enu
        from visualization_msgs.msg import Marker
        from std_msgs.msg import ColorRGBA
        from geometry_msgs.msg import Point
        
        # Convert NED to ENU for visualization
        pos_enu = ned_to_enu(position)
        vel_enu = ned_to_enu(velocity)
        vel_norm = np.linalg.norm(vel_enu)
        
        markers = MarkerArray()
        
        # Create large sphere for target position
        sphere = Marker()
        sphere.header.frame_id = 'map'
        sphere.header.stamp = self.get_clock().now().to_msg()
        sphere.ns = 'target_current'
        sphere.id = 0
        sphere.type = Marker.SPHERE
        sphere.action = Marker.ADD
        sphere.pose.position.x = float(pos_enu[0])
        sphere.pose.position.y = float(pos_enu[1])
        sphere.pose.position.z = float(pos_enu[2])
        sphere.pose.orientation.w = 1.0
        sphere.scale.x = 0.5  # Larger than prediction spheres
        sphere.scale.y = 0.5
        sphere.scale.z = 0.5
        sphere.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.9)  # Red for current target
        markers.markers.append(sphere)
        
        # Create arrow showing current velocity
        if vel_norm > 0.01:
            arrow = Marker()
            arrow.header.frame_id = 'map'
            arrow.header.stamp = self.get_clock().now().to_msg()
            arrow.ns = 'target_velocity'
            arrow.id = 1
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            
            start_point = Point()
            start_point.x = float(pos_enu[0])
            start_point.y = float(pos_enu[1])
            start_point.z = float(pos_enu[2])
            
            arrow_length = min(vel_norm * 1.0, 3.0)  # Longer arrow for current velocity
            vel_unit = vel_enu / vel_norm
            
            end_point = Point()
            end_point.x = float(pos_enu[0] + vel_unit[0] * arrow_length)
            end_point.y = float(pos_enu[1] + vel_unit[1] * arrow_length)
            end_point.z = float(pos_enu[2] + vel_unit[2] * arrow_length)
            
            arrow.points.append(start_point)
            arrow.points.append(end_point)
            
            arrow.scale.x = 0.08  # Shaft diameter
            arrow.scale.y = 0.15  # Head diameter
            arrow.scale.z = 0.20  # Head length
            arrow.color = ColorRGBA(r=1.0, g=0.5, b=0.0, a=0.9)  # Orange for current velocity
            markers.markers.append(arrow)
        
        # Create text label
        text = Marker()
        text.header.frame_id = 'map'
        text.header.stamp = self.get_clock().now().to_msg()
        text.ns = 'target_label'
        text.id = 2
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.x = float(pos_enu[0])
        text.pose.position.y = float(pos_enu[1])
        text.pose.position.z = float(pos_enu[2] + 0.8)
        text.pose.orientation.w = 1.0
        text.scale.z = 0.5
        text.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
        text.text = f"TARGET\nv={vel_norm:.2f}m/s"
        markers.markers.append(text)
        
        self.target_marker_pub.publish(markers)
    
    def prediction_callback(self):
        """
        Timer callback to publish predictions at regular intervals.
        """
        if not self.cv_model.initialized:
            # Debug: Waiting for initialization
            if not hasattr(self, '_waiting_logged'):
                print("[CV Node] ⏳ Waiting for filter initialization...")
                self._waiting_logged = True
            return
        
        current_time = self.get_clock().now().nanoseconds / 1e9
        
        # Generate predictions at all horizons
        predictions = []
        for horizon in self.prediction_horizons:
            try:
                pos, vel, full_cov = self.cv_model.predict(horizon)
                
                # Extract position and velocity covariances from full 6x6 covariance
                pos_cov = full_cov[0:3, 0:3]
                vel_cov = full_cov[3:6, 3:6]
                
                # Create PredictorOutput
                pred_output = PredictorOutput(
                    timestamp=current_time,
                    prediction_horizon=horizon,
                    predicted_position=pos,
                    predicted_velocity=vel,
                    predicted_acceleration=np.zeros(3),  # CV assumes zero acceleration
                    position_covariance=pos_cov,
                    velocity_covariance=vel_cov,
                    model_probability=1.0,
                    innovation=self.cv_model.get_innovation(),
                    is_valid=True,
                    predictor_type='cv'
                )
                
                predictions.append((horizon, pred_output))
                
            except Exception as e:
                self.get_logger().error(f'Prediction failed for horizon {horizon}: {e}')
        
        if not predictions:
            return
        
        # Publish primary prediction (shortest horizon)
        primary_pred = predictions[0][1]
        pred_msg = self._create_prediction_message(primary_pred, current_time)
        self.prediction_pub.publish(pred_msg)
        
        # Publish visualization markers
        if self.publish_markers and len(predictions) > 0:
            try:
                markers = create_prediction_markers(
                    predictions,
                    frame_id='map',
                    namespace='cv_predictor',
                    scale=self.marker_scale
                )
                self.marker_pub.publish(markers)
            except Exception as e:
                self.get_logger().error(f'Marker creation failed: {e}')
        
        # Debug: Prediction output
        if not hasattr(self, '_prediction_counter'):
            self._prediction_counter = 0
            print("\n[CV Node] ✓ First prediction published!")
        self._prediction_counter += 1
        
        if self._prediction_counter % 20 == 0:  # Every 2 seconds at 10 Hz
            print(f"\n[CV Node] Prediction #{self._prediction_counter} (all horizons):")
            for horizon, pred in predictions:
                print(f"  t+{horizon:.1f}s: pos=[{pred.predicted_position[0]:6.2f}, "
                      f"{pred.predicted_position[1]:6.2f}, {pred.predicted_position[2]:6.2f}], "
                      f"vel=[{pred.predicted_velocity[0]:5.2f}, {pred.predicted_velocity[1]:5.2f}, "
                      f"{pred.predicted_velocity[2]:5.2f}]")
            
            # Show covariance uncertainty
            pos_std = np.sqrt(np.diag(primary_pred.position_covariance))
            print(f"  Position uncertainty (σ): [{pos_std[0]:.3f}, {pos_std[1]:.3f}, {pos_std[2]:.3f}]m")
        
        # Log prediction info (throttled)
        self.get_logger().info(
            f'CV Prediction: pos=[{primary_pred.predicted_position[0]:.2f}, '
            f'{primary_pred.predicted_position[1]:.2f}, {primary_pred.predicted_position[2]:.2f}], '
            f'vel=[{primary_pred.predicted_velocity[0]:.2f}, '
            f'{primary_pred.predicted_velocity[1]:.2f}, {primary_pred.predicted_velocity[2]:.2f}]',
            throttle_duration_sec=2.0
        )
    
    def _create_prediction_message(
        self,
        prediction: PredictorOutput,
        current_time: float
    ) -> VehicleLocalPosition:
        """
        Create VehicleLocalPosition message from PredictorOutput.
        
        Args:
            prediction: Prediction output
            current_time: Current ROS time (seconds)
        
        Returns:
            VehicleLocalPosition message
        """
        msg = VehicleLocalPosition()
        
        # Timestamp (in microseconds)
        msg.timestamp = int((current_time + prediction.prediction_horizon) * 1e6)
        
        # Position (NED frame)
        msg.x = float(prediction.predicted_position[0])
        msg.y = float(prediction.predicted_position[1])
        msg.z = float(prediction.predicted_position[2])
        
        # Velocity (NED frame)
        msg.vx = float(prediction.predicted_velocity[0])
        msg.vy = float(prediction.predicted_velocity[1])
        msg.vz = float(prediction.predicted_velocity[2])
        
        # Acceleration (CV assumes zero)
        msg.ax = 0.0
        msg.ay = 0.0
        msg.az = 0.0
        
        # Validity flags
        msg.xy_valid = prediction.is_valid
        msg.z_valid = prediction.is_valid
        msg.v_xy_valid = prediction.is_valid
        msg.v_z_valid = prediction.is_valid
        
        # Reference frame
        msg.ref_timestamp = msg.timestamp
        msg.ref_lat = 0.0  # Local frame
        msg.ref_lon = 0.0
        msg.ref_alt = 0.0
        
        return msg


def main(args=None):
    """Main entry point for CV predictor node."""
    rclpy.init(args=args)
    
    try:
        node = CVPredictorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error in CV predictor node: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

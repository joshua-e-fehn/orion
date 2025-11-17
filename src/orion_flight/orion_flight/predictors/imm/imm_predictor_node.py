#!/usr/bin/env python3
"""
Interacting Multiple Model (IMM) Target Predictor Node.

This node subscribes to target position/velocity and publishes predictions
using an adaptive combination of CV and CA models.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleLocalPosition
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import String
import numpy as np
import json

from .imm_filter import IMMFilter
from ..cv.cv_model import CVModel
from ..ca.ca_model import CAModel
from ..common.types import PredictorInput, PredictorOutput
from ..common.visualization import create_prediction_markers


class IMMPredictorNode(Node):
    """ROS2 node for IMM target prediction."""
    
    def __init__(self):
        super().__init__('imm_predictor_node')
        
        # Declare and get parameters
        self._declare_parameters()
        self._get_parameters()
        
        # Initialize CV model
        cv_process_noise = {
            'position': self.param_cv_q_pos,
            'velocity': self.param_cv_q_vel
        }
        cv_measurement_noise = {
            'position': self.param_r_pos,
            'velocity': self.param_r_vel
        }
        cv_model = CVModel(cv_process_noise, cv_measurement_noise)
        
        # Initialize CA model
        # CRITICAL: Set use_acceleration_measurements=False for IMM compatibility
        # This ensures CA model uses 6D measurements (pos+vel) like CV model
        # for fair likelihood comparison in IMM filter
        ca_process_noise = {
            'position': self.param_ca_q_pos,
            'velocity': self.param_ca_q_vel,
            'acceleration': self.param_ca_q_acc
        }
        ca_measurement_noise = {
            'position': self.param_r_pos,
            'velocity': self.param_r_vel,
            'acceleration': self.param_r_acc
        }
        ca_model = CAModel(
            ca_process_noise, 
            ca_measurement_noise,
            use_acceleration_measurements=False  # IMM mode: 6D measurements only
        )
        
        # Initialize IMM filter
        transition_matrix = np.array([
            [self.param_p_cv_cv, self.param_p_cv_ca],
            [self.param_p_ca_cv, self.param_p_ca_ca]
        ])
        initial_probs = np.array([self.param_init_p_cv, self.param_init_p_ca])
        
        self.imm_filter = IMMFilter(
            cv_model=cv_model,
            ca_model=ca_model,
            transition_matrix=transition_matrix,
            initial_mode_probabilities=initial_probs
        )
        
        # QoS profile matching PX4 publishers (BEST_EFFORT reliability)
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
        
        self.status_pub = self.create_publisher(
            String,
            '/target/predictor_status',
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
        
        # Create timer for status updates
        self.status_timer = self.create_timer(
            1.0,  # 1 Hz status updates
            self.status_callback
        )
        
        # State - track timestamp in microseconds (PX4 format)
        self.last_measurement_timestamp_us = None
        self.last_update_time = None
        
        self.get_logger().info(
            f'IMM Predictor initialized:\n'
            f'  Target namespace: {self.target_namespace}\n'
            f'  Update rate: {self.update_rate} Hz\n'
            f'  Prediction horizons: {self.prediction_horizons}\n'
            f'  CV process noise (pos, vel): ({self.param_cv_q_pos}, {self.param_cv_q_vel})\n'
            f'  CA process noise (pos, vel, acc): ({self.param_ca_q_pos}, {self.param_ca_q_vel}, {self.param_ca_q_acc})\n'
            f'  Measurement noise (pos, vel, acc): ({self.param_r_pos}, {self.param_r_vel}, {self.param_r_acc})\n'
            f'  Transition matrix:\n{transition_matrix}\n'
            f'  Initial probabilities: CV={self.param_init_p_cv}, CA={self.param_init_p_ca}'
        )
    
    def _declare_parameters(self):
        """Declare ROS2 parameters."""
        self.declare_parameter('target_namespace', 'px4_2')
        self.declare_parameter('update_rate', 10.0)
        self.declare_parameter('prediction_horizons', [0.5, 1.0, 2.0, 3.0])
        
        # CV model process noise
        self.declare_parameter('cv_process_noise.position', 0.1)
        self.declare_parameter('cv_process_noise.velocity', 0.5)
        
        # CA model process noise
        self.declare_parameter('ca_process_noise.position', 0.1)
        self.declare_parameter('ca_process_noise.velocity', 0.5)
        self.declare_parameter('ca_process_noise.acceleration', 1.0)
        
        # Measurement noise (shared)
        self.declare_parameter('measurement_noise.position', 0.05)
        self.declare_parameter('measurement_noise.velocity', 0.1)
        self.declare_parameter('measurement_noise.acceleration', 0.5)
        
        # IMM parameters
        self.declare_parameter('imm.transition_prob_stay', 0.95)  # P(stay in same mode)
        self.declare_parameter('imm.initial_prob_cv', 0.5)
        self.declare_parameter('imm.initial_prob_ca', 0.5)
        
        # Visualization
        self.declare_parameter('publish_markers', True)
        self.declare_parameter('marker_scale', 1.0)
    
    def _get_parameters(self):
        """Get parameter values."""
        self.target_namespace = self.get_parameter('target_namespace').value
        self.update_rate = self.get_parameter('update_rate').value
        self.prediction_horizons = self.get_parameter('prediction_horizons').value
        
        # CV process noise
        self.param_cv_q_pos = self.get_parameter('cv_process_noise.position').value
        self.param_cv_q_vel = self.get_parameter('cv_process_noise.velocity').value
        
        # CA process noise
        self.param_ca_q_pos = self.get_parameter('ca_process_noise.position').value
        self.param_ca_q_vel = self.get_parameter('ca_process_noise.velocity').value
        self.param_ca_q_acc = self.get_parameter('ca_process_noise.acceleration').value
        
        # Measurement noise
        self.param_r_pos = self.get_parameter('measurement_noise.position').value
        self.param_r_vel = self.get_parameter('measurement_noise.velocity').value
        self.param_r_acc = self.get_parameter('measurement_noise.acceleration').value
        
        # IMM parameters
        p_stay = self.get_parameter('imm.transition_prob_stay').value
        p_switch = 1.0 - p_stay
        self.param_p_cv_cv = p_stay
        self.param_p_cv_ca = p_switch
        self.param_p_ca_cv = p_switch
        self.param_p_ca_ca = p_stay
        
        self.param_init_p_cv = self.get_parameter('imm.initial_prob_cv').value
        self.param_init_p_ca = self.get_parameter('imm.initial_prob_ca').value
        
        # Normalize initial probabilities
        total = self.param_init_p_cv + self.param_init_p_ca
        if total > 0:
            self.param_init_p_cv /= total
            self.param_init_p_ca /= total
        
        # Visualization
        self.publish_markers = self.get_parameter('publish_markers').value
        self.marker_scale = self.get_parameter('marker_scale').value
    
    def target_callback(self, msg: VehicleLocalPosition):
        """
        Callback for target position measurements.
        
        Updates the IMM filter with new measurement.
        """
        # Check validity flags first
        if not (msg.xy_valid and msg.z_valid and msg.v_xy_valid and msg.v_z_valid):
            self.get_logger().warn('Received invalid target measurement', throttle_duration_sec=1.0)
            return
        
        # Extract position and velocity (already in NED from PX4)
        position = np.array([msg.x, msg.y, msg.z])
        velocity = np.array([msg.vx, msg.vy, msg.vz])
        
        # Extract acceleration if available (PX4 provides it)
        acceleration = np.array([msg.ax, msg.ay, msg.az])
        
        # CRITICAL: Calculate dt from PX4 timestamps (microseconds)
        current_timestamp_us = msg.timestamp
        
        if self.last_measurement_timestamp_us is not None:
            dt_us = current_timestamp_us - self.last_measurement_timestamp_us
            
            # Check for time going backwards
            if dt_us < 0:
                self.get_logger().error(
                    f'Time went backwards! current={current_timestamp_us}, '
                    f'last={self.last_measurement_timestamp_us}, dt={dt_us}'
                )
                # Reset and skip this measurement
                self.last_measurement_timestamp_us = current_timestamp_us
                return
            
            # Convert microseconds to seconds
            dt = dt_us * 1e-6
            
            # Sanity check: reject unreasonable dt values
            if dt > 1.0:  # More than 1 second gap
                self.get_logger().warn(
                    f'Large dt detected: {dt:.3f}s - possible data gap or timestamp issue',
                    throttle_duration_sec=2.0
                )
                # Use a reasonable default instead of rejecting
                dt = 0.1
            elif dt < 1e-6:  # Less than 1 microsecond
                self.get_logger().debug(
                    f'Very small dt: {dt:.9f}s - skipping update',
                    throttle_duration_sec=2.0
                )
                return
        else:
            # First measurement - use default dt
            dt = 0.1
            self.get_logger().info(f'First measurement received, using default dt={dt:.3f}s')
        
        # Update IMM filter with properly calculated dt
        self.imm_filter.update(position, velocity, dt, acceleration)
        
        # Store timestamp for next iteration
        self.last_measurement_timestamp_us = current_timestamp_us
        
        # Log diagnostics (throttled)
        if self.imm_filter.initialized:
            mode_probs = self.imm_filter.get_mode_probabilities()
            self.get_logger().debug(
                f'IMM Update: dt={dt:.4f}s (from timestamps), P(CV)={mode_probs[0]:.3f}, P(CA)={mode_probs[1]:.3f}',
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
        if not self.imm_filter.initialized:
            return
        
        current_time = self.get_clock().now().nanoseconds / 1e9
        
        # Get mode probabilities
        mode_probs = self.imm_filter.get_mode_probabilities()
        
        # Generate predictions at all horizons
        predictions = []
        for horizon in self.prediction_horizons:
            try:
                pos, vel, acc, full_cov = self.imm_filter.predict(horizon)
                
                # Extract diagonal blocks for PredictorOutput compatibility
                pos_cov = full_cov[0:3, 0:3]
                vel_cov = full_cov[3:6, 3:6]
                
                # Create PredictorOutput
                pred_output = PredictorOutput(
                    timestamp=current_time,
                    prediction_horizon=horizon,
                    predicted_position=pos,
                    predicted_velocity=vel,
                    predicted_acceleration=acc,
                    position_covariance=pos_cov,
                    velocity_covariance=vel_cov,
                    model_probability=mode_probs[1],  # CA probability as primary indicator
                    is_valid=True,
                    predictor_type='imm'
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
                    namespace='imm_predictor',
                    scale=self.marker_scale
                )
                self.marker_pub.publish(markers)
            except Exception as e:
                self.get_logger().error(f'Marker creation failed: {e}')
        
        # Log prediction info (throttled)
        self.get_logger().info(
            f'IMM Prediction [P(CV)={mode_probs[0]:.2f}, P(CA)={mode_probs[1]:.2f}]: '
            f'pos=[{primary_pred.predicted_position[0]:.2f}, '
            f'{primary_pred.predicted_position[1]:.2f}, {primary_pred.predicted_position[2]:.2f}], '
            f'vel=[{primary_pred.predicted_velocity[0]:.2f}, '
            f'{primary_pred.predicted_velocity[1]:.2f}, {primary_pred.predicted_velocity[2]:.2f}], '
            f'acc=[{primary_pred.predicted_acceleration[0]:.2f}, '
            f'{primary_pred.predicted_acceleration[1]:.2f}, {primary_pred.predicted_acceleration[2]:.2f}]',
            throttle_duration_sec=2.0
        )
    
    def status_callback(self):
        """Publish predictor status information."""
        if not self.imm_filter.initialized:
            return
        
        mode_probs = self.imm_filter.get_mode_probabilities()
        
        # Get individual model predictions for diagnostics
        model_preds = self.imm_filter.get_model_predictions(horizon=1.0)
        
        status_dict = {
            'predictor_type': 'imm',
            'initialized': self.imm_filter.initialized,
            'mode_probabilities': {
                'cv': float(mode_probs[0]),
                'ca': float(mode_probs[1])
            },
            'active_model': 'ca' if mode_probs[1] > mode_probs[0] else 'cv',
            'model_predictions': {
                'cv': {
                    'position': model_preds['cv']['position'].tolist(),
                    'velocity': model_preds['cv']['velocity'].tolist()
                },
                'ca': {
                    'position': model_preds['ca']['position'].tolist(),
                    'velocity': model_preds['ca']['velocity'].tolist(),
                    'acceleration': model_preds['ca']['acceleration'].tolist()
                }
            }
        }
        
        status_msg = String()
        status_msg.data = json.dumps(status_dict, indent=2)
        self.status_pub.publish(status_msg)
    
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
        
        # Acceleration (NED frame)
        msg.ax = float(prediction.predicted_acceleration[0])
        msg.ay = float(prediction.predicted_acceleration[1])
        msg.az = float(prediction.predicted_acceleration[2])
        
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
    """Main entry point for IMM predictor node."""
    rclpy.init(args=args)
    
    try:
        node = IMMPredictorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error in IMM predictor node: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

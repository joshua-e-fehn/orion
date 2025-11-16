#!/usr/bin/env python3
"""
Interacting Multiple Model (IMM) Target Predictor Node.

This node subscribes to target position/velocity and publishes predictions
using an adaptive combination of CV and CA models.
"""

import rclpy
from rclpy.node import Node
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
        ca_model = CAModel(ca_process_noise, ca_measurement_noise)
        
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
        
        # Create subscribers
        target_topic = f'/{self.target_namespace}/fmu/out/vehicle_local_position'
        self.target_sub = self.create_subscription(
            VehicleLocalPosition,
            target_topic,
            self.target_callback,
            10
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
        
        # State
        self.last_measurement_time = None
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
        current_time = self.get_clock().now().nanoseconds / 1e9
        
        # Check validity flags
        if not (msg.xy_valid and msg.z_valid and msg.v_xy_valid and msg.v_z_valid):
            self.get_logger().warn('Received invalid target measurement', throttle_duration_sec=1.0)
            return
        
        # Extract position and velocity (already in NED from PX4)
        position = np.array([msg.x, msg.y, msg.z])
        velocity = np.array([msg.vx, msg.vy, msg.vz])
        
        # Extract acceleration if available (PX4 provides it)
        acceleration = np.array([msg.ax, msg.ay, msg.az])
        
        # Calculate dt
        if self.last_measurement_time is not None:
            dt = current_time - self.last_measurement_time
            if dt < 0:
                self.get_logger().error('Negative dt - time went backwards!')
                return
        else:
            dt = 0.1  # Default dt for first measurement
        
        # Update IMM filter
        self.imm_filter.update(position, velocity, dt, acceleration)
        
        self.last_measurement_time = current_time
        
        # Log diagnostics (throttled)
        if self.imm_filter.initialized:
            mode_probs = self.imm_filter.get_mode_probabilities()
            self.get_logger().debug(
                f'IMM Update: dt={dt:.3f}s, P(CV)={mode_probs[0]:.3f}, P(CA)={mode_probs[1]:.3f}',
                throttle_duration_sec=1.0
            )
    
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
                pos, vel, acc, pos_cov, vel_cov = self.imm_filter.predict(horizon)
                
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

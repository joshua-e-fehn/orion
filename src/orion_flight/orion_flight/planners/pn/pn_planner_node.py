#!/usr/bin/env python3
"""Proportional Navigation (PN) planner ROS2 node."""

import rclpy
import numpy as np
import json

from px4_msgs.msg import VehicleLocalPosition, TrajectorySetpoint, OffboardControlMode
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import String

from orion_flight.planners.common.planner_base import PlannerBase
from orion_flight.planners.common.types import PlannerInput, PlannerOutput
from orion_flight.planners.common.visualization import create_guidance_markers

from orion_flight.planners.pn.pn_algorithm import ProportionalNavigationAlgorithm


class PNPlannerNode(PlannerBase):
    """
    ROS2 node for Proportional Navigation guidance law.
    
    Subscribes to interceptor and target states, computes PN guidance,
    and publishes trajectory setpoints to PX4.
    """
    
    def __init__(self):
        super().__init__('pn_planner')
        
        # Declare ROS parameters
        self.declare_parameter('N', 3.0)  # Navigation constant
        self.declare_parameter('amax', [4.0, 4.0, 2.0])
        self.declare_parameter('control_rate', 20.0)
        self.declare_parameter('interceptor_namespace', 'px4_1')
        self.declare_parameter('target_namespace', 'px4_2')
        self.declare_parameter('use_predictor', True)
        self.declare_parameter('convergence_distance', 1.0)  # meters
        self.declare_parameter('min_tgo', 0.05)  # seconds
        self.declare_parameter('v_eps', 0.1)  # m/s
        
        # Get parameters
        params = {
            'N': self.get_parameter('N').value,
            'amax': self.get_parameter('amax').value,
            'dt': 1.0 / self.get_parameter('control_rate').value,
            'min_tgo': self.get_parameter('min_tgo').value,
            'v_eps': self.get_parameter('v_eps').value
        }
        
        self.control_rate = self.get_parameter('control_rate').value
        self.interceptor_ns = self.get_parameter('interceptor_namespace').value
        self.target_ns = self.get_parameter('target_namespace').value
        self.use_predictor = self.get_parameter('use_predictor').value
        self.convergence_distance = self.get_parameter('convergence_distance').value
        
        # Initialize PN algorithm
        self.algorithm = ProportionalNavigationAlgorithm(params)
        
        # State storage
        self.interceptor_state = None
        self.target_state = None
        self.predicted_state = None
        
        # Create subscriptions
        self._create_subscriptions()
        
        # Create publishers
        self._create_publishers()
        
        # Create timer for control loop
        self.control_timer = self.create_timer(
            1.0 / self.control_rate,
            self.control_loop_callback
        )
        
        # Mark as initialized
        self.set_initialized(True)
        
        self.get_logger().info(
            f'PN Planner initialized: N={params["N"]}, '
            f'amax={params["amax"]}, rate={self.control_rate} Hz'
        )
    
    def _create_subscriptions(self):
        """Create ROS2 subscriptions."""
        # Interceptor state (ego drone)
        self.create_subscription(
            VehicleLocalPosition,
            f'/{self.interceptor_ns}/fmu/out/vehicle_local_position',
            self.interceptor_callback,
            10
        )
        
        # Target current state
        self.create_subscription(
            VehicleLocalPosition,
            f'/{self.target_ns}/fmu/out/vehicle_local_position',
            self.target_callback,
            10
        )
        
        # Target predicted state (from predictor)
        if self.use_predictor:
            self.create_subscription(
                VehicleLocalPosition,
                '/target/predicted_state',
                self.predicted_callback,
                10
            )
    
    def _create_publishers(self):
        """Create ROS2 publishers."""
        # Trajectory setpoint (primary control output)
        self.trajectory_pub = self.create_publisher(
            TrajectorySetpoint,
            f'/{self.interceptor_ns}/fmu/in/trajectory_setpoint',
            10
        )
        
        # Offboard control mode (heartbeat)
        self.offboard_pub = self.create_publisher(
            OffboardControlMode,
            f'/{self.interceptor_ns}/fmu/in/offboard_control_mode',
            10
        )
        
        # Visualization markers
        self.markers_pub = self.create_publisher(
            MarkerArray,
            '/planner/guidance_markers',
            10
        )
        
        # Status messages
        self.status_pub = self.create_publisher(
            String,
            '/planner/status',
            10
        )
    
    def interceptor_callback(self, msg: VehicleLocalPosition):
        """Callback for interceptor state updates."""
        self.interceptor_state = msg
    
    def target_callback(self, msg: VehicleLocalPosition):
        """Callback for target current state updates."""
        self.target_state = msg
    
    def predicted_callback(self, msg: VehicleLocalPosition):
        """Callback for target predicted state updates."""
        self.predicted_state = msg
    
    def control_loop_callback(self):
        """Main control loop (runs at control_rate Hz)."""
        # Check if we have necessary data
        if self.interceptor_state is None:
            self.get_logger().warn('No interceptor state available', throttle_duration_sec=2.0)
            return
        
        # Use predicted state if available and enabled, otherwise use current state
        if self.use_predictor and self.predicted_state is not None:
            target = self.predicted_state
            using_prediction = True
        else:
            target = self.target_state
            using_prediction = False
        
        if target is None:
            self.get_logger().warn('No target state available', throttle_duration_sec=2.0)
            return
        
        # Build planner input
        planner_input = self._build_planner_input(self.interceptor_state, target, using_prediction)
        
        # Compute guidance
        planner_output = self.compute_guidance(planner_input)
        
        if planner_output.is_valid:
            # Publish trajectory setpoint
            self._publish_trajectory_setpoint(planner_output)
            
            # Publish offboard heartbeat
            self._publish_offboard_heartbeat()
            
            # Publish visualization (at lower rate)
            if self.algorithm.get_command_count() % 4 == 0:  # ~5 Hz if control at 20 Hz
                self._publish_visualization(planner_input, planner_output)
            
            # Publish status (at lower rate)
            if self.algorithm.get_command_count() % 20 == 0:  # ~1 Hz if control at 20 Hz
                self._publish_status(planner_output)
    
    def _build_planner_input(self, interceptor_msg, target_msg, using_prediction: bool) -> PlannerInput:
        """Build PlannerInput from ROS messages."""
        return PlannerInput(
            timestamp=self.get_clock().now().nanoseconds / 1e9,
            interceptor_position=np.array([interceptor_msg.x, interceptor_msg.y, interceptor_msg.z]),
            interceptor_velocity=np.array([interceptor_msg.vx, interceptor_msg.vy, interceptor_msg.vz]),
            interceptor_acceleration=np.array([interceptor_msg.ax, interceptor_msg.ay, interceptor_msg.az]),
            target_position=np.array([target_msg.x, target_msg.y, target_msg.z]),
            target_velocity=np.array([target_msg.vx, target_msg.vy, target_msg.vz]),
            target_acceleration=np.array([target_msg.ax, target_msg.ay, target_msg.az]),
            target_position_covariance=np.eye(3),  # Default
            target_velocity_covariance=np.eye(3),
            interceptor_valid=True,
            target_valid=True,
            prediction_available=using_prediction
        )
    
    def compute_guidance(self, planner_input: PlannerInput) -> PlannerOutput:
        """
        Compute Proportional Navigation guidance command.
        
        Args:
            planner_input: Current states
        
        Returns:
            PlannerOutput with commanded setpoints
        """
        # Call PN algorithm
        result = self.algorithm.compute_command(
            p_i=planner_input.interceptor_position,
            v_i=planner_input.interceptor_velocity,
            p_t=planner_input.target_position,
            v_t=planner_input.target_velocity
        )
        
        # Build output
        return PlannerOutput(
            timestamp=planner_input.timestamp,
            commanded_acceleration=result['acceleration'],
            commanded_velocity=result['velocity'],
            commanded_position=result['position'],
            commanded_yaw=0.0,  # PN doesn't specify yaw
            commanded_yaw_rate=0.0,
            time_to_go=result['tgo'],
            closing_velocity=result['closing_velocity'],
            miss_distance=result['miss_distance'],
            is_valid=True,
            guidance_active=True
        )
    
    def _publish_trajectory_setpoint(self, output: PlannerOutput):
        """Publish trajectory setpoint to PX4."""
        msg = TrajectorySetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        
        # Position (NED)
        msg.position = [float(x) for x in output.commanded_position]
        
        # Velocity (NED)
        msg.velocity = [float(v) for v in output.commanded_velocity]
        
        # Acceleration (NED)
        msg.acceleration = [float(a) for a in output.commanded_acceleration]
        
        # Yaw (default: face direction of motion)
        msg.yaw = float(output.commanded_yaw)
        
        self.trajectory_pub.publish(msg)
    
    def _publish_offboard_heartbeat(self):
        """Publish offboard control mode heartbeat."""
        msg = OffboardControlMode()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = True
        msg.velocity = True
        msg.acceleration = True
        
        self.offboard_pub.publish(msg)
    
    def _publish_visualization(self, input_data: PlannerInput, output: PlannerOutput):
        """Publish visualization markers for RViz."""
        markers = create_guidance_markers(
            interceptor_pos=input_data.interceptor_position,
            target_pos=input_data.target_position,
            acceleration=output.commanded_acceleration,
            frame_id='map',
            namespace='pn_planner',
            scale_accel=0.5
        )
        
        self.markers_pub.publish(markers)
    
    def _publish_status(self, output: PlannerOutput):
        """Publish planner status as JSON."""
        status = {
            'planner': 'proportional_navigation',
            'active': output.guidance_active,
            'tgo': float(output.time_to_go),
            'closing_velocity': float(output.closing_velocity),
            'miss_distance': float(output.miss_distance),
            'converged': self.is_converged(),
            'commands_issued': self.algorithm.get_command_count()
        }
        
        msg = String()
        msg.data = json.dumps(status)
        self.status_pub.publish(msg)
    
    def reset(self):
        """Reset planner state."""
        self.algorithm.reset()
        self.get_logger().info('PN Planner reset')
    
    def is_converged(self) -> bool:
        """Check if intercept is complete."""
        if self.interceptor_state is None or self.target_state is None:
            return False
        
        # Compute distance to target
        dp = np.array([
            self.target_state.x - self.interceptor_state.x,
            self.target_state.y - self.interceptor_state.y,
            self.target_state.z - self.interceptor_state.z
        ])
        
        distance = np.linalg.norm(dp)
        
        return distance < self.convergence_distance
    
    def get_planner_type(self) -> str:
        """Get planner type identifier."""
        return 'pn'


def main(args=None):
    """Main entry point for PN planner node."""
    rclpy.init(args=args)
    node = PNPlannerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('PN Planner shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

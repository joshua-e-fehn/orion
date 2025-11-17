#!/usr/bin/env python3
"""Pure Pursuit (PP) planner ROS2 node."""

import rclpy
import numpy as np
import json

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import VehicleLocalPosition, TrajectorySetpoint, OffboardControlMode, VehicleCommand, VehicleStatus, VehicleCommandAck
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import String

from orion_flight.planners.common.planner_base import PlannerBase
from orion_flight.planners.common.types import PlannerInput, PlannerOutput
from orion_flight.planners.common.visualization import create_guidance_markers

from orion_flight.planners.pp.pp_algorithm import PurePursuitAlgorithm


class PPPlannerNode(PlannerBase):
    """
    ROS2 node for Pure Pursuit guidance law.
    
    Subscribes to interceptor and target states, computes PP guidance,
    and publishes trajectory setpoints to PX4.
    """
    
    def __init__(self):
        super().__init__('pp_planner')
        
        # Declare ROS parameters
        self.declare_parameter('G_pp', 2.0)
        self.declare_parameter('amax', [4.0, 4.0, 2.0])
        self.declare_parameter('control_rate', 20.0)
        self.declare_parameter('interceptor_namespace', 'px4_2')
        self.declare_parameter('target_namespace', 'px4_1')
        self.declare_parameter('use_predictor', True)
        self.declare_parameter('convergence_distance', 1.0)  # meters
        
        # Get parameters
        params = {
            'G_pp': self.get_parameter('G_pp').value,
            'amax': self.get_parameter('amax').value,
            'dt': 1.0 / self.get_parameter('control_rate').value
        }
        
        self.control_rate = self.get_parameter('control_rate').value
        self.interceptor_ns = 'px4_2'#self.get_parameter('interceptor_namespace').value
        self.target_ns = self.get_parameter('target_namespace').value
        self.use_predictor = self.get_parameter('use_predictor').value
        self.convergence_distance = self.get_parameter('convergence_distance').value
        
        # Max acceleration for safety
        self.MAX_ACCEL = 1.0  # m/s²
        
        # Initialize PP algorithm
        self.algorithm = PurePursuitAlgorithm(params)
        
        # State storage
        self.interceptor_state = None
        self.target_state = None
        self.predicted_state = None
        self.interceptor_status = None
        
        # Auto-arm/offboard state tracking
        self.flight_phase = "INIT"
        self.offboard_setpoint_counter = 0
        self.is_armed = False
        self.is_offboard = False
        
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
            f'PP Planner initialized: G_pp={params["G_pp"]}, '
            f'amax={params["amax"]}, rate={self.control_rate} Hz'
        )
    
    def _create_subscriptions(self):
        """Create ROS2 subscriptions."""
        # QoS profile for PX4 topics (BEST_EFFORT reliability)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Interceptor state (ego drone)
        self.create_subscription(
            VehicleLocalPosition,
            f'/{self.interceptor_ns}/fmu/out/vehicle_local_position_v1',
            self.interceptor_callback,
            qos_profile
        )
        
        # Target current state
        self.create_subscription(
            VehicleLocalPosition,
            f'/{self.target_ns}/fmu/out/vehicle_local_position_v1',
            self.target_callback,
            qos_profile
        )
        
        # Target predicted state (from predictor)
        if self.use_predictor:
            self.create_subscription(
                VehicleLocalPosition,
                '/target/predicted_state',
                self.predicted_callback,
                qos_profile
            )
        
        # Vehicle status (for arming and offboard mode tracking)
        self.create_subscription(
            VehicleStatus,
            f'/{self.interceptor_ns}/fmu/out/vehicle_status_v1',
            self.vehicle_status_callback,
            qos_profile
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
        
        # Vehicle command publisher (for arming and mode changes)
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand,
            f'/{self.interceptor_ns}/fmu/in/vehicle_command',
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
    
    def vehicle_status_callback(self, msg: VehicleStatus):
        """Callback for vehicle status updates."""
        self.is_armed = (msg.arming_state == 2)  # ARMING_STATE_ARMED
        self.is_offboard = (msg.nav_state == 14)  # NAVIGATION_STATE_OFFBOARD
    
    def arm(self):
        """Send arm command to vehicle."""
        cmd = VehicleCommand()
        cmd.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        cmd.command = 400  # VEHICLE_CMD_COMPONENT_ARM_DISARM
        cmd.param1 = 1.0  # 1 to arm
        cmd.param2 = 21196.0  # Force arm
        cmd.target_system = 0  # 0 = broadcast to local system
        cmd.target_component = 1
        cmd.source_system = 1
        cmd.source_component = 1
        cmd.from_external = True
        
        self.vehicle_command_pub.publish(cmd)
        self.get_logger().info('Arm command sent')
    
    def engage_offboard_mode(self):
        """Send offboard mode command to vehicle."""
        cmd = VehicleCommand()
        cmd.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        cmd.command = 176  # VEHICLE_CMD_DO_SET_MODE
        cmd.param1 = 1.0  # Base mode: custom
        cmd.param2 = 6.0  # Custom main mode: OFFBOARD
        cmd.target_system = 0
        cmd.target_component = 1
        cmd.source_system = 1
        cmd.source_component = 1
        cmd.from_external = True
        
        self.vehicle_command_pub.publish(cmd)
        self.get_logger().info('Offboard mode command sent')
    
    def control_loop_callback(self):
        """Main control loop (runs at control_rate Hz)."""
        # Always send offboard heartbeat and setpoints
        self._publish_offboard_heartbeat()
        
        # Check if we have necessary data
        if self.interceptor_state is None:
            self.get_logger().warn('No interceptor state available', throttle_duration_sec=2.0)
            return
        
        # Send a default hover setpoint during initialization
        if self.flight_phase == "INIT":
            self._publish_hover_setpoint()
        
        # Auto-initialization state machine
        if self.flight_phase == "INIT":
            # Send setpoints for at least 20 iterations before engaging OFFBOARD
            self.offboard_setpoint_counter += 1
            
            if self.offboard_setpoint_counter == 10:
                self.get_logger().info("Sending initial setpoints before OFFBOARD mode...")
            elif self.offboard_setpoint_counter == 20:
                self.get_logger().info("Engaging OFFBOARD mode and arming...")
                self.engage_offboard_mode()
            elif self.offboard_setpoint_counter == 25:
                self.arm()
            elif self.offboard_setpoint_counter >= 30:
                # Wait for drone to be armed and in OFFBOARD mode
                if self.is_armed and self.is_offboard:
                    self.flight_phase = "RUNNING"
                    self.get_logger().info("✓ Armed and in OFFBOARD mode, starting guidance!")
                else:
                    # Keep trying to engage offboard and arm every 50 iterations (~2.5s)
                    if self.offboard_setpoint_counter % 50 == 0:
                        armed_status = "ARMED" if self.is_armed else "DISARMED"
                        mode_status = "OFFBOARD" if self.is_offboard else "NOT_OFFBOARD"
                        self.get_logger().warn(f"Waiting for ready state: {armed_status}, {mode_status}")
                        if not self.is_offboard:
                            self.engage_offboard_mode()
                        if not self.is_armed:
                            self.arm()
            return
        
        # RUNNING phase - normal guidance operation
        if not self.is_armed or not self.is_offboard:
            self.get_logger().warn('Vehicle not armed/offboard, waiting...', throttle_duration_sec=2.0)
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
        Compute Pure Pursuit guidance command pointing directly toward the target.
        
        Args:
            planner_input: Current states
        
        Returns:
            PlannerOutput with commanded setpoints
        """
        # Vector from interceptor to target
        direction = planner_input.target_position - planner_input.interceptor_position
        distance = np.linalg.norm(direction)
        
        if distance > 0.0:
            direction_unit = direction / distance
        else:
            direction_unit = np.zeros(3)
        
        # Pure Pursuit acceleration command
        acc_command = self.algorithm.G_pp * direction_unit
        
        # Clamp acceleration magnitude to MAX_ACCEL
        acc_norm = np.linalg.norm(acc_command)
        if acc_norm > self.MAX_ACCEL:
            acc_command = acc_command / acc_norm * self.MAX_ACCEL
        
        # Optional: compute velocity command as smoothed approach toward target
        desired_speed = min(np.linalg.norm(planner_input.interceptor_velocity) + 1.0, 5.0)  # m/s
        vel_command = direction_unit * desired_speed
        
        # Position command: current position + small step toward target
        pos_command = planner_input.interceptor_position + direction_unit * 0.5  # 0.5 m step
        
        # Time-to-go and miss distance for reporting
        tgo = distance / max(np.linalg.norm(vel_command), 1e-3)
        miss_distance = distance
        
        return PlannerOutput(
            timestamp=planner_input.timestamp,
            commanded_acceleration=acc_command,
            commanded_velocity=vel_command,
            commanded_position=pos_command,
            commanded_yaw=0.0,
            commanded_yaw_rate=0.0,
            time_to_go=tgo,
            closing_velocity=np.dot(planner_input.target_velocity - planner_input.interceptor_velocity, direction_unit),
            miss_distance=miss_distance,
            is_valid=True,
            guidance_active=True
        )


    def _publish_trajectory_setpoint(self, output: PlannerOutput):
        """
        Publish trajectory setpoint to PX4 with proper direction and clamped magnitude.
        """
        msg = TrajectorySetpoint()
        now = self.get_clock().now()
        msg.timestamp = int(now.nanoseconds / 1000)

        # Position: move slightly toward target
        msg.position = [
            float(output.commanded_position[0]),  # add 0.5 m offset in x
            float(output.commanded_position[1]),
            float(output.commanded_position[2])
        ]

        # Velocity: direct toward target
        msg.velocity = [float(v) for v in output.commanded_velocity[:3]]

        # Acceleration: clamp vector magnitude
        acc = np.array(output.commanded_acceleration[:3], dtype=float)
        acc_norm = np.linalg.norm(acc)
        if acc_norm > self.MAX_ACCEL:
            acc = acc / acc_norm * self.MAX_ACCEL
        msg.acceleration = acc.tolist()

        # Yaw (optional, can keep 0)
        msg.yaw = float(output.commanded_yaw) if output.commanded_yaw is not None else 0.0

        # Publish
        self.trajectory_pub.publish(msg)

        # Debug logging every 20 commands
        if self.algorithm.get_command_count() % 20 == 0:
            self.get_logger().info(
                f'Setpoint -> pos: {msg.position}, vel: {msg.velocity}, accel: {msg.acceleration}'
            )


    
    def _publish_offboard_heartbeat(self):
        """Publish offboard control mode heartbeat."""
        msg = OffboardControlMode()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = True
        msg.velocity = True
        msg.acceleration = True
        msg.attitude = False
        msg.body_rate = False
        
        self.offboard_pub.publish(msg)
    
    def _publish_hover_setpoint(self):
        """Publish a hover setpoint at current position during initialization."""
        if self.interceptor_state is None:
            return
        
        msg = TrajectorySetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        
        # Hover at current position
        msg.position = [float(self.interceptor_state.x), 
                       float(self.interceptor_state.y), 
                       float(self.interceptor_state.z)]
        msg.velocity = [0.0, 0.0, 0.0]
        msg.acceleration = [0.0, 0.0, 0.0]
        msg.yaw = 0.0
        
        self.trajectory_pub.publish(msg)
    
    def _publish_visualization(self, input_data: PlannerInput, output: PlannerOutput):
        """Publish visualization markers for RViz."""
        markers = create_guidance_markers(
            interceptor_pos=input_data.interceptor_position,
            target_pos=input_data.target_position,
            acceleration=output.commanded_acceleration,
            frame_id='map',
            namespace='pp_planner',
            scale_accel=0.5
        )
        
        self.markers_pub.publish(markers)
    
    def _publish_status(self, output: PlannerOutput):
        """Publish planner status as JSON."""
        status = {
            'planner': 'pure_pursuit',
            'active': bool(output.guidance_active),
            'tgo': float(output.time_to_go),
            'closing_velocity': float(output.closing_velocity),
            'miss_distance': float(output.miss_distance),
            'converged': bool(self.is_converged()),
            'commands_issued': int(self.algorithm.get_command_count())
        }
        
        msg = String()
        msg.data = json.dumps(status)
        self.status_pub.publish(msg)
    
    def reset(self):
        """Reset planner state."""
        self.algorithm.reset()
        self.get_logger().info('PP Planner reset')
    
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
        return 'pp'


def main(args=None):
    """Main entry point for PP planner node."""
    rclpy.init(args=args)
    node = PPPlannerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('PP Planner shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

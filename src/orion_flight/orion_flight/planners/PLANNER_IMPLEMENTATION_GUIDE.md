# Planner Implementation Guide

## Introduction

This guide provides step-by-step instructions for implementing new guidance and planning algorithms that integrate seamlessly with the Orion interception system.

**Target Audience**: Engineers implementing guidance laws for drone interception.

**Prerequisites**:
- Understanding of guidance and control theory
- Familiarity with ROS2 Python programming
- Knowledge of the guidance algorithm you want to implement
- Review of `algo_guide.md` for algorithm details

## Implementation Steps

### Step 1: Define Your Guidance Law

First, mathematically define your guidance algorithm:

**Inputs:**
- Interceptor state: `p_i, v_i, a_i`
- Target state: `p_t, v_t, a_t` (current or predicted)
- Algorithm parameters: gains, limits, etc.

**Outputs:**
- Commanded acceleration: `a_cmd`

**Key Metrics:**
- Time-to-go: `t_go`
- Closing velocity: `V_c`
- Line-of-sight (LOS) vector: `λ`

**Example (Pure Pursuit)**:
```
a_cmd = G_pp * (p_t - p_i)
```

**Example (Proportional Navigation)**:
```
λ = p_t - p_i
λ_dot = (λ × (v_t - v_i)) / ||λ||²
V_c = -(λ · (v_t - v_i)) / ||λ||
a_cmd = N * V_c * λ_dot
```

### Step 2: Create Planner Directory

```bash
cd src/orion_flight/planners
mkdir my_planner
cd my_planner
touch __init__.py
touch my_planner_node.py
touch my_algorithm.py
```

### Step 3: Implement the Algorithm Class

Create `my_algorithm.py` with the guidance law logic:

```python
"""My Planner guidance law implementation."""

import numpy as np
from typing import Dict


class MyPlannerAlgorithm:
    """
    Implementation of My Planner guidance law.
    
    This class contains the core algorithm logic, separate from ROS2 integration.
    """
    
    def __init__(self, params: Dict):
        """
        Initialize the planner algorithm.
        
        Args:
            params: Dictionary with algorithm parameters
                - 'G': Proportional gain
                - 'amax': Max acceleration [ax, ay, az] (m/s²)
                - 'min_tgo': Minimum time-to-go safeguard (seconds)
                - ... other algorithm-specific parameters
        """
        # Extract parameters
        self.G = params.get('G', 10.0)
        self.amax = np.array(params.get('amax', [4.0, 4.0, 2.0]))
        self.min_tgo = params.get('min_tgo', 0.05)
        
        # Internal state (if needed)
        self.last_command = np.zeros(3)
    
    def compute_command(self,
                       p_i: np.ndarray,  # Interceptor position
                       v_i: np.ndarray,  # Interceptor velocity
                       p_t: np.ndarray,  # Target position
                       v_t: np.ndarray,  # Target velocity
                       dt: float = 0.02  # Time step
                       ) -> Dict:
        """
        Compute guidance command.
        
        Args:
            p_i: Interceptor position [x, y, z] in NED (m)
            v_i: Interceptor velocity [vx, vy, vz] in NED (m/s)
            p_t: Target position [x, y, z] in NED (m)
            v_t: Target velocity [vx, vy, vz] in NED (m/s)
            dt: Time step for integration (seconds)
        
        Returns:
            Dictionary with:
                - 'acceleration': Commanded acceleration [ax, ay, az] (m/s²)
                - 'velocity': Commanded velocity [vx, vy, vz] (m/s)
                - 'position': Commanded position [x, y, z] (m)
                - 'tgo': Time-to-go estimate (seconds)
                - 'closing_velocity': Closing speed (m/s)
                - 'miss_distance': Current miss distance (m)
        """
        # Compute relative state
        dp = p_t - p_i  # Position error
        dv = v_t - v_i  # Velocity error
        
        # Compute guidance metrics
        miss_distance = np.linalg.norm(dp)
        
        # Time-to-go estimate (safeguarded)
        dv_norm = np.linalg.norm(dv)
        if dv_norm < 1e-3:
            tgo = max(self.min_tgo, miss_distance / max(np.linalg.norm(v_i), 1e-3))
        else:
            tgo = max(self.min_tgo, miss_distance / dv_norm)
        
        # Closing velocity
        if miss_distance > 1e-6:
            closing_velocity = -np.dot(dp, dv) / miss_distance
        else:
            closing_velocity = 0.0
        
        # CORE GUIDANCE LAW COMPUTATION
        # ==============================
        # Example: Simple proportional guidance
        a_cmd = self.G * dp
        
        # Apply acceleration limits (per-axis clamping)
        a_cmd = np.clip(a_cmd, -self.amax, self.amax)
        
        # Integrate to get velocity and position commands
        v_cmd = v_i + a_cmd * dt
        p_cmd = p_i + v_i * dt + 0.5 * a_cmd * (dt ** 2)
        
        # Store for next iteration
        self.last_command = a_cmd
        
        # Return command package
        return {
            'acceleration': a_cmd,
            'velocity': v_cmd,
            'position': p_cmd,
            'tgo': tgo,
            'closing_velocity': closing_velocity,
            'miss_distance': miss_distance
        }
    
    def reset(self):
        """Reset internal state."""
        self.last_command = np.zeros(3)


# Helper functions used by guidance laws
def norm(v: np.ndarray) -> float:
    """Compute Euclidean norm of vector."""
    return np.linalg.norm(v)


def clamp_vec(v: np.ndarray, v_max: np.ndarray) -> np.ndarray:
    """Clamp vector per-axis to ±v_max."""
    return np.clip(v, -v_max, v_max)


def safe_normalize(v: np.ndarray, eps: float = 1e-6) -> np.ndarray:
    """Safely normalize vector (returns zero if norm too small)."""
    v_norm = np.linalg.norm(v)
    if v_norm < eps:
        return np.zeros_like(v)
    return v / v_norm
```

### Step 4: Implement the ROS2 Node

Create `my_planner_node.py` inheriting from `PlannerBase`:

```python
"""My Planner ROS2 node."""

import rclpy
from rclpy.node import Node
import numpy as np

from px4_msgs.msg import VehicleLocalPosition, TrajectorySetpoint, OffboardControlMode
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import String

from ..common.planner_base import PlannerBase
from ..common.types import PlannerInput, PlannerOutput
from ..common.utils import ned_to_enu
from ..common.visualization import create_guidance_markers

from .my_algorithm import MyPlannerAlgorithm


class MyPlannerNode(PlannerBase):
    """
    ROS2 node for My Planner guidance law.
    """
    
    def __init__(self):
        super().__init__('my_planner')
        
        # Declare ROS parameters
        self.declare_parameter('G', 10.0)
        self.declare_parameter('amax', [4.0, 4.0, 2.0])
        self.declare_parameter('min_tgo', 0.05)
        self.declare_parameter('control_rate', 20.0)
        self.declare_parameter('interceptor_namespace', 'px4_1')
        self.declare_parameter('target_namespace', 'px4_2')
        
        # Get parameters
        params = {
            'G': self.get_parameter('G').value,
            'amax': self.get_parameter('amax').value,
            'min_tgo': self.get_parameter('min_tgo').value
        }
        
        self.control_rate = self.get_parameter('control_rate').value
        self.interceptor_ns = self.get_parameter('interceptor_namespace').value
        self.target_ns = self.get_parameter('target_namespace').value
        
        # Initialize algorithm
        self.algorithm = MyPlannerAlgorithm(params)
        
        # Initialize state
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
        
        self.get_logger().info(f'My Planner initialized (G={params["G"]}, rate={self.control_rate} Hz)')
    
    def _create_subscriptions(self):
        """Create ROS2 subscriptions."""
        # Interceptor state
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
        
        # Status
        self.status_pub = self.create_publisher(
            String,
            '/planner/status',
            10
        )
    
    def interceptor_callback(self, msg: VehicleLocalPosition):
        """Callback for interceptor state."""
        self.interceptor_state = msg
    
    def target_callback(self, msg: VehicleLocalPosition):
        """Callback for target current state."""
        self.target_state = msg
    
    def predicted_callback(self, msg: VehicleLocalPosition):
        """Callback for target predicted state."""
        self.predicted_state = msg
    
    def control_loop_callback(self):
        """Main control loop."""
        # Check if we have necessary data
        if self.interceptor_state is None:
            return
        
        # Use predicted state if available, otherwise current state
        target = self.predicted_state if self.predicted_state is not None else self.target_state
        
        if target is None:
            return
        
        # Build planner input
        planner_input = self._build_planner_input(self.interceptor_state, target)
        
        # Compute guidance
        planner_output = self.compute_guidance(planner_input)
        
        if planner_output.is_valid:
            # Publish trajectory setpoint
            self._publish_trajectory_setpoint(planner_output)
            
            # Publish offboard heartbeat
            self._publish_offboard_heartbeat()
            
            # Publish visualization
            self._publish_visualization(planner_input, planner_output)
            
            # Publish status
            self._publish_status(planner_output)
    
    def _build_planner_input(self, interceptor_msg, target_msg) -> PlannerInput:
        """Build PlannerInput from ROS messages."""
        return PlannerInput(
            timestamp=self.get_clock().now().nanoseconds / 1e9,
            interceptor_position=np.array([interceptor_msg.x, interceptor_msg.y, interceptor_msg.z]),
            interceptor_velocity=np.array([interceptor_msg.vx, interceptor_msg.vy, interceptor_msg.vz]),
            interceptor_acceleration=np.array([interceptor_msg.ax, interceptor_msg.ay, interceptor_msg.az]),
            target_position=np.array([target_msg.x, target_msg.y, target_msg.z]),
            target_velocity=np.array([target_msg.vx, target_msg.vy, target_msg.vz]),
            target_acceleration=np.array([target_msg.ax, target_msg.ay, target_msg.az]),
            target_position_covariance=np.eye(3),  # Default if not available
            target_velocity_covariance=np.eye(3),
            interceptor_valid=True,
            target_valid=True,
            prediction_available=(self.predicted_state is not None)
        )
    
    def compute_guidance(self, planner_input: PlannerInput) -> PlannerOutput:
        """
        Compute guidance command (implements abstract method).
        
        Args:
            planner_input: Current states
        
        Returns:
            PlannerOutput with command
        """
        # Call algorithm
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
            commanded_yaw=0.0,  # Optional
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
        
        # Yaw
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
        """Publish visualization markers."""
        markers = create_guidance_markers(
            interceptor_pos=input_data.interceptor_position,
            target_pos=input_data.target_position,
            acceleration=output.commanded_acceleration,
            frame_id='map',
            namespace='my_planner'
        )
        
        self.markers_pub.publish(markers)
    
    def _publish_status(self, output: PlannerOutput):
        """Publish planner status."""
        import json
        status = {
            'planner': 'my_planner',
            'active': output.guidance_active,
            'tgo': output.time_to_go,
            'closing_velocity': output.closing_velocity,
            'miss_distance': output.miss_distance
        }
        
        msg = String()
        msg.data = json.dumps(status)
        self.status_pub.publish(msg)
    
    def reset(self):
        """Reset planner."""
        self.algorithm.reset()
    
    def is_converged(self) -> bool:
        """Check if intercept complete."""
        if self.interceptor_state is None or self.target_state is None:
            return False
        
        # Converged if within threshold distance
        dp = np.array([self.target_state.x - self.interceptor_state.x,
                      self.target_state.y - self.interceptor_state.y,
                      self.target_state.z - self.interceptor_state.z])
        
        return np.linalg.norm(dp) < 1.0  # 1 meter threshold
    
    def get_planner_type(self) -> str:
        """Get planner type."""
        return 'my_planner'


def main(args=None):
    rclpy.init(args=args)
    node = MyPlannerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Step 5: Update `__init__.py`

```python
"""My Planner package."""

from .my_planner_node import MyPlannerNode
from .my_algorithm import MyPlannerAlgorithm

__all__ = ['MyPlannerNode', 'MyPlannerAlgorithm']
```

### Step 6: Add Entry Point to `setup.py`

Edit `src/orion_flight/setup.py`:

```python
entry_points={
    'console_scripts': [
        # ... existing entries ...
        'my_planner = planners.my_planner.my_planner_node:main',
    ],
},
```

### Step 7: Create Launch File

Create `src/orion_flight/launch/my_planner.launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('G', default_value='10.0'),
        DeclareLaunchArgument('amax_x', default_value='4.0'),
        DeclareLaunchArgument('amax_y', default_value='4.0'),
        DeclareLaunchArgument('amax_z', default_value='2.0'),
        DeclareLaunchArgument('control_rate', default_value='20.0'),
        
        Node(
            package='orion_flight',
            executable='my_planner',
            name='my_planner_node',
            output='screen',
            parameters=[{
                'G': LaunchConfiguration('G'),
                'amax': [
                    LaunchConfiguration('amax_x'),
                    LaunchConfiguration('amax_y'),
                    LaunchConfiguration('amax_z')
                ],
                'control_rate': LaunchConfiguration('control_rate'),
                'interceptor_namespace': 'px4_1',
                'target_namespace': 'px4_2'
            }]
        )
    ])
```

## Testing Your Planner

### Unit Tests

Create `tests/test_my_planner.py`:

```python
import numpy as np
from planners.my_planner.my_algorithm import MyPlannerAlgorithm


def test_stationary_target():
    """Test guidance to stationary target."""
    params = {'G': 10.0, 'amax': [4.0, 4.0, 2.0]}
    planner = MyPlannerAlgorithm(params)
    
    # Interceptor at origin, target at (10, 0, 0)
    p_i = np.array([0.0, 0.0, 0.0])
    v_i = np.array([0.0, 0.0, 0.0])
    p_t = np.array([10.0, 0.0, 0.0])
    v_t = np.array([0.0, 0.0, 0.0])
    
    result = planner.compute_command(p_i, v_i, p_t, v_t)
    
    # Should command acceleration toward target
    assert result['acceleration'][0] > 0  # Positive x acceleration
    assert abs(result['acceleration'][1]) < 0.1  # Near-zero y
    assert abs(result['acceleration'][2]) < 0.1  # Near-zero z


def test_acceleration_limits():
    """Test that acceleration is clamped."""
    params = {'G': 100.0, 'amax': [1.0, 1.0, 1.0]}
    planner = MyPlannerAlgorithm(params)
    
    p_i = np.array([0.0, 0.0, 0.0])
    v_i = np.array([0.0, 0.0, 0.0])
    p_t = np.array([100.0, 100.0, 100.0])
    v_t = np.array([0.0, 0.0, 0.0])
    
    result = planner.compute_command(p_i, v_i, p_t, v_t)
    
    # Should be clamped to amax
    assert np.all(np.abs(result['acceleration']) <= 1.0)
```

Run tests:
```bash
cd ~/Documents/Orion/orion_arm
colcon test --packages-select orion_flight
```

### SITL Integration Test

```bash
# Terminal 1: Start PX4 SITL
cd ~/PX4-Autopilot
make px4_sitl gz_x500

# Terminal 2: Start Micro XRCE-DDS Agent
MicroXRCEAgent udp4 -p 8888

# Terminal 3: Launch planner
cd ~/Documents/Orion/orion_arm
source install/setup.bash
ros2 launch orion_flight my_planner.launch.py

# Terminal 4: Monitor status
ros2 topic echo /planner/status

# Terminal 5: Visualize in RViz
rviz2 -d config/planner_visualization.rviz
```

## Algorithm-Specific Implementation Notes

### Pure Pursuit (PP)

**Key equation:**
```
a_cmd = G_pp * (p_t - p_i)
```

**Considerations:**
- Very simple, no velocity term
- Can overshoot on fast-moving targets
- Good baseline for testing

### Proportional Navigation (PN)

**Key equations:**
```
λ = p_t - p_i
λ_dot = (λ × (v_t - v_i)) / ||λ||²
V_c = -(λ · (v_t - v_i)) / ||λ||
a_cmd = N * V_c * λ_dot
```

**Considerations:**
- Handle small ||λ|| (division by zero)
- Handle V_c → 0 (parallel motion)
- Use `np.cross` for 3D cross product

### Linearized PN (LPN)

**Key equation:**
```
a_cmd = G * (Δp + Δv * tgo) / tgo²
```

**Considerations:**
- Safeguard `tgo` with `min_tgo`
- More robust than canonical PN

### Fast Response PN (FRPN)

**Key equation:**
```
a_cmd = G * ((1-W) * (Δp + Δv * tgo) / tgo²  +  W * Δp)
```

**Considerations:**
- Blend LPN and PP terms
- Typical: G ≈ 20, W ≈ 0.05
- Best performance for fast interception

### Model Predictive Control (MPC)

**Requires:**
- QP/NLP solver (OSQP, ACADOS, qpOASES)
- Discrete dynamics model
- Cost function formulation
- Constraint handling

**See:** `mpc/mpc_solver.py` for example implementation.

## Common Pitfalls

1. **Coordinate frame confusion**: Always use NED for computation, convert to ENU only for visualization
2. **Division by zero**: Safeguard all norm operations and tgo calculations
3. **Acceleration limits**: Always clamp before publishing
4. **Timestamp synchronization**: Use consistent clock source
5. **Offboard heartbeat**: Must publish at high rate (>2 Hz) for PX4 safety

## Performance Tuning

### Gains
- Start conservatively (low gain)
- Increase until oscillations appear, then back off 20-30%
- Tune in simulation before hardware

### Control Rate
- Minimum: 10 Hz
- Recommended: 20-50 Hz
- Higher rate = more responsive but more CPU

### Acceleration Limits
- Conservative: [2, 2, 1] m/s² (safe for testing)
- Aggressive: [5, 5, 3] m/s² (requires good tuning)
- Match to drone capabilities

## Debugging Checklist

- [ ] ROS topics publishing/subscribing correctly?
- [ ] Coordinate frames consistent (NED)?
- [ ] Acceleration limits enforced?
- [ ] Offboard mode enabled in PX4?
- [ ] Time-to-go safeguards working?
- [ ] Visualization markers appearing in RViz?
- [ ] Status messages make sense?

## Additional Resources

- See `PLANNER_README.md` for architecture overview
- See `algo_guide.md` for algorithm details
- See `../predictors/PREDICTOR_IMPLEMENTATION_GUIDE.md` for predictor integration
- PX4 docs: https://docs.px4.io/

# Predictor Implementation Guide

## Introduction

This guide provides step-by-step instructions for implementing new target prediction algorithms that integrate seamlessly with the Orion interception system.

## Prerequisites

- Understanding of state estimation and Kalman filtering
- Familiarity with ROS2 Python programming
- Knowledge of the prediction algorithm you want to implement

## Implementation Steps

### Step 1: Define Your Prediction Model

First, mathematically define your prediction model:

**State Vector**: What do you track?
```
x = [position, velocity, acceleration, ...]ᵀ
```

**State Transition**: How does state evolve?
```
x(k+1) = F·x(k) + w(k)   where w ~ N(0, Q)
```

**Measurement Model**: What do you observe?
```
z(k) = H·x(k) + v(k)     where v ~ N(0, R)
```

**Example (Constant Velocity)**:
```python
# State: [x, y, z, vx, vy, vz]
# State transition for Δt:
F = [[1, 0, 0, Δt, 0,  0 ],
     [0, 1, 0, 0,  Δt, 0 ],
     [0, 0, 1, 0,  0,  Δt],
     [0, 0, 0, 1,  0,  0 ],
     [0, 0, 0, 0,  1,  0 ],
     [0, 0, 0, 0,  0,  1 ]]

# Measurement: [x, y, z, vx, vy, vz]
H = I_6x6
```

### Step 2: Create Predictor Directory

```bash
cd src/orion_flight/predictors
mkdir my_predictor
cd my_predictor
touch __init__.py
touch my_predictor_node.py
touch my_model.py
```

### Step 3: Implement the Model Class

Create `my_model.py` with the prediction algorithm:

```python
import numpy as np
from typing import Tuple

class MyPredictorModel:
    """
    Implementation of My Prediction Algorithm.
    """
    
    def __init__(self, process_noise: dict, measurement_noise: dict):
        """
        Initialize the predictor model.
        
        Args:
            process_noise: Dictionary with noise parameters
            measurement_noise: Dictionary with measurement noise parameters
        """
        # State dimension
        self.state_dim = 6  # [x, y, z, vx, vy, vz]
        
        # Initialize state and covariance
        self.x = np.zeros(self.state_dim)  # State vector
        self.P = np.eye(self.state_dim) * 10.0  # Initial covariance (high uncertainty)
        
        # Process noise covariance Q
        self.Q = self._build_process_noise(process_noise)
        
        # Measurement noise covariance R
        self.R = self._build_measurement_noise(measurement_noise)
        
        # Measurement matrix H (observe position and velocity)
        self.H = np.eye(self.state_dim)
        
        self.initialized = False
    
    def _build_process_noise(self, params: dict) -> np.ndarray:
        """Build process noise covariance matrix Q."""
        # Example: Different noise for position and velocity
        q_pos = params.get('position', 0.1)
        q_vel = params.get('velocity', 0.5)
        
        Q = np.zeros((self.state_dim, self.state_dim))
        Q[0:3, 0:3] = np.eye(3) * q_pos  # Position noise
        Q[3:6, 3:6] = np.eye(3) * q_vel  # Velocity noise
        return Q
    
    def _build_measurement_noise(self, params: dict) -> np.ndarray:
        """Build measurement noise covariance matrix R."""
        r_pos = params.get('position', 0.05)
        r_vel = params.get('velocity', 0.1)
        
        R = np.zeros((self.state_dim, self.state_dim))
        R[0:3, 0:3] = np.eye(3) * r_pos
        R[3:6, 3:6] = np.eye(3) * r_vel
        return R
    
    def initialize(self, position: np.ndarray, velocity: np.ndarray):
        """Initialize state with first measurement."""
        self.x[0:3] = position
        self.x[3:6] = velocity
        self.initialized = True
    
    def update(self, position: np.ndarray, velocity: np.ndarray, dt: float):
        """
        Update state estimate with new measurement.
        
        Args:
            position: Measured position [x, y, z]
            velocity: Measured velocity [vx, vy, vz]
            dt: Time since last update (seconds)
        """
        if not self.initialized:
            self.initialize(position, velocity)
            return
        
        # 1. Prediction step
        F = self._build_transition_matrix(dt)
        self.x = F @ self.x  # Predicted state
        self.P = F @ self.P @ F.T + self.Q  # Predicted covariance
        
        # 2. Update step (Kalman update)
        z = np.concatenate([position, velocity])  # Measurement
        y = z - (self.H @ self.x)  # Innovation
        S = self.H @ self.P @ self.H.T + self.R  # Innovation covariance
        K = self.P @ self.H.T @ np.linalg.inv(S)  # Kalman gain
        
        self.x = self.x + K @ y  # Updated state
        self.P = (np.eye(self.state_dim) - K @ self.H) @ self.P  # Updated covariance
    
    def _build_transition_matrix(self, dt: float) -> np.ndarray:
        """
        Build state transition matrix F for time step dt.
        Override this method to implement your specific model.
        """
        # Example: Constant Velocity Model
        F = np.eye(self.state_dim)
        F[0, 3] = dt  # x = x + vx*dt
        F[1, 4] = dt  # y = y + vy*dt
        F[2, 5] = dt  # z = z + vz*dt
        return F
    
    def predict(self, horizon: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Predict state at time t + horizon.
        
        Args:
            horizon: Prediction time horizon (seconds)
        
        Returns:
            predicted_position: [x, y, z]
            predicted_velocity: [vx, vy, vz]
            covariance: State covariance matrix
        """
        # Propagate state forward
        F = self._build_transition_matrix(horizon)
        x_pred = F @ self.x
        P_pred = F @ self.P @ F.T + self._scale_process_noise(horizon)
        
        position = x_pred[0:3]
        velocity = x_pred[3:6]
        
        return position, velocity, P_pred
    
    def _scale_process_noise(self, dt: float) -> np.ndarray:
        """Scale process noise for prediction horizon."""
        # Process noise grows with time
        return self.Q * dt
    
    def get_state(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Get current state estimate."""
        return self.x[0:3], self.x[3:6], self.P
    
    def reset(self):
        """Reset predictor to initial state."""
        self.x = np.zeros(self.state_dim)
        self.P = np.eye(self.state_dim) * 10.0
        self.initialized = False
```

### Step 4: Implement the ROS2 Node

Create `my_predictor_node.py`:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleLocalPosition
from visualization_msgs.msg import MarkerArray
import numpy as np

# Import your model
from .my_model import MyPredictorModel

# Import common types (we'll create these)
from ..common.types import PredictorInput, PredictorOutput
from ..common.visualization import create_prediction_markers


class MyPredictorNode(Node):
    """ROS2 node for My Prediction Algorithm."""
    
    def __init__(self):
        super().__init__('my_predictor_node')
        
        # Declare parameters
        self.declare_parameter('target_namespace', 'px4_2')
        self.declare_parameter('update_rate', 10.0)
        self.declare_parameter('prediction_horizons', [0.5, 1.0, 2.0, 3.0])
        self.declare_parameter('process_noise.position', 0.1)
        self.declare_parameter('process_noise.velocity', 0.5)
        self.declare_parameter('measurement_noise.position', 0.05)
        self.declare_parameter('measurement_noise.velocity', 0.1)
        self.declare_parameter('publish_markers', True)
        
        # Get parameters
        self.target_ns = self.get_parameter('target_namespace').value
        self.update_rate = self.get_parameter('update_rate').value
        self.horizons = self.get_parameter('prediction_horizons').value
        
        # Initialize predictor model
        process_noise = {
            'position': self.get_parameter('process_noise.position').value,
            'velocity': self.get_parameter('process_noise.velocity').value
        }
        measurement_noise = {
            'position': self.get_parameter('measurement_noise.position').value,
            'velocity': self.get_parameter('measurement_noise.velocity').value
        }
        
        self.predictor = MyPredictorModel(process_noise, measurement_noise)
        
        # Subscribers
        target_topic = f'/{self.target_ns}/fmu/out/vehicle_local_position'
        self.target_sub = self.create_subscription(
            VehicleLocalPosition,
            target_topic,
            self.target_callback,
            10
        )
        
        # Publishers
        self.prediction_pub = self.create_publisher(
            VehicleLocalPosition,
            '/target/predicted_state',
            10
        )
        
        if self.get_parameter('publish_markers').value:
            self.marker_pub = self.create_publisher(
                MarkerArray,
                '/target/prediction_markers',
                10
            )
        
        # Timer for prediction updates
        self.timer = self.create_timer(
            1.0 / self.update_rate,
            self.prediction_callback
        )
        
        # State
        self.last_measurement_time = None
        self.latest_prediction = None
        
        self.get_logger().info(
            f'My Predictor initialized for target: {self.target_ns}'
        )
    
    def target_callback(self, msg: VehicleLocalPosition):
        """Callback for target position updates."""
        current_time = self.get_clock().now().nanoseconds / 1e9
        
        # Extract measurement
        position = np.array([msg.x, msg.y, msg.z])
        velocity = np.array([msg.vx, msg.vy, msg.vz])
        
        # Calculate dt
        if self.last_measurement_time is not None:
            dt = current_time - self.last_measurement_time
        else:
            dt = 0.1  # Initial dt
        
        # Update predictor
        if msg.xy_valid and msg.v_xy_valid:
            self.predictor.update(position, velocity, dt)
        
        self.last_measurement_time = current_time
    
    def prediction_callback(self):
        """Timer callback to publish predictions."""
        if not self.predictor.initialized:
            return
        
        current_time = self.get_clock().now().nanoseconds / 1e9
        
        # Predict at multiple horizons
        predictions = []
        for horizon in self.horizons:
            pos, vel, cov = self.predictor.predict(horizon)
            
            # Create prediction message
            pred_msg = VehicleLocalPosition()
            pred_msg.timestamp = int((current_time + horizon) * 1e6)
            pred_msg.x, pred_msg.y, pred_msg.z = pos
            pred_msg.vx, pred_msg.vy, pred_msg.vz = vel
            pred_msg.xy_valid = True
            pred_msg.z_valid = True
            pred_msg.v_xy_valid = True
            pred_msg.v_z_valid = True
            
            predictions.append((horizon, pred_msg, cov))
        
        # Publish primary prediction (first horizon)
        if predictions:
            self.prediction_pub.publish(predictions[0][1])
            self.latest_prediction = predictions[0]
        
        # Publish visualization markers
        if self.get_parameter('publish_markers').value and predictions:
            markers = create_prediction_markers(
                predictions,
                frame_id='map',
                namespace='my_predictor'
            )
            self.marker_pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = MyPredictorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Step 5: Create Common Types

We need to create the shared data structures referenced above.

### Step 6: Update Package Configuration

Add your predictor to `setup.py`:

```python
entry_points={
    'console_scripts': [
        # ... existing entries ...
        'my_predictor_node = orion_flight.predictors.my_predictor.my_predictor_node:main',
    ],
},
```

### Step 7: Create Configuration File

Create `config/my_predictor.yaml`:

```yaml
/**:
  ros__parameters:
    target_namespace: "px4_2"
    update_rate: 20.0
    prediction_horizons: [0.5, 1.0, 2.0, 3.0]
    
    process_noise:
      position: 0.1
      velocity: 0.5
    
    measurement_noise:
      position: 0.05
      velocity: 0.1
    
    publish_markers: true
    marker_scale: 1.0
    show_uncertainty: true
```

### Step 8: Test Your Predictor

```bash
# Build
cd ~/Documents/Orion/orion_arm
colcon build --packages-select orion_flight

# Source
source install/setup.bash

# Run
ros2 run orion_flight my_predictor_node --ros-args \
    --params-file src/orion_flight/config/my_predictor.yaml
```

## Best Practices

### 1. Numerical Stability
```python
# Use SVD for matrix inversions
U, S, Vt = np.linalg.svd(S_matrix)
S_inv = Vt.T @ np.diag(1.0 / S) @ U.T

# Ensure covariance symmetry
P = 0.5 * (P + P.T)

# Prevent negative eigenvalues
eigenvalues, eigenvectors = np.linalg.eigh(P)
eigenvalues = np.maximum(eigenvalues, 1e-6)
P = eigenvectors @ np.diag(eigenvalues) @ eigenvectors.T
```

### 2. Coordinate Frame Handling
```python
# Always work in NED internally
# Convert from ROS (ENU) if needed
def enu_to_ned(pos_enu):
    # ENU: [East, North, Up] -> NED: [North, East, Down]
    return np.array([pos_enu[1], pos_enu[0], -pos_enu[2]])

def ned_to_enu(pos_ned):
    # NED: [North, East, Down] -> ENU: [East, North, Up]
    return np.array([pos_ned[1], pos_ned[0], -pos_ned[2]])
```

### 3. Parameter Tuning
```python
# Log innovation statistics for tuning
innovation_norm = np.linalg.norm(y)
if innovation_norm > 3.0 * np.sqrt(np.trace(S)):
    self.get_logger().warn(
        f'Large innovation: {innovation_norm:.2f} '
        f'(expected < {3.0 * np.sqrt(np.trace(S)):.2f})'
    )
```

### 4. Performance Monitoring
```python
# Track computation time
import time
start = time.time()
self.predictor.update(position, velocity, dt)
elapsed = time.time() - start

if elapsed > 0.010:  # 10ms warning threshold
    self.get_logger().warn(f'Slow update: {elapsed*1000:.1f}ms')
```

## Common Pitfalls

❌ **Don't**: Forget to initialize before using
```python
# Wrong
pos, vel, cov = self.predictor.predict(1.0)  # May crash if not initialized
```

✅ **Do**: Check initialization
```python
# Correct
if self.predictor.initialized:
    pos, vel, cov = self.predictor.predict(1.0)
```

❌ **Don't**: Ignore dt=0 edge case
```python
# Wrong
F = np.eye(6)
F[0, 3] = dt  # Division by zero if dt=0
```

✅ **Do**: Handle edge cases
```python
# Correct
if dt < 1e-6:
    return self.x.copy()  # No prediction for dt≈0
```

❌ **Don't**: Mix coordinate frames
```python
# Wrong - mixing NED and ENU
pred_ned = self.predictor.predict(horizon)
marker.position.x = pred_ned[0]  # RViz expects ENU!
```

✅ **Do**: Convert explicitly
```python
# Correct
pred_ned = self.predictor.predict(horizon)
pred_enu = ned_to_enu(pred_ned)
marker.position.x = pred_enu[0]
```

## Advanced Topics

### Multi-Rate Processing
If measurements arrive at different rates (e.g., position at 50Hz, velocity at 10Hz):

```python
def update_position(self, position: np.ndarray, dt: float):
    """Update with position-only measurement."""
    H_pos = np.zeros((3, self.state_dim))
    H_pos[0:3, 0:3] = np.eye(3)
    # Use partial measurement update
    
def update_velocity(self, velocity: np.ndarray, dt: float):
    """Update with velocity-only measurement."""
    H_vel = np.zeros((3, self.state_dim))
    H_vel[0:3, 3:6] = np.eye(3)
    # Use partial measurement update
```

### Adaptive Noise Estimation
Automatically tune Q and R based on innovation statistics:

```python
# If innovations consistently large -> increase R
# If state jumps around -> increase Q
self.R *= innovation_factor
self.Q *= consistency_factor
```

### Multiple Prediction Modes
Support different behavior modes (cruise, maneuver, etc.):

```python
class MyPredictorModel:
    def set_mode(self, mode: str):
        if mode == 'cruise':
            self.Q = self.Q_cruise
        elif mode == 'maneuver':
            self.Q = self.Q_maneuver
```

## Testing Checklist

- [ ] Unit tests for prediction algorithm
- [ ] Integration test with simulated target
- [ ] Test with real multi-drone simulation
- [ ] Verify RViz visualization
- [ ] Measure computation time (< 10ms target)
- [ ] Validate prediction accuracy against ground truth
- [ ] Test edge cases (initialization, dt=0, invalid measurements)
- [ ] Check parameter sensitivity

## References

- Kalman Filtering: Bar-Shalom, Y., et al. "Estimation with Applications to Tracking and Navigation"
- IMM Filter: Blom, H. A., & Bar-Shalom, Y. "The interacting multiple model algorithm for systems with Markovian switching coefficients"
- ROS2 Python: https://docs.ros.org/en/humble/Tutorials.html

## Support

For questions or issues:
1. Check [`PREDICTOR_README.md`](PREDICTOR_README.md) for interface details
2. Review existing implementations in `cv/`, `ca/`, `imm/`
3. Test with simulation: [`SIMULATION_SETUP.md`](../../../SIMULATION_SETUP.md)

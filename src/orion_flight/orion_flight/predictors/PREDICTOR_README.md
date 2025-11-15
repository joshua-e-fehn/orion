# Target Prediction Framework

## Overview

This directory contains the target prediction framework for the Orion interception system. All predictors follow a common interface to enable seamless switching between different prediction algorithms and support for the Interacting Multiple Model (IMM) framework.

## Directory Structure

```
predictors/
├── PREDICTOR_README.md                  # This file
├── PREDICTOR_IMPLEMENTATION_GUIDE.md    # Guide for implementing new predictors
├── common/
│   ├── __init__.py                      # Package initialization
│   ├── predictor_base.py                # Abstract base class for all predictors
│   ├── types.py                         # Common data types and structures
│   └── visualization.py                 # Shared visualization utilities
├── cv/
│   ├── __init__.py
│   ├── cv_predictor_node.py            # Constant Velocity predictor ROS2 node
│   └── cv_model.py                      # CV prediction algorithm
├── ca/
│   ├── __init__.py
│   ├── ca_predictor_node.py            # Constant Acceleration predictor ROS2 node
│   └── ca_model.py                      # CA prediction algorithm
└── imm/
    ├── __init__.py
    ├── imm_predictor_node.py           # IMM predictor ROS2 node
    └── imm_filter.py                    # IMM algorithm implementation
```

## Common Interface

All predictors implement the `PredictorBase` abstract class which defines:

### Input Interface

**ROS2 Subscription:**
- **Topic**: `/{target_namespace}/fmu/out/vehicle_local_position`
- **Type**: `px4_msgs.msg.VehicleLocalPosition`
- **Rate**: ~50 Hz (from PX4)

**Data Structure (`PredictorInput`):**
```python
@dataclass
class PredictorInput:
    timestamp: float              # ROS time in seconds
    position: np.ndarray          # [x, y, z] in NED frame (meters)
    velocity: np.ndarray          # [vx, vy, vz] in NED frame (m/s)
    acceleration: np.ndarray      # [ax, ay, az] in NED frame (m/s²)
    position_valid: bool          # Position validity flag
    velocity_valid: bool          # Velocity validity flag
```

### Output Interface

**ROS2 Publications:**

1. **Predicted State**
   - **Topic**: `/target/predicted_state`
   - **Type**: `px4_msgs.msg.VehicleLocalPosition`
   - **Rate**: 10-20 Hz
   - **Content**: Predicted target state at future time horizon(s)

2. **Prediction Markers**
   - **Topic**: `/target/prediction_markers`
   - **Type**: `visualization_msgs.msg.MarkerArray`
   - **Rate**: 5-10 Hz
   - **Content**: Visualization of predicted trajectory and uncertainty

3. **Predictor Status**
   - **Topic**: `/target/predictor_status`
   - **Type**: `std_msgs.msg.String` (JSON format)
   - **Rate**: 1 Hz
   - **Content**: Predictor health, model probabilities (IMM), covariance norms

**Data Structure (`PredictorOutput`):**
```python
@dataclass
class PredictorOutput:
    timestamp: float                    # Prediction time (seconds)
    prediction_horizon: float           # How far ahead (seconds)
    
    # Predicted state
    predicted_position: np.ndarray      # [x, y, z] in NED (meters)
    predicted_velocity: np.ndarray      # [vx, vy, vz] in NED (m/s)
    predicted_acceleration: np.ndarray  # [ax, ay, az] in NED (m/s²)
    
    # Uncertainty (covariance matrices)
    position_covariance: np.ndarray     # 3x3 position covariance
    velocity_covariance: np.ndarray     # 3x3 velocity covariance
    
    # Additional info
    model_probability: float            # For IMM: probability of this model
    innovation: np.ndarray              # Measurement residual
    is_valid: bool                      # Prediction validity flag
```

### Required Methods

Every predictor must implement:

```python
class PredictorBase(ABC):
    @abstractmethod
    def update(self, measurement: PredictorInput) -> None:
        """Update internal state with new measurement"""
        pass
    
    @abstractmethod
    def predict(self, horizon: float) -> PredictorOutput:
        """Predict target state at time t + horizon"""
        pass
    
    @abstractmethod
    def reset(self) -> None:
        """Reset predictor to initial state"""
        pass
    
    @abstractmethod
    def get_state_estimate(self) -> PredictorOutput:
        """Get current state estimate (horizon = 0)"""
        pass
```

## Coordinate Frames

### NED (North-East-Down) - PX4 Native
- **X**: North (forward)
- **Y**: East (right)
- **Z**: Down (positive downward)

This is the primary working frame for all predictors.

### Visualization (ENU for RViz)
- **X**: East
- **Y**: North
- **Z**: Up

Visualization markers are automatically converted to ENU in `visualization.py`.

## Prediction Horizons

Predictors support multiple prediction horizons configured via ROS parameters:

```python
# Example: Predict at 0.5s, 1.0s, 2.0s, 3.0s into the future
prediction_horizons: [0.5, 1.0, 2.0, 3.0]
```

Each horizon produces a separate `PredictorOutput` published in a marker array.

## Uncertainty Representation

### Covariance Propagation
All predictors must propagate uncertainty through predictions using:
- **Process noise**: `Q` matrix (tunable per predictor)
- **Measurement noise**: `R` matrix (from sensor characteristics)
- **State covariance**: `P` matrix (updated with each measurement)

### Visualization
Uncertainty is visualized as:
- **Ellipsoids**: 3D confidence ellipsoids at σ, 2σ, 3σ levels
- **Color coding**:
  - Green: High confidence (small covariance)
  - Yellow: Medium confidence
  - Red: Low confidence (large covariance)

## ROS2 Parameters

Standard parameters for all predictors:

```yaml
predictor:
  target_namespace: "px4_2"              # Target drone namespace
  update_rate: 10.0                      # Prediction update rate (Hz)
  prediction_horizons: [0.5, 1.0, 2.0]   # Future times to predict (seconds)
  
  # Noise parameters (tuned per predictor)
  process_noise_position: 0.1            # Process noise for position (m²/s³)
  process_noise_velocity: 0.5            # Process noise for velocity (m²/s⁵)
  measurement_noise_position: 0.05       # Measurement noise for position (m²)
  measurement_noise_velocity: 0.1        # Measurement noise for velocity (m²/s²)
  
  # Visualization
  publish_markers: true                  # Enable visualization markers
  marker_scale: 1.0                      # Marker size scaling
  show_uncertainty: true                 # Show uncertainty ellipsoids
```

## Available Predictors

### 1. Constant Velocity (CV)
**Model**: Target maintains current velocity
```
x(t+Δt) = x(t) + v(t)·Δt
v(t+Δt) = v(t)
```
**Use Case**: Targets moving in straight lines, cruise phase
**Node**: `cv_predictor_node.py`

### 2. Constant Acceleration (CA)
**Model**: Target maintains current acceleration
```
x(t+Δt) = x(t) + v(t)·Δt + 0.5·a(t)·Δt²
v(t+Δt) = v(t) + a(t)·Δt
a(t+Δt) = a(t)
```
**Use Case**: Targets performing maneuvers, turns, climbs
**Node**: `ca_predictor_node.py`

### 3. Interacting Multiple Model (IMM)
**Model**: Weighted combination of CV and CA
```
x̂_IMM = Σ μᵢ·x̂ᵢ  where μᵢ is model probability
```
**Use Case**: Adaptive prediction for mixed behavior
**Node**: `imm_predictor_node.py`

## Usage Examples

### Launch CV Predictor
```bash
ros2 run orion_flight cv_predictor_node --ros-args \
    -p target_namespace:=px4_2 \
    -p update_rate:=20.0 \
    -p prediction_horizons:=[1.0,2.0,3.0]
```

### Launch via Launch File
```bash
ros2 launch orion_flight predictors.launch.py \
    predictor_type:=cv \
    target_namespace:=px4_2 \
    config:=config/cv_predictor.yaml
```

### Subscribe to Predictions (Python)
```python
from px4_msgs.msg import VehicleLocalPosition

def prediction_callback(msg):
    print(f"Predicted position: {msg.x}, {msg.y}, {msg.z}")
    print(f"Predicted velocity: {msg.vx}, {msg.vy}, {msg.vz}")

node.create_subscription(
    VehicleLocalPosition,
    '/target/predicted_state',
    prediction_callback,
    10
)
```

### Subscribe to Predictions (C++)
```cpp
auto subscription = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
    "/target/predicted_state", 10,
    [this](const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), 
            "Predicted position: [%.2f, %.2f, %.2f]",
            msg->x, msg->y, msg->z);
    });
```

## Performance Metrics

Predictors log performance metrics:
- **Prediction error**: RMS error between prediction and actual
- **Update latency**: Time from measurement to prediction (should be <10ms)
- **Computational load**: CPU usage per update cycle

Access via:
```bash
ros2 topic echo /target/predictor_status
```

## Integration with Guidance

Guidance nodes consume predictions:
```python
# In guidance node
predicted_state = self.get_latest_prediction()
guidance_command = self.compute_intercept(
    ego_state=self.ego_state,
    target_predicted=predicted_state
)
```

See [`algo_guide.md`](../../../algo_guide.md) for guidance law integration.

## Troubleshooting

**No predictions published**
- Check target namespace is correct
- Verify `/px4_2/fmu/out/vehicle_local_position` is publishing
- Check predictor node is running: `ros2 node list`

**High prediction error**
- Tune process/measurement noise parameters
- Increase update rate
- Switch to CA or IMM for maneuvering targets

**Visualization not showing**
- Ensure RViz is configured for topic `/target/prediction_markers`
- Check fixed frame is set to `map`
- Verify marker publishing is enabled

## Next Steps

1. See [`PREDICTOR_IMPLEMENTATION_GUIDE.md`](PREDICTOR_IMPLEMENTATION_GUIDE.md) for implementing new predictors
2. Review individual predictor implementations in `cv/`, `ca/`, `imm/`
3. Test predictors with simulation: [`SIMULATION_SETUP.md`](../../../SIMULATION_SETUP.md)

# Constant Acceleration (CA) Predictor

## Overview

The Constant Acceleration (CA) predictor implements a 9-state Kalman filter that estimates and predicts target drone motion assuming constant acceleration. This predictor is more sophisticated than the CV predictor and can better track maneuvering targets.

## State Vector

The CA model maintains a 9-dimensional state:
```
x = [x, y, z, vx, vy, vz, ax, ay, az]ᵀ
```

Where:
- **Position**: (x, y, z) in NED frame [meters]
- **Velocity**: (vx, vy, vz) in NED frame [m/s]
- **Acceleration**: (ax, ay, az) in NED frame [m/s²]

## Motion Model

The CA model assumes constant acceleration:

```
x(t+Δt) = x(t) + v(t)·Δt + 0.5·a(t)·Δt²
v(t+Δt) = v(t) + a(t)·Δt
a(t+Δt) = a(t)
```

## Acceleration Estimation

Since PX4's `VehicleLocalPosition` message typically does not include acceleration measurements, the CA predictor estimates acceleration using finite differences:

```
a(t) ≈ [v(t) - v(t-1)] / Δt
```

The estimated acceleration is:
- Clamped to ±10 m/s² to reject outliers
- Used to update the Kalman filter state
- Propagated through predictions

## Usage

### Launch CA Predictor

```bash
# Source the workspace
source ~/Documents/Orion/orion_arm/install/setup.bash

# Run as a module
python3 -m orion_flight.predictors.ca.ca_predictor_node

# Or with custom config
python3 -m orion_flight.predictors.ca.ca_predictor_node --ros-args \
    --params-file src/orion_flight/config/ca_predictor.yaml
```

### Configuration

Edit `config/ca_predictor.yaml`:

```yaml
/**:
  ros__parameters:
    target_namespace: "px4_2"          # Target drone namespace
    update_rate: 20.0                  # Prediction update rate (Hz)
    prediction_horizons: [0.5, 1.0, 2.0, 3.0, 5.0]  # Future times (seconds)
    
    process_noise:
      position: 0.1                    # Position process noise (m²/s³)
      velocity: 0.5                    # Velocity process noise (m²/s⁵)
      acceleration: 1.0                # Acceleration process noise (m²/s⁷)
    
    measurement_noise:
      position: 0.05                   # Position measurement noise (m²)
      velocity: 0.1                    # Velocity measurement noise (m²/s²)
      acceleration: 0.5                # Acceleration noise (if measured)
    
    use_acceleration_from_msg: false   # Use msg.ax/ay/az if available
    publish_markers: true
    marker_scale: 1.0
```

### Subscriptions

- **Input**: `/{target_namespace}/fmu/out/vehicle_local_position`
  - Type: `px4_msgs/VehicleLocalPosition`
  - Rate: ~50 Hz (from PX4)
  - Fields used: x, y, z, vx, vy, vz

### Publications

1. **Predicted State**: `/target/predicted_state`
   - Type: `px4_msgs/VehicleLocalPosition`
   - Rate: 10-20 Hz
   - Includes predicted position, velocity, and acceleration

2. **Visualization**: `/target/prediction_markers`
   - Type: `visualization_msgs/MarkerArray`
   - Rate: 5-10 Hz
   - Shows trajectory and uncertainty ellipsoids

## When to Use CA vs CV

**Use CA Predictor when:**
- Target performs maneuvers (turns, climbs, acceleration changes)
- Prediction horizon is short-to-medium (< 5 seconds)
- You need to capture curvature in target trajectory
- Target shows clear acceleration patterns

**Use CV Predictor when:**
- Target flies in straight lines at constant velocity
- Computational efficiency is critical
- Prediction horizon is very short (< 1 second)
- Target motion is nearly linear

**Use IMM (when implemented):**
- Target switches between cruise and maneuver modes
- Uncertain about target behavior
- Need adaptive prediction

## Performance Characteristics

| Metric | Value |
|--------|-------|
| State dimension | 9 |
| Computation time | ~2-5 ms per update |
| Memory usage | ~1 KB per predictor instance |
| Prediction accuracy | Better than CV for maneuvering targets |

## Testing

Run unit tests (when implemented):
```bash
pytest src/orion_flight/orion_flight/predictors/ca/tests/
```

## Implementation Details

### Process Noise Matrix

The CA predictor uses a continuous white noise jerk model. The process noise covariance Q grows with time:

```
Q = G·Q_cont·G^T·dt
```

Where G is the noise gain matrix that couples position, velocity, and acceleration.

### Numerical Stability

The implementation includes:
- **Covariance symmetry enforcement**: `P = 0.5(P + P^T)`
- **Positive definite guarantee**: Eigenvalue clamping
- **Joseph form update**: `P = (I - KH)P(I - KH)^T + KRK^T`
- **Outlier rejection**: Acceleration clamped to physical limits

## Troubleshooting

**High acceleration estimates**
- Increase `process_noise.acceleration` to trust estimates less
- Check for noisy velocity measurements
- Verify target is actually maneuvering

**Poor prediction accuracy**
- Tune process noise parameters
- Increase update rate
- Check measurement noise settings match sensor characteristics

**Predictions diverge**
- Reduce prediction horizons
- Increase measurement update rate
- Check for missed/delayed measurements

## References

- Kalman Filtering: Bar-Shalom, Y., et al. "Estimation with Applications to Tracking and Navigation"
- Process Noise Models: Li, X. R., & Jilkov, V. P. "Survey of maneuvering target tracking"

## See Also

- [CV Predictor](../cv/README.md)
- [IMM Predictor](../imm/README.md) (when implemented)
- [Predictor Framework](../PREDICTOR_README.md)
- [Implementation Guide](../PREDICTOR_IMPLEMENTATION_GUIDE.md)

# IMM Predictor Implementation

## Overview

This implementation provides a complete Interacting Multiple Model (IMM) predictor for target motion prediction in the Orion flight package. The IMM predictor adaptively combines Constant Velocity (CV) and Constant Acceleration (CA) models to handle targets with varying maneuver patterns.

## Files Implemented

### Constant Acceleration (CA) Predictor
- **`ca/ca_model.py`** (324 lines): Core CA prediction model
  - 9-state Kalman filter: [x, y, z, vx, vy, vz, ax, ay, az]
  - Constant acceleration motion model with continuous white noise jerk
  - Automatic acceleration estimation via finite differences when not measured
  - Full covariance propagation with Joseph form updates

- **`ca/ca_predictor_node.py`** (291 lines): ROS2 node
  - Subscribes to `/{target_namespace}/fmu/out/vehicle_local_position`
  - Publishes predictions on `/target/predicted_state`
  - Visualization markers on `/target/prediction_markers`
  - Configurable update rate and prediction horizons

### Interacting Multiple Model (IMM) Filter
- **`imm/imm_filter.py`** (379 lines): Core IMM algorithm
  - Combines CV and CA models with adaptive weighting
  - Model mixing with configurable transition probability matrix
  - Gaussian likelihood computation for innovation-based mode selection
  - Mode probability updates using Bayesian filtering
  - State and covariance fusion with spread term

- **`imm/imm_predictor_node.py`** (413 lines): ROS2 node
  - Integrates CV and CA models in IMM framework
  - Publishes fused predictions with model probabilities
  - Status topic (`/target/predictor_status`) with mode probabilities and diagnostics
  - Visualization of combined predicted trajectory

### Configuration
- **`config/ca_predictor.yaml`**: Configuration for standalone CA predictor
- **`config/imm_predictor.yaml`**: Configuration for IMM with tunable parameters

## Usage

### Running CA Predictor
```bash
ros2 run orion_flight ca_predictor_node --ros-args \
    --params-file src/orion_flight/config/ca_predictor.yaml
```

### Running IMM Predictor
```bash
ros2 run orion_flight imm_predictor_node --ros-args \
    --params-file src/orion_flight/config/imm_predictor.yaml
```

### Configurable Parameters

#### CA Predictor
- `target_namespace`: Target drone namespace (default: "px4_2")
- `update_rate`: Prediction update rate in Hz (default: 20.0)
- `prediction_horizons`: List of future time horizons in seconds
- `process_noise.position`: Position process noise (default: 0.1)
- `process_noise.velocity`: Velocity process noise (default: 0.5)
- `process_noise.acceleration`: Acceleration process noise (default: 1.0)
- `measurement_noise.*`: Measurement noise parameters

#### IMM Predictor
All CA parameters plus:
- `cv_process_noise.*`: CV model-specific process noise
- `ca_process_noise.*`: CA model-specific process noise
- `imm.transition_prob_stay`: Probability of staying in current mode (default: 0.95)
- `imm.initial_prob_cv`: Initial CV model probability (default: 0.5)
- `imm.initial_prob_ca`: Initial CA model probability (default: 0.5)

## Algorithm Details

### CA Model
The CA model assumes constant acceleration motion:
```
x(t+Δt) = x(t) + v(t)·Δt + 0.5·a(t)·Δt²
v(t+Δt) = v(t) + a(t)·Δt
a(t+Δt) = a(t)
```

Process noise models continuous white noise jerk (derivative of acceleration).

### IMM Algorithm
The IMM filter follows the standard algorithm:

1. **Mixing**: Compute mixing probabilities μ_{i|j} for interaction
2. **Filtering**: Update each model (CV and CA) with new measurements
3. **Likelihood**: Compute Gaussian likelihood for each model's innovation
4. **Update**: Update mode probabilities using Bayes' rule
5. **Fusion**: Combine predictions weighted by mode probabilities with spread term

The fused prediction is:
```
x̂ = Σ_j μ_j · x̂_j
P = Σ_j μ_j · (P_j + (x̂_j - x̂)(x̂_j - x̂)ᵀ)
```

## Interface Compliance

All predictors follow the common `PredictorBase` interface:
- `update(measurement)`: Update with new target measurement
- `predict(horizon)`: Predict state at future time
- `predict_multiple(horizons)`: Predict at multiple horizons
- `reset()`: Reset to initial state
- `get_state_estimate()`: Get current filtered state

## Coordinate Frames

- **Internal computations**: NED (North-East-Down) frame (PX4 native)
- **Visualization**: Automatically converted to ENU (East-North-Up) for RViz

## Performance Considerations

- **Update latency**: Target < 10ms per update cycle
- **Numerical stability**: 
  - Joseph form covariance updates
  - Covariance symmetry enforcement
  - Positive eigenvalue enforcement
- **Edge case handling**:
  - Negative/zero dt detection
  - Singular matrix handling
  - Acceleration magnitude limiting

## Testing

The implementation includes:
- Syntax validation (all files compile)
- YAML configuration validation
- Security scanning (CodeQL) - 0 vulnerabilities found
- Mathematical validation tests (see test_predictors.py)

## Integration with Guidance

Guidance nodes consume predictions via:
```python
predicted_state = latest_prediction  # From /target/predicted_state topic
guidance_command = compute_intercept(ego_state, predicted_state)
```

## References

- **Algo Guide**: See `algo_guide.md` Section 3.3 for IMM algorithm details
- **Implementation Guide**: See `PREDICTOR_IMPLEMENTATION_GUIDE.md`
- **Common Interface**: See `predictors/common/predictor_base.py`

## Future Enhancements

Potential improvements:
- Support for additional motion models (e.g., Coordinated Turn)
- Adaptive noise estimation based on innovation statistics
- Multi-rate measurement processing
- Integration with uncertainty-aware guidance laws

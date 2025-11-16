# Linearized PN (LPN) Planner Implementation

## Overview

This document describes the implementation of the Linearized Proportional Navigation (LPN) planner for the Orion autonomous drone interception system.

## Implementation Summary

The LPN planner has been fully implemented following the architecture and algorithm specifications from the `algo_guide.md` and `PLANNER_README.md`.

### Files Created

1. **`lpn_algorithm.py`** (157 lines)
   - Core LPN guidance law implementation
   - Robust time-to-go computation
   - Acceleration clamping and integration
   - Guidance metrics computation

2. **`lpn_planner_node.py`** (339 lines)
   - ROS2 node implementing PlannerBase interface
   - State management and control loop
   - ROS2 publishers and subscribers
   - Visualization and status reporting

3. **`lpn_planner.launch.py`** (93 lines)
   - Launch file with configurable parameters
   - Default values from algo_guide recommendations

4. **Updated `__init__.py`** and **`setup.py`**
   - Package exports and entry point registration

## Algorithm Details

### LPN Guidance Law

The LPN algorithm implements the following formula:

```
a_cmd = G_lpn * ((Δp + Δv * tgo) / tgo²)
```

Where:
- **Δp** = p_target - p_interceptor (position error)
- **Δv** = v_target - v_interceptor (velocity error)
- **tgo** = time-to-go estimate (safeguarded)
- **G_lpn** = LPN gain parameter

### Time-to-Go Computation

Time-to-go is computed with robust safeguards:

```python
if ||Δv|| < v_eps:
    tgo = max(min_tgo, ||Δp|| / (||v_i|| + v_eps))
else:
    tgo = max(min_tgo, ||Δp|| / ||Δv||)
```

This prevents division by zero when relative velocity is small.

### Key Features

1. **Robustness**: Handles edge cases like zero relative velocity
2. **Safety**: Enforces per-axis acceleration limits
3. **Integration**: Computes velocity and position setpoints for PX4
4. **Metrics**: Provides time-to-go, closing velocity, miss distance

## Parameters

### Default Values (from algo_guide.md)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `G_lpn` | 20.0 | LPN gain (aggressiveness) |
| `min_tgo` | 0.05 s | Minimum time-to-go safeguard |
| `amax` | [4.0, 4.0, 2.0] | Max acceleration [ax, ay, az] in m/s² |
| `control_rate` | 20.0 Hz | Control loop frequency |
| `v_eps` | 0.001 m/s | Velocity epsilon for safeguards |

### Tuning Recommendations

- **Higher G_lpn**: More aggressive interception (faster convergence but higher control effort)
- **Lower G_lpn**: Gentler approach (smoother trajectory but slower convergence)
- **min_tgo**: Should be set based on system dynamics and control latency

## ROS2 Interface

### Subscriptions

1. **Interceptor State**
   - Topic: `/{interceptor_ns}/fmu/out/vehicle_local_position`
   - Type: `VehicleLocalPosition`
   - Rate: ~50 Hz

2. **Target State (Current)**
   - Topic: `/{target_ns}/fmu/out/vehicle_local_position`
   - Type: `VehicleLocalPosition`
   - Rate: ~50 Hz

3. **Target State (Predicted)** - Optional
   - Topic: `/target/predicted_state`
   - Type: `VehicleLocalPosition`
   - Rate: 10-20 Hz

### Publications

1. **Trajectory Setpoint**
   - Topic: `/{interceptor_ns}/fmu/in/trajectory_setpoint`
   - Type: `TrajectorySetpoint`
   - Rate: 20 Hz (configurable)

2. **Offboard Control Mode**
   - Topic: `/{interceptor_ns}/fmu/in/offboard_control_mode`
   - Type: `OffboardControlMode`
   - Rate: 20 Hz (heartbeat)

3. **Visualization Markers**
   - Topic: `/planner/guidance_markers`
   - Type: `MarkerArray`
   - Rate: ~5 Hz

4. **Status**
   - Topic: `/planner/status`
   - Type: `String` (JSON)
   - Rate: ~1 Hz

## Usage

### Basic Launch

```bash
ros2 launch orion_flight lpn_planner.launch.py
```

### Custom Parameters

```bash
ros2 launch orion_flight lpn_planner.launch.py \
    interceptor_namespace:=px4_1 \
    target_namespace:=px4_2 \
    G_lpn:=25.0 \
    min_tgo:=0.1 \
    use_predictor:=true
```

### With Predictor

```bash
# Terminal 1: Start predictor
ros2 launch orion_flight predictors.launch.py predictor_type:=cv

# Terminal 2: Start LPN planner
ros2 launch orion_flight lpn_planner.launch.py use_predictor:=true
```

## Testing Results

### Algorithm Verification

All tests passed successfully:

1. ✅ **Basic Intercept**: Correct acceleration computation
2. ✅ **Small Relative Velocity**: Edge case handled properly
3. ✅ **Acceleration Limits**: Clamping enforced correctly
4. ✅ **Formula Verification**: Manual calculation matches implementation
5. ✅ **LPN vs PP Comparison**: LPN accounts for target velocity
6. ✅ **Edge Cases**: Zero velocity, close targets, saturation handled

### Key Differences from Pure Pursuit

| Aspect | Pure Pursuit | Linearized PN |
|--------|--------------|---------------|
| Formula | `a = G * Δp` | `a = G * ((Δp + Δv*tgo) / tgo²)` |
| Target Velocity | Ignored | Accounted for |
| Intercept Point | Current position | Predicted position |
| Closing Geometry | Not optimized | Optimized for intercept |
| Performance | Fair | Very Good |

## Integration with Framework

### Predictor Integration

The LPN planner seamlessly integrates with predictors:

```
Predictor (CV/CA/IMM) → /target/predicted_state → LPN Planner
```

When `use_predictor=true`, LPN uses the predicted future state instead of current state.

### Common Interface

LPN implements the `PlannerBase` abstract interface:

- ✅ `compute_guidance()` - Core guidance computation
- ✅ `reset()` - Reset planner state
- ✅ `is_converged()` - Check intercept completion
- ✅ `get_planner_type()` - Returns 'lpn'

## Performance Characteristics

### Advantages

1. **Predictive**: Leads moving targets to intercept point
2. **Robust**: Handles edge cases (small closing velocity, near-zero relative velocity)
3. **Efficient**: Computationally lightweight (closed-form solution)
4. **Proven**: Based on established guidance theory

### Limitations

1. **No Constraints**: Doesn't handle velocity/position constraints (use MPC for that)
2. **No Obstacles**: Doesn't avoid obstacles (add to future enhancement)
3. **Assumes Point Mass**: Doesn't model rotational dynamics

## Comparison with Other Planners

| Planner | Complexity | Performance | Use Case |
|---------|-----------|-------------|----------|
| PP | ⭐ Simple | ⭐⭐ Fair | Baseline testing |
| LPN | ⭐⭐ Medium | ⭐⭐⭐⭐ Very Good | General interception |
| FRPN | ⭐⭐⭐ Medium | ⭐⭐⭐⭐⭐ Excellent | **Recommended** |
| MPC | ⭐⭐⭐⭐ High | ⭐⭐⭐⭐⭐ Excellent | Constraint-aware |

## Future Enhancements

Potential improvements for future work:

1. **Uncertainty-Aware**: Scale gain based on prediction covariance
2. **3D Optimization**: Altitude-specific gain scheduling
3. **Adaptive Gain**: Auto-tune G_lpn based on geometry
4. **Performance Metrics**: Add interception success rate tracking

## References

- Pliska et al., "Towards Safe Mid-Air Drone Interception", IEEE RA-L 2024
- `algo_guide.md` - Algorithm theory and pseudocode
- `PLANNER_README.md` - Framework architecture
- `PLANNER_IMPLEMENTATION_GUIDE.md` - Implementation guide

## Conclusion

The LPN planner has been successfully implemented and tested. It provides a robust, efficient guidance solution for drone interception that significantly outperforms Pure Pursuit while remaining computationally lightweight.

**Status**: ✅ Ready for integration and testing

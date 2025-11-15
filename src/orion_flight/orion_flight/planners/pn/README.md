# Proportional Navigation (PN) Planner

## Overview

The Proportional Navigation (PN) planner implements a canonical guidance law used in guided missiles and interception systems. It commands acceleration perpendicular to the line-of-sight (LOS) proportional to the LOS rate times closing speed.

**Guidance Law:**
```
a_cmd = N * V_c * λ_dot
```

Where:
- `N` is the navigation constant (gain)
- `V_c` is the closing velocity
- `λ_dot` is the LOS rate vector

## Key Features

- **Robust LOS Rate Computation**: Uses cross product for 3D LOS rate calculation
- **Closing Velocity Safeguards**: Handles edge cases when relative velocity is small
- **Per-axis Acceleration Limits**: Respects drone platform constraints
- **Integration with Predictors**: Can use predicted target states
- **Real-time Visualization**: RViz markers for debugging

## Quick Start

### 1. Build the Package

```bash
cd ~/Documents/Orion/orion_arm
colcon build --packages-select orion_flight
source install/setup.bash
```

### 2. Launch the PN Planner

```bash
# Using launch file with default parameters
ros2 launch orion_flight pn_planner.launch.py

# Or with custom parameters
ros2 launch orion_flight pn_planner.launch.py \
    N:=3.5 \
    amax_x:=5.0 \
    amax_y:=5.0 \
    amax_z:=3.0

# Or run directly
ros2 run orion_flight pn_planner --ros-args \
    -p N:=3.0 \
    -p interceptor_namespace:=px4_1 \
    -p target_namespace:=px4_2
```

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `N` | float | 3.0 | Navigation constant (PN gain) |
| `amax` | list[float] | [4.0, 4.0, 2.0] | Max acceleration [x, y, z] (m/s²) |
| `control_rate` | float | 20.0 | Control loop frequency (Hz) |
| `interceptor_namespace` | str | 'px4_1' | Interceptor drone namespace |
| `target_namespace` | str | 'px4_2' | Target drone namespace |
| `use_predictor` | bool | true | Use predicted target state |
| `convergence_distance` | float | 1.0 | Intercept success threshold (m) |
| `min_tgo` | float | 0.05 | Minimum time-to-go safeguard (s) |
| `v_eps` | float | 0.1 | Velocity epsilon for safeguarding (m/s) |

## Algorithm Details

### Line-of-Sight (LOS) Rate Computation

```python
λ = p_t - p_i                           # LOS vector
Δv = v_t - v_i                          # Relative velocity
λ_dot = (λ × Δv) / ||λ||²              # LOS rate (3D)
```

### Closing Velocity

```python
V_c = -(λ · Δv) / ||λ||                # Positive when closing
```

### Proportional Navigation Command

```python
a_cmd = N * V_c * λ_dot                # Lateral acceleration
a_cmd = clamp(a_cmd, amax)             # Apply limits
```

## Parameter Tuning

### Navigation Constant (N)

- **N = 2-3**: Conservative, smooth approach
- **N = 3-4**: Standard for most scenarios (recommended)
- **N = 4-5**: Aggressive, faster intercept
- **N > 5**: May cause oscillations

**Tuning Tips:**
- Start with N=3.0
- Increase for faster intercept
- Decrease if seeing oscillations
- PN requires N > 1 for convergence

### Acceleration Limits

Adjust based on drone capabilities:

```bash
# For agile drone
ros2 launch orion_flight pn_planner.launch.py \
    amax_x:=6.0 amax_y:=6.0 amax_z:=3.0

# For conservative/heavy drone
ros2 launch orion_flight pn_planner.launch.py \
    amax_x:=3.0 amax_y:=3.0 amax_z:=1.5
```

## Integration with Predictor

The PN planner can use predicted target states from a predictor node:

```bash
# Terminal 1: Start CV predictor
ros2 run orion_flight cv_predictor_node --ros-args \
    -p target_namespace:=px4_2

# Terminal 2: Start PN planner with predictor
ros2 launch orion_flight pn_planner.launch.py use_predictor:=true
```

## Topics

### Subscriptions

- `/{interceptor_ns}/fmu/out/vehicle_local_position` - Interceptor state
- `/{target_ns}/fmu/out/vehicle_local_position` - Target current state
- `/target/predicted_state` - Target predicted state (optional)

### Publications

- `/{interceptor_ns}/fmu/in/trajectory_setpoint` - Control commands
- `/{interceptor_ns}/fmu/in/offboard_control_mode` - Offboard heartbeat
- `/planner/guidance_markers` - Visualization markers
- `/planner/status` - Status JSON

## Monitoring

### View Status

```bash
ros2 topic echo /planner/status
```

**Example output:**
```json
{
  "planner": "proportional_navigation",
  "active": true,
  "tgo": 2.3,
  "closing_velocity": 5.4,
  "miss_distance": 12.5,
  "converged": false,
  "commands_issued": 245
}
```

### Visualize in RViz

Add these displays:
1. **MarkerArray** - Topic: `/planner/guidance_markers`
   - Green line: LOS vector
   - Red arrow: Commanded acceleration
   - Blue/red spheres: Interceptor/target positions

## Comparison with Pure Pursuit

| Feature | Pure Pursuit | Proportional Navigation |
|---------|--------------|------------------------|
| **Complexity** | Simple | Medium |
| **Performance** | Fair | Good |
| **Target Motion** | Better for stationary | Better for moving |
| **Optimality** | Not optimal | Near-optimal for constant velocity targets |
| **Oscillations** | Can overshoot | More stable with proper N |
| **Implementation** | Position error only | Uses velocity + geometry |

## Troubleshooting

### Planner oscillates
- Reduce `N` (try N=2.0)
- Reduce `amax`
- Check if closing velocity is too small

### Slow convergence
- Increase `N` (try N=4.0)
- Increase `amax` if drone can handle it
- Verify target velocity is being measured

### No acceleration command
- Check if LOS vector is degenerate (already at target)
- Verify relative velocity is non-zero
- Check `v_eps` safeguard threshold

### Commands too aggressive
- Reduce `amax`
- Reduce `N`
- Consider using predictor for smoother commands

## References

- Algorithm details: `algo_guide.md` Section 4.2
- Framework overview: `PLANNER_README.md`
- Implementation guide: `PLANNER_IMPLEMENTATION_GUIDE.md`
- Quick start: `PLANNER_QUICKSTART.md`

## Next Steps

1. Test with stationary target
2. Test with moving target
3. Tune N and amax for your scenario
4. Compare with PP planner performance
5. Consider implementing FRPN for better performance

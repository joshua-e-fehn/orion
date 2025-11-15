# Hover Demo with Target Prediction

## Overview

This demo combines the hover flight trajectory (straight-line motion) with real-time target prediction visualization. You can observe how the CV (Constant Velocity) or CA (Constant Acceleration) predictor forecasts the target drone's future positions.

## Quick Start

### 1. Start PX4 SITL

In a **separate terminal**, launch PX4 with Gazebo:

```bash
cd ~/PX4-Autopilot
make px4_sitl gz_x500
```

Wait until you see:
- Gazebo window opens with the drone
- PX4 console shows `pxh>`

### 2. Run the Demo

In your workspace terminal:

```bash
cd ~/Documents/Orion/orion_arm
./hover_predictor_demo.sh
```

The script will:
1. Check if PX4 SITL is running
2. Ask you to select a predictor type (CV or CA)
3. Launch the hover controller and predictor
4. Open RViz with visualization

## What You'll See in RViz

### Target Drone (Red)
- **Current Position**: Red sphere
- **Trajectory Trail**: Line showing past positions
- **Velocity Vector**: Red arrow

### Predicted Positions (Green/Yellow/Red)
The predictor shows future positions at multiple time horizons:
- **t+0.5s**: Very close prediction (small uncertainty)
- **t+1.0s**: Short-term prediction
- **t+2.0s**: Medium-term prediction  
- **t+3.0s**: Longer-term prediction
- **t+5.0s**: Long-term prediction (larger uncertainty)

### Uncertainty Visualization
- **Ellipsoids**: 3D uncertainty regions around each prediction
- **Color Coding**:
  - **Green**: High confidence (small uncertainty)
  - **Yellow**: Medium confidence
  - **Red**: Low confidence (large uncertainty)
- Uncertainty **grows with prediction horizon** (further into future = less certain)

### Predicted Trajectory
- **Line connecting predictions**: Shows expected future path
- For CV predictor: Straight line (assumes constant velocity)
- For CA predictor: Curved path (accounts for acceleration)

## Manual Launch (Without Script)

If you prefer to launch manually:

```bash
# Source workspace
cd ~/Documents/Orion/orion_arm
source install/setup.bash

# Launch with CV predictor
ros2 launch orion_flight hover_with_predictor.launch.py predictor_type:=cv

# OR launch with CA predictor
ros2 launch orion_flight hover_with_predictor.launch.py predictor_type:=ca
```

### Custom Parameters

```bash
ros2 launch orion_flight hover_with_predictor.launch.py \
    predictor_type:=cv \
    target_namespace:=px4_2 \
    flight_height:=5.0 \
    prediction_update_rate:=20.0 \
    prediction_horizons:="[0.5, 1.0, 2.0, 3.0, 5.0]"
```

## Understanding the Predictions

### CV Predictor (Constant Velocity)
- **Assumes**: Target maintains current velocity
- **Best for**: Straight-line motion, cruise flight
- **Prediction**: `p(t+Δt) = p(t) + v(t)·Δt`
- **You'll see**: Straight-line predictions extending from current position

### CA Predictor (Constant Acceleration)
- **Assumes**: Target maintains current acceleration
- **Best for**: Maneuvering targets, turns, climbs
- **Prediction**: `p(t+Δt) = p(t) + v(t)·Δt + 0.5·a(t)·Δt²`
- **You'll see**: Curved predictions if target is accelerating

## Comparing Predictors

For the **hover demo** (straight-line motion):
- **CV predictor**: Should be very accurate (target is at constant velocity)
- **CA predictor**: Should also work well, might show slight curvature if estimating small accelerations from noise

To test which works better:
1. Run demo with CV predictor, observe prediction accuracy
2. Stop and restart with CA predictor
3. Compare how predictions match actual target motion

## RViz Configuration

The demo uses a custom RViz configuration showing:
- **3D view**: Top-down or perspective view
- **Grid**: Ground reference
- **TF frames**: Coordinate system visualization
- **Markers**: Target and predictions
- **Axes**: World frame orientation

### Adjusting the View

In RViz:
- **Mouse scroll**: Zoom in/out
- **Left drag**: Rotate view
- **Middle drag**: Pan view
- **Right-click marker**: Show details

## Monitoring Prediction Quality

### Terminal Output

The predictor node prints diagnostic information:

```
[INFO] [cv_predictor_node]: CV Prediction: 
  pos=[10.23, 5.45, -5.00], 
  vel=[2.00, 0.00, 0.00]
```

### Topics to Inspect

```bash
# View predicted state
ros2 topic echo /target/predicted_state

# View target actual state
ros2 topic echo /px4_2/fmu/out/vehicle_local_position

# View prediction markers
ros2 topic echo /target/prediction_markers
```

### Compute Prediction Error

```bash
# In a separate terminal, compute error between prediction and actual
ros2 run orion_flight prediction_error_monitor
```

(Note: This tool needs to be implemented separately)

## Troubleshooting

### No predictions visible
- Check predictor node is running: `ros2 node list` should show `cv_predictor_node` or `ca_predictor_node`
- Check markers are published: `ros2 topic hz /target/prediction_markers`
- Verify target is publishing: `ros2 topic hz /px4_2/fmu/out/vehicle_local_position`

### Predictions don't match target
- Target might be maneuvering: switch from CV to CA predictor
- Check process noise parameters in config file
- Increase update rate for faster convergence

### RViz crashes or freezes
- Reduce marker count by decreasing prediction horizons
- Reduce update rate
- Close other applications to free memory

## Next Steps

### 1. Test with Circular Motion

Instead of straight-line hover, try circular trajectory:

```bash
ros2 launch orion_flight circle_trajectory.launch.py
```

Then run predictor:

```bash
ros2 launch orion_flight predictors.launch.py predictor_type:=ca
```

The **CA predictor** should perform much better for circular motion!

### 2. Implement Interceptor

Once you're comfortable with predictions, implement a guidance law:

```bash
ros2 launch orion_flight pp_planner.launch.py target_namespace:=px4_2
```

### 3. Try IMM Predictor (When Available)

The Interacting Multiple Model (IMM) predictor adaptively switches between CV and CA:

```bash
ros2 launch orion_flight hover_with_predictor.launch.py predictor_type:=imm
```

## Files in This Demo

- `hover_predictor_demo.sh`: Interactive demo launcher script
- `launch/hover_with_predictor.launch.py`: Combined launch file
- `launch/predictors.launch.py`: Standalone predictor launcher
- `config/cv_predictor.yaml`: CV predictor configuration
- `config/ca_predictor.yaml`: CA predictor configuration

## References

- [CV Predictor README](../src/orion_flight/orion_flight/predictors/cv/README.md)
- [CA Predictor README](../src/orion_flight/orion_flight/predictors/ca/README.md)
- [Predictor Framework](../src/orion_flight/orion_flight/predictors/PREDICTOR_README.md)
- [Algorithm Guide](algo_guide.md)

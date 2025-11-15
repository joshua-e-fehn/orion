# Dual-Drone Simulation Setup Guide

## Overview

The Orion workspace supports multi-drone simulations for testing interception and prediction algorithms. The typical setup involves:
- **Drone 1 (Ego/Interceptor)**: The drone you control to intercept the target
- **Drone 2 (Target/Enemy)**: The target drone whose behavior you want to predict and intercept

Both drones run in the same Gazebo simulation, each with their own PX4 SITL instance.

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    Gazebo Simulation                        │
│  ┌──────────────┐              ┌──────────────┐            │
│  │   Drone 1    │              │   Drone 2    │            │
│  │ (Interceptor)│              │   (Target)   │            │
│  └──────┬───────┘              └──────┬───────┘            │
└─────────┼───────────────────────────────┼──────────────────┘
          │                               │
    ┌─────▼─────┐                   ┌─────▼─────┐
    │  PX4 SITL │                   │  PX4 SITL │
    │ Instance 1│                   │ Instance 2│
    └─────┬─────┘                   └─────┬─────┘
          │                               │
    ┌─────▼──────┐                  ┌─────▼──────┐
    │ uXRCE-DDS  │                  │ uXRCE-DDS  │
    │ Agent :8888│                  │ Agent :8889│
    └─────┬──────┘                  └─────┬──────┘
          │                               │
          └───────────────┬───────────────┘
                          │
                  ┌───────▼────────┐
                  │   ROS 2 DDS    │
                  │  (Namespaced)  │
                  └────────────────┘
```

## Namespaced ROS2 Topics

Each drone publishes and subscribes to topics under its own namespace to avoid conflicts.

### Drone 1 (Interceptor) - Namespace: `/px4_1`

**State Estimation (Subscribe from):**
```
/px4_1/fmu/out/vehicle_local_position     # Position, velocity, acceleration
/px4_1/fmu/out/vehicle_attitude           # Attitude (quaternion, euler)
/px4_1/fmu/out/vehicle_status             # Arming state, flight mode
/px4_1/fmu/out/vehicle_odometry           # Full odometry
/px4_1/fmu/out/vehicle_angular_velocity   # Angular rates
```

**Control Commands (Publish to):**
```
/px4_1/fmu/in/offboard_control_mode       # Enable offboard mode (heartbeat)
/px4_1/fmu/in/trajectory_setpoint         # Position/velocity/acceleration commands
/px4_1/fmu/in/vehicle_command             # Arm, disarm, mode changes
/px4_1/fmu/in/vehicle_rates_setpoint      # Direct rate commands (advanced)
```

### Drone 2 (Target) - Namespace: `/px4_2`

**State Estimation (Subscribe from):**
```
/px4_2/fmu/out/vehicle_local_position     # Target position & velocity
/px4_2/fmu/out/vehicle_attitude           # Target attitude
/px4_2/fmu/out/vehicle_status             # Target status
/px4_2/fmu/out/vehicle_odometry           # Target full odometry
```

**Control Commands (Publish to):**
```
/px4_2/fmu/in/offboard_control_mode       # Control target drone (for testing)
/px4_2/fmu/in/trajectory_setpoint         # Command target maneuvers
/px4_2/fmu/in/vehicle_command             # Arm/disarm target
```

## Starting Multi-Drone Simulation

### Method 1: Manual Launch (Recommended for Learning)

**Terminal 1: Start PX4 Instance 1**
```bash
cd ~/PX4-Autopilot
PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE="0,0" PX4_GZ_MODEL_NAME=x500_1 \
./build/px4_sitl_default/bin/px4 -i 1
```

**Terminal 2: Start PX4 Instance 2**
```bash
cd ~/PX4-Autopilot
PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE="3,0" PX4_GZ_MODEL_NAME=x500_2 \
./build/px4_sitl_default/bin/px4 -i 2
```

**Terminal 3: Start Gazebo**
```bash
gz sim -v4 -r multi_uav_hitl.sdf
```

**Terminal 4: Start uXRCE-DDS Agent for Drone 1**
```bash
MicroXRCEAgent udp4 -p 8888
```

**Terminal 5: Start uXRCE-DDS Agent for Drone 2**
```bash
MicroXRCEAgent udp4 -p 8889
```

### Method 2: Using PX4 Multi-Vehicle Script

```bash
cd ~/PX4-Autopilot
# This will start 2 vehicles with automatic port configuration
./Tools/simulation/gazebo-classic/sitl_multiple_run.sh -n 2 -m x500
```

## Verifying Communication

Check that ROS2 can see both drones:

```bash
# List all topics
ros2 topic list

# Should see topics like:
# /px4_1/fmu/out/vehicle_local_position
# /px4_2/fmu/out/vehicle_local_position
# etc.

# Echo drone 1 position
ros2 topic echo /px4_1/fmu/out/vehicle_local_position

# Echo drone 2 position
ros2 topic echo /px4_2/fmu/out/vehicle_local_position
```

## Typical Workflow

### Basic Interception Setup

1. **Start simulation** (all terminals above)
2. **Disable safety checks** for both drones:
   ```bash
   python3 disable_all_safety.py --namespace px4_1
   python3 disable_all_safety.py --namespace px4_2
   ```
3. **Launch predictor node** (monitors drone 2, optional but recommended):
   ```bash
   ros2 run orion_flight cv_predictor_node --ros-args \
       -p target_namespace:=px4_2
   ```
4. **Launch planner/guidance** for drone 1 (interceptor):
   ```bash
   # Pure Pursuit (baseline)
   ros2 run orion_flight pp_planner --ros-args \
       -p interceptor_namespace:=px4_1 \
       -p target_namespace:=px4_2 \
       -p G_pp:=2.0 \
       -p use_predictor:=true
   
   # Or use launch file (when available):
   # ros2 launch orion_flight pp_planner.launch.py
   ```
5. **Enable offboard mode** for drone 1:
   ```bash
   # Arm and switch to offboard mode
   ros2 topic pub --once /px4_1/fmu/in/vehicle_command px4_msgs/msg/VehicleCommand \
       "{command: 400, param1: 1.0}"  # ARM
   
   ros2 topic pub --once /px4_1/fmu/in/vehicle_command px4_msgs/msg/VehicleCommand \
       "{command: 176, param1: 6.0}"  # Set to OFFBOARD mode
   ```

## Important Notes

### Coordinate Frames
- **Local NED Frame**: North-East-Down (PX4 native)
  - X: North (forward)
  - Y: East (right)
  - Z: Down (positive is down!)
- **ROS/RViz uses ENU**: East-North-Up
  - Conversion needed for visualization

### Timing Considerations
- PX4 runs at ~250 Hz internal loop
- `/fmu/out/vehicle_local_position` publishes at ~50 Hz
- Offboard control requires heartbeat at ≥2 Hz (recommend 10 Hz)
- Prediction nodes typically run at 10-20 Hz

### Common Issues

**Problem: Topics not appearing**
- Check uXRCE-DDS agents are running on correct ports
- Verify PX4 shows "Client connected" in console
- Check ROS_DOMAIN_ID matches (default: 0)

**Problem: Drones spawn at same location**
- Ensure `PX4_GZ_MODEL_POSE` sets different positions
- Format: `"x,y"` in meters (e.g., `"0,0"` and `"3,0"`)

**Problem: One drone won't arm**
- Run safety disable script for that specific namespace
- Check arming status: `ros2 topic echo /px4_X/fmu/out/vehicle_status`
- Use diagnostic: `python3 px4_diagnostic.py --namespace px4_X`

## Testing Predictors

To test prediction algorithms:

1. **Manual control**: Fly drone 2 manually while predictor tracks it
2. **Scripted maneuvers**: Use trajectory node to command drone 2 through known patterns
3. **Evasive maneuvers**: Implement evasive behavior on drone 2 to test predictor robustness

Example test scenario:
```bash
# Terminal 1: Start predictor
ros2 run orion_flight cv_predictor_node --ros-args -p target_namespace:=px4_2

# Terminal 2: Command target to fly circles
ros2 run orion_flight circle_trajectory_node --ros-args \
    -r __ns:=/px4_2 \
    -p circle_radius:=10.0 \
    -p angular_velocity:=0.5
```

## Testing Planners

The planner framework provides multiple guidance algorithms for drone interception. See [`planners/PLANNER_README.md`](src/orion_flight/planners/PLANNER_README.md) for detailed documentation.

### Available Planners

| Planner | Status | Description | Use Case |
|---------|--------|-------------|----------|
| **PP** (Pure Pursuit) | ✅ Implemented | Simple proportional guidance | Baseline testing |
| **PN** (Proportional Navigation) | 🚧 Planned | Classic missile guidance | Academic comparison |
| **LPN** (Linearized PN) | 🚧 Planned | Cartesian formulation | General interception |
| **FRPN** (Fast Response PN) | 🚧 Planned | **RECOMMENDED** - Paper's main contribution | Production use |
| **MPC** (Model Predictive Control) | 🚧 Planned | Constraint-aware optimal control | Safety-critical scenarios |

### Pure Pursuit Planner Testing

**Basic test (stationary target):**
```bash
# Terminal 1: Start PP planner
ros2 run orion_flight pp_planner --ros-args \
    -p interceptor_namespace:=px4_1 \
    -p target_namespace:=px4_2 \
    -p G_pp:=2.0 \
    -p amax:="[4.0, 4.0, 2.0]" \
    -p use_predictor:=false

# Terminal 2: Monitor status
ros2 topic echo /planner/status

# Terminal 3: Visualize in RViz
rviz2 -d src/orion_flight/config/planner_visualization.rviz
```

**Advanced test (moving target with predictor):**
```bash
# Terminal 1: Start predictor
ros2 run orion_flight cv_predictor_node --ros-args \
    -p target_namespace:=px4_2

# Terminal 2: Start PP planner with predictor
ros2 run orion_flight pp_planner --ros-args \
    -p interceptor_namespace:=px4_1 \
    -p target_namespace:=px4_2 \
    -p G_pp:=3.0 \
    -p use_predictor:=true

# Terminal 3: Command target to move
ros2 run orion_flight circle_trajectory_node --ros-args \
    -r __ns:=/px4_2 \
    -p circle_radius:=15.0 \
    -p angular_velocity:=0.3
```

### Planner Parameter Tuning

**Pure Pursuit parameters:**
- `G_pp`: Proportional gain (default: 2.0)
  - Lower (1.0-2.0): Slower, smoother approach
  - Higher (3.0-5.0): Faster, more aggressive (may overshoot)
- `amax`: Max acceleration `[ax, ay, az]` in m/s² (default: [4.0, 4.0, 2.0])
  - Conservative: [2.0, 2.0, 1.0]
  - Aggressive: [6.0, 6.0, 3.0]
- `control_rate`: Control loop frequency in Hz (default: 20.0)
  - Minimum: 10 Hz
  - Recommended: 20-50 Hz
- `convergence_distance`: Intercept success threshold in meters (default: 1.0)

**Testing procedure:**
1. Start with conservative gains (G_pp=2.0)
2. Test with stationary target first
3. Gradually increase target speed/maneuverability
4. Tune gains based on overshoot/convergence time
5. Add predictor for moving targets

### Integration: Predictor + Planner

Complete interception pipeline:
```bash
# 1. Start PX4 SITL for both drones (see above)

# 2. Start predictor (CV for testing)
ros2 run orion_flight cv_predictor_node --ros-args \
    -p target_namespace:=px4_2 \
    -p prediction_horizons:="[0.5, 1.0, 2.0]"

# 3. Start planner (PP for baseline)
ros2 run orion_flight pp_planner --ros-args \
    -p interceptor_namespace:=px4_1 \
    -p target_namespace:=px4_2 \
    -p use_predictor:=true \
    -p G_pp:=2.5

# 4. Command target drone to perform maneuvers
ros2 run orion_flight circle_trajectory_node --ros-args \
    -r __ns:=/px4_2 \
    -p circle_radius:=20.0

# 5. Monitor planner status
ros2 topic echo /planner/status

# 6. Visualize in RViz
rviz2
```

**Topic flow:**
```
Target PX4 (px4_2) 
  → /px4_2/fmu/out/vehicle_local_position 
  → CV Predictor 
  → /target/predicted_state 
  → PP Planner 
  → /px4_1/fmu/in/trajectory_setpoint 
  → Interceptor PX4 (px4_1)
```

## Visualization in RViz

Launch RViz with multi-drone configuration:
```bash
rviz2 -d src/orion_flight/config/multi_drone.rviz
```

Add displays for:
- **Drone 1 path**: Topic `/px4_1/fmu/out/vehicle_local_position`
- **Drone 2 path**: Topic `/px4_2/fmu/out/vehicle_local_position`
- **Predicted path**: Topic `/target/prediction_markers`
- **TF frames**: `map`, `px4_1_base_link`, `px4_2_base_link`

## Next Steps

- See [`predictors/PREDICTOR_README.md`](src/orion_flight/predictors/PREDICTOR_README.md) for predictor architecture
- See [`predictors/PREDICTOR_IMPLEMENTATION_GUIDE.md`](src/orion_flight/predictors/PREDICTOR_IMPLEMENTATION_GUIDE.md) for implementing new predictors
- See [`planners/PLANNER_README.md`](src/orion_flight/planners/PLANNER_README.md) for planner architecture and guidance law selection
- See [`planners/PLANNER_IMPLEMENTATION_GUIDE.md`](src/orion_flight/planners/PLANNER_IMPLEMENTATION_GUIDE.md) for implementing new guidance laws
- See [`algo_guide.md`](algo_guide.md) for detailed algorithm descriptions and pseudocode

## References

- Pliska et al., "Towards Safe Mid-Air Drone Interception: Strategies for Tracking & Capture", IEEE RA-L 2024
- PX4 Offboard Control: https://docs.px4.io/main/en/flight_modes/offboard.html
- PX4 Multi-Vehicle Simulation: https://docs.px4.io/main/en/simulation/multi-vehicle-simulation.html

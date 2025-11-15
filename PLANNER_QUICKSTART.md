# Planner Quick Start Guide

Quick guide to get the Pure Pursuit planner running for drone interception.

## Prerequisites

✅ PX4 SITL installed and working  
✅ Micro XRCE-DDS Agent installed  
✅ ROS2 (Foxy/Galactic/Humble) installed  
✅ Orion workspace built

## Build the Planner

```bash
cd ~/Documents/Orion/orion_arm

# Build the workspace
colcon build --packages-select orion_flight

# Source the workspace
source install/setup.bash
```

## Quick Test (5 Minutes)

### Terminal 1: PX4 SITL (Interceptor - Drone 1)
```bash
cd ~/PX4-Autopilot
PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE="0,0" PX4_GZ_MODEL_NAME=x500_1 \
./build/px4_sitl_default/bin/px4 -i 1
```

### Terminal 2: PX4 SITL (Target - Drone 2)
```bash
cd ~/PX4-Autopilot
PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE="10,0" PX4_GZ_MODEL_NAME=x500_2 \
./build/px4_sitl_default/bin/px4 -i 2
```

### Terminal 3: Gazebo
```bash
gz sim -v4 -r multi_uav_hitl.sdf
```

### Terminal 4: Micro XRCE-DDS Agent (Drone 1)
```bash
MicroXRCEAgent udp4 -p 8888
```

### Terminal 5: Micro XRCE-DDS Agent (Drone 2)
```bash
MicroXRCEAgent udp4 -p 8889
```

### Terminal 6: Pure Pursuit Planner
```bash
cd ~/Documents/Orion/orion_arm
source install/setup.bash

# Launch PP planner with launch file
ros2 launch orion_flight pp_planner.launch.py

# OR run directly with custom parameters:
ros2 run orion_flight pp_planner --ros-args \
    -p interceptor_namespace:=px4_1 \
    -p target_namespace:=px4_2 \
    -p G_pp:=2.0 \
    -p use_predictor:=false
```

### Terminal 7: Enable Offboard Mode
```bash
cd ~/Documents/Orion/orion_arm
source install/setup.bash

# Disable safety checks for drone 1
python3 disable_all_safety.py --namespace px4_1

# Wait a few seconds, then arm and enable offboard
ros2 topic pub --once /px4_1/fmu/in/vehicle_command px4_msgs/msg/VehicleCommand \
    "{command: 400, param1: 1.0}"  # ARM

sleep 2

ros2 topic pub --once /px4_1/fmu/in/vehicle_command px4_msgs/msg/VehicleCommand \
    "{command: 176, param1: 6.0}"  # OFFBOARD mode
```

### Terminal 8: Monitor Status
```bash
cd ~/Documents/Orion/orion_arm
source install/setup.bash

# Watch planner status
ros2 topic echo /planner/status
```

**Expected behavior:**
- Drone 1 (interceptor) should move toward Drone 2 (target)
- Status messages show decreasing `miss_distance`
- When distance < 1m, `converged: true`

## With Moving Target

Add a predictor and moving target:

### Terminal 9: Predictor (Optional but Recommended)
```bash
cd ~/Documents/Orion/orion_arm
source install/setup.bash

ros2 run orion_flight cv_predictor_node --ros-args \
    -p target_namespace:=px4_2
```

### Terminal 10: Target Trajectory
```bash
cd ~/Documents/Orion/orion_arm
source install/setup.bash

# Command target to fly in circles
ros2 run orion_flight circle_trajectory_node --ros-args \
    -r __ns:=/px4_2 \
    -p circle_radius:=15.0 \
    -p angular_velocity:=0.3
```

**Update planner to use predictor:**
```bash
# Restart planner with predictor enabled
ros2 launch orion_flight pp_planner.launch.py use_predictor:=true
```

## Visualization

```bash
# Launch RViz
rviz2
```

**Add displays:**
1. **MarkerArray**: Topic `/planner/guidance_markers`
   - Shows: LOS line (green), acceleration arrow (red), drone positions
2. **Path**: Topic `/px4_1/fmu/out/vehicle_local_position`
   - Shows: Interceptor trajectory
3. **Path**: Topic `/px4_2/fmu/out/vehicle_local_position`
   - Shows: Target trajectory

**Note:** RViz uses ENU frame (X=East, Y=North, Z=Up), PX4 uses NED (X=North, Y=East, Z=Down). Markers are auto-converted.

## Parameter Tuning

Edit launch file or pass as arguments:

```bash
ros2 launch orion_flight pp_planner.launch.py \
    G_pp:=3.0 \
    amax_x:=5.0 \
    amax_y:=5.0 \
    amax_z:=3.0 \
    control_rate:=30.0
```

**Tuning tips:**
- **G_pp too low (< 1.0)**: Slow convergence
- **G_pp too high (> 5.0)**: Oscillations, overshoot
- **amax too low**: Cannot reach target if it's maneuvering
- **amax too high**: Aggressive, may violate drone limits

Start with defaults (G_pp=2.0, amax=[4,4,2]), tune from there.

## Troubleshooting

### Planner starts but drone doesn't move
- ✅ Check offboard mode is enabled: `ros2 topic echo /px4_1/fmu/out/vehicle_status`
- ✅ Check planner is publishing: `ros2 topic hz /px4_1/fmu/in/trajectory_setpoint`
- ✅ Check drone is armed

### No target state available
- ✅ Verify drone 2 is running and publishing: `ros2 topic hz /px4_2/fmu/out/vehicle_local_position`
- ✅ Check namespace matches parameter

### RViz shows nothing
- ✅ Set Fixed Frame to `map`
- ✅ Check topic names match exactly
- ✅ Markers publish at ~5 Hz, be patient

### Planner oscillates/overshoots
- ⚙️ Reduce `G_pp` (try 1.5 or 1.0)
- ⚙️ Reduce `amax`
- ⚙️ Increase `control_rate` for smoother commands

## Next Steps

Once Pure Pursuit is working:

1. **Test with predictor**: Add CV predictor node
2. **Tune parameters**: Find optimal gains for your scenario
3. **Implement PN/FRPN**: More advanced guidance laws
4. **Add MPC**: For constraint-aware planning

See:
- [`PLANNER_README.md`](../src/orion_flight/planners/PLANNER_README.md) - Full documentation
- [`PLANNER_IMPLEMENTATION_GUIDE.md`](../src/orion_flight/planners/PLANNER_IMPLEMENTATION_GUIDE.md) - Implementation details
- [`algo_guide.md`](algo_guide.md) - Algorithm theory and pseudocode

## Success Criteria

✅ Planner node starts without errors  
✅ Drone 1 moves toward drone 2  
✅ `miss_distance` decreases over time  
✅ Visualization markers appear in RViz  
✅ Status messages show `converged: true` when close  

Congratulations! You have a working interception planner. 🚁🎯

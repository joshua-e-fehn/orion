# Orion Launch System - Summary

## Overview

The Orion drone interception system now has three modular, easy-to-use launch files that handle all aspects of simulation, attacker behavior, and interceptor control.

## Launch Files Created

### 1. **`simulation.launch.py`** 
**Location:** `src/orion_flight/launch/simulation.launch.py`

**Purpose:** Complete simulation environment setup

**Features:**
- Launches 2 PX4 SITL instances (px4_1 and px4_2)
- Starts Gazebo with both drones
- Automatically starts MicroXRCE-DDS agents
- Sets up RViz visualization
- Configures coordinate frames
- Safety features disabled automatically

**Usage:**
```bash
ros2 launch orion_flight simulation.launch.py
```

---

### 2. **`attacker.launch.py`**
**Location:** `src/orion_flight/launch/attacker.launch.py`

**Purpose:** Control attacker drone (px4_1) behavior

**Modes Available:**
- ✅ `hover` - Straight-line hover flight
- ✅ `circle` - Circular trajectory
- 🚧 `waypoint` - Waypoint following (future)
- 🚧 `evasive` - Evasive maneuvers (future)

**Usage:**
```bash
# Hover mode
ros2 launch orion_flight attacker.launch.py mode:=hover flight_height:=5.0

# Circle mode
ros2 launch orion_flight attacker.launch.py mode:=circle radius:=10.0 altitude:=5.0
```

---

### 3. **`interceptor.launch.py`**
**Location:** `src/orion_flight/launch/interceptor.launch.py`

**Purpose:** Control interceptor drone (px4_2) with prediction and planning

**Modes:**
- `prediction_only` - Test predictors
- `planning_only` - Test planners with ground truth
- `full` - Complete system (predictor + planner)

**Predictors:**
- ✅ `cv` - Constant Velocity (best for straight lines)
- ✅ `ca` - Constant Acceleration (best for maneuvers)
- 🚧 `imm` - Interacting Multiple Model (future)

**Planners:**
- ✅ `pp` - Pure Pursuit
- 🚧 `apf` - Artificial Potential Field (future)
- 🚧 `mpc` - Model Predictive Control (future)

**Usage:**
```bash
# Full system with CA predictor and PP planner
ros2 launch orion_flight interceptor.launch.py \
    mode:=full \
    predictor:=ca \
    planner:=pp \
    lookahead_distance:=3.0
```

---

## Quick Start Guide

### Method 1: Using Launch Files Directly

**Step 1: Start Simulation**
```bash
# Terminal 1
cd ~/Documents/Orion/orion_arm
source install/setup.bash
ros2 launch orion_flight simulation.launch.py
```
*Wait for Gazebo to fully load!*

**Step 2: Launch Attacker**
```bash
# Terminal 2
cd ~/Documents/Orion/orion_arm
source install/setup.bash
ros2 launch orion_flight attacker.launch.py mode:=circle radius:=10.0
```

**Step 3: Launch Interceptor**
```bash
# Terminal 3
cd ~/Documents/Orion/orion_arm
source install/setup.bash
ros2 launch orion_flight interceptor.launch.py mode:=full predictor:=ca planner:=pp
```

### Method 2: Using Interactive Script

```bash
cd ~/Documents/Orion/orion_arm
./quick_launch.sh
```

Then follow the interactive prompts!

---

## Key Features

### ✅ Fully Automated Simulation
- No manual PX4 startup required
- No manual agent configuration
- All safety features automatically disabled
- Both drones ready to fly

### ✅ Flexible Behavior Control
- Easy parameter tuning
- Multiple flight modes
- Real-time visualization

### ✅ Modular Prediction & Planning
- Test components independently
- Easy algorithm comparison
- Configurable parameters

### ✅ Clean Architecture
- One launch file per responsibility
- Clear parameter naming
- Comprehensive documentation

---

## Documentation

1. **`LAUNCH_FILES_GUIDE.md`** - Complete user guide with examples
2. **`quick_launch.sh`** - Interactive launch helper script
3. This summary document

---

## File Structure

```
orion_arm/
├── src/orion_flight/launch/
│   ├── simulation.launch.py    ← Full simulation setup
│   ├── attacker.launch.py      ← Attacker behavior control
│   ├── interceptor.launch.py   ← Interceptor system
│   ├── predictors.launch.py    ← (old, still works)
│   └── ...
├── LAUNCH_FILES_GUIDE.md       ← Detailed documentation
├── quick_launch.sh             ← Interactive launcher
└── README.md
```

---

## Example Workflows

### Test CV Predictor on Hover
```bash
# Terminal 1: Simulation
ros2 launch orion_flight simulation.launch.py

# Terminal 2: Hover attacker
ros2 launch orion_flight attacker.launch.py mode:=hover

# Terminal 3: CV predictor only
ros2 launch orion_flight interceptor.launch.py mode:=prediction_only predictor:=cv
```

### Test CA Predictor on Circle
```bash
# Terminal 1: Simulation
ros2 launch orion_flight simulation.launch.py

# Terminal 2: Circle attacker
ros2 launch orion_flight attacker.launch.py mode:=circle radius:=10.0

# Terminal 3: CA predictor only
ros2 launch orion_flight interceptor.launch.py mode:=prediction_only predictor:=ca
```

### Full Interception Demo
```bash
# Terminal 1: Simulation
ros2 launch orion_flight simulation.launch.py

# Terminal 2: Circle attacker
ros2 launch orion_flight attacker.launch.py mode:=circle radius:=12.0 altitude:=8.0

# Terminal 3: Full system
ros2 launch orion_flight interceptor.launch.py \
    mode:=full \
    predictor:=ca \
    planner:=pp \
    lookahead_distance:=4.0 \
    max_speed:=7.0
```

---

## Benefits Over Old System

### Before:
- ❌ Manual PX4 startup in separate terminal
- ❌ Manual MicroXRCE-DDS agent startup
- ❌ Complex bash scripts with many steps
- ❌ Hard to change parameters
- ❌ Difficult to test individual components

### Now:
- ✅ One command to start complete simulation
- ✅ One command per drone behavior
- ✅ Easy parameter modification via launch args
- ✅ Modular testing of predictors/planners
- ✅ Clean separation of concerns
- ✅ Interactive helper script available

---

## Next Steps

1. **Test the System:**
   ```bash
   ./quick_launch.sh
   ```

2. **Try Different Configurations:**
   - Compare CV vs CA predictors
   - Test different circle radii
   - Tune planner parameters

3. **Extend the System:**
   - Add waypoint mode to attacker
   - Implement IMM predictor
   - Add APF/MPC planners
   - Create custom RViz configs

4. **Create Launch Combinations:**
   - Save your favorite parameter sets
   - Create scenario-specific launch files
   - Build automated test sequences

---

## Troubleshooting

**Problem:** Simulation doesn't start
```bash
# Check PX4 installation
ls ~/PX4-Autopilot

# Specify custom path if needed
ros2 launch orion_flight simulation.launch.py px4_dir:=/your/path
```

**Problem:** Drones don't move
- Wait for Gazebo to fully load
- Check that agents show "successfully created client"
- Verify topics: `ros2 topic list | grep px4_1`

**Problem:** "Package not found"
```bash
cd ~/Documents/Orion/orion_arm
colcon build
source install/setup.bash
```

---

## Performance Tips

1. **Always start simulation first** - Let it fully load before behaviors
2. **Use appropriate predictor** - CV for straight, CA for maneuvers  
3. **Tune lookahead distance** - Smaller = tighter following, larger = smoother
4. **Monitor update rates** - Use `ros2 topic hz` to check performance
5. **Record successful configs** - Document what works well

---

## Summary

You now have a complete, modular launch system that:

✅ Starts full simulation with one command  
✅ Controls attacker behavior with simple parameters  
✅ Runs interceptor with flexible predictor/planner selection  
✅ Provides interactive launching via `quick_launch.sh`  
✅ Includes comprehensive documentation  
✅ Separates concerns for easy testing and development  

**Happy flying! 🚁**

---

**Created:** November 15, 2025  
**Author:** Orion ARM Team

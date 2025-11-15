# Orion Launch Files User Guide

This guide explains how to use the three main launch files for the Orion drone interception system.

## Overview

The system is divided into three modular launch files:

1. **`simulation.launch.py`** - Complete simulation environment with 2 drones
2. **`attacker.launch.py`** - Attacker drone (px4_1) behavior control
3. **`interceptor.launch.py`** - Interceptor drone (px4_2) with prediction & planning

## Quick Start

### Step 1: Start the Simulation

```bash
# Terminal 1: Launch complete simulation with 2 drones
cd ~/Documents/Orion/orion_arm
source install/setup.bash
ros2 launch orion_flight simulation.launch.py
```

This will:
- Start Gazebo with 2 drones (px4_1 at [0,0], px4_2 at [0,3])
- Launch MicroXRCE-DDS agents for both drones
- Open RViz for visualization
- Set up coordinate frames

**Wait for Gazebo to fully load before proceeding!** (You should see both drones in the simulation)

### Step 2: Launch Attacker Behavior

```bash
# Terminal 2: Make attacker fly in a circle
cd ~/Documents/Orion/orion_arm
source install/setup.bash
ros2 launch orion_flight attacker.launch.py mode:=circle radius:=10.0
```

### Step 3: Launch Interceptor System

```bash
# Terminal 3: Launch interceptor with CV predictor and Pure Pursuit planner
cd ~/Documents/Orion/orion_arm
source install/setup.bash
ros2 launch orion_flight interceptor.launch.py mode:=full predictor:=cv planner:=pp
```

---

## Launch File Details

### 1. `simulation.launch.py` - Simulation Environment

**Purpose:** Starts the complete PX4 SITL simulation with Gazebo and ROS2 bridge.

**Usage:**
```bash
ros2 launch orion_flight simulation.launch.py [parameters]
```

**Parameters:**

| Parameter | Default | Description |
|-----------|---------|-------------|
| `world` | `default` | Gazebo world to load |
| `gui` | `true` | Show Gazebo GUI |
| `headless` | `false` | Run headless (no GUI) |
| `rviz` | `true` | Launch RViz visualization |
| `px4_dir` | `~/PX4-Autopilot` | Path to PX4-Autopilot directory |

**Examples:**

```bash
# Standard launch with GUI
ros2 launch orion_flight simulation.launch.py

# Headless simulation (no Gazebo GUI)
ros2 launch orion_flight simulation.launch.py headless:=true

# Custom PX4 directory
ros2 launch orion_flight simulation.launch.py px4_dir:=/path/to/PX4-Autopilot

# No RViz (if you want to launch it separately)
ros2 launch orion_flight simulation.launch.py rviz:=false
```

**What Gets Launched:**
- PX4 SITL instance 1 (px4_1) at position [0, 0]
- PX4 SITL instance 2 (px4_2) at position [0, 3]
- MicroXRCE-DDS agent on port 8888 for px4_1
- MicroXRCE-DDS agent on port 8889 for px4_2
- Gazebo simulation environment
- RViz with simulation config
- Static TF transforms for both drones

**Notes:**
- Each PX4 instance and agent runs in a separate gnome-terminal window
- Drone 2 launches 5 seconds after drone 1 to avoid conflicts
- Agents start after a delay to ensure PX4 is ready
- Close all terminals when stopping the simulation

---

### 2. `attacker.launch.py` - Attacker Drone Behavior

**Purpose:** Controls the behavior of the attacker drone (px4_1).

**Usage:**
```bash
ros2 launch orion_flight attacker.launch.py mode:=<MODE> [parameters]
```

**Available Modes:**

| Mode | Status | Description |
|------|--------|-------------|
| `hover` | ✅ Ready | Straight-line hover flight |
| `circle` | ✅ Ready | Circular trajectory |
| `waypoint` | 🚧 Future | Follow waypoint sequence |
| `evasive` | 🚧 Future | Evasive maneuvers |

**Common Parameters:**

| Parameter | Default | Description |
|-----------|---------|-------------|
| `mode` | `hover` | Behavior mode |
| `namespace` | `px4_1` | Drone namespace |
| `rviz` | `true` | Show RViz |
| `trail_length` | `50` | Visualization trail length |

**Hover Mode Parameters:**

| Parameter | Default | Description |
|-----------|---------|-------------|
| `flight_height` | `5.0` | Flight altitude (meters, positive) |
| `hover_duration` | `30.0` | Duration to hover (seconds) |

**Circle Mode Parameters:**

| Parameter | Default | Description |
|-----------|---------|-------------|
| `radius` | `5.0` | Circle radius (meters) |
| `altitude` | `5.0` | Flight altitude (meters, positive) |
| `angular_velocity` | `0.5` | Angular velocity (rad/s) |

**Examples:**

```bash
# Hover flight at 7 meters for 60 seconds
ros2 launch orion_flight attacker.launch.py \
    mode:=hover \
    flight_height:=7.0 \
    hover_duration:=60.0

# Circle flight with 15m radius at 10m altitude
ros2 launch orion_flight attacker.launch.py \
    mode:=circle \
    radius:=15.0 \
    altitude:=10.0 \
    angular_velocity:=0.3

# Fast circle
ros2 launch orion_flight attacker.launch.py \
    mode:=circle \
    radius:=8.0 \
    angular_velocity:=1.0

# Hover without RViz (if already running)
ros2 launch orion_flight attacker.launch.py \
    mode:=hover \
    rviz:=false
```

**What Gets Launched:**
- Behavior control node (hover_node or circle_node)
- RViz with appropriate configuration
- Static TF transforms

---

### 3. `interceptor.launch.py` - Interceptor System

**Purpose:** Controls the interceptor drone (px4_2) with prediction and planning.

**Usage:**
```bash
ros2 launch orion_flight interceptor.launch.py mode:=<MODE> [parameters]
```

**Operation Modes:**

| Mode | Description | Components |
|------|-------------|------------|
| `prediction_only` | Track attacker with predictor | Predictor only |
| `planning_only` | Plan path using ground truth | Planner only |
| `full` | Complete system | Predictor + Planner |

**Common Parameters:**

| Parameter | Default | Description |
|-----------|---------|-------------|
| `mode` | `full` | Operation mode |
| `namespace` | `px4_2` | Interceptor namespace |
| `target_namespace` | `px4_1` | Target (attacker) namespace |
| `rviz` | `true` | Launch RViz |

**Predictor Parameters:**

| Parameter | Default | Options | Description |
|-----------|---------|---------|-------------|
| `predictor` | `cv` | `cv`, `ca`, `imm` | Predictor type |
| `prediction_horizon` | `2.0` | seconds | How far to predict |

**Predictor Types:**
- **`cv`** - Constant Velocity (6-state Kalman filter)
  - Best for: Straight-line motion, hover
  - States: [x, y, z, vx, vy, vz]
  
- **`ca`** - Constant Acceleration (9-state Kalman filter)
  - Best for: Maneuvering targets, circles
  - States: [x, y, z, vx, vy, vz, ax, ay, az]
  
- **`imm`** - Interacting Multiple Model (future)
  - Best for: Mixed/unknown motion patterns

**Planner Parameters:**

| Parameter | Default | Options | Description |
|-----------|---------|---------|-------------|
| `planner` | `pp` | `pp`, `apf`, `mpc` | Planner type |
| `lookahead_distance` | `3.0` | meters | [PP] Lookahead distance |
| `max_speed` | `5.0` | m/s | Max interceptor speed |

**Planner Types:**
- **`pp`** - Pure Pursuit
  - Simple geometric path following
  - Good for smooth interception
  
- **`apf`** - Artificial Potential Field (future)
  - Force-based navigation
  
- **`mpc`** - Model Predictive Control (future)
  - Optimal control with constraints

**Examples:**

```bash
# Full system with CV predictor and Pure Pursuit
ros2 launch orion_flight interceptor.launch.py \
    mode:=full \
    predictor:=cv \
    planner:=pp

# Full system with CA predictor for maneuvering target
ros2 launch orion_flight interceptor.launch.py \
    mode:=full \
    predictor:=ca \
    planner:=pp \
    lookahead_distance:=5.0

# Prediction only to test CV predictor
ros2 launch orion_flight interceptor.launch.py \
    mode:=prediction_only \
    predictor:=cv \
    prediction_horizon:=3.0

# Planning only with ground truth
ros2 launch orion_flight interceptor.launch.py \
    mode:=planning_only \
    planner:=pp \
    max_speed:=8.0

# Aggressive interception parameters
ros2 launch orion_flight interceptor.launch.py \
    mode:=full \
    predictor:=ca \
    planner:=pp \
    lookahead_distance:=2.0 \
    max_speed:=10.0 \
    prediction_horizon:=1.5

# Test CA predictor on circle trajectory
ros2 launch orion_flight interceptor.launch.py \
    mode:=prediction_only \
    predictor:=ca \
    prediction_horizon:=2.5
```

---

## Complete Workflow Examples

### Example 1: Test CV Predictor on Hover Flight

```bash
# Terminal 1: Start simulation
ros2 launch orion_flight simulation.launch.py

# Terminal 2: Attacker hovers in straight line
ros2 launch orion_flight attacker.launch.py mode:=hover flight_height:=5.0

# Terminal 3: Test CV predictor
ros2 launch orion_flight interceptor.launch.py \
    mode:=prediction_only \
    predictor:=cv
```

**Expected Result:** RViz shows predicted trajectory matching hover path

---

### Example 2: Test CA Predictor on Circle Flight

```bash
# Terminal 1: Start simulation
ros2 launch orion_flight simulation.launch.py

# Terminal 2: Attacker flies in circle
ros2 launch orion_flight attacker.launch.py \
    mode:=circle \
    radius:=10.0 \
    altitude:=8.0 \
    angular_velocity:=0.5

# Terminal 3: Test CA predictor
ros2 launch orion_flight interceptor.launch.py \
    mode:=prediction_only \
    predictor:=ca \
    prediction_horizon:=3.0
```

**Expected Result:** CA predictor tracks circular motion better than CV

---

### Example 3: Full Interception System

```bash
# Terminal 1: Start simulation
ros2 launch orion_flight simulation.launch.py

# Terminal 2: Attacker flies circle
ros2 launch orion_flight attacker.launch.py \
    mode:=circle \
    radius:=12.0 \
    altitude:=6.0

# Terminal 3: Full interceptor system
ros2 launch orion_flight interceptor.launch.py \
    mode:=full \
    predictor:=ca \
    planner:=pp \
    lookahead_distance:=4.0 \
    max_speed:=7.0
```

**Expected Result:** Interceptor predicts target motion and plans interception path

---

### Example 4: Compare CV vs CA Predictors

**Setup 1: CV Predictor on Circle**
```bash
# Terminal 1: Simulation
ros2 launch orion_flight simulation.launch.py

# Terminal 2: Circle flight
ros2 launch orion_flight attacker.launch.py mode:=circle radius:=10.0

# Terminal 3: CV predictor
ros2 launch orion_flight interceptor.launch.py \
    mode:=prediction_only \
    predictor:=cv
```

**Setup 2: CA Predictor on Circle**
```bash
# Keep terminals 1 & 2 running, restart terminal 3:
ros2 launch orion_flight interceptor.launch.py \
    mode:=prediction_only \
    predictor:=ca
```

**Expected Result:** CA predictor handles acceleration better, less lag

---

## Troubleshooting

### Simulation doesn't start

**Check:**
```bash
# Verify PX4 directory exists
ls ~/PX4-Autopilot

# If not, specify custom path
ros2 launch orion_flight simulation.launch.py \
    px4_dir:=/your/path/to/PX4-Autopilot
```

### Drones don't arm/move

**Check:**
1. Wait for "pxh>" prompt in PX4 terminals
2. Ensure MicroXRCE agents show "successfully created client"
3. Verify topics exist:
   ```bash
   ros2 topic list | grep px4_1
   ros2 topic list | grep px4_2
   ```

### "Package not found" errors

**Build the workspace:**
```bash
cd ~/Documents/Orion/orion_arm
colcon build --packages-select orion_flight attack_drone interceptor
source install/setup.bash
```

### RViz shows nothing

**Check:**
1. Ensure drones are publishing position data:
   ```bash
   ros2 topic echo /px4_1/fmu/out/vehicle_local_position --once
   ```
2. Check TF frames:
   ```bash
   ros2 run tf2_tools view_frames
   ```

### Predictor not tracking

**Debug:**
```bash
# Check if predictor is receiving data
ros2 topic hz /px4_1/fmu/out/vehicle_local_position

# Check predictor output
ros2 topic echo /target/predicted_state

# Check for errors
ros2 node info /cv_predictor_node
```

---

## Advanced Usage

### Running Headless (No GUI)

```bash
# Simulation without Gazebo or RViz GUI
ros2 launch orion_flight simulation.launch.py \
    headless:=true \
    rviz:=false

# Run attacker
ros2 launch orion_flight attacker.launch.py \
    mode:=circle \
    rviz:=false

# Run interceptor
ros2 launch orion_flight interceptor.launch.py \
    mode:=full \
    rviz:=false
```

### Custom RViz Config

```bash
# Launch with your own RViz config
ros2 launch orion_flight interceptor.launch.py \
    mode:=full \
    rviz:=false

# In separate terminal
rviz2 -d /path/to/your/config.rviz
```

### Recording Data

```bash
# Record all topics for analysis
ros2 bag record -a

# Record specific topics
ros2 bag record \
    /px4_1/fmu/out/vehicle_local_position \
    /px4_2/fmu/out/vehicle_local_position \
    /target/predicted_state \
    /target/prediction_markers
```

---

## Tips & Best Practices

1. **Always start simulation first** - Wait for Gazebo to fully load before launching behaviors

2. **One behavior at a time** - Don't launch multiple attacker or interceptor modes simultaneously

3. **Use appropriate predictor** - CV for straight lines, CA for maneuvers

4. **Tune parameters** - Adjust lookahead distance and speed based on scenario

5. **Monitor performance** - Use `ros2 topic hz` to check update rates

6. **Save successful configs** - Document parameter combinations that work well

7. **Clean shutdown** - Use Ctrl+C in all terminals, close simulation last

---

## Next Steps

- Test different predictor/planner combinations
- Tune parameters for your specific scenarios
- Implement new attacker behaviors (waypoint, evasive)
- Add new predictors (IMM) and planners (APF, MPC)
- Create custom RViz configurations
- Record and analyze performance data

---

**Orion ARM Team - November 15, 2025**

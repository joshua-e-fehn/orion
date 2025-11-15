# Autonomous Circular Flight Demo

This demo provides a complete autonomous flight system that makes your PX4 drone fly in a circular pattern at a specified height, with real-time trajectory visualization in RViz2.

## Features

- ✈️ **Autonomous Circular Flight**: Configurable radius, height, and speed
- 📊 **Real-time Visualization**: RViz2 displays the last 10 drone positions
- 🎯 **Complete Flight Sequence**: Automatic arming, takeoff, circular flight, and landing
- ⚙️ **Configurable Parameters**: Easy customization via launch arguments
- 🚀 **Simple Execution**: One-command launch script

## Quick Start

### Prerequisites

1. **PX4 SITL with Gazebo** must be running:
   ```bash
   cd ~/PX4-Autopilot
   make px4_sitl gz_x500
   # OR for classic Gazebo:
   make px4_sitl gazebo-classic
   ```

2. **Workspace Built**:
   ```bash
   cd ~/Documents/Orion/orion_arm
   colcon build --packages-select px4_ros_com --symlink-install
   ```

### Run the Demo

Simply execute the convenience script:

```bash
cd ~/Documents/Orion/orion_arm
./run_circle_demo.sh
```

This script will:
- Source the workspace
- Check if PX4 SITL is running
- Optionally start micro-ros-agent
- Launch the circular flight controller
- Open RViz2 with pre-configured visualization

### Manual Launch

If you prefer to launch manually:

```bash
source ~/Documents/Orion/orion_arm/install/setup.bash
ros2 launch px4_ros_com circle_trajectory.launch.py
```

## Configuration

You can customize the flight parameters by passing arguments to the launch file:

```bash
ros2 launch px4_ros_com circle_trajectory.launch.py \
    circle_radius:=5.0 \
    flight_height:=8.0 \
    angular_velocity:=0.5 \
    num_circles:=3 \
    trail_length:=15
```

### Available Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `circle_radius` | 4.0 | Radius of the circular trajectory (meters) |
| `flight_height` | 5.0 | Flight height above ground (meters, positive value) |
| `angular_velocity` | 0.3 | Angular velocity for circular motion (rad/s) |
| `num_circles` | 2 | Number of complete circles to fly before landing |
| `trail_length` | 10 | Number of position markers to show in RViz trail |
| `use_micro_ros_agent` | false | Auto-start micro-ros-agent for PX4 communication |

## What You'll See

### In Gazebo
- The drone will automatically arm and takeoff
- It will climb to the specified height
- Begin flying in a circular pattern
- Complete the specified number of circles
- Automatically land

### In RViz2
- **Blue sphere**: Current drone position
- **Colored trail**: Last 10 positions (red → green gradient)
- **Cyan line**: Connected trajectory path
- **Grid**: Ground reference plane
- **Axes**: World coordinate frame

## Flight Phases

The autonomous flight follows this state machine:

1. **INIT**: Send initial setpoints and prepare for offboard mode
2. **TAKEOFF**: Climb to target altitude
3. **CIRCLE**: Execute circular trajectory
4. **LAND**: Descend and land
5. **COMPLETE**: Mission complete

## Architecture

### Files Created

```
orion_arm/
├── src/px4_ros_com/
│   ├── src/examples/offboard_py/
│   │   └── circle_trajectory_node.py    # Main control node
│   ├── launch/
│   │   └── circle_trajectory.launch.py  # Launch file
│   └── config/
│       └── circle_trajectory.rviz       # RViz configuration
└── run_circle_demo.sh                   # Convenience script
```

### Node Communication

```
circle_trajectory_node
├── Subscribes:
│   ├── /fmu/out/vehicle_local_position  (drone state)
│   └── /fmu/out/vehicle_status          (flight mode)
└── Publishes:
    ├── /fmu/in/offboard_control_mode    (offboard heartbeat)
    ├── /fmu/in/trajectory_setpoint      (position commands)
    ├── /fmu/in/vehicle_command          (arm/disarm/mode)
    └── /trajectory_markers              (visualization)
```

## Troubleshooting

### Drone doesn't takeoff
- Ensure PX4 SITL is running and showing `pxh>` prompt
- Check that uXRCE-DDS bridge is active (PX4 should show client connected)
- Verify topics are being published: `ros2 topic list`

### RViz shows nothing
- Check that the fixed frame is set to "map"
- Enable the MarkerArray display
- Verify markers are being published: `ros2 topic echo /trajectory_markers`

### Build errors
- Ensure all dependencies are installed: `rosdep install --from-paths src --ignore-src -r -y`
- Clean build: `rm -rf build install log && colcon build`

### micro-ros-agent issues
- PX4 auto-starts uXRCE-DDS on UDP port 8888
- Usually micro-ros-agent is not needed if using default PX4 settings
- Try launching with `use_micro_ros_agent:=false`

## Customization

### Modify Circle Parameters
Edit the launch file or pass parameters at runtime to change flight characteristics.

### Change Visualization
Edit `config/circle_trajectory.rviz` to add more displays (TF tree, camera view, etc.)

### Extend Functionality
The `circle_trajectory_node.py` can be extended to:
- Add obstacle avoidance
- Implement different trajectory patterns (figure-8, square, etc.)
- Add waypoint navigation
- Include sensor data visualization

## Safety Notes

⚠️ **This is demonstration code for simulation only!**

Before using on real hardware:
- Add safety checks and geofencing
- Implement failsafe behaviors
- Test thoroughly in simulation
- Start with conservative parameters
- Always have manual override ready

## Technical Details

- **Coordinate Frame**: NED (North-East-Down) for PX4, converted to Z-up for RViz
- **Control Rate**: 10 Hz (PX4 requires minimum 2 Hz for offboard mode)
- **QoS Profile**: BEST_EFFORT reliability, TRANSIENT_LOCAL durability
- **ROS2 Version**: Humble
- **PX4 Version**: Compatible with uXRCE-DDS bridge

## Credits

Created for the Orion ARM project, November 13, 2025.
Based on PX4 offboard control examples.

## License

BSD 3-Clause (same as px4_ros_com package)

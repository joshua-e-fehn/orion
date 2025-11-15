# Orion Flight - Autonomous Drone Control Package

**Author:** Joshua Fehn  
**Email:** joshua.fehn@tum.de  
**Date:** November 15, 2025  
**License:** BSD-3-Clause

## Overview

`orion_flight` is a custom ROS2 package for autonomous drone flight control with PX4. It provides clean, minimal implementations of flight controllers without the bloat of example code.

## Features

- ✈️ **Circular Trajectory Flight**: Autonomous circular flight with configurable parameters
- 📊 **Real-time Visualization**: RViz2 integration with position trails
- 🎯 **Complete Autonomy**: Takeoff, flight, and landing automation
- 🚀 **Minimal Dependencies**: Only depends on `px4_msgs` and ROS2 core
- 🔧 **Easy Configuration**: Launch file parameters for quick adjustments

## Package Structure

```
orion_flight/
├── orion_flight/
│   ├── __init__.py
│   └── circle_trajectory_node.py    # Circular flight controller
├── launch/
│   └── circle_trajectory.launch.py  # Launch configuration
├── config/
│   └── circle_trajectory.rviz       # RViz visualization config
├── package.xml                      # Package dependencies
├── CMakeLists.txt                   # Build configuration
└── setup.py                         # Python package setup
```

## Dependencies

### Required:
- ROS2 Humble
- `px4_msgs` - PX4 message definitions
- `rclpy` - ROS2 Python client library
- `visualization_msgs` - For RViz markers
- `geometry_msgs` - For geometric data types

### Optional:
- `rviz2` - For visualization (recommended)
- `tf2_ros` - For coordinate frame transforms
- `micro-ros-agent` - If not using PX4's built-in uXRCE-DDS

## Installation

1. **Clone into your workspace:**
   ```bash
   cd ~/your_ros2_workspace/src
   # This package is already part of the orion repository
   ```

2. **Install dependencies:**
   ```bash
   cd ~/your_ros2_workspace
   rosdep install --from-paths src --ignore-src -r -y
   ```

3. **Build the package:**
   ```bash
   colcon build --packages-select orion_flight --symlink-install
   ```

4. **Source the workspace:**
   ```bash
   source install/setup.bash
   ```

## Usage

### Quick Start

Use the convenience script from the workspace root:
```bash
./run_circle_demo.sh
```

### Manual Launch

```bash
# Basic launch
ros2 launch orion_flight circle_trajectory.launch.py

# With custom parameters
ros2 launch orion_flight circle_trajectory.launch.py \
    circle_radius:=5.0 \
    flight_height:=8.0 \
    angular_velocity:=0.5 \
    num_circles:=3
```

### Launch Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `circle_radius` | float | 4.0 | Circle radius in meters |
| `flight_height` | float | 5.0 | Altitude in meters (positive, converted to NED) |
| `angular_velocity` | float | 0.3 | Angular velocity in rad/s |
| `num_circles` | int | 2 | Number of complete circles before landing |
| `trail_length` | int | 10 | Number of position markers in visualization |
| `use_micro_ros_agent` | bool | false | Auto-start micro-ros-agent |

## Nodes

### circle_trajectory_node.py

Autonomous circular trajectory flight controller.

**Subscribed Topics:**
- `/fmu/out/vehicle_local_position` (px4_msgs/VehicleLocalPosition)
- `/fmu/out/vehicle_status` (px4_msgs/VehicleStatus)
- `/fmu/out/vehicle_command_ack` (px4_msgs/VehicleCommandAck)

**Published Topics:**
- `/fmu/in/offboard_control_mode` (px4_msgs/OffboardControlMode)
- `/fmu/in/trajectory_setpoint` (px4_msgs/TrajectorySetpoint)
- `/fmu/in/vehicle_command` (px4_msgs/VehicleCommand)
- `/trajectory_markers` (visualization_msgs/MarkerArray)

**Parameters:**
- See Launch Parameters above

## Flight Phases

The node implements a state machine with the following phases:

1. **INIT** - Initialize offboard mode and send initial setpoints
2. **TAKEOFF** - Climb to target altitude
3. **CIRCLE** - Execute circular trajectory
4. **LAND** - Descend and land
5. **COMPLETE** - Mission complete

## Safety Notes

⚠️ **This code is for SIMULATION ONLY!**

Before using on real hardware:
- Implement comprehensive safety checks
- Add geofencing
- Include failsafe behaviors
- Test extensively in simulation
- Have manual override ready
- Follow local regulations

## Examples

### Example 1: Small Fast Circle
```bash
ros2 launch orion_flight circle_trajectory.launch.py \
    circle_radius:=2.0 \
    flight_height:=3.0 \
    angular_velocity:=0.8 \
    num_circles:=5
```

### Example 2: Large Slow Circle
```bash
ros2 launch orion_flight circle_trajectory.launch.py \
    circle_radius:=10.0 \
    flight_height:=8.0 \
    angular_velocity:=0.2 \
    num_circles:=1
```

## Troubleshooting

### Node doesn't start
- Ensure PX4 SITL is running: `make px4_sitl gz_x500`
- Check that workspace is built and sourced
- Verify dependencies: `rosdep check orion_flight`

### Drone doesn't arm
- Run safety disabler: `python3 disable_all_safety.py`
- Check PX4 parameters: `python3 px4_diagnostic.py`

### No visualization in RViz
- Ensure RViz fixed frame is set to "map"
- Check that markers are published: `ros2 topic echo /trajectory_markers`

## Contributing

This package is part of the Orion project. For contributions:
1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Submit a pull request

## License

BSD 3-Clause License - Same as PX4 and ROS2 core packages

## Credits

- **Orion ARM Team** - Package development
- **PX4 Development Team** - PX4 Autopilot and message definitions
- **ROS2 Community** - Framework and tools

## Related Packages

- `px4_msgs` - PX4 message definitions (required dependency)
- `px4_ros_com` - PX4 ROS2 examples (kept for reference)

## Version History

- **1.0.0** (2025-11-15) - Initial release
  - Circular trajectory flight
  - RViz visualization
  - Clean minimal implementation

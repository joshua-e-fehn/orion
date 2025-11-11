# Project Summary: Autonomous Drone Simulation

## Overview

This project provides a complete Docker-based autonomous drone simulation environment supporting both ARM64 and AMD64 architectures. The simulation includes ROS2 Humble, MAVROS, PX4 Autopilot (SITL), Gazebo Classic, and RViz2, with a pre-configured autonomous flight demo.

## What's Included

### Docker Infrastructure
✅ Multi-architecture Dockerfile (ARM64 and AMD64)
✅ Docker Compose configuration for easy deployment
✅ Optimized build with layer caching
✅ Proper environment setup and entrypoint script

### Software Stack
✅ ROS2 Humble (Robot Operating System 2)
✅ PX4 Autopilot v1.14.3 (SITL mode)
✅ MAVROS (MAVLink to ROS2 bridge)
✅ Gazebo Classic 11 (3D simulation)
✅ RViz2 (Visualization)

### Autonomous Flight Package
✅ Complete ROS2 package (`autonomous_drone`)
✅ Waypoint navigation node with state machine
✅ Offboard control implementation
✅ Launch files for full simulation
✅ RViz configuration for visualization

### Documentation
✅ README.md - Comprehensive user guide
✅ QUICKSTART.md - Quick start guide
✅ ARCHITECTURE.md - System architecture overview
✅ TESTING.md - Testing procedures
✅ TROUBLESHOOTING.md - Common issues and solutions

### Helper Scripts
✅ `build_images.sh` - Build both architecture images
✅ `run_simulation.sh` - One-command simulation launch
✅ `test_basic.sh` - Basic validation tests

### CI/CD
✅ GitHub Actions workflow for automated builds
✅ Multi-architecture build and test pipeline

## Quick Start

### 1. Build and Run (Easiest)
```bash
./run_simulation.sh
```

### 2. Using Docker Compose
```bash
xhost +local:docker
docker-compose up
```

### 3. Manual Build
```bash
# Build for your architecture
./build_images.sh

# Run manually
docker run -it --rm \
  --privileged \
  --network host \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  autonomous_drone_sim:amd64
```

## Autonomous Flight Demo

The included demo performs:

1. **Initialization** - Connect to PX4 and MAVROS
2. **Pre-flight** - Send initial setpoints
3. **Mode Change** - Switch to OFFBOARD mode
4. **Arming** - Arm the motors
5. **Flight** - Fly to waypoint (5m forward, 3m up)
6. **Hold** - Maintain position at waypoint

### Expected Behavior

- PX4 SITL starts in Gazebo
- Iris quadcopter appears in 3D world
- MAVROS connects to PX4
- Autonomous navigator takes control
- Drone arms and takes off
- Flies to predefined waypoint
- Holds position (hovering)

### Customization

To change the waypoint, edit `autonomous_drone/scripts/waypoint_navigator.py`:

```python
self.target_pose.pose.position.x = 5.0  # Forward (meters)
self.target_pose.pose.position.y = 0.0  # Left (meters)
self.target_pose.pose.position.z = 3.0  # Up (meters)
```

## Architecture

### System Components

```
Docker Container
├── ROS2 Humble
│   ├── Core framework
│   └── DDS communication
├── PX4 Autopilot (SITL)
│   ├── Flight controller
│   └── MAVLink interface
├── MAVROS
│   ├── MAVLink ↔ ROS2 bridge
│   └── Topic/Service interface
├── Gazebo Classic 11
│   ├── Physics simulation
│   └── Sensor simulation
├── RViz2
│   └── 3D visualization
└── Autonomous Drone Package
    ├── Waypoint navigator
    ├── Launch files
    └── Configuration
```

### Data Flow

```
Navigator → MAVROS → PX4 → Gazebo → Sensors → PX4 → MAVROS → Navigator
                                ↓
                              RViz
```

## File Structure

```
orion/
├── Dockerfile                          # Multi-arch Docker image
├── docker-compose.yml                  # Docker Compose config
├── entrypoint.sh                       # Container entrypoint
├── build_images.sh                     # Build script
├── run_simulation.sh                   # Run script
├── test_basic.sh                       # Test script
├── LICENSE                             # MIT License
├── README.md                           # Main documentation
├── QUICKSTART.md                       # Quick start guide
├── ARCHITECTURE.md                     # Architecture overview
├── TESTING.md                          # Testing guide
├── TROUBLESHOOTING.md                  # Troubleshooting guide
├── .github/
│   └── workflows/
│       └── docker-build.yml            # CI/CD workflow
└── autonomous_drone/                   # ROS2 package
    ├── package.xml                     # Package manifest
    ├── CMakeLists.txt                  # Build configuration
    ├── autonomous_drone/               # Python package
    │   └── __init__.py
    ├── scripts/                        # Executable scripts
    │   ├── waypoint_navigator.py       # Main navigator
    │   └── offboard_control.py         # Offboard control
    ├── launch/                         # Launch files
    │   ├── simulation.launch.py        # Full simulation
    │   └── mavros.launch.py            # MAVROS only
    └── config/                         # Configuration
        └── drone_view.rviz             # RViz config
```

## Technical Details

### Supported Architectures
- **AMD64** (x86_64) - Intel/AMD processors
- **ARM64** (aarch64) - ARM processors (Raspberry Pi 4, Apple Silicon, etc.)

### Software Versions
- ROS2: Humble Hawksbill
- PX4: v1.14.3
- Gazebo: Classic 11
- Ubuntu: 22.04 (via ROS2 base image)

### Resource Requirements
- **Minimum**: 4 cores, 8 GB RAM, 20 GB disk
- **Recommended**: 8 cores, 16 GB RAM, 30 GB disk, GPU

### Network Ports
- 14540: MAVROS → PX4 (UDP)
- 14557: PX4 → MAVROS (UDP)
- All communication is localhost

## Testing

### Basic Tests
```bash
./test_basic.sh
```

Tests include:
- Container starts
- ROS2 installation
- Workspace build
- MAVROS availability
- PX4 SITL binary
- Script permissions

### Full Simulation Test
```bash
./run_simulation.sh
```

Validates:
- X11 forwarding
- Gazebo rendering
- PX4 connection
- MAVROS bridge
- Autonomous navigation
- RViz visualization

## Common Use Cases

### 1. Development
Mount source code for live editing:
```bash
docker run -it --rm \
  -v $(pwd)/autonomous_drone:/root/ros2_ws/src/autonomous_drone \
  autonomous_drone_sim:amd64 bash
```

### 2. Headless Testing
Run without GUI for CI/CD:
```bash
docker run --rm \
  -e HEADLESS=1 \
  autonomous_drone_sim:amd64 \
  bash -c "cd /root/PX4-Autopilot && HEADLESS=1 make px4_sitl gazebo-classic"
```

### 3. Multi-Drone Simulation
Launch multiple instances with different IDs and ports.

## Next Steps

### Immediate
1. Build the Docker image
2. Run the basic simulation
3. Observe autonomous flight
4. Review logs and topics

### Enhancements
1. Add multiple waypoints
2. Implement path planning
3. Add obstacle avoidance
4. Integrate vision sensors
5. Test different vehicles
6. Deploy to real hardware

## Support & Contribution

### Documentation
- Main README for usage
- QUICKSTART for getting started
- ARCHITECTURE for design details
- TESTING for validation
- TROUBLESHOOTING for common issues

### Getting Help
1. Check TROUBLESHOOTING.md
2. Review logs
3. Enable debug output
4. Open GitHub issue

### Contributing
1. Fork the repository
2. Create feature branch
3. Make changes
4. Test thoroughly
5. Submit pull request

## License

MIT License - See LICENSE file for details

## Acknowledgments

- PX4 Autopilot Team
- MAVROS Maintainers
- ROS2 Community
- Gazebo Team

---

**Project Status**: ✅ Complete and Ready to Use

All components are integrated, tested, and documented. The simulation provides a solid foundation for autonomous drone development and can be easily extended for more complex scenarios.

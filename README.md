# Autonomous Drone Simulation

This repository provides Docker images for running autonomous drone simulations using ROS2, MAVROS, PX4, Gazebo, and RViz. The simulation includes a simple autonomous flight demo where the drone flies to a predefined waypoint and holds position.

## Features

- **Multi-architecture support**: ARM64 and AMD64 Docker images
- **Complete stack**: ROS2 Humble, MAVROS, PX4 SITL, Gazebo Classic, and RViz2
- **Autonomous flight**: Pre-configured waypoint navigation demo
- **Easy deployment**: Docker Compose setup for quick start
- **Visualization**: RViz2 configuration for monitoring drone state

## Prerequisites

- Docker (version 20.10 or later)
- Docker Buildx (for multi-architecture builds)
- X11 server (for GUI applications like Gazebo and RViz)
  - Linux: Already available
  - macOS: Install XQuartz
  - Windows: Install VcXsrv or Xming

## Quick Start

### Using Docker Compose (Recommended)

1. **Clone the repository**:
   ```bash
   git clone https://github.com/joshua-e-fehn/orion.git
   cd orion
   ```

2. **Allow X11 connections** (for GUI):
   ```bash
   xhost +local:docker
   ```

3. **Run the simulation**:
   ```bash
   docker-compose up
   ```

This will:
- Start the PX4 SITL (Software-in-the-Loop) simulation
- Launch Gazebo with the Iris quadcopter
- Start MAVROS for ROS2-PX4 communication
- Run the autonomous waypoint navigator
- Open RViz2 for visualization

### Manual Docker Build and Run

1. **Build the Docker image** for your architecture:

   For AMD64:
   ```bash
   docker buildx build --platform linux/amd64 -t autonomous_drone_sim:amd64 --load .
   ```

   For ARM64:
   ```bash
   docker buildx build --platform linux/arm64 -t autonomous_drone_sim:arm64 --load .
   ```

   Or use the build script:
   ```bash
   ./build_images.sh
   ```

2. **Run the container**:

   For AMD64:
   ```bash
   docker run -it --rm \
     --privileged \
     --network host \
     -e DISPLAY=$DISPLAY \
     -v /tmp/.X11-unix:/tmp/.X11-unix \
     autonomous_drone_sim:amd64
   ```

   For ARM64:
   ```bash
   docker run -it --rm \
     --privileged \
     --network host \
     -e DISPLAY=$DISPLAY \
     -v /tmp/.X11-unix:/tmp/.X11-unix \
     autonomous_drone_sim:arm64
   ```

3. **Inside the container, run the simulation**:
   ```bash
   ros2 launch autonomous_drone simulation.launch.py
   ```

## Architecture

### Components

- **PX4 Autopilot**: Flight controller software running in SITL mode
- **Gazebo Classic**: 3D robotics simulator
- **MAVROS**: MAVLink to ROS2 bridge
- **RViz2**: 3D visualization tool
- **Autonomous Drone Package**: Custom ROS2 package for waypoint navigation

### Autonomous Flight Demo

The included demo performs the following sequence:

1. **Connection**: Waits for PX4 SITL to connect
2. **Initialization**: Sends initial setpoints (required by PX4)
3. **Mode Change**: Switches to OFFBOARD mode
4. **Arming**: Arms the drone motors
5. **Navigation**: Flies to waypoint (5m forward, 3m up)
6. **Hold**: Maintains position at the waypoint

The target waypoint can be modified in `autonomous_drone/scripts/waypoint_navigator.py`:
```python
self.target_pose.pose.position.x = 5.0  # Forward (meters)
self.target_pose.pose.position.y = 0.0  # Left (meters)
self.target_pose.pose.position.z = 3.0  # Up (meters)
```

## Project Structure

```
orion/
├── Dockerfile                          # Multi-arch Docker image
├── docker-compose.yml                  # Docker Compose configuration
├── entrypoint.sh                       # Container entrypoint script
├── build_images.sh                     # Build script for both architectures
├── autonomous_drone/                   # ROS2 package
│   ├── package.xml                     # Package manifest
│   ├── CMakeLists.txt                  # Build configuration
│   ├── autonomous_drone/               # Python package
│   │   └── __init__.py
│   ├── scripts/                        # Executable scripts
│   │   ├── waypoint_navigator.py       # Main autonomous navigation node
│   │   └── offboard_control.py         # Basic offboard control
│   ├── launch/                         # Launch files
│   │   ├── simulation.launch.py        # Full simulation launch
│   │   └── mavros.launch.py            # MAVROS only launch
│   └── config/                         # Configuration files
│       └── drone_view.rviz             # RViz configuration
└── README.md                           # This file
```

## Advanced Usage

### Running Components Separately

1. **Start PX4 SITL**:
   ```bash
   cd /root/PX4-Autopilot
   HEADLESS=1 make px4_sitl gazebo-classic
   ```

2. **In another terminal, start MAVROS**:
   ```bash
   ros2 launch autonomous_drone mavros.launch.py
   ```

3. **In another terminal, run the navigator**:
   ```bash
   ros2 run autonomous_drone waypoint_navigator.py
   ```

4. **In another terminal, start RViz**:
   ```bash
   ros2 run rviz2 rviz2 -d /root/ros2_ws/src/autonomous_drone/config/drone_view.rviz
   ```

### Monitoring Topics

View available ROS2 topics:
```bash
ros2 topic list
```

Monitor drone state:
```bash
ros2 topic echo /mavros/state
```

Monitor local position:
```bash
ros2 topic echo /mavros/local_position/pose
```

## Customization

### Modifying the Waypoint

Edit `autonomous_drone/scripts/waypoint_navigator.py` and change the target position:
```python
self.target_pose.pose.position.x = 10.0  # Your desired X
self.target_pose.pose.position.y = 5.0   # Your desired Y
self.target_pose.pose.position.z = 5.0   # Your desired Z
```

### Changing the Vehicle

Modify the launch file or pass as argument:
```bash
ros2 launch autonomous_drone simulation.launch.py vehicle:=plane
```

Supported vehicles: iris, iris_opt_flow, iris_vision, typhoon_h480, plane, standard_vtol, etc.

### Using Different Gazebo Worlds

```bash
ros2 launch autonomous_drone simulation.launch.py world:=windy
```

## Troubleshooting

### Gazebo/RViz Not Displaying

Ensure X11 forwarding is enabled:
```bash
xhost +local:docker
```

For macOS, ensure XQuartz is running and:
```bash
xhost + 127.0.0.1
export DISPLAY=:0
```

### PX4 Connection Issues

Check if MAVROS is connected:
```bash
ros2 topic echo /mavros/state
```

The `connected` field should be `true`.

### Build Errors

Make sure Docker Buildx is installed:
```bash
docker buildx version
```

If not available, install it:
```bash
docker buildx create --use
```

### Container Permission Issues

The container needs privileged mode for hardware access:
```bash
docker run --privileged ...
```

## References

- [PX4 Autopilot](https://px4.io/)
- [MAVROS](https://github.com/mavlink/mavros)
- [ROS2 Humble](https://docs.ros.org/en/humble/)
- [Gazebo](https://gazebosim.org/)

## License

MIT License - See LICENSE file for details

## Contributing

Contributions are welcome! Please open an issue or submit a pull request.

## Support

For issues and questions, please open an issue on GitHub.

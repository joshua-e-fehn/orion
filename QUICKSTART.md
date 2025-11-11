# Quick Start Guide - Autonomous Drone Simulation

## Installation

### Option 1: Using the Quick Run Script (Easiest)

1. Clone the repository:
   ```bash
   git clone https://github.com/joshua-e-fehn/orion.git
   cd orion
   ```

2. Run the simulation:
   ```bash
   ./run_simulation.sh
   ```

The script will:
- Detect your architecture (ARM64 or AMD64)
- Build the Docker image if needed
- Enable X11 forwarding
- Start the complete simulation

### Option 2: Using Docker Compose

1. Clone the repository:
   ```bash
   git clone https://github.com/joshua-e-fehn/orion.git
   cd orion
   ```

2. Enable X11 forwarding:
   ```bash
   xhost +local:docker
   ```

3. Start the simulation:
   ```bash
   docker-compose up
   ```

### Option 3: Manual Build and Run

See the main README.md for detailed instructions.

## What You'll See

When you run the simulation, the following will happen:

1. **Terminal Output**: You'll see logs from:
   - PX4 SITL starting up
   - MAVROS connecting to PX4
   - The waypoint navigator initializing
   - Flight status updates

2. **Gazebo Window**: 3D simulation showing:
   - The Iris quadcopter drone
   - The simulated environment
   - Real-time physics simulation

3. **RViz Window**: Visualization showing:
   - TF frames
   - Drone position and orientation
   - Any additional visualization markers

## Flight Sequence

The autonomous flight demo executes these steps:

1. **Startup** (0-10 seconds)
   - Connect to PX4
   - Initialize MAVROS
   - Wait for FCU connection

2. **Pre-flight** (10-15 seconds)
   - Send initial setpoints
   - Switch to OFFBOARD mode
   - Arm the motors

3. **Flight** (15-40 seconds)
   - Take off
   - Fly to waypoint (5m forward, 3m up)
   - Slow down as it approaches

4. **Hold** (40+ seconds)
   - Maintain position at waypoint
   - Continue holding until stopped

## Customizing the Waypoint

To change where the drone flies, edit the file:
`autonomous_drone/scripts/waypoint_navigator.py`

Find these lines (around line 68):
```python
self.target_pose.pose.position.x = 5.0  # Forward (meters)
self.target_pose.pose.position.y = 0.0  # Left (meters)  
self.target_pose.pose.position.z = 3.0  # Up (meters)
```

Change the values to your desired coordinates and rebuild the Docker image.

## Monitoring the Simulation

### Check Connection Status

In a new terminal, enter the container:
```bash
docker exec -it drone_sim bash
```

Check if MAVROS is connected:
```bash
ros2 topic echo /mavros/state
```

You should see `connected: true`

### Monitor Position

View the current drone position:
```bash
ros2 topic echo /mavros/local_position/pose
```

### List All Topics

See all available ROS2 topics:
```bash
ros2 topic list
```

## Stopping the Simulation

Press `Ctrl+C` in the terminal where the simulation is running.

## Troubleshooting

### No GUI Windows Appear

Make sure X11 forwarding is enabled:
```bash
xhost +local:docker
```

For macOS with XQuartz:
```bash
xhost + 127.0.0.1
export DISPLAY=:0
```

### PX4 Won't Connect

Wait a bit longer - PX4 SITL can take 10-20 seconds to fully initialize.

Check the logs for any error messages.

### Build Fails

Make sure you have Docker Buildx:
```bash
docker buildx version
```

If not installed:
```bash
docker buildx create --use
```

### Performance Issues

The simulation is resource-intensive. Recommended minimum:
- 4 CPU cores
- 8 GB RAM
- GPU acceleration (optional but helps)

## Next Steps

Once you've successfully run the basic demo:

1. Modify waypoint coordinates
2. Add multiple waypoints
3. Implement path planning
4. Add obstacle avoidance
5. Integrate with vision sensors
6. Test different vehicle types

See the main README.md for more advanced usage and customization options.

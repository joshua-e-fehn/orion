# Testing the Autonomous Drone Simulation

This document provides guidance on testing and validating the autonomous drone simulation.

## Pre-flight Checks

Before running the simulation, ensure:

1. **Docker is installed and running**:
   ```bash
   docker --version
   docker info
   ```

2. **Docker Buildx is available** (for multi-arch builds):
   ```bash
   docker buildx version
   ```

3. **X11 server is running** (Linux/macOS):
   ```bash
   echo $DISPLAY
   ```
   Should return something like `:0` or `:1`

4. **Sufficient resources**:
   - At least 4 CPU cores
   - At least 8 GB RAM
   - At least 20 GB free disk space

## Building the Image

### Test AMD64 Build

```bash
docker buildx build --platform linux/amd64 -t autonomous_drone_sim:amd64 --load .
```

Expected outcome:
- Build completes without errors
- Image size is approximately 5-8 GB
- All layers cached for subsequent builds

### Test ARM64 Build

```bash
docker buildx build --platform linux/arm64 -t autonomous_drone_sim:arm64 --load .
```

Expected outcome:
- Build completes without errors (may take longer on non-ARM hosts)
- Image size is approximately 5-8 GB
- Cross-compilation works correctly

## Running Basic Tests

### Test 1: Container Starts

```bash
docker run --rm autonomous_drone_sim:amd64 echo "Container works"
```

Expected output: `Container works`

### Test 2: ROS2 Environment

```bash
docker run --rm autonomous_drone_sim:amd64 bash -c "source /opt/ros/humble/setup.bash && ros2 --version"
```

Expected output: ROS2 version information

### Test 3: Workspace Built

```bash
docker run --rm autonomous_drone_sim:amd64 bash -c "source /root/ros2_ws/install/setup.bash && ros2 pkg list | grep autonomous_drone"
```

Expected output: `autonomous_drone`

### Test 4: MAVROS Available

```bash
docker run --rm autonomous_drone_sim:amd64 bash -c "source /opt/ros/humble/setup.bash && ros2 pkg list | grep mavros"
```

Expected output: List of MAVROS packages

### Test 5: PX4 SITL Available

```bash
docker run --rm autonomous_drone_sim:amd64 bash -c "ls /root/PX4-Autopilot/build/px4_sitl_default/bin/px4"
```

Expected output: Path to px4 binary

## Running the Full Simulation

### Test 6: Headless Simulation (No GUI)

For testing without display:

```bash
docker run --rm \
  -e HEADLESS=1 \
  autonomous_drone_sim:amd64 \
  bash -c "
    source /opt/ros/humble/setup.bash && \
    source /root/ros2_ws/install/setup.bash && \
    cd /root/PX4-Autopilot && \
    HEADLESS=1 make px4_sitl_default gazebo-classic &
    sleep 10 && \
    ros2 run mavros mavros_node --ros-args -p fcu_url:=udp://:14540@127.0.0.1:14557 &
    sleep 5 && \
    echo 'Simulation started successfully'
  "
```

Expected behavior:
- PX4 SITL starts
- MAVROS connects
- No errors in logs

### Test 7: Full Simulation with GUI

```bash
xhost +local:docker
docker run -it --rm \
  --privileged \
  --network host \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  autonomous_drone_sim:amd64 \
  bash -c "
    source /opt/ros/humble/setup.bash && \
    source /root/ros2_ws/install/setup.bash && \
    ros2 launch autonomous_drone simulation.launch.py
  "
```

Expected behavior:
- Gazebo window opens showing quadcopter
- RViz window opens with visualization
- Terminal shows connection and flight logs
- Drone arms and flies to waypoint
- No crashes or errors

## Validation Checklist

### PX4 SITL
- [ ] PX4 SITL starts without errors
- [ ] Gazebo Classic launches and displays the Iris quadcopter
- [ ] Vehicle is visible in Gazebo
- [ ] Sensor data is being published

### MAVROS Connection
- [ ] MAVROS connects to PX4 (check `/mavros/state` topic)
- [ ] `connected: true` in state message
- [ ] No connection timeout errors
- [ ] MAVLink messages are flowing

### ROS2 Topics
- [ ] All expected topics are published
- [ ] `/mavros/state` shows connection
- [ ] `/mavros/local_position/pose` is updating
- [ ] `/mavros/setpoint_position/local` is being published

### Autonomous Flight
- [ ] Navigator node starts successfully
- [ ] Mode switches to OFFBOARD
- [ ] Vehicle arms
- [ ] Drone takes off
- [ ] Drone flies to waypoint (5m forward, 3m up)
- [ ] Drone holds position at waypoint
- [ ] No crashes or unexpected behavior

### Visualization
- [ ] RViz opens successfully
- [ ] TF frames are visible
- [ ] Grid is displayed
- [ ] Visualization updates in real-time

## Common Issues and Solutions

### Issue: Gazebo doesn't start

**Solution**: Ensure X11 forwarding is enabled:
```bash
xhost +local:docker
```

### Issue: MAVROS doesn't connect

**Solution**: Wait longer for PX4 to fully initialize (10-20 seconds)

### Issue: Build fails on ARM64

**Solution**: Ensure QEMU is installed for cross-compilation:
```bash
docker run --privileged --rm tonistiigi/binfmt --install all
```

### Issue: Out of memory during build

**Solution**: Increase Docker memory limit in Docker Desktop settings or use:
```bash
docker buildx build --memory 8g ...
```

### Issue: Container crashes immediately

**Solution**: Check logs:
```bash
docker logs <container_id>
```

## Performance Metrics

Expected performance on recommended hardware:

- **Build time**: 15-30 minutes (first build)
- **Container startup**: 5-10 seconds
- **PX4 SITL startup**: 10-15 seconds
- **MAVROS connection**: 5-10 seconds
- **Total time to flight**: ~30 seconds
- **Gazebo FPS**: 30-60 (with GPU acceleration)

## Automated Testing

For CI/CD pipelines, you can run headless tests:

```bash
#!/bin/bash
# test_simulation.sh

set -e

echo "Building image..."
docker buildx build --platform linux/amd64 -t autonomous_drone_sim:test --load .

echo "Testing ROS2 installation..."
docker run --rm autonomous_drone_sim:test bash -c "source /opt/ros/humble/setup.bash && ros2 --version"

echo "Testing workspace..."
docker run --rm autonomous_drone_sim:test bash -c "source /root/ros2_ws/install/setup.bash && ros2 pkg list | grep autonomous_drone"

echo "Testing PX4..."
docker run --rm autonomous_drone_sim:test ls /root/PX4-Autopilot/build/px4_sitl_default/bin/px4

echo "All tests passed!"
```

## Next Steps

After successful testing:

1. Tag and push images to a registry (optional)
2. Document any architecture-specific issues
3. Optimize performance if needed
4. Add more complex flight scenarios
5. Integrate with CI/CD pipeline

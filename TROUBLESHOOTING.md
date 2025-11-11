# Troubleshooting Guide

Common issues and solutions for the autonomous drone simulation.

## Build Issues

### Problem: Docker build fails with "executor failed running"

**Cause**: Insufficient memory or disk space

**Solution**:
```bash
# Check available space
df -h

# Clean up Docker
docker system prune -a

# Increase Docker memory (Docker Desktop)
# Settings > Resources > Memory > Set to at least 8GB
```

### Problem: "failed to solve with frontend dockerfile.v0"

**Cause**: Docker Buildx not properly configured

**Solution**:
```bash
# Install/update buildx
docker buildx create --use --name multiarch
docker buildx inspect --bootstrap
```

### Problem: ARM64 build fails with QEMU errors

**Cause**: QEMU not installed or outdated

**Solution**:
```bash
# Install QEMU binfmt
docker run --privileged --rm tonistiigi/binfmt --install all

# Verify
docker buildx ls
```

### Problem: PX4 build fails inside Docker

**Cause**: Missing dependencies or network issues

**Solution**:
```bash
# Build with verbose output
docker buildx build --progress=plain --no-cache .

# Check for specific error messages
```

## Runtime Issues

### Problem: "Cannot connect to X server"

**Cause**: X11 forwarding not enabled

**Solution**:
```bash
# Linux
xhost +local:docker

# macOS (with XQuartz)
xhost + 127.0.0.1
export DISPLAY=:0

# WSL2
export DISPLAY=$(cat /etc/resolv.conf | grep nameserver | awk '{print $2}'):0
```

### Problem: Gazebo window is black or frozen

**Cause**: GPU acceleration issues

**Solution**:
```bash
# Disable GPU acceleration
docker run -it --rm \
  -e DISPLAY=$DISPLAY \
  -e LIBGL_ALWAYS_SOFTWARE=1 \
  ...

# Or use software rendering
export LIBGL_ALWAYS_INDIRECT=1
```

### Problem: "Failed to connect to MAVROS"

**Cause**: PX4 SITL not fully started

**Solution**:
- Wait 15-30 seconds for PX4 to fully initialize
- Check PX4 is running: `ps aux | grep px4`
- Verify port 14540 is listening: `netstat -an | grep 14540`

### Problem: Container exits immediately

**Cause**: Entrypoint script error

**Solution**:
```bash
# Run with shell to debug
docker run -it --rm \
  --entrypoint /bin/bash \
  autonomous_drone_sim:amd64

# Inside container, manually run commands
source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash
```

### Problem: "No module named 'rclpy'"

**Cause**: ROS2 environment not sourced

**Solution**:
```bash
# Always source both setup files
source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash
```

## Simulation Issues

### Problem: Drone doesn't arm

**Possible Causes**:
1. Not in OFFBOARD mode
2. No setpoint received
3. Safety checks failing

**Solution**:
```bash
# Check state
ros2 topic echo /mavros/state

# Check if setpoints are being published
ros2 topic hz /mavros/setpoint_position/local

# Manual arming (for testing)
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}"
```

### Problem: Drone arms but doesn't take off

**Cause**: Setpoint not high enough or mode issues

**Solution**:
- Check setpoint z-value is > 0 (altitude)
- Verify OFFBOARD mode is active
- Check logs for errors: `ros2 node list` and `ros2 topic echo /rosout`

### Problem: Drone flies erratically

**Cause**: Incorrect coordinate frame or noisy setpoints

**Solution**:
- Verify frame_id is "map" in setpoint messages
- Check setpoint publishing rate (should be >2Hz, recommended 20Hz)
- Reduce aggressive position changes

### Problem: Gazebo crashes

**Cause**: GPU issues or insufficient resources

**Solution**:
```bash
# Check Gazebo logs
cat ~/.gazebo/server.log

# Run headless (no GUI)
HEADLESS=1 make px4_sitl gazebo-classic

# Reduce physics update rate in Gazebo
```

## Performance Issues

### Problem: Simulation runs slowly

**Solutions**:
- Close other applications
- Disable real-time factor in Gazebo
- Use headless mode
- Reduce sensor update rates
- Use lighter Gazebo world

### Problem: High CPU usage

**Solutions**:
```bash
# Limit CPU cores
docker run --cpus=4 ...

# Reduce Gazebo physics rate
# Edit world file: max_step_size and real_time_update_rate
```

### Problem: High memory usage

**Solution**:
```bash
# Limit memory
docker run --memory=8g ...

# Monitor usage
docker stats
```

## Network Issues

### Problem: "Connection refused" on MAVROS

**Cause**: Wrong port or PX4 not listening

**Solution**:
```bash
# Check PX4 UDP ports
netstat -an | grep 145

# Verify MAVROS configuration
ros2 param list /mavros

# Test with different URL
ros2 run mavros mavros_node --ros-args -p fcu_url:=udp://:14540@localhost:14557
```

### Problem: Topics not visible

**Cause**: ROS_DOMAIN_ID mismatch

**Solution**:
```bash
# Ensure same ROS_DOMAIN_ID
export ROS_DOMAIN_ID=0

# Check DDS discovery
ros2 daemon stop
ros2 daemon start
```

## Development Issues

### Problem: Changes not reflected in container

**Cause**: Image not rebuilt

**Solution**:
```bash
# Rebuild without cache
docker buildx build --no-cache -t autonomous_drone_sim:amd64 --load .

# Or just rebuild the workspace
docker run -it --rm \
  -v $(pwd)/autonomous_drone:/root/ros2_ws/src/autonomous_drone \
  autonomous_drone_sim:amd64 \
  bash -c "cd /root/ros2_ws && colcon build --symlink-install"
```

### Problem: Python script changes not working

**Cause**: Not using symlink install

**Solution**:
```bash
# Always use --symlink-install
colcon build --symlink-install

# Or mount source directory
docker run -v $(pwd)/autonomous_drone:/root/ros2_ws/src/autonomous_drone ...
```

## Diagnostic Commands

### Check ROS2 Topics
```bash
ros2 topic list
ros2 topic echo /mavros/state
ros2 topic hz /mavros/local_position/pose
```

### Check ROS2 Nodes
```bash
ros2 node list
ros2 node info /mavros
```

### Check Services
```bash
ros2 service list
ros2 service type /mavros/cmd/arming
```

### Check Parameters
```bash
ros2 param list /mavros
ros2 param get /mavros fcu_url
```

### Monitor System
```bash
# Inside container
htop
nvidia-smi  # if GPU available

# Outside container
docker stats
```

## Getting Help

If you're still having issues:

1. Check the logs:
   ```bash
   ros2 launch autonomous_drone simulation.launch.py 2>&1 | tee simulation.log
   ```

2. Enable debug logging:
   ```bash
   export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}] [{name}]: {message}"
   export RCUTILS_COLORIZED_OUTPUT=1
   ```

3. Check PX4 logs:
   ```bash
   ls -la /root/PX4-Autopilot/build/px4_sitl_default/logs/
   ```

4. Create an issue on GitHub with:
   - Your system information (OS, Docker version, architecture)
   - Complete error messages
   - Steps to reproduce
   - Relevant log files

## Useful References

- [PX4 Documentation](https://docs.px4.io/)
- [MAVROS Documentation](https://github.com/mavlink/mavros/tree/ros2/mavros)
- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)
- [Gazebo Documentation](http://gazebosim.org/tutorials)
- [Docker Documentation](https://docs.docker.com/)

# Architecture Overview

## System Components

This autonomous drone simulation consists of several integrated components:

```
┌─────────────────────────────────────────────────────────┐
│                    Docker Container                      │
│                                                          │
│  ┌────────────┐  ┌──────────┐  ┌──────────┐            │
│  │  ROS2      │  │ MAVROS   │  │ Waypoint │            │
│  │  Humble    │◄─┤ Bridge   │◄─┤ Navigator│            │
│  └────────────┘  └──────────┘  └──────────┘            │
│        ▲              ▲                                  │
│        │              │ MAVLink                          │
│        │              ▼                                  │
│  ┌────────────┐  ┌──────────┐                          │
│  │  RViz2     │  │   PX4    │                          │
│  │Visualization│  │   SITL   │                          │
│  └────────────┘  └──────────┘                          │
│        ▲              │                                  │
│        │              ▼                                  │
│  ┌────────────────────────────┐                         │
│  │    Gazebo Classic 11       │                         │
│  │   (Physics & Rendering)    │                         │
│  └────────────────────────────┘                         │
│                                                          │
└─────────────────────────────────────────────────────────┘
```

## Component Details

### 1. ROS2 Humble
- **Purpose**: Robot Operating System framework
- **Role**: Message passing, service calls, lifecycle management
- **Key Features**:
  - DDS-based communication
  - Quality of Service (QoS) policies
  - Launch system for orchestration

### 2. PX4 Autopilot (SITL)
- **Purpose**: Flight controller software
- **Mode**: Software-in-the-Loop simulation
- **Role**: 
  - Flight control algorithms
  - Sensor fusion
  - Motor mixing
  - Safety features
- **Communication**: MAVLink protocol over UDP

### 3. MAVROS
- **Purpose**: MAVLink to ROS2 bridge
- **Role**: 
  - Translates MAVLink messages to ROS2 topics/services
  - Provides ROS2 interface to PX4
  - Handles different coordinate frames
- **Key Topics**:
  - `/mavros/state` - FCU connection state
  - `/mavros/local_position/pose` - Current position
  - `/mavros/setpoint_position/local` - Desired position
- **Key Services**:
  - `/mavros/cmd/arming` - Arm/disarm motors
  - `/mavros/set_mode` - Change flight mode

### 4. Gazebo Classic 11
- **Purpose**: 3D robotics simulator
- **Role**:
  - Physics simulation (ODE engine)
  - Sensor simulation (IMU, GPS, cameras)
  - Rendering
  - Environmental effects
- **Models**: Iris quadcopter, worlds, obstacles

### 5. RViz2
- **Purpose**: 3D visualization
- **Role**:
  - Visualize robot state
  - Display TF frames
  - Show sensor data
  - Monitor topics

### 6. Waypoint Navigator (Custom)
- **Purpose**: Autonomous navigation logic
- **Role**:
  - Mission planning
  - Flight mode management
  - Position control
  - State machine

## Data Flow

### Startup Sequence

1. **Container Start**
   ```
   Docker → Entrypoint Script → Source ROS2 Environment
   ```

2. **Launch Sequence**
   ```
   Launch File → PX4 SITL → Gazebo → MAVROS → Navigator → RViz
   ```

3. **Connection Establishment**
   ```
   PX4 (UDP:14557) ←→ MAVROS (UDP:14540) → ROS2 Topics
   ```

### Flight Control Loop

```
1. Navigator publishes setpoint
   ↓
2. MAVROS converts to MAVLink
   ↓
3. PX4 receives setpoint
   ↓
4. PX4 runs control algorithms
   ↓
5. PX4 sends actuator commands
   ↓
6. Gazebo simulates physics
   ↓
7. Gazebo updates sensors
   ↓
8. PX4 reads sensor data
   ↓
9. MAVROS publishes state to ROS2
   ↓
10. Navigator reads state
    ↓
    (Loop back to 1)
```

### Visualization Loop

```
Gazebo → Physics State → ROS2 TF → RViz Display
```

## Coordinate Frames

### PX4 Frames
- **FRD** (Forward-Right-Down): Body frame
- **NED** (North-East-Down): Local frame
- **LLA** (Latitude-Longitude-Altitude): Global frame

### ROS2 Frames
- **base_link**: Robot body
- **map**: Fixed world frame
- **odom**: Odometry frame

### Transformations
MAVROS handles conversions between PX4 (NED) and ROS2 (ENU - East-North-Up)

## Communication Protocols

### MAVLink
- **Version**: MAVLink 2.0
- **Transport**: UDP
- **Messages**:
  - HEARTBEAT
  - POSITION_TARGET_LOCAL_NED
  - LOCAL_POSITION_NED
  - COMMAND_LONG
  - SET_MODE

### ROS2 DDS
- **Implementation**: Fast-DDS
- **QoS Policies**:
  - Reliability: Best Effort (for sensor data)
  - Durability: Transient Local
  - History: Keep Last

## Network Architecture

```
┌──────────────┐     UDP 14540      ┌──────────┐
│   MAVROS     │ ← ← ← ← ← ← ← ← ← ─│   PX4    │
│   (ROS2)     │                     │  SITL    │
│              │ ─ → → → → → → → → →│          │
└──────────────┘     UDP 14557      └──────────┘
       ↕
    DDS/RTC
       ↕
┌──────────────┐
│  Navigator   │
│    Node      │
└──────────────┘
```

## File Structure

```
/root/
├── PX4-Autopilot/              # PX4 source and builds
│   ├── build/
│   │   └── px4_sitl_default/   # SITL build
│   └── Tools/
│       └── simulation/         # Gazebo models & worlds
└── ros2_ws/                    # ROS2 workspace
    ├── src/
    │   ├── mavros/             # MAVROS source
    │   └── autonomous_drone/   # Custom package
    └── install/                # Built packages
```

## Environment Variables

Key environment variables:

- `ROS_DISTRO=humble` - ROS2 distribution
- `WORKSPACE=/root/ros2_ws` - ROS2 workspace path
- `PX4_HOME=/root/PX4-Autopilot` - PX4 installation
- `GAZEBO_MODEL_PATH` - Path to Gazebo models
- `GAZEBO_PLUGIN_PATH` - Path to Gazebo plugins

## Resource Requirements

### Minimum
- CPU: 4 cores
- RAM: 8 GB
- Disk: 20 GB
- Network: Localhost only

### Recommended
- CPU: 8 cores (for better real-time performance)
- RAM: 16 GB
- Disk: 30 GB (with caching)
- GPU: For Gazebo rendering acceleration

## Performance Characteristics

### Update Rates
- PX4 Loop: 250 Hz
- Gazebo Physics: 100 Hz
- MAVROS: 20-50 Hz
- Navigator: 20 Hz
- RViz: 30 Hz

### Latencies
- MAVROS ↔ PX4: <10 ms
- Setpoint to Motor: <50 ms
- End-to-end: <100 ms

## Security Considerations

- Container runs in privileged mode (for hardware access)
- Network mode: host (for UDP communication)
- X11 forwarding (for GUI)
- No secrets or credentials stored
- Localhost-only communication

## Extensibility Points

### Adding Sensors
1. Add to Gazebo model (SDF/URDF)
2. Configure PX4 sensor drivers
3. Create ROS2 publisher in MAVROS
4. Subscribe in custom nodes

### Custom Flight Modes
1. Implement in waypoint_navigator.py
2. Use existing MAVROS services
3. Follow PX4 state machine

### Multi-Vehicle
1. Launch multiple PX4 instances
2. Configure different MAVLink IDs
3. Separate MAVROS instances
4. Unique ROS2 namespaces

### Vision/Sensors
1. Add camera plugins to Gazebo
2. Use ROS2 camera drivers
3. Integrate with computer vision nodes
4. Feed to navigator

## Testing Strategy

### Unit Tests
- Python node logic
- Service call handlers
- State machine transitions

### Integration Tests
- MAVROS ↔ PX4 communication
- Topic publication/subscription
- Service availability

### System Tests
- Full mission execution
- Multiple waypoints
- Error recovery
- Performance benchmarks

## Monitoring & Debugging

### Logs
- PX4: `/root/PX4-Autopilot/build/px4_sitl_default/logs/`
- ROS2: `ros2 topic echo /rosout`
- Gazebo: `~/.gazebo/server.log`

### Topics to Monitor
- `/mavros/state` - Connection status
- `/mavros/local_position/pose` - Position
- `/mavros/setpoint_position/local` - Commands
- `/rosout` - ROS2 logs

### Common Metrics
- Connection uptime
- Position error
- Control loop frequency
- CPU/Memory usage

## Future Enhancements

Potential improvements:

1. **Multi-waypoint missions** - Chain of waypoints
2. **Obstacle avoidance** - Using sensors
3. **Path planning** - Optimal trajectory
4. **Vision-based navigation** - Camera integration
5. **SLAM** - Simultaneous Localization and Mapping
6. **Swarm coordination** - Multiple drones
7. **Real hardware testing** - Deploy to physical drone
8. **Web interface** - Browser-based control

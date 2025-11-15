# Guidance & Planning Framework

## Overview

This directory contains the guidance and planning framework for the Orion drone interception system. All planners follow a common interface to enable seamless switching between different guidance algorithms and support for comparative evaluation.

The planning framework implements guidance laws from the paper **"Towards Safe Mid-Air Drone Interception: Strategies for Tracking & Capture"** (Pliska et al., IEEE RA-L 2024), including:
- Pure Pursuit (PP) - baseline
- Proportional Navigation (PN) - canonical missile guidance
- Linearized Proportional Navigation (LPN) - Cartesian formulation
- General Proportional Navigation (GPN) - variant with gain scheduling
- Fast Response Proportional Navigation (FRPN) - **recommended**, paper's main contribution
- Model Predictive Control (MPC) - constraint-aware optimal control

## Directory Structure

```
planners/
├── PLANNER_README.md                    # This file
├── PLANNER_IMPLEMENTATION_GUIDE.md      # Guide for implementing new planners
├── common/
│   ├── __init__.py                      # Package initialization
│   ├── planner_base.py                  # Abstract base class for all planners
│   ├── types.py                         # Common data types and structures
│   ├── utils.py                         # Shared utilities (norm, clamp, coordinate transforms)
│   └── visualization.py                 # Shared visualization utilities
├── pp/
│   ├── __init__.py
│   ├── pp_planner_node.py              # Pure Pursuit planner ROS2 node
│   └── pp_algorithm.py                  # PP guidance law implementation
├── pn/
│   ├── __init__.py
│   ├── pn_planner_node.py              # Proportional Navigation planner ROS2 node
│   └── pn_algorithm.py                  # PN guidance law implementation
├── lpn/
│   ├── __init__.py
│   ├── lpn_planner_node.py             # Linearized PN planner ROS2 node
│   └── lpn_algorithm.py                 # LPN guidance law implementation
├── gpn/
│   ├── __init__.py
│   ├── gpn_planner_node.py             # General PN planner ROS2 node
│   └── gpn_algorithm.py                 # GPN guidance law implementation
├── frpn/
│   ├── __init__.py
│   ├── frpn_planner_node.py            # Fast Response PN planner ROS2 node (RECOMMENDED)
│   └── frpn_algorithm.py                # FRPN guidance law implementation
└── mpc/
    ├── __init__.py
    ├── mpc_planner_node.py             # MPC planner ROS2 node
    └── mpc_solver.py                    # MPC problem formulation and solver
```

## Common Interface

All planners implement the `PlannerBase` abstract class which defines:

### Input Interface

**ROS2 Subscriptions:**

1. **Interceptor State** (ego drone)
   - **Topic**: `/{interceptor_namespace}/fmu/out/vehicle_local_position`
   - **Type**: `px4_msgs.msg.VehicleLocalPosition`
   - **Rate**: ~50 Hz (from PX4)
   - **Content**: Current interceptor position, velocity, acceleration

2. **Target Predicted State** (from predictor)
   - **Topic**: `/target/predicted_state`
   - **Type**: `px4_msgs.msg.VehicleLocalPosition`
   - **Rate**: 10-20 Hz
   - **Content**: Predicted target state at future horizon(s)

3. **Target Current State** (fallback if no predictor)
   - **Topic**: `/{target_namespace}/fmu/out/vehicle_local_position`
   - **Type**: `px4_msgs.msg.VehicleLocalPosition`
   - **Rate**: ~50 Hz (from PX4)
   - **Content**: Current target position, velocity (used if predictor unavailable)

**Data Structure (`PlannerInput`):**
```python
@dataclass
class PlannerInput:
    timestamp: float                    # ROS time in seconds
    
    # Interceptor state (ego drone)
    interceptor_position: np.ndarray    # [x, y, z] in NED frame (meters)
    interceptor_velocity: np.ndarray    # [vx, vy, vz] in NED frame (m/s)
    interceptor_acceleration: np.ndarray # [ax, ay, az] in NED frame (m/s²)
    
    # Target state (current or predicted)
    target_position: np.ndarray         # [x, y, z] in NED frame (meters)
    target_velocity: np.ndarray         # [vx, vy, vz] in NED frame (m/s)
    target_acceleration: np.ndarray     # [ax, ay, az] in NED frame (m/s²)
    
    # Prediction uncertainty (from predictor, optional)
    target_position_covariance: np.ndarray  # 3x3 covariance matrix
    target_velocity_covariance: np.ndarray  # 3x3 covariance matrix
    
    # Validity flags
    interceptor_valid: bool
    target_valid: bool
    prediction_available: bool
```

### Output Interface

**ROS2 Publications:**

1. **Trajectory Setpoint** (primary control output)
   - **Topic**: `/{interceptor_namespace}/fmu/in/trajectory_setpoint`
   - **Type**: `px4_msgs.msg.TrajectorySetpoint`
   - **Rate**: 20-50 Hz (high rate for responsive guidance)
   - **Content**: Commanded position, velocity, acceleration setpoints

2. **Guidance Markers** (visualization)
   - **Topic**: `/planner/guidance_markers`
   - **Type**: `visualization_msgs.msg.MarkerArray`
   - **Rate**: 5-10 Hz
   - **Content**: Visualization of commanded acceleration, LOS vector, predicted intercept point

3. **Planner Status** (debugging and monitoring)
   - **Topic**: `/planner/status`
   - **Type**: `std_msgs.msg.String` (JSON format)
   - **Rate**: 1-5 Hz
   - **Content**: Planner health, algorithm-specific metrics (e.g., time-to-go, closing velocity)

**Data Structure (`PlannerOutput`):**
```python
@dataclass
class PlannerOutput:
    timestamp: float                    # Command time (seconds)
    
    # Commanded acceleration (primary output of guidance law)
    commanded_acceleration: np.ndarray  # [ax, ay, az] in NED (m/s²)
    
    # Derived setpoints for trajectory controller
    commanded_velocity: np.ndarray      # [vx, vy, vz] in NED (m/s)
    commanded_position: np.ndarray      # [x, y, z] in NED (meters)
    
    # Optional yaw control
    commanded_yaw: float                # Yaw angle (radians)
    commanded_yaw_rate: float           # Yaw rate (rad/s)
    
    # Guidance metrics
    time_to_go: float                   # Estimated time to intercept (seconds)
    closing_velocity: float             # Closing speed (m/s)
    miss_distance: float                # Current distance to target (meters)
    
    # Validity
    is_valid: bool                      # Command validity flag
    guidance_active: bool               # Guidance law is actively guiding
```

### Required Methods

Every planner must implement:

```python
class PlannerBase(ABC):
    @abstractmethod
    def compute_guidance(self, planner_input: PlannerInput) -> PlannerOutput:
        """
        Compute guidance command based on current state.
        
        Args:
            planner_input: Current interceptor and target states
        
        Returns:
            PlannerOutput with commanded acceleration and derived setpoints
        """
        pass
    
    @abstractmethod
    def reset(self) -> None:
        """Reset planner to initial state."""
        pass
    
    @abstractmethod
    def is_converged(self) -> bool:
        """Check if interception criteria are met."""
        pass
    
    @abstractmethod
    def get_planner_type(self) -> str:
        """Get the type identifier of this planner."""
        pass
```

## Coordinate Frames

### NED (North-East-Down) - PX4 Native
- **X**: North (forward)
- **Y**: East (right)
- **Z**: Down (positive downward)

**All guidance computations are performed in NED frame.**

### Visualization (ENU for RViz)
- **X**: East
- **Y**: North
- **Z**: Up

Visualization markers are automatically converted to ENU in `visualization.py`.

## Guidance Law Selection Guide

| Planner | Use Case | Pros | Cons | Recommended For |
|---------|----------|------|------|-----------------|
| **PP** | Baseline, simple testing | Very simple, robust | Slow, overshoots on moving targets | Initial testing only |
| **PN** | Classic missile guidance | Well-established, works for closing geometry | Fails when closing speed → 0 | Academic comparison |
| **LPN** | Linearized variant | More robust than PN, Cartesian formulation | Still issues with small relative velocity | General interception |
| **GPN** | Gain-scheduled variant | Handles edge cases better | Complex tuning | Specialized scenarios |
| **FRPN** | **RECOMMENDED** | Fast response, robust to all geometries, handles low closing speed | Requires tuning of G and W | **Production use** |
| **MPC** | Constraint-aware scenarios | Optimal, handles constraints, smoother | Computationally expensive, requires solver | Obstacle avoidance, safety-critical |

## Parameter Tuning

### FRPN (Recommended Starting Point)
```yaml
frpn_planner:
  G: 19.7              # Global gain (aggressiveness)
  W: 0.051             # Blending weight (PP component)
  min_tgo: 0.05        # Minimum time-to-go safeguard (seconds)
  v_eps: 0.1           # Velocity epsilon for denominator safeguard (m/s)
  amax: [4.0, 4.0, 2.0]  # Max acceleration [ax, ay, az] (m/s²)
  alpha_uncertainty: 0.0  # Uncertainty scaling factor (0 = disabled)
```

### Pure Pursuit (PP)
```yaml
pp_planner:
  G_pp: 2.0            # Proportional gain
  amax: [4.0, 4.0, 2.0]  # Max acceleration (m/s²)
```

### Proportional Navigation (PN)
```yaml
pn_planner:
  N: 3.0               # Navigation constant (typically 3-5)
  amax: [4.0, 4.0, 2.0]  # Max acceleration (m/s²)
```

### LPN
```yaml
lpn_planner:
  G_lpn: 20.0          # LPN gain
  min_tgo: 0.05        # Minimum time-to-go (seconds)
  amax: [4.0, 4.0, 2.0]  # Max acceleration (m/s²)
```

### MPC
```yaml
mpc_planner:
  horizon_steps: 15    # Prediction horizon (steps)
  dt: 0.1              # Time step (seconds)
  Q_position: [10.0, 10.0, 5.0]   # Position tracking weights
  Q_velocity: [1.0, 1.0, 0.5]     # Velocity tracking weights
  R_control: [0.1, 0.1, 0.1]      # Control effort weights
  vmax: [10.0, 10.0, 5.0]         # Max velocity (m/s)
  amax: [4.0, 4.0, 2.0]           # Max acceleration (m/s²)
  solver: 'osqp'       # Solver: 'osqp', 'acados', 'qpoases'
```

## Integration with Predictors

Planners integrate seamlessly with the predictor framework:

1. **Standalone Mode**: Planners can use current target state directly (no predictor)
2. **Predictor Mode**: Planners subscribe to `/target/predicted_state` for future target position
3. **Uncertainty-Aware**: FRPN and MPC can scale behavior based on prediction covariance

**Topic Flow:**
```
Target PX4 → Predictor Node → /target/predicted_state → Planner Node
Interceptor PX4 → Planner Node → /px4_1/fmu/in/trajectory_setpoint → PX4 Controller
```

## ROS2 Launch

Launch files are provided for each planner in `src/orion_flight/launch/`:

```bash
# Launch FRPN planner (recommended)
ros2 launch orion_flight frpn_planner.launch.py

# Launch PP planner (baseline)
ros2 launch orion_flight pp_planner.launch.py

# Launch with custom parameters
ros2 launch orion_flight frpn_planner.launch.py G:=25.0 W:=0.03
```

## Visualization in RViz

All planners publish visualization markers to `/planner/guidance_markers`:

- **Red Arrow**: Commanded acceleration vector
- **Green Line**: Line-of-sight (LOS) from interceptor to target
- **Blue Sphere**: Predicted intercept point
- **Yellow Trail**: Commanded trajectory path
- **Cyan Ellipsoid**: Target position uncertainty (if available from predictor)

**RViz Config**: Use `config/planner_visualization.rviz` for pre-configured displays.

## Testing and Validation

See `PLANNER_IMPLEMENTATION_GUIDE.md` for:
- Unit testing guidance algorithms
- SITL integration testing
- Performance metrics and benchmarking
- Tuning procedures

## Cross-Reference with Predictors

**Similarities to Predictor Framework:**
- Both use abstract base classes for polymorphism
- Common data structures for state representation
- ROS2 pub/sub pattern with namespaced topics
- Visualization via MarkerArray
- NED coordinate frame convention

**Key Differences:**
- Predictors: passive state estimation (input: measurements → output: predictions)
- Planners: active control generation (input: states → output: commands)
- Predictors run at ~10-20 Hz (estimation rate)
- Planners run at 20-50 Hz (control rate)

**Integration:**
- Planners consume predictor outputs as inputs
- Predictors operate independently of planners
- Both support fallback modes (predictor failure → use current state)

## Safety Considerations

1. **Acceleration Limits**: All planners enforce `amax` per-axis clamping
2. **Velocity Limits**: MPC enforces velocity constraints explicitly
3. **Fallback Behavior**: If target state invalid, planners command hover or safe trajectory
4. **Time-to-Go Safeguards**: PN/LPN/FRPN use `min_tgo` to prevent division by zero
5. **Offboard Mode**: Ensure offboard mode is enabled before planners publish commands

## References

- Pliska et al., "Towards Safe Mid-Air Drone Interception: Strategies for Tracking & Capture", IEEE RA-L 2024
- See `algo_guide.md` for detailed algorithm descriptions and pseudocode
- PX4 Offboard Control: https://docs.px4.io/main/en/flight_modes/offboard.html

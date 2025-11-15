"""Common data types for planner framework."""

from dataclasses import dataclass
import numpy as np
from typing import Optional


@dataclass
class PlannerInput:
    """
    Input data structure for all planners.
    
    Contains interceptor (ego) and target states needed for guidance computation.
    """
    
    timestamp: float  # ROS time in seconds
    
    # Interceptor state (ego drone)
    interceptor_position: np.ndarray  # [x, y, z] in NED frame (meters)
    interceptor_velocity: np.ndarray  # [vx, vy, vz] in NED frame (m/s)
    interceptor_acceleration: np.ndarray  # [ax, ay, az] in NED frame (m/s²)
    
    # Target state (current or predicted)
    target_position: np.ndarray  # [x, y, z] in NED frame (meters)
    target_velocity: np.ndarray  # [vx, vy, vz] in NED frame (m/s)
    target_acceleration: np.ndarray  # [ax, ay, az] in NED frame (m/s²)
    
    # Prediction uncertainty (from predictor, optional)
    target_position_covariance: Optional[np.ndarray] = None  # 3x3 covariance matrix
    target_velocity_covariance: Optional[np.ndarray] = None  # 3x3 covariance matrix
    
    # Validity flags
    interceptor_valid: bool = True
    target_valid: bool = True
    prediction_available: bool = False  # True if using predicted (not current) target state


@dataclass
class PlannerOutput:
    """
    Output data structure for all planners.
    
    Contains commanded setpoints and guidance metrics.
    """
    
    timestamp: float  # Command time (seconds)
    
    # Commanded acceleration (primary output of guidance law)
    commanded_acceleration: np.ndarray  # [ax, ay, az] in NED (m/s²)
    
    # Derived setpoints for trajectory controller
    commanded_velocity: np.ndarray  # [vx, vy, vz] in NED (m/s)
    commanded_position: np.ndarray  # [x, y, z] in NED (meters)
    
    # Optional yaw control
    commanded_yaw: float = 0.0  # Yaw angle (radians)
    commanded_yaw_rate: float = 0.0  # Yaw rate (rad/s)
    
    # Guidance metrics
    time_to_go: float = 0.0  # Estimated time to intercept (seconds)
    closing_velocity: float = 0.0  # Closing speed (m/s)
    miss_distance: float = 0.0  # Current distance to target (meters)
    
    # Validity flags
    is_valid: bool = True  # Command validity flag
    guidance_active: bool = True  # Guidance law is actively guiding


@dataclass
class GuidanceMetrics:
    """
    Additional metrics for analysis and visualization.
    """
    
    # Line-of-sight (LOS) vector and rate
    los_vector: Optional[np.ndarray] = None  # Unit vector from interceptor to target
    los_rate: Optional[np.ndarray] = None  # Rate of change of LOS (rad/s)
    
    # Relative state
    relative_position: Optional[np.ndarray] = None  # Target position - interceptor position
    relative_velocity: Optional[np.ndarray] = None  # Target velocity - interceptor velocity
    
    # Navigation constant (for PN variants)
    effective_N: Optional[float] = None  # Effective navigation gain
    
    # Algorithm-specific data
    extra_data: Optional[dict] = None  # Dictionary for algorithm-specific metrics

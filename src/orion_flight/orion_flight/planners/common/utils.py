"""Utility functions for planner framework."""

import numpy as np
from typing import Tuple


def norm(v: np.ndarray) -> float:
    """
    Compute Euclidean norm of vector.
    
    Args:
        v: Input vector
    
    Returns:
        Euclidean norm
    """
    return np.linalg.norm(v)


def clamp_vec(v: np.ndarray, v_max: np.ndarray) -> np.ndarray:
    """
    Clamp vector per-axis to ±v_max.
    
    Args:
        v: Input vector to clamp
        v_max: Maximum absolute values per axis
    
    Returns:
        Clamped vector
    """
    return np.clip(v, -v_max, v_max)


def safe_normalize(v: np.ndarray, eps: float = 1e-6) -> np.ndarray:
    """
    Safely normalize vector (returns zero vector if norm too small).
    
    Args:
        v: Input vector
        eps: Minimum norm threshold
    
    Returns:
        Normalized vector or zero vector if norm < eps
    """
    v_norm = np.linalg.norm(v)
    if v_norm < eps:
        return np.zeros_like(v)
    return v / v_norm


def safe_divide(numerator: float, denominator: float, 
                min_denominator: float = 1e-6, 
                default: float = 0.0) -> float:
    """
    Safely divide with safeguard for small denominator.
    
    Args:
        numerator: Numerator value
        denominator: Denominator value
        min_denominator: Minimum allowed absolute denominator
        default: Value to return if denominator too small
    
    Returns:
        Division result or default
    """
    if abs(denominator) < min_denominator:
        return default
    return numerator / denominator


def compute_time_to_go(relative_position: np.ndarray, 
                       relative_velocity: np.ndarray,
                       min_tgo: float = 0.05,
                       v_eps: float = 0.1) -> float:
    """
    Compute time-to-go estimate with safeguards.
    
    Uses the formula: tgo = ||Δp|| / ||Δv||
    with safeguards for small relative velocity.
    
    Args:
        relative_position: Position error (target - interceptor)
        relative_velocity: Velocity error (target - interceptor)
        min_tgo: Minimum time-to-go (seconds)
        v_eps: Velocity epsilon for denominator safeguard
    
    Returns:
        Time-to-go estimate (seconds)
    """
    dp_norm = np.linalg.norm(relative_position)
    dv_norm = np.linalg.norm(relative_velocity)
    
    if dv_norm < v_eps:
        # Use a heuristic when relative velocity is small
        tgo = dp_norm / v_eps
    else:
        tgo = dp_norm / dv_norm
    
    return max(min_tgo, tgo)


def compute_closing_velocity(relative_position: np.ndarray,
                             relative_velocity: np.ndarray) -> float:
    """
    Compute closing velocity (rate of approach).
    
    Closing velocity is the component of relative velocity along LOS.
    Positive = closing, negative = opening.
    
    Args:
        relative_position: Position error (target - interceptor)
        relative_velocity: Velocity error (target - interceptor)
    
    Returns:
        Closing velocity (m/s)
    """
    dp_norm = np.linalg.norm(relative_position)
    
    if dp_norm < 1e-6:
        return 0.0
    
    # Project relative velocity onto LOS
    # Vc = -(λ · Δv) / ||λ|| where λ = Δp
    closing_velocity = -np.dot(relative_position, relative_velocity) / dp_norm
    
    return closing_velocity


def compute_los_vector(interceptor_position: np.ndarray,
                       target_position: np.ndarray) -> np.ndarray:
    """
    Compute line-of-sight (LOS) unit vector from interceptor to target.
    
    Args:
        interceptor_position: Interceptor position [x, y, z]
        target_position: Target position [x, y, z]
    
    Returns:
        LOS unit vector (or zero if positions coincident)
    """
    los = target_position - interceptor_position
    return safe_normalize(los)


def compute_los_rate(los_vector: np.ndarray,
                    relative_velocity: np.ndarray,
                    relative_distance: float) -> np.ndarray:
    """
    Compute line-of-sight (LOS) rate vector.
    
    For canonical PN: λ_dot = (λ × Δv) / ||λ||²
    
    Args:
        los_vector: LOS vector (not necessarily unit)
        relative_velocity: Relative velocity (target - interceptor)
        relative_distance: Distance between interceptor and target
    
    Returns:
        LOS rate vector (rad/s in 3D)
    """
    if relative_distance < 1e-6:
        return np.zeros(3)
    
    los_rate = np.cross(los_vector, relative_velocity) / (relative_distance ** 2)
    
    return los_rate


def ned_to_enu(position_ned: np.ndarray) -> np.ndarray:
    """
    Convert position from NED to ENU frame for visualization.
    
    NED: x=North, y=East, z=Down
    ENU: x=East, y=North, z=Up
    
    Transformation:
    x_enu = y_ned
    y_enu = x_ned
    z_enu = -z_ned
    
    Args:
        position_ned: Position in NED frame [x, y, z]
    
    Returns:
        Position in ENU frame [x, y, z]
    """
    return np.array([position_ned[1], position_ned[0], -position_ned[2]])


def enu_to_ned(position_enu: np.ndarray) -> np.ndarray:
    """
    Convert position from ENU to NED frame.
    
    Args:
        position_enu: Position in ENU frame [x, y, z]
    
    Returns:
        Position in NED frame [x, y, z]
    """
    return np.array([position_enu[1], position_enu[0], -position_enu[2]])


def accel_to_velocity_setpoint(v_current: np.ndarray,
                               a_cmd: np.ndarray,
                               dt: float) -> np.ndarray:
    """
    Convert acceleration command to velocity setpoint using integration.
    
    Args:
        v_current: Current velocity
        a_cmd: Commanded acceleration
        dt: Integration time step
    
    Returns:
        Commanded velocity
    """
    return v_current + a_cmd * dt


def accel_to_position_setpoint(p_current: np.ndarray,
                               v_current: np.ndarray,
                               a_cmd: np.ndarray,
                               dt: float) -> np.ndarray:
    """
    Convert acceleration command to position setpoint using integration.
    
    Uses kinematic equation: p = p0 + v0*dt + 0.5*a*dt²
    
    Args:
        p_current: Current position
        v_current: Current velocity
        a_cmd: Commanded acceleration
        dt: Integration time step
    
    Returns:
        Commanded position
    """
    return p_current + v_current * dt + 0.5 * a_cmd * (dt ** 2)


def compute_intercept_point(interceptor_pos: np.ndarray,
                           interceptor_vel: np.ndarray,
                           target_pos: np.ndarray,
                           target_vel: np.ndarray,
                           max_iterations: int = 10,
                           tolerance: float = 0.01) -> Tuple[np.ndarray, float]:
    """
    Compute predicted intercept point for constant velocity motion.
    
    Solves for time t such that:
    ||p_i + v_i * t - (p_t + v_t * t)|| = 0
    
    This is a simplified intercept point calculation assuming
    constant velocities (no guidance).
    
    Args:
        interceptor_pos: Interceptor position
        interceptor_vel: Interceptor velocity
        target_pos: Target position
        target_vel: Target velocity
        max_iterations: Maximum Newton-Raphson iterations
        tolerance: Convergence tolerance
    
    Returns:
        Tuple of (intercept_point, intercept_time)
    """
    # Initial guess: time-to-go based on current geometry
    dp = target_pos - interceptor_pos
    dv = target_vel - interceptor_vel
    
    if np.linalg.norm(dv) < 1e-3:
        # Parallel motion, no intercept or immediate intercept
        return target_pos, 0.0
    
    # Newton-Raphson iteration
    t = np.linalg.norm(dp) / np.linalg.norm(interceptor_vel) if np.linalg.norm(interceptor_vel) > 0.1 else 1.0
    
    for _ in range(max_iterations):
        # Position at time t
        p_i_t = interceptor_pos + interceptor_vel * t
        p_t_t = target_pos + target_vel * t
        
        # Miss distance
        miss = p_t_t - p_i_t
        miss_norm = np.linalg.norm(miss)
        
        if miss_norm < tolerance:
            break
        
        # Derivative of miss distance w.r.t. time
        dmiss_dt = target_vel - interceptor_vel
        
        # Newton-Raphson update
        t = t + np.dot(miss, dmiss_dt) / (np.linalg.norm(dmiss_dt) ** 2 + 1e-6)
        
        # Clamp time to positive
        t = max(0.0, t)
    
    # Compute intercept point
    intercept_point = target_pos + target_vel * t
    
    return intercept_point, t


def angle_wrap(angle: float) -> float:
    """
    Wrap angle to [-π, π].
    
    Args:
        angle: Angle in radians
    
    Returns:
        Wrapped angle in [-π, π]
    """
    return (angle + np.pi) % (2 * np.pi) - np.pi

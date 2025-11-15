"""Common data types for target prediction framework."""

from dataclasses import dataclass
from typing import Optional
import numpy as np


@dataclass
class PredictorInput:
    """
    Input data for predictor from target measurements.
    All positions/velocities in NED (North-East-Down) frame.
    """
    timestamp: float              # ROS time in seconds
    position: np.ndarray          # [x, y, z] in NED frame (meters)
    velocity: np.ndarray          # [vx, vy, vz] in NED frame (m/s)
    acceleration: np.ndarray      # [ax, ay, az] in NED frame (m/s²)
    position_valid: bool          # Position measurement validity
    velocity_valid: bool          # Velocity measurement validity
    acceleration_valid: bool      # Acceleration measurement validity


@dataclass
class PredictorOutput:
    """
    Output from predictor with predicted target state.
    All predictions in NED (North-East-Down) frame.
    """
    timestamp: float                    # Current time (seconds)
    prediction_horizon: float           # How far ahead this prediction is (seconds)
    
    # Predicted state
    predicted_position: np.ndarray      # [x, y, z] in NED (meters)
    predicted_velocity: np.ndarray      # [vx, vy, vz] in NED (m/s)
    predicted_acceleration: np.ndarray  # [ax, ay, az] in NED (m/s²) - if available
    
    # Uncertainty quantification
    position_covariance: np.ndarray     # 3x3 position covariance matrix
    velocity_covariance: np.ndarray     # 3x3 velocity covariance matrix
    full_covariance: Optional[np.ndarray] = None  # Full state covariance (if available)
    
    # Metadata
    model_probability: float = 1.0      # For IMM: probability of this model being correct
    innovation: Optional[np.ndarray] = None  # Measurement residual/innovation
    is_valid: bool = True               # Whether this prediction is valid
    predictor_type: str = "unknown"     # Type of predictor (cv, ca, imm, etc.)


@dataclass
class TargetState:
    """
    Complete target state representation.
    Combines measurement and prediction for guidance algorithms.
    """
    timestamp: float
    
    # Current state (measured)
    current_position: np.ndarray
    current_velocity: np.ndarray
    current_acceleration: np.ndarray
    
    # Predicted state (at various horizons)
    predictions: list[PredictorOutput]
    
    # State validity
    is_valid: bool = True
    time_since_update: float = 0.0


def ned_to_enu(vector_ned: np.ndarray) -> np.ndarray:
    """
    Convert vector from NED to ENU coordinate frame.
    
    NED: North-East-Down (PX4 native)
    ENU: East-North-Up (ROS/RViz standard)
    
    Args:
        vector_ned: Vector in NED frame [North, East, Down]
    
    Returns:
        vector_enu: Vector in ENU frame [East, North, Up]
    """
    if len(vector_ned) != 3:
        raise ValueError(f"Expected 3D vector, got shape {vector_ned.shape}")
    
    # NED -> ENU: swap and negate
    # [N, E, D] -> [E, N, -D]
    return np.array([vector_ned[1], vector_ned[0], -vector_ned[2]])


def enu_to_ned(vector_enu: np.ndarray) -> np.ndarray:
    """
    Convert vector from ENU to NED coordinate frame.
    
    Args:
        vector_enu: Vector in ENU frame [East, North, Up]
    
    Returns:
        vector_ned: Vector in NED frame [North, East, Down]
    """
    if len(vector_enu) != 3:
        raise ValueError(f"Expected 3D vector, got shape {vector_enu.shape}")
    
    # ENU -> NED: swap and negate
    # [E, N, U] -> [N, E, -U]
    return np.array([vector_enu[1], vector_enu[0], -vector_enu[2]])


def rotate_covariance_ned_to_enu(cov_ned: np.ndarray) -> np.ndarray:
    """
    Rotate covariance matrix from NED to ENU frame.
    
    Args:
        cov_ned: 3x3 covariance matrix in NED frame
    
    Returns:
        cov_enu: 3x3 covariance matrix in ENU frame
    """
    if cov_ned.shape != (3, 3):
        raise ValueError(f"Expected 3x3 covariance, got shape {cov_ned.shape}")
    
    # Rotation matrix: NED -> ENU
    # [N, E, D] -> [E, N, -D]
    R = np.array([
        [0, 1,  0],
        [1, 0,  0],
        [0, 0, -1]
    ])
    
    # Rotate covariance: C_enu = R * C_ned * R^T
    return R @ cov_ned @ R.T


def ensure_covariance_valid(P: np.ndarray, min_eigenvalue: float = 1e-6) -> np.ndarray:
    """
    Ensure covariance matrix is valid (symmetric, positive semi-definite).
    
    Args:
        P: Covariance matrix
        min_eigenvalue: Minimum allowed eigenvalue
    
    Returns:
        P_valid: Valid covariance matrix
    """
    # Ensure symmetry
    P = 0.5 * (P + P.T)
    
    # Ensure positive eigenvalues
    eigenvalues, eigenvectors = np.linalg.eigh(P)
    eigenvalues = np.maximum(eigenvalues, min_eigenvalue)
    P_valid = eigenvectors @ np.diag(eigenvalues) @ eigenvectors.T
    
    return P_valid

"""Constant Velocity prediction model implementation."""

import numpy as np
from typing import List, Tuple

from ..common.types import PredictorInput, PredictorOutput, ensure_covariance_valid


class CVModel:
    """
    Constant Velocity (CV) prediction model.
    
    Model assumes target maintains constant velocity:
        x(t+Δt) = x(t) + v(t)·Δt
        v(t+Δt) = v(t)
    
    State vector: [x, y, z, vx, vy, vz]
    """
    
    def __init__(self, process_noise: dict, measurement_noise: dict):
        """
        Initialize CV model.
        
        Args:
            process_noise: Dict with 'position' and 'velocity' noise parameters
            measurement_noise: Dict with 'position' and 'velocity' noise parameters
        """
        # State dimension: [x, y, z, vx, vy, vz]
        self.state_dim = 6
        
        # State vector and covariance
        self.x = np.zeros(self.state_dim)
        self.P = np.eye(self.state_dim) * 100.0  # High initial uncertainty
        
        # Process noise Q
        self.q_pos = process_noise.get('position', 0.1)
        self.q_vel = process_noise.get('velocity', 0.5)
        
        # Measurement noise R
        self.r_pos = measurement_noise.get('position', 0.05)
        self.r_vel = measurement_noise.get('velocity', 0.1)
        
        # Measurement matrix H (we observe position and velocity)
        self.H = np.eye(self.state_dim)
        
        # Build measurement noise covariance
        self.R = self._build_measurement_noise()
        
        self.initialized = False
        self.last_innovation = np.zeros(self.state_dim)
    
    def _build_measurement_noise(self) -> np.ndarray:
        """Build measurement noise covariance matrix R."""
        R = np.zeros((self.state_dim, self.state_dim))
        R[0:3, 0:3] = np.eye(3) * self.r_pos  # Position noise
        R[3:6, 3:6] = np.eye(3) * self.r_vel  # Velocity noise
        return R
    
    def _build_process_noise(self, dt: float) -> np.ndarray:
        """
        Build process noise covariance matrix Q for time step dt.
        
        Uses continuous white noise acceleration model:
        Q = G·Q_cont·G^T·dt
        
        where G is the noise gain matrix.
        """
        # Continuous time process noise (acceleration noise)
        q_cont_pos = self.q_pos
        q_cont_vel = self.q_vel
        
        # Discrete time process noise
        Q = np.zeros((self.state_dim, self.state_dim))
        
        # Position noise (affected by velocity integration)
        Q[0:3, 0:3] = np.eye(3) * (q_cont_pos * dt + q_cont_vel * dt**3 / 3.0)
        Q[0:3, 3:6] = np.eye(3) * (q_cont_vel * dt**2 / 2.0)
        Q[3:6, 0:3] = np.eye(3) * (q_cont_vel * dt**2 / 2.0)
        
        # Velocity noise
        Q[3:6, 3:6] = np.eye(3) * (q_cont_vel * dt)
        
        return Q
    
    def _build_transition_matrix(self, dt: float) -> np.ndarray:
        """
        Build state transition matrix F for time step dt.
        
        For CV model:
            x_new = x + vx*dt
            y_new = y + vy*dt
            z_new = z + vz*dt
            vx_new = vx
            vy_new = vy
            vz_new = vz
        """
        F = np.eye(self.state_dim)
        F[0, 3] = dt  # x += vx*dt
        F[1, 4] = dt  # y += vy*dt
        F[2, 5] = dt  # z += vz*dt
        return F
    
    def initialize(self, position: np.ndarray, velocity: np.ndarray):
        """
        Initialize state with first measurement.
        
        Args:
            position: Initial position [x, y, z]
            velocity: Initial velocity [vx, vy, vz]
        """
        self.x[0:3] = position
        self.x[3:6] = velocity
        
        # Reduce initial uncertainty after first measurement
        self.P = np.eye(self.state_dim) * 1.0
        
        self.initialized = True
        
        print(f"[CV Model] Initialized with:")
        print(f"  Position: [{position[0]:.3f}, {position[1]:.3f}, {position[2]:.3f}]")
        print(f"  Velocity: [{velocity[0]:.3f}, {velocity[1]:.3f}, {velocity[2]:.3f}]")
    
    def update(self, position: np.ndarray, velocity: np.ndarray, dt: float):
        """
        Update state estimate with new measurement (Kalman filter update).
        
        Args:
            position: Measured position [x, y, z] in NED
            velocity: Measured velocity [vx, vy, vz] in NED
            dt: Time since last update (seconds)
        """
        if not self.initialized:
            self.initialize(position, velocity)
            return
        
        if dt < 1e-6:
            # dt too small, skip prediction step
            dt = 1e-6
        
        # PREDICTION STEP
        F = self._build_transition_matrix(dt)
        Q = self._build_process_noise(dt)
        
        # Predict state
        x_pred = F @ self.x
        
        # Predict covariance
        P_pred = F @ self.P @ F.T + Q
        P_pred = ensure_covariance_valid(P_pred)
        
        # UPDATE STEP (Kalman correction)
        z = np.concatenate([position, velocity])  # Measurement vector
        
        # Innovation (measurement residual)
        y = z - (self.H @ x_pred)
        self.last_innovation = y
        
        # Innovation covariance
        S = self.H @ P_pred @ self.H.T + self.R
        S = ensure_covariance_valid(S)
        
        # Kalman gain
        try:
            K = P_pred @ self.H.T @ np.linalg.inv(S)
        except np.linalg.LinAlgError:
            # Singular matrix - skip update
            self.x = x_pred
            self.P = P_pred
            return
        
        # Update state
        self.x = x_pred + K @ y
        
        # Update covariance (Joseph form for numerical stability)
        I_KH = np.eye(self.state_dim) - K @ self.H
        self.P = I_KH @ P_pred @ I_KH.T + K @ self.R @ K.T
        self.P = ensure_covariance_valid(self.P)
        
        # Debug output (throttled by caller)
        innovation_norm = np.linalg.norm(y[0:3])
        if innovation_norm > 0.5:  # Only log significant innovations
            print(f"[CV Model] Large innovation detected: {innovation_norm:.3f}m")
    
    def predict(self, horizon: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """
        Predict state at future time t + horizon.
        
        Args:
            horizon: Prediction time horizon (seconds)
        
        Returns:
            predicted_position: [x, y, z]
            predicted_velocity: [vx, vy, vz]
            position_covariance: 3x3 position covariance
            velocity_covariance: 3x3 velocity covariance
        """
        if horizon < 0:
            raise ValueError(f"Negative horizon: {horizon}")
        
        # Propagate state forward
        F = self._build_transition_matrix(horizon)
        x_pred = F @ self.x
        
        # Propagate covariance
        Q_horizon = self._build_process_noise(horizon)
        P_pred = F @ self.P @ F.T + Q_horizon
        P_pred = ensure_covariance_valid(P_pred)
        
        # Extract position and velocity
        position = x_pred[0:3]
        velocity = x_pred[3:6]
        
        # Extract covariances
        pos_cov = P_pred[0:3, 0:3]
        vel_cov = P_pred[3:6, 3:6]
        
        return position, velocity, pos_cov, vel_cov
    
    def get_state(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """
        Get current state estimate.
        
        Returns:
            position: [x, y, z]
            velocity: [vx, vy, vz]
            position_covariance: 3x3 position covariance
            velocity_covariance: 3x3 velocity covariance
        """
        position = self.x[0:3]
        velocity = self.x[3:6]
        pos_cov = self.P[0:3, 0:3]
        vel_cov = self.P[3:6, 3:6]
        
        return position, velocity, pos_cov, vel_cov
    
    def get_innovation(self) -> np.ndarray:
        """Get last measurement innovation (residual)."""
        return self.last_innovation
    
    def reset(self):
        """Reset model to uninitialized state."""
        self.x = np.zeros(self.state_dim)
        self.P = np.eye(self.state_dim) * 100.0
        self.initialized = False
        self.last_innovation = np.zeros(self.state_dim)

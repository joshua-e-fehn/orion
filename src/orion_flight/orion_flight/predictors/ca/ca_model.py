"""Constant Acceleration prediction model implementation."""

import numpy as np
from typing import List, Tuple

from ..common.types import PredictorInput, PredictorOutput, ensure_covariance_valid


class CAModel:
    """
    Constant Acceleration (CA) prediction model.
    
    Model assumes target maintains constant acceleration:
        x(t+Δt) = x(t) + v(t)·Δt + 0.5·a(t)·Δt²
        v(t+Δt) = v(t) + a(t)·Δt
        a(t+Δt) = a(t)
    
    State vector: [x, y, z, vx, vy, vz, ax, ay, az]
    """
    
    def __init__(self, process_noise: dict, measurement_noise: dict):
        """
        Initialize CA model.
        
        Args:
            process_noise: Dict with 'position', 'velocity', and 'acceleration' noise parameters
            measurement_noise: Dict with 'position', 'velocity', and 'acceleration' noise parameters
        """
        # State dimension: [x, y, z, vx, vy, vz, ax, ay, az]
        self.state_dim = 9
        
        # State vector and covariance
        self.x = np.zeros(self.state_dim)
        self.P = np.eye(self.state_dim) * 100.0  # High initial uncertainty
        
        # Process noise Q
        self.q_pos = process_noise.get('position', 0.1)
        self.q_vel = process_noise.get('velocity', 0.5)
        self.q_acc = process_noise.get('acceleration', 1.0)
        
        # Measurement noise R
        self.r_pos = measurement_noise.get('position', 0.05)
        self.r_vel = measurement_noise.get('velocity', 0.1)
        self.r_acc = measurement_noise.get('acceleration', 0.5)
        
        # Measurement matrix H (we observe position, velocity, and acceleration)
        self.H = np.eye(self.state_dim)
        
        # Build measurement noise covariance
        self.R = self._build_measurement_noise()
        
        # For estimating acceleration when not directly measured
        self.prev_velocity = None
        self.prev_time = None
        
        self.initialized = False
        self.last_innovation = np.zeros(self.state_dim)
    
    def _build_measurement_noise(self) -> np.ndarray:
        """Build measurement noise covariance matrix R."""
        R = np.zeros((self.state_dim, self.state_dim))
        R[0:3, 0:3] = np.eye(3) * self.r_pos  # Position noise
        R[3:6, 3:6] = np.eye(3) * self.r_vel  # Velocity noise
        R[6:9, 6:9] = np.eye(3) * self.r_acc  # Acceleration noise (higher uncertainty)
        return R
    
    def _build_process_noise(self, dt: float) -> np.ndarray:
        """
        Build process noise covariance matrix Q for time step dt.
        
        Uses continuous white noise jerk model:
        The jerk (derivative of acceleration) is modeled as white noise.
        """
        # Continuous time process noise intensities
        q_cont_pos = self.q_pos
        q_cont_vel = self.q_vel
        q_cont_acc = self.q_acc
        
        # Discrete time process noise matrix
        Q = np.zeros((self.state_dim, self.state_dim))
        
        dt2 = dt * dt
        dt3 = dt2 * dt
        dt4 = dt3 * dt
        dt5 = dt4 * dt
        
        # Position block (affected by velocity, acceleration, and jerk)
        Q[0:3, 0:3] = np.eye(3) * (q_cont_pos * dt + q_cont_vel * dt3 / 3.0 + q_cont_acc * dt5 / 20.0)
        Q[0:3, 3:6] = np.eye(3) * (q_cont_vel * dt2 / 2.0 + q_cont_acc * dt4 / 8.0)
        Q[0:3, 6:9] = np.eye(3) * (q_cont_acc * dt3 / 6.0)
        
        # Velocity block (affected by acceleration and jerk)
        Q[3:6, 0:3] = np.eye(3) * (q_cont_vel * dt2 / 2.0 + q_cont_acc * dt4 / 8.0)
        Q[3:6, 3:6] = np.eye(3) * (q_cont_vel * dt + q_cont_acc * dt3 / 3.0)
        Q[3:6, 6:9] = np.eye(3) * (q_cont_acc * dt2 / 2.0)
        
        # Acceleration block (affected by jerk)
        Q[6:9, 0:3] = np.eye(3) * (q_cont_acc * dt3 / 6.0)
        Q[6:9, 3:6] = np.eye(3) * (q_cont_acc * dt2 / 2.0)
        Q[6:9, 6:9] = np.eye(3) * (q_cont_acc * dt)
        
        return Q
    
    def _build_transition_matrix(self, dt: float) -> np.ndarray:
        """
        Build state transition matrix F for time step dt.
        
        For CA model:
            x_new = x + vx*dt + 0.5*ax*dt²
            y_new = y + vy*dt + 0.5*ay*dt²
            z_new = z + vz*dt + 0.5*az*dt²
            vx_new = vx + ax*dt
            vy_new = vy + ay*dt
            vz_new = vz + az*dt
            ax_new = ax
            ay_new = ay
            az_new = az
        """
        F = np.eye(self.state_dim)
        
        dt2 = 0.5 * dt * dt
        
        # Position updates
        F[0, 3] = dt      # x += vx*dt
        F[0, 6] = dt2     # x += 0.5*ax*dt²
        F[1, 4] = dt      # y += vy*dt
        F[1, 7] = dt2     # y += 0.5*ay*dt²
        F[2, 5] = dt      # z += vz*dt
        F[2, 8] = dt2     # z += 0.5*az*dt²
        
        # Velocity updates
        F[3, 6] = dt      # vx += ax*dt
        F[4, 7] = dt      # vy += ay*dt
        F[5, 8] = dt      # vz += az*dt
        
        # Acceleration remains constant (already identity)
        
        return F
    
    def _estimate_acceleration(self, velocity: np.ndarray, dt: float) -> np.ndarray:
        """
        Estimate acceleration from velocity using finite difference.
        
        Args:
            velocity: Current velocity [vx, vy, vz]
            dt: Time step (seconds)
        
        Returns:
            Estimated acceleration [ax, ay, az]
        """
        if self.prev_velocity is None or dt < 1e-6:
            # No previous velocity or dt too small
            return np.zeros(3)
        
        # Finite difference: a ≈ (v - v_prev) / dt
        acceleration = (velocity - self.prev_velocity) / dt
        
        # Limit acceleration magnitude to reasonable values (e.g., 10 m/s²)
        acc_magnitude = np.linalg.norm(acceleration)
        max_acc = 10.0
        if acc_magnitude > max_acc:
            acceleration = acceleration * (max_acc / acc_magnitude)
        
        return acceleration
    
    def initialize(self, position: np.ndarray, velocity: np.ndarray, acceleration: np.ndarray = None):
        """
        Initialize state with first measurement.
        
        Args:
            position: Initial position [x, y, z]
            velocity: Initial velocity [vx, vy, vz]
            acceleration: Initial acceleration [ax, ay, az] (optional)
        """
        self.x[0:3] = position
        self.x[3:6] = velocity
        
        if acceleration is not None:
            self.x[6:9] = acceleration
        else:
            self.x[6:9] = np.zeros(3)  # Start with zero acceleration
        
        # Reduce initial uncertainty after first measurement
        self.P = np.eye(self.state_dim) * 1.0
        # Higher uncertainty for acceleration initially
        self.P[6:9, 6:9] = np.eye(3) * 10.0
        
        self.prev_velocity = velocity
        self.initialized = True
    
    def update(self, position: np.ndarray, velocity: np.ndarray, dt: float, 
               acceleration: np.ndarray = None):
        """
        Update state estimate with new measurement (Kalman filter update).
        
        Args:
            position: Measured position [x, y, z] in NED
            velocity: Measured velocity [vx, vy, vz] in NED
            dt: Time since last update (seconds)
            acceleration: Measured acceleration [ax, ay, az] in NED (optional)
        """
        if not self.initialized:
            self.initialize(position, velocity, acceleration)
            return
        
        if dt < 1e-6:
            # dt too small, skip prediction step
            dt = 1e-6
        
        # Estimate acceleration if not provided
        if acceleration is None:
            acceleration = self._estimate_acceleration(velocity, dt)
        
        # PREDICTION STEP
        F = self._build_transition_matrix(dt)
        Q = self._build_process_noise(dt)
        
        # Predict state
        x_pred = F @ self.x
        
        # Predict covariance
        P_pred = F @ self.P @ F.T + Q
        P_pred = ensure_covariance_valid(P_pred)
        
        # UPDATE STEP (Kalman correction)
        z = np.concatenate([position, velocity, acceleration])  # Measurement vector
        
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
            self.prev_velocity = velocity
            return
        
        # Update state
        self.x = x_pred + K @ y
        
        # Update covariance (Joseph form for numerical stability)
        I_KH = np.eye(self.state_dim) - K @ self.H
        self.P = I_KH @ P_pred @ I_KH.T + K @ self.R @ K.T
        self.P = ensure_covariance_valid(self.P)
        
        # Store velocity for next acceleration estimate
        self.prev_velocity = velocity
    
    def predict(self, horizon: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """
        Predict state at future time t + horizon.
        
        Args:
            horizon: Prediction time horizon (seconds)
        
        Returns:
            predicted_position: [x, y, z]
            predicted_velocity: [vx, vy, vz]
            predicted_acceleration: [ax, ay, az]
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
        
        # Extract position, velocity, and acceleration
        position = x_pred[0:3]
        velocity = x_pred[3:6]
        acceleration = x_pred[6:9]
        
        # Extract covariances
        pos_cov = P_pred[0:3, 0:3]
        vel_cov = P_pred[3:6, 3:6]
        
        return position, velocity, acceleration, pos_cov, vel_cov
    
    def get_state(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """
        Get current state estimate.
        
        Returns:
            position: [x, y, z]
            velocity: [vx, vy, vz]
            acceleration: [ax, ay, az]
            position_covariance: 3x3 position covariance
            velocity_covariance: 3x3 velocity covariance
        """
        position = self.x[0:3]
        velocity = self.x[3:6]
        acceleration = self.x[6:9]
        pos_cov = self.P[0:3, 0:3]
        vel_cov = self.P[3:6, 3:6]
        
        return position, velocity, acceleration, pos_cov, vel_cov
    
    def get_innovation(self) -> np.ndarray:
        """Get last measurement innovation (residual)."""
        return self.last_innovation
    
    def reset(self):
        """Reset model to uninitialized state."""
        self.x = np.zeros(self.state_dim)
        self.P = np.eye(self.state_dim) * 100.0
        self.prev_velocity = None
        self.prev_time = None
        self.initialized = False
        self.last_innovation = np.zeros(self.state_dim)

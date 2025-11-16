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
        
        # Measurement matrix H (we observe position, velocity, and optionally acceleration)
        # For now, we assume we only measure position and velocity directly
        # Acceleration is estimated from velocity changes
        self.H = np.zeros((6, self.state_dim))
        self.H[0:3, 0:3] = np.eye(3)  # Measure position
        self.H[3:6, 3:6] = np.eye(3)  # Measure velocity
        
        # Build measurement noise covariance
        self.R = self._build_measurement_noise()
        
        self.initialized = False
        self.last_innovation = np.zeros(6)  # 6D innovation (pos + vel)
        self.last_velocity = None
        self.last_measurement_time = None
    
    def _build_measurement_noise(self) -> np.ndarray:
        """Build measurement noise covariance matrix R."""
        R = np.zeros((6, 6))
        R[0:3, 0:3] = np.eye(3) * self.r_pos  # Position noise
        R[3:6, 3:6] = np.eye(3) * self.r_vel  # Velocity noise
        return R
    
    def _build_process_noise(self, dt: float) -> np.ndarray:
        """
        Build process noise covariance matrix Q for time step dt.
        
        Uses continuous white noise jerk model:
        Q = G·Q_cont·G^T·dt
        
        where G is the noise gain matrix.
        """
        # Continuous time process noise intensities
        q_cont_pos = self.q_pos
        q_cont_vel = self.q_vel
        q_cont_acc = self.q_acc
        
        # Discrete time process noise
        Q = np.zeros((self.state_dim, self.state_dim))
        
        dt2 = dt * dt
        dt3 = dt2 * dt
        dt4 = dt3 * dt
        dt5 = dt4 * dt
        
        # Position-position block
        Q[0:3, 0:3] = np.eye(3) * (q_cont_pos * dt + q_cont_vel * dt3/3.0 + q_cont_acc * dt5/20.0)
        
        # Position-velocity block
        Q[0:3, 3:6] = np.eye(3) * (q_cont_vel * dt2/2.0 + q_cont_acc * dt4/8.0)
        Q[3:6, 0:3] = Q[0:3, 3:6]  # Symmetric
        
        # Position-acceleration block
        Q[0:3, 6:9] = np.eye(3) * (q_cont_acc * dt3/6.0)
        Q[6:9, 0:3] = Q[0:3, 6:9]  # Symmetric
        
        # Velocity-velocity block
        Q[3:6, 3:6] = np.eye(3) * (q_cont_vel * dt + q_cont_acc * dt3/3.0)
        
        # Velocity-acceleration block
        Q[3:6, 6:9] = np.eye(3) * (q_cont_acc * dt2/2.0)
        Q[6:9, 3:6] = Q[3:6, 6:9]  # Symmetric
        
        # Acceleration-acceleration block
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
        
        dt2_half = 0.5 * dt * dt
        
        # Position updates
        F[0, 3] = dt       # x += vx*dt
        F[1, 4] = dt       # y += vy*dt
        F[2, 5] = dt       # z += vz*dt
        F[0, 6] = dt2_half # x += 0.5*ax*dt²
        F[1, 7] = dt2_half # y += 0.5*ay*dt²
        F[2, 8] = dt2_half # z += 0.5*az*dt²
        
        # Velocity updates
        F[3, 6] = dt       # vx += ax*dt
        F[4, 7] = dt       # vy += ay*dt
        F[5, 8] = dt       # vz += az*dt
        
        # Acceleration stays constant (F[6:9, 6:9] = I already set)
        
        return F
    
    def _estimate_acceleration(self, velocity: np.ndarray, dt: float) -> np.ndarray:
        """
        Estimate acceleration from velocity change.
        
        Args:
            velocity: Current velocity [vx, vy, vz]
            dt: Time since last measurement
        
        Returns:
            Estimated acceleration [ax, ay, az]
        """
        if self.last_velocity is None or dt < 1e-6:
            return np.zeros(3)
        
        # Finite difference: a ≈ (v - v_prev) / dt
        acc = (velocity - self.last_velocity) / dt
        
        # Clamp to reasonable values to avoid outliers
        max_acc = 10.0  # m/s² (reasonable for drones)
        acc = np.clip(acc, -max_acc, max_acc)
        
        return acc
    
    def initialize(self, position: np.ndarray, velocity: np.ndarray, acceleration: np.ndarray = None):
        """
        Initialize state with first measurement.
        
        Args:
            position: Initial position [x, y, z]
            velocity: Initial velocity [vx, vy, vz]
            acceleration: Initial acceleration [ax, ay, az] (optional, defaults to zero)
        """
        self.x[0:3] = position
        self.x[3:6] = velocity
        
        if acceleration is not None:
            self.x[6:9] = acceleration
        else:
            self.x[6:9] = np.zeros(3)
        
        # Reduce initial uncertainty after first measurement
        self.P = np.eye(self.state_dim) * 1.0
        # Higher uncertainty for acceleration since it's not directly measured
        self.P[6:9, 6:9] = np.eye(3) * 10.0
        
        self.last_velocity = velocity
        self.initialized = True
    
    def update(self, position: np.ndarray, velocity: np.ndarray, dt: float, 
               acceleration: np.ndarray = None):
        """
        Update state estimate with new measurement (Kalman filter update).
        
        Args:
            position: Measured position [x, y, z] in NED
            velocity: Measured velocity [vx, vy, vz] in NED
            dt: Time since last update (seconds)
            acceleration: Measured acceleration [ax, ay, az] (optional)
        """
        # Estimate acceleration from velocity change if not provided
        if acceleration is None:
            acc_estimate = self._estimate_acceleration(velocity, dt)
        else:
            acc_estimate = acceleration
        
        if not self.initialized:
            self.initialize(position, velocity, acc_estimate)
            self.last_measurement_time = 0.0
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
        # We measure position and velocity (6D measurement)
        z = np.concatenate([position, velocity])
        
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
            self.last_velocity = velocity
            return
        
        # Update state
        self.x = x_pred + K @ y
        
        # Update covariance (Joseph form for numerical stability)
        I_KH = np.eye(self.state_dim) - K @ self.H
        self.P = I_KH @ P_pred @ I_KH.T + K @ self.R @ K.T
        self.P = ensure_covariance_valid(self.P)
        
        # Store velocity for next acceleration estimate
        self.last_velocity = velocity
    
    def predict(self, horizon: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
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
            acceleration_covariance: 3x3 acceleration covariance
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
        acc_cov = P_pred[6:9, 6:9]
        
        return position, velocity, acceleration, pos_cov, vel_cov, acc_cov
    
    def get_state(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """
        Get current state estimate.
        
        Returns:
            position: [x, y, z]
            velocity: [vx, vy, vz]
            acceleration: [ax, ay, az]
            position_covariance: 3x3 position covariance
            velocity_covariance: 3x3 velocity covariance
            acceleration_covariance: 3x3 acceleration covariance
        """
        position = self.x[0:3]
        velocity = self.x[3:6]
        acceleration = self.x[6:9]
        pos_cov = self.P[0:3, 0:3]
        vel_cov = self.P[3:6, 3:6]
        acc_cov = self.P[6:9, 6:9]
        
        return position, velocity, acceleration, pos_cov, vel_cov, acc_cov
    
    def get_innovation(self) -> np.ndarray:
        """Get last measurement innovation (residual)."""
        return self.last_innovation
    
    def reset(self):
        """Reset model to uninitialized state."""
        self.x = np.zeros(self.state_dim)
        self.P = np.eye(self.state_dim) * 100.0
        self.initialized = False
        self.last_innovation = np.zeros(6)
        self.last_velocity = None
        self.last_measurement_time = None

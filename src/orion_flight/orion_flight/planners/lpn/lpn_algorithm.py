"""Linearized Proportional Navigation (LPN) guidance algorithm implementation."""

import numpy as np
from typing import Dict
from orion_flight.planners.common.utils import norm, clamp_vec


class LinearizedPNAlgorithm:
    """
    Linearized Proportional Navigation (LPN) guidance law implementation.
    
    LPN is a Cartesian/linearized formulation that yields an acceleration command
    using position & velocity errors and time-to-go estimate.
    
    Command: a_cmd = G * ((Δp + Δv * tgo) / tgo²)
    
    where:
    - Δp = p_t - p_i (position error)
    - Δv = v_t - v_i (velocity error)
    - tgo = ||Δp|| / ||Δv|| (time-to-go, safeguarded)
    - G is the LPN gain
    
    LPN is more robust than canonical PN in many geometries and handles
    edge cases better when closing velocity is small.
    """
    
    def __init__(self, params: Dict):
        """
        Initialize Linearized PN algorithm.
        
        Args:
            params: Dictionary with algorithm parameters
                - 'G_lpn': LPN gain (default: 20.0)
                - 'min_tgo': Minimum time-to-go safeguard in seconds (default: 0.05)
                - 'amax': Max acceleration [ax, ay, az] in m/s² (default: [4.0, 4.0, 2.0])
                - 'dt': Integration time step (default: 0.02)
                - 'v_eps': Velocity epsilon for denominator safeguard (default: 1e-3)
        """
        self.G_lpn = params.get('G_lpn', 20.0)
        self.min_tgo = params.get('min_tgo', 0.05)
        self.amax = np.array(params.get('amax', [4.0, 4.0, 2.0]))
        self.dt = params.get('dt', 0.02)
        self.v_eps = params.get('v_eps', 1e-3)
        
        # Internal state
        self.last_command = np.zeros(3)
        self.command_count = 0
    
    def compute_command(self,
                       p_i: np.ndarray,
                       v_i: np.ndarray,
                       p_t: np.ndarray,
                       v_t: np.ndarray,
                       dt: float = None) -> Dict:
        """
        Compute Linearized PN guidance command.
        
        Args:
            p_i: Interceptor position [x, y, z] in NED (m)
            v_i: Interceptor velocity [vx, vy, vz] in NED (m/s)
            p_t: Target position [x, y, z] in NED (m)
            v_t: Target velocity [vx, vy, vz] in NED (m/s)
            dt: Time step for integration (seconds), uses self.dt if None
        
        Returns:
            Dictionary with:
                - 'acceleration': Commanded acceleration [ax, ay, az] (m/s²)
                - 'velocity': Commanded velocity [vx, vy, vz] (m/s)
                - 'position': Commanded position [x, y, z] (m)
                - 'tgo': Time-to-go estimate (seconds)
                - 'closing_velocity': Closing speed (m/s)
                - 'miss_distance': Current miss distance (m)
        """
        if dt is None:
            dt = self.dt
        
        # Compute position error (relative position)
        dp = p_t - p_i
        
        # Compute relative velocity
        dv = v_t - v_i
        
        # Compute guidance metrics
        miss_distance = norm(dp)
        dv_norm = norm(dv)
        
        # ====================================
        # TIME-TO-GO COMPUTATION WITH SAFEGUARDS
        # ====================================
        # If relative velocity is small, use interceptor speed as fallback
        if dv_norm < self.v_eps:
            tgo = max(self.min_tgo, miss_distance / (norm(v_i) + self.v_eps))
        else:
            tgo = max(self.min_tgo, miss_distance / dv_norm)
        
        # Closing velocity (projection of relative velocity onto LOS)
        if miss_distance > 1e-6:
            closing_velocity = -np.dot(dp, dv) / miss_distance
        else:
            closing_velocity = 0.0
        
        # ====================================
        # LINEARIZED PN GUIDANCE LAW
        # ====================================
        # a_cmd = G * ((Δp + Δv * tgo) / tgo²)
        lpn_term = (dp + dv * tgo) / (tgo * tgo)
        a_cmd = self.G_lpn * lpn_term
        
        # Apply acceleration limits (per-axis clamping)
        a_cmd = clamp_vec(a_cmd, self.amax)
        
        # ====================================
        # INTEGRATE TO GET VELOCITY & POSITION SETPOINTS
        # ====================================
        # v_cmd = v_i + a_cmd * dt
        v_cmd = v_i + a_cmd * dt
        
        # p_cmd = p_i + v_i * dt + 0.5 * a_cmd * dt²
        p_cmd = p_i + v_i * dt + 0.5 * a_cmd * (dt ** 2)
        
        # Store for history
        self.last_command = a_cmd.copy()
        self.command_count += 1
        
        # Return command package
        return {
            'acceleration': a_cmd,
            'velocity': v_cmd,
            'position': p_cmd,
            'tgo': tgo,
            'closing_velocity': closing_velocity,
            'miss_distance': miss_distance
        }
    
    def reset(self):
        """Reset internal state."""
        self.last_command = np.zeros(3)
        self.command_count = 0
    
    def get_last_command(self) -> np.ndarray:
        """Get last commanded acceleration."""
        return self.last_command.copy()
    
    def get_command_count(self) -> int:
        """Get total number of commands generated."""
        return self.command_count

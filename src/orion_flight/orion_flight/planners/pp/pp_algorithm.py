"""Pure Pursuit (PP) guidance algorithm implementation."""

import numpy as np
from typing import Dict
from orion_flight.planners.common.utils import norm, clamp_vec


class PurePursuitAlgorithm:
    """
    Pure Pursuit guidance law implementation.
    
    PP is a simple proportional guidance law that commands acceleration
    proportional to the position error (target position - interceptor position).
    
    Command: a_cmd = G_pp * (p_t - p_i)
    
    This is the simplest guidance law and serves as a baseline. It works
    well for stationary targets but can overshoot on moving targets.
    """
    
    def __init__(self, params: Dict):
        """
        Initialize Pure Pursuit algorithm.
        
        Args:
            params: Dictionary with algorithm parameters
                - 'G_pp': Proportional gain (default: 2.0)
                - 'amax': Max acceleration [ax, ay, az] in m/s² (default: [4.0, 4.0, 2.0])
                - 'dt': Integration time step (default: 0.02)
        """
        self.G_pp = params.get('G_pp', 2.0)
        self.amax = np.array(params.get('amax', [4.0, 4.0, 2.0]))
        self.dt = params.get('dt', 0.02)
        
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
        Compute Pure Pursuit guidance command.
        
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
        
        # Time-to-go estimate (simple: distance / relative speed)
        dv_norm = norm(dv)
        if dv_norm < 1e-3:
            # If relative velocity is small, use interceptor speed as heuristic
            tgo = miss_distance / max(norm(v_i), 1e-3)
        else:
            tgo = miss_distance / dv_norm
        
        # Ensure minimum tgo
        tgo = max(0.05, tgo)
        
        # Closing velocity (projection of relative velocity onto LOS)
        if miss_distance > 1e-6:
            closing_velocity = -np.dot(dp, dv) / miss_distance
        else:
            closing_velocity = 0.0
        
        # ====================================
        # PURE PURSUIT GUIDANCE LAW
        # ====================================
        # Command acceleration proportional to position error
        a_cmd = self.G_pp * dp
        
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

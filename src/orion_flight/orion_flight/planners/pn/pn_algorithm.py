#!/usr/bin/env python3
"""Proportional Navigation (PN) guidance algorithm implementation."""

import numpy as np
from typing import Dict
from orion_flight.planners.common.utils import (
    norm, clamp_vec, compute_los_rate, compute_closing_velocity
)


class ProportionalNavigationAlgorithm:
    """
    Proportional Navigation (PN) guidance law implementation.
    
    PN is a canonical guidance law used in guided missiles and interception systems.
    It commands acceleration perpendicular to the line-of-sight (LOS) proportional
    to the LOS rate times closing speed.
    
    Command: a_cmd = N * V_c * λ_dot
    
    Where:
        - N is the navigation constant (gain)
        - V_c is the closing velocity
        - λ_dot is the LOS rate vector
    
    Reference: algo_guide.md Section 4.2
    """
    
    def __init__(self, params: Dict):
        """
        Initialize Proportional Navigation algorithm.
        
        Args:
            params: Dictionary with algorithm parameters
                - 'N': Navigation constant (gain), default: 3.0
                - 'amax': Max acceleration [ax, ay, az] in m/s² (default: [4.0, 4.0, 2.0])
                - 'dt': Integration time step (default: 0.02)
                - 'min_tgo': Minimum time-to-go safeguard (default: 0.05)
                - 'v_eps': Velocity epsilon for safeguarding (default: 0.1)
        """
        self.N = params.get('N', 3.0)
        self.amax = np.array(params.get('amax', [4.0, 4.0, 2.0]))
        self.dt = params.get('dt', 0.02)
        self.min_tgo = params.get('min_tgo', 0.05)
        self.v_eps = params.get('v_eps', 0.1)
        
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
        Compute Proportional Navigation guidance command.
        
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
        
        # ====================================
        # COMPUTE GEOMETRY
        # ====================================
        # Line-of-sight (LOS) vector: λ = p_t - p_i
        lam = p_t - p_i
        lam_norm = norm(lam)
        
        # Relative velocity: Δv = v_t - v_i
        rel_v = v_t - v_i
        rel_v_norm = norm(rel_v)
        
        # Miss distance (current separation)
        miss_distance = lam_norm
        
        # ====================================
        # COMPUTE CLOSING VELOCITY
        # ====================================
        # V_c = -(λ · Δv) / ||λ||
        # Positive when closing, negative when opening
        if lam_norm < 1e-6:
            # Already at target
            closing_velocity = 0.0
        else:
            closing_velocity = -np.dot(lam, rel_v) / lam_norm
        
        # ====================================
        # COMPUTE TIME-TO-GO ESTIMATE
        # ====================================
        # tgo = ||Δp|| / ||Δv|| with safeguards
        if rel_v_norm < self.v_eps:
            # Use interceptor speed as heuristic when relative velocity is small
            tgo = miss_distance / max(norm(v_i), self.v_eps)
        else:
            tgo = miss_distance / rel_v_norm
        
        # Ensure minimum tgo
        tgo = max(self.min_tgo, tgo)
        
        # ====================================
        # PROPORTIONAL NAVIGATION GUIDANCE LAW
        # ====================================
        # Compute LOS rate: λ_dot = (λ × Δv) / ||λ||²
        if lam_norm < 1e-6:
            # Degenerate case: already at target
            a_cmd = np.zeros(3)
        else:
            # LOS rate vector (3D)
            lam_dot = compute_los_rate(lam, rel_v, lam_norm)
            
            # PN command: a_cmd = N * V_c * λ_dot
            a_cmd = self.N * closing_velocity * lam_dot
        
        # ====================================
        # APPLY ACCELERATION LIMITS
        # ====================================
        # Apply per-axis clamping
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

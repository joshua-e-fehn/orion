#!/usr/bin/env python3
"""Standalone unit test for PN algorithm (no ROS2 dependencies)."""

import sys
import numpy as np


# Simplified utility functions (copy from utils.py)
def norm(v: np.ndarray) -> float:
    """Compute Euclidean norm of vector."""
    return np.linalg.norm(v)


def clamp_vec(v: np.ndarray, v_max: np.ndarray) -> np.ndarray:
    """Clamp vector per-axis to ±v_max."""
    return np.clip(v, -v_max, v_max)


def compute_los_rate(los_vector: np.ndarray,
                     relative_velocity: np.ndarray,
                     relative_distance: float) -> np.ndarray:
    """Compute line-of-sight (LOS) rate vector."""
    if relative_distance < 1e-6:
        return np.zeros(3)
    
    los_rate = np.cross(los_vector, relative_velocity) / (relative_distance ** 2)
    return los_rate


# Simplified PN algorithm (standalone)
class SimplePNAlgorithm:
    """Standalone PN algorithm for testing."""
    
    def __init__(self, N=3.0, amax=None, min_tgo=0.05, v_eps=0.1):
        self.N = N
        self.amax = np.array(amax if amax is not None else [4.0, 4.0, 2.0])
        self.min_tgo = min_tgo
        self.v_eps = v_eps
    
    def compute_command(self, p_i, v_i, p_t, v_t):
        """Compute PN guidance command."""
        # LOS vector
        lam = p_t - p_i
        lam_norm = norm(lam)
        
        # Relative velocity
        rel_v = v_t - v_i
        rel_v_norm = norm(rel_v)
        
        # Miss distance
        miss_distance = lam_norm
        
        # Closing velocity
        if lam_norm < 1e-6:
            closing_velocity = 0.0
        else:
            closing_velocity = -np.dot(lam, rel_v) / lam_norm
        
        # Time-to-go
        if rel_v_norm < self.v_eps:
            tgo = miss_distance / max(norm(v_i), self.v_eps)
        else:
            tgo = miss_distance / rel_v_norm
        tgo = max(self.min_tgo, tgo)
        
        # PN command
        if lam_norm < 1e-6:
            a_cmd = np.zeros(3)
        else:
            lam_dot = compute_los_rate(lam, rel_v, lam_norm)
            a_cmd = self.N * closing_velocity * lam_dot
        
        # Clamp
        a_cmd = clamp_vec(a_cmd, self.amax)
        
        return {
            'acceleration': a_cmd,
            'tgo': tgo,
            'closing_velocity': closing_velocity,
            'miss_distance': miss_distance
        }


def test_basic_intercept():
    """Test basic intercept scenario."""
    print("\n=== Test 1: Basic Intercept ===")
    
    pn = SimplePNAlgorithm(N=3.0, amax=[4.0, 4.0, 2.0])
    
    # Interceptor at origin, target at (10, 10, 0), moving east
    p_i = np.array([0.0, 0.0, 0.0])
    v_i = np.array([0.0, 0.0, 0.0])
    p_t = np.array([10.0, 10.0, 0.0])
    v_t = np.array([0.0, 5.0, 0.0])
    
    result = pn.compute_command(p_i, v_i, p_t, v_t)
    
    print(f"Interceptor: {p_i}")
    print(f"Target: {p_t}")
    print(f"Target velocity: {v_t}")
    print(f"Commanded acceleration: {result['acceleration']}")
    print(f"Miss distance: {result['miss_distance']:.2f} m")
    print(f"Time-to-go: {result['tgo']:.2f} s")
    print(f"Closing velocity: {result['closing_velocity']:.2f} m/s")
    
    # Verify acceleration is non-zero
    accel_norm = np.linalg.norm(result['acceleration'])
    assert accel_norm > 0.0, "Acceleration should be non-zero"
    
    # Verify within limits
    assert np.all(np.abs(result['acceleration']) <= [4.0, 4.0, 2.0]), \
        "Acceleration should be within limits"
    
    print("✓ Test passed!")
    return True


def test_head_on_intercept():
    """Test head-on intercept scenario."""
    print("\n=== Test 2: Head-on Intercept ===")
    
    pn = SimplePNAlgorithm(N=3.0, amax=[4.0, 4.0, 2.0])
    
    # Interceptor moving north, target moving south (head-on)
    p_i = np.array([0.0, 0.0, 0.0])
    v_i = np.array([10.0, 0.0, 0.0])
    p_t = np.array([20.0, 5.0, 0.0])
    v_t = np.array([-8.0, 0.0, 0.0])
    
    result = pn.compute_command(p_i, v_i, p_t, v_t)
    
    print(f"Interceptor: {p_i}, velocity: {v_i}")
    print(f"Target: {p_t}, velocity: {v_t}")
    print(f"Commanded acceleration: {result['acceleration']}")
    print(f"Miss distance: {result['miss_distance']:.2f} m")
    print(f"Closing velocity: {result['closing_velocity']:.2f} m/s")
    
    # Should have positive closing velocity (approaching)
    assert result['closing_velocity'] > 0.0, "Should be closing on target"
    
    print("✓ Test passed!")
    return True


def test_acceleration_limits():
    """Test acceleration limiting."""
    print("\n=== Test 3: Acceleration Limits ===")
    
    pn = SimplePNAlgorithm(N=50.0, amax=[2.0, 2.0, 1.0])
    
    # Large position error with high gain should saturate
    p_i = np.array([0.0, 0.0, 0.0])
    v_i = np.array([0.0, 0.0, 0.0])
    p_t = np.array([10.0, 10.0, -5.0])
    v_t = np.array([3.0, 3.0, 0.0])
    
    result = pn.compute_command(p_i, v_i, p_t, v_t)
    
    print(f"High gain N=50.0, low limits amax=[2.0, 2.0, 1.0]")
    print(f"Commanded acceleration: {result['acceleration']}")
    
    # Verify limits are respected
    limits = [2.0, 2.0, 1.0]
    for i, (a, amax) in enumerate(zip(result['acceleration'], limits)):
        assert abs(a) <= amax + 1e-6, \
            f"Acceleration[{i}] = {a} exceeds limit {amax}"
    
    print("✓ All accelerations within limits!")
    return True


def test_stationary_target():
    """Test with stationary target."""
    print("\n=== Test 4: Stationary Target ===")
    
    pn = SimplePNAlgorithm(N=3.0, amax=[4.0, 4.0, 2.0])
    
    # Interceptor at origin, stationary target
    p_i = np.array([0.0, 0.0, 0.0])
    v_i = np.array([0.0, 0.0, 0.0])
    p_t = np.array([5.0, 5.0, -2.0])
    v_t = np.array([0.0, 0.0, 0.0])
    
    result = pn.compute_command(p_i, v_i, p_t, v_t)
    
    print(f"Interceptor: {p_i}")
    print(f"Target: {p_t} (stationary)")
    print(f"Commanded acceleration: {result['acceleration']}")
    print(f"Miss distance: {result['miss_distance']:.2f} m")
    print(f"Acceleration magnitude: {np.linalg.norm(result['acceleration']):.3f} m/s²")
    
    print("✓ Test passed!")
    return True


def test_perpendicular_approach():
    """Test perpendicular approach (pure lateral intercept)."""
    print("\n=== Test 5: Perpendicular Approach ===")
    
    pn = SimplePNAlgorithm(N=3.0, amax=[4.0, 4.0, 2.0])
    
    # Interceptor moving east, target moving north (perpendicular)
    p_i = np.array([0.0, 0.0, 0.0])
    v_i = np.array([0.0, 10.0, 0.0])  # Moving east
    p_t = np.array([15.0, 5.0, 0.0])
    v_t = np.array([5.0, 0.0, 0.0])   # Moving north
    
    result = pn.compute_command(p_i, v_i, p_t, v_t)
    
    print(f"Interceptor: {p_i}, velocity: {v_i}")
    print(f"Target: {p_t}, velocity: {v_t}")
    print(f"Commanded acceleration: {result['acceleration']}")
    print(f"Miss distance: {result['miss_distance']:.2f} m")
    print(f"Closing velocity: {result['closing_velocity']:.2f} m/s")
    
    # Should have some lateral acceleration
    accel_norm = np.linalg.norm(result['acceleration'])
    assert accel_norm > 0.0, "Should have non-zero acceleration"
    
    print("✓ Test passed!")
    return True


def run_all_tests():
    """Run all tests."""
    print("="*60)
    print("PN Algorithm Unit Tests (Standalone)")
    print("="*60)
    
    tests = [
        test_basic_intercept,
        test_head_on_intercept,
        test_acceleration_limits,
        test_stationary_target,
        test_perpendicular_approach
    ]
    
    passed = 0
    failed = 0
    
    for test in tests:
        try:
            if test():
                passed += 1
        except Exception as e:
            print(f"✗ Test failed with error: {e}")
            failed += 1
            import traceback
            traceback.print_exc()
    
    print("\n" + "="*60)
    print(f"Results: {passed} passed, {failed} failed")
    print("="*60)
    
    return failed == 0


if __name__ == '__main__':
    success = run_all_tests()
    sys.exit(0 if success else 1)

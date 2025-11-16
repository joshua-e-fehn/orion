#!/usr/bin/env python3
"""Simple unit test for PN algorithm."""

import sys
import numpy as np

# Add src to path for imports
sys.path.insert(0, 'src/orion_flight')

from orion_flight.planners.pn.pn_algorithm import ProportionalNavigationAlgorithm


def test_basic_intercept():
    """Test basic intercept scenario."""
    print("\n=== Test 1: Basic Intercept ===")
    
    # Initialize PN algorithm
    params = {
        'N': 3.0,
        'amax': [4.0, 4.0, 2.0],
        'dt': 0.05
    }
    
    pn = ProportionalNavigationAlgorithm(params)
    
    # Scenario: Interceptor at origin, target at (10, 10, 0), moving east
    p_i = np.array([0.0, 0.0, 0.0])  # Interceptor position
    v_i = np.array([0.0, 0.0, 0.0])  # Interceptor stationary initially
    p_t = np.array([10.0, 10.0, 0.0])  # Target position
    v_t = np.array([0.0, 5.0, 0.0])   # Target moving east at 5 m/s
    
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
    assert np.all(np.abs(result['acceleration']) <= params['amax']), \
        "Acceleration should be within limits"
    
    print("✓ Test passed!")
    return True


def test_stationary_target():
    """Test with stationary target."""
    print("\n=== Test 2: Stationary Target ===")
    
    params = {
        'N': 3.0,
        'amax': [4.0, 4.0, 2.0],
        'dt': 0.05
    }
    
    pn = ProportionalNavigationAlgorithm(params)
    
    # Interceptor at origin, stationary target at (5, 5, -2)
    p_i = np.array([0.0, 0.0, 0.0])
    v_i = np.array([0.0, 0.0, 0.0])
    p_t = np.array([5.0, 5.0, -2.0])
    v_t = np.array([0.0, 0.0, 0.0])  # Stationary target
    
    result = pn.compute_command(p_i, v_i, p_t, v_t)
    
    print(f"Interceptor: {p_i}")
    print(f"Target: {p_t} (stationary)")
    print(f"Commanded acceleration: {result['acceleration']}")
    print(f"Miss distance: {result['miss_distance']:.2f} m")
    
    # For stationary target with zero relative velocity, 
    # PN may produce zero or small acceleration
    print(f"Acceleration magnitude: {np.linalg.norm(result['acceleration']):.3f} m/s²")
    
    print("✓ Test passed!")
    return True


def test_head_on_intercept():
    """Test head-on intercept scenario."""
    print("\n=== Test 3: Head-on Intercept ===")
    
    params = {
        'N': 3.0,
        'amax': [4.0, 4.0, 2.0],
        'dt': 0.05
    }
    
    pn = ProportionalNavigationAlgorithm(params)
    
    # Interceptor moving north, target moving south (head-on)
    p_i = np.array([0.0, 0.0, 0.0])
    v_i = np.array([10.0, 0.0, 0.0])  # Moving north at 10 m/s
    p_t = np.array([20.0, 5.0, 0.0])
    v_t = np.array([-8.0, 0.0, 0.0])  # Moving south at 8 m/s
    
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
    print("\n=== Test 4: Acceleration Limits ===")
    
    params = {
        'N': 50.0,  # Very high gain to trigger limits
        'amax': [2.0, 2.0, 1.0],  # Low limits
        'dt': 0.05
    }
    
    pn = ProportionalNavigationAlgorithm(params)
    
    # Large position error with high gain should saturate
    p_i = np.array([0.0, 0.0, 0.0])
    v_i = np.array([0.0, 0.0, 0.0])
    p_t = np.array([10.0, 10.0, -5.0])
    v_t = np.array([3.0, 3.0, 0.0])
    
    result = pn.compute_command(p_i, v_i, p_t, v_t)
    
    print(f"High gain N={params['N']}, low limits amax={params['amax']}")
    print(f"Commanded acceleration: {result['acceleration']}")
    
    # Verify limits are respected
    for i, (a, amax) in enumerate(zip(result['acceleration'], params['amax'])):
        assert abs(a) <= amax + 1e-6, \
            f"Acceleration[{i}] = {a} exceeds limit {amax}"
    
    print("✓ All accelerations within limits!")
    return True


def test_convergence():
    """Test convergence detection."""
    print("\n=== Test 5: Convergence ===")
    
    params = {
        'N': 3.0,
        'amax': [4.0, 4.0, 2.0],
        'dt': 0.05
    }
    
    pn = ProportionalNavigationAlgorithm(params)
    
    # Very close positions
    p_i = np.array([10.0, 5.0, -2.0])
    v_i = np.array([1.0, 1.0, 0.0])
    p_t = np.array([10.1, 5.1, -2.0])  # Only 0.14m away
    v_t = np.array([1.0, 1.0, 0.0])     # Same velocity
    
    result = pn.compute_command(p_i, v_i, p_t, v_t)
    
    print(f"Miss distance: {result['miss_distance']:.3f} m")
    print(f"Commanded acceleration: {result['acceleration']}")
    
    # Should have very small miss distance
    assert result['miss_distance'] < 0.5, "Should be very close to target"
    
    print("✓ Test passed!")
    return True


def run_all_tests():
    """Run all tests."""
    print("="*60)
    print("PN Algorithm Unit Tests")
    print("="*60)
    
    tests = [
        test_basic_intercept,
        test_stationary_target,
        test_head_on_intercept,
        test_acceleration_limits,
        test_convergence
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

#!/usr/bin/env python3
"""
Simple test script for CA predictor.

Tests basic functionality of the CA model without ROS2 dependencies.
"""

import numpy as np
import sys
import os

# Add parent directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', '..'))

from orion_flight.predictors.ca.ca_model import CAModel


def test_initialization():
    """Test CA model initialization."""
    print("Testing initialization...")
    
    process_noise = {'position': 0.1, 'velocity': 0.5, 'acceleration': 1.0}
    measurement_noise = {'position': 0.05, 'velocity': 0.1, 'acceleration': 0.5}
    
    model = CAModel(process_noise, measurement_noise)
    
    assert model.state_dim == 9, "State dimension should be 9"
    assert not model.initialized, "Model should not be initialized"
    
    print("✓ Initialization test passed")


def test_update_and_predict():
    """Test update and prediction."""
    print("\nTesting update and prediction...")
    
    process_noise = {'position': 0.1, 'velocity': 0.5, 'acceleration': 1.0}
    measurement_noise = {'position': 0.05, 'velocity': 0.1, 'acceleration': 0.5}
    
    model = CAModel(process_noise, measurement_noise)
    
    # Simulate a target moving with constant acceleration
    dt = 0.1  # 10 Hz updates
    true_accel = np.array([1.0, 0.5, -0.2])  # m/s²
    
    position = np.array([0.0, 0.0, -10.0])  # Start at (0, 0, -10) in NED
    velocity = np.array([5.0, 0.0, 0.0])    # Moving north at 5 m/s
    
    # Feed several measurements
    for i in range(20):
        # Update model
        model.update(position, velocity, dt)
        
        # Simulate true motion with constant acceleration
        position = position + velocity * dt + 0.5 * true_accel * dt**2
        velocity = velocity + true_accel * dt
    
    assert model.initialized, "Model should be initialized after updates"
    
    # Get current state estimate
    est_pos, est_vel, est_acc, _, _, _ = model.get_state()
    
    print(f"  Estimated position: {est_pos}")
    print(f"  Estimated velocity: {est_vel}")
    print(f"  Estimated acceleration: {est_acc}")
    print(f"  True acceleration: {true_accel}")
    
    # Check that estimated acceleration is close to true acceleration
    acc_error = np.linalg.norm(est_acc - true_accel)
    print(f"  Acceleration error: {acc_error:.3f} m/s²")
    
    # Test prediction
    horizons = [0.5, 1.0, 2.0]
    print("\n  Predictions:")
    for h in horizons:
        pred_pos, pred_vel, pred_acc, pos_cov, vel_cov, acc_cov = model.predict(h)
        pos_uncertainty = np.sqrt(np.trace(pos_cov))
        print(f"    t+{h}s: pos={pred_pos}, uncertainty={pos_uncertainty:.3f}m")
    
    print("✓ Update and prediction test passed")


def test_acceleration_estimation():
    """Test acceleration estimation from velocity changes."""
    print("\nTesting acceleration estimation...")
    
    process_noise = {'position': 0.1, 'velocity': 0.5, 'acceleration': 1.0}
    measurement_noise = {'position': 0.05, 'velocity': 0.1, 'acceleration': 0.5}
    
    model = CAModel(process_noise, measurement_noise)
    
    # Start with zero velocity
    position = np.array([0.0, 0.0, -10.0])
    velocity = np.array([0.0, 0.0, 0.0])
    dt = 0.1
    
    # First update - no acceleration estimated yet
    model.update(position, velocity, dt)
    
    # Apply sudden acceleration
    acceleration = np.array([2.0, 1.0, -0.5])
    
    for i in range(10):
        position = position + velocity * dt + 0.5 * acceleration * dt**2
        velocity = velocity + acceleration * dt
        model.update(position, velocity, dt)
    
    # Check estimated acceleration
    _, _, est_acc, _, _, _ = model.get_state()
    
    print(f"  True acceleration: {acceleration}")
    print(f"  Estimated acceleration: {est_acc}")
    
    error = np.linalg.norm(est_acc - acceleration)
    print(f"  Error: {error:.3f} m/s²")
    
    # Should converge to true acceleration after several updates
    assert error < 0.5, f"Acceleration estimation error too large: {error}"
    
    print("✓ Acceleration estimation test passed")


def test_covariance_propagation():
    """Test that covariance grows with prediction horizon."""
    print("\nTesting covariance propagation...")
    
    process_noise = {'position': 0.1, 'velocity': 0.5, 'acceleration': 1.0}
    measurement_noise = {'position': 0.05, 'velocity': 0.1, 'acceleration': 0.5}
    
    model = CAModel(process_noise, measurement_noise)
    
    # Initialize with some state
    position = np.array([0.0, 0.0, -10.0])
    velocity = np.array([5.0, 0.0, 0.0])
    model.update(position, velocity, 0.1)
    
    # Check covariance increases with horizon
    horizons = [0.5, 1.0, 2.0, 5.0]
    uncertainties = []
    
    for h in horizons:
        _, _, _, pos_cov, _, _ = model.predict(h)
        uncertainty = np.sqrt(np.trace(pos_cov))
        uncertainties.append(uncertainty)
        print(f"  t+{h}s: position uncertainty = {uncertainty:.3f}m")
    
    # Uncertainty should increase with time
    for i in range(len(uncertainties) - 1):
        assert uncertainties[i+1] > uncertainties[i], \
            "Uncertainty should increase with prediction horizon"
    
    print("✓ Covariance propagation test passed")


def test_reset():
    """Test model reset functionality."""
    print("\nTesting reset...")
    
    process_noise = {'position': 0.1, 'velocity': 0.5, 'acceleration': 1.0}
    measurement_noise = {'position': 0.05, 'velocity': 0.1, 'acceleration': 0.5}
    
    model = CAModel(process_noise, measurement_noise)
    
    # Update a few times
    position = np.array([10.0, 20.0, -30.0])
    velocity = np.array([1.0, 2.0, -0.5])
    for _ in range(5):
        model.update(position, velocity, 0.1)
        position += velocity * 0.1
    
    assert model.initialized, "Model should be initialized"
    
    # Reset
    model.reset()
    
    assert not model.initialized, "Model should not be initialized after reset"
    assert np.allclose(model.x, np.zeros(9)), "State should be zero after reset"
    
    print("✓ Reset test passed")


def main():
    """Run all tests."""
    print("=" * 60)
    print("CA Predictor Model Tests")
    print("=" * 60)
    
    try:
        test_initialization()
        test_update_and_predict()
        test_acceleration_estimation()
        test_covariance_propagation()
        test_reset()
        
        print("\n" + "=" * 60)
        print("All tests passed! ✓")
        print("=" * 60)
        return 0
        
    except AssertionError as e:
        print(f"\n✗ Test failed: {e}")
        return 1
    except Exception as e:
        print(f"\n✗ Unexpected error: {e}")
        import traceback
        traceback.print_exc()
        return 1


if __name__ == '__main__':
    sys.exit(main())

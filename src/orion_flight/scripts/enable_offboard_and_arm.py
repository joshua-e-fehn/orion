#!/usr/bin/env python3
"""
Enable OFFBOARD mode and ARM a specific PX4 instance
Usage: enable_offboard_and_arm.py <instance_id>
"""
import sys
import time
from pymavlink import mavutil

def enable_offboard_and_arm(instance_id):
    """Enable OFFBOARD mode and ARM for a specific PX4 instance."""
    
    # Calculate MAVLink port for this instance
    # Instance 1: 14540, Instance 2: 14541, etc.
    mavlink_port = 14540 + (instance_id - 1)
    
    print(f"\n{'='*60}")
    print(f"  Enable OFFBOARD & ARM for PX4 Instance {instance_id}")
    print(f"  MAVLink Port: {mavlink_port}")
    print(f"{'='*60}\n")
    
    # Connect to PX4
    connection_string = f'udp:127.0.0.1:{mavlink_port}'
    print(f"Connecting to {connection_string}...")
    
    try:
        mav = mavutil.mavlink_connection(connection_string, timeout=30)
        print("Waiting for heartbeat...")
        mav.wait_heartbeat(timeout=30)
        print(f"✓ Connected to PX4 instance {instance_id}\n")
    except Exception as e:
        print(f"✗ Failed to connect: {e}")
        print(f"  Make sure PX4 instance {instance_id} is running!")
        return False
    
    # Send initial setpoint (required before enabling OFFBOARD)
    print("Sending initial setpoint stream...")
    for _ in range(10):
        mav.mav.set_position_target_local_ned_send(
            0,  # time_boot_ms
            mav.target_system,
            mav.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b0000111111111000,  # type_mask (only positions enabled)
            0, 0, 0,  # x, y, z positions (NED)
            0, 0, 0,  # x, y, z velocity
            0, 0, 0,  # x, y, z acceleration
            0, 0  # yaw, yaw_rate
        )
        time.sleep(0.1)
    
    print("✓ Initial setpoints sent\n")
    
    # Enable OFFBOARD mode
    print("Enabling OFFBOARD mode...")
    mav.mav.command_long_send(
        mav.target_system,
        mav.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE,
        0,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        6,  # OFFBOARD mode
        0, 0, 0, 0, 0
    )
    
    # Wait for acknowledgment
    time.sleep(1)
    print("✓ OFFBOARD mode command sent\n")
    
    # ARM the drone
    print("Arming drone...")
    mav.mav.command_long_send(
        mav.target_system,
        mav.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1,  # 1 to arm, 0 to disarm
        0, 0, 0, 0, 0, 0
    )
    
    # Wait for acknowledgment
    time.sleep(1)
    print("✓ ARM command sent\n")
    
    # Continue sending setpoints for a bit to keep OFFBOARD mode active
    print("Maintaining OFFBOARD mode with setpoints...")
    for _ in range(20):
        mav.mav.set_position_target_local_ned_send(
            0,
            mav.target_system,
            mav.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b0000111111111000,
            0, 0, 0,
            0, 0, 0,
            0, 0, 0,
            0, 0
        )
        time.sleep(0.05)
    
    print(f"\n✓ OFFBOARD mode enabled and drone {instance_id} armed!")
    print(f"  The drone is now ready to receive commands from ROS2 nodes")
    print(f"{'='*60}\n")
    
    mav.close()
    return True

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: enable_offboard_and_arm.py <instance_id>")
        sys.exit(1)
    
    instance_id = int(sys.argv[1])
    success = enable_offboard_and_arm(instance_id)
    sys.exit(0 if success else 1)

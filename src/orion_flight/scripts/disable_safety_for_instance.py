#!/usr/bin/env python3
"""
Disable PX4 safety checks for a specific instance
Usage: disable_safety_for_instance.py <instance_id>
"""
import sys
import time
from pymavlink import mavutil

def disable_safety(instance_id):
    """Disable safety features for a specific PX4 instance."""
    
    # Calculate MAVLink port for this instance
    # Instance 1: 14540, Instance 2: 14541, etc.
    mavlink_port = 14540 + (instance_id - 1)
    
    print(f"\n{'='*60}")
    print(f"  Disabling Safety for PX4 Instance {instance_id}")
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
    
    # Safety parameters to disable
    params = {
        # Circuit breakers (disable safety checks)
        'CBRK_SUPPLY_CHK': 894281,   # Disable power supply check
        'CBRK_USB_CHK': 197848,      # Disable USB check
        'CBRK_AIRSPD_CHK': 162128,   # Disable airspeed check
        'CBRK_ENGINEFAIL': 284953,   # Disable engine failure check
        'CBRK_FLIGHTTERM': 121212,   # Disable flight termination
        'CBRK_GPSFAIL': 240024,      # Disable GPS failure check
        
        # Commander settings
        'COM_RCL_EXCEPT': 4,         # RC loss exceptions (offboard)
        'COM_RC_IN_MODE': 4,         # RC input disabled
        'COM_ARM_WO_GPS': 1,         # Allow arm without GPS
        'COM_ARM_AUTH': 0,           # Disable arm authorization
        'COM_OBS_AVOID': 0,          # Disable obstacle avoidance
        
        # Navigation settings
        'NAV_RCL_ACT': 0,            # No action on RC loss
        'NAV_DLL_ACT': 0,            # No action on data link loss
    }
    
    print("Setting parameters...")
    success_count = 0
    for param_name, param_value in params.items():
        try:
            print(f"  {param_name:20s} = {param_value}")
            mav.mav.param_set_send(
                mav.target_system,
                mav.target_component,
                param_name.encode('utf-8'),
                param_value,
                mavutil.mavlink.MAV_PARAM_TYPE_INT32
            )
            time.sleep(0.2)
            success_count += 1
        except Exception as e:
            print(f"  ✗ Failed to set {param_name}: {e}")
    
    print(f"\n✓ Set {success_count}/{len(params)} parameters successfully!")
    print(f"✓ Safety checks disabled for PX4 instance {instance_id}!\n")
    print(f"{'='*60}\n")
    
    mav.close()
    return True

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: disable_safety_for_instance.py <instance_id>")
        sys.exit(1)
    
    instance_id = int(sys.argv[1])
    success = disable_safety(instance_id)
    sys.exit(0 if success else 1)

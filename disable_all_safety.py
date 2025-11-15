#!/usr/bin/env python3
"""
Disable ALL PX4 safety checks for offboard development
"""
from pymavlink import mavutil
import time

mav = mavutil.mavlink_connection('udp:127.0.0.1:14540')
mav.wait_heartbeat()
print("✓ Connected to PX4\n")

# Comprehensive list of safety parameters to disable
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
for param_name, param_value in params.items():
    print(f"  {param_name} = {param_value}")
    mav.mav.param_set_send(
        mav.target_system,
        mav.target_component,
        param_name.encode('utf-8'),
        param_value,
        mavutil.mavlink.MAV_PARAM_TYPE_INT32
    )
    time.sleep(0.3)

print("\n✓ All safety checks disabled!")
print("\nNow PX4 should accept arm and offboard commands.")
print("Run: ./run_circle_demo.sh")

#!/bin/bash

echo "========================================="
echo "  Setting PX4 Parameters for Offboard"
echo "========================================="

# Send commands to PX4 console via MAVLink
# Using pymavlink to set parameters

python3 - << 'EOF'
from pymavlink import mavutil
import time

# Connect to PX4
print("[INFO] Connecting to PX4 at udp:127.0.0.1:14540...")
mav = mavutil.mavlink_connection('udp:127.0.0.1:14540')
mav.wait_heartbeat()
print("[SUCCESS] Connected to PX4!")

# Parameters to set
params = {
    'COM_RCL_EXCEPT': 4,    # RC loss exceptions (allow offboard)
    'COM_RC_IN_MODE': 4,    # RC input mode (disabled)
    'NAV_RCL_ACT': 0,       # No action on RC loss
    'COM_ARM_SWISBTN': 0,   # Disable arm switch
    'COM_OBS_AVOID': 0,     # Disable obstacle avoidance requirement
}

print("\n[INFO] Setting parameters...")
for param_name, param_value in params.items():
    print(f"  Setting {param_name} = {param_value}")
    mav.mav.param_set_send(
        mav.target_system,
        mav.target_component,
        param_name.encode('utf-8'),
        param_value,
        mavutil.mavlink.MAV_PARAM_TYPE_INT32
    )
    time.sleep(0.5)

print("\n[SUCCESS] Parameters set!")
print("\nNow you can run: ./run_circle_demo.sh")
EOF

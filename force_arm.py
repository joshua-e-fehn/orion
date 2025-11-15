#!/usr/bin/env python3
"""
Force PX4 to arm and enter offboard mode
"""
from pymavlink import mavutil
import time

mav = mavutil.mavlink_connection('udp:127.0.0.1:14540')
mav.wait_heartbeat()
print(f"✓ Connected to PX4")

# Force offboard mode
print("Setting OFFBOARD mode...")
mav.set_mode('OFFBOARD')
time.sleep(1)

# Force arm with override
print("Force arming...")
mav.arducopter_arm()
time.sleep(0.5)

# Alternative: raw command
mav.mav.command_long_send(
    mav.target_system,
    mav.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    1,  # arm
    21196,  # force
    0, 0, 0, 0, 0
)

print("✓ Commands sent!")

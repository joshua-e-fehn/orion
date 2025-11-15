#!/usr/bin/env python3

"""
PX4 Pre-flight Check Diagnostic Tool
Shows why PX4 won't arm
"""

from pymavlink import mavutil
import time

print("="*50)
print("  PX4 Pre-flight Check Diagnostic")
print("="*50)

# Connect to PX4
print("\n[INFO] Connecting to PX4...")
mav = mavutil.mavlink_connection('udp:127.0.0.1:14540')
mav.wait_heartbeat()
print(f"[SUCCESS] Connected to system {mav.target_system}")

# Request detailed status
print("\n[INFO] Requesting extended status...")
mav.mav.command_long_send(
    mav.target_system,
    mav.target_component,
    mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
    0,
    mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS,
    0, 0, 0, 0, 0, 0
)

# Listen for messages
print("\n[INFO] Listening for status messages...\n")
start_time = time.time()

while time.time() - start_time < 5:
    msg = mav.recv_match(blocking=True, timeout=1)
    if msg:
        msg_type = msg.get_type()
        
        if msg_type == 'HEARTBEAT':
            mode = mavutil.mode_string_v10(msg)
            armed = 'ARMED' if msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED else 'DISARMED'
            print(f"[HEARTBEAT] Mode: {mode}, Status: {armed}")
            
        elif msg_type == 'SYS_STATUS':
            print(f"\n[SYS_STATUS]:")
            print(f"  Sensors enabled: {bin(msg.onboard_control_sensors_enabled)}")
            print(f"  Sensors health:  {bin(msg.onboard_control_sensors_health)}")
            print(f"  Battery: {msg.battery_remaining}%")
            
        elif msg_type == 'STATUSTEXT':
            print(f"[STATUS] {msg.text}")

print("\n[INFO] Diagnostic complete!")
print("\nTo arm PX4, ensure:")
print("  1. GPS lock (if required)")
print("  2. All sensors healthy")  
print("  3. Safety switch off (if physical)")
print("  4. Pre-arm checks passed")
print("\nTry manually in PX4 console:")
print("  commander arm")

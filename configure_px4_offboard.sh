#!/bin/bash

# Configure PX4 parameters for offboard mode without RC
# This script uses MAVProxy to set parameters via MAVLink

echo "========================================="
echo "  Configuring PX4 for Offboard Mode"
echo "========================================="

# Check if MAVProxy is installed
if ! command -v mavproxy.py &> /dev/null; then
    echo "[ERROR] MAVProxy not found. Installing..."
    pip3 install MAVProxy
fi

# Connect to PX4 and set parameters
echo "[INFO] Connecting to PX4 via MAVLink (UDP 14540)..."

# Set parameters using MAVLink commands
# COM_RCL_EXCEPT: RC loss exceptions (bit 2 = offboard)
# COM_RC_IN_MODE: RC input mode (4 = RC disabled)
# COM_ARM_AUTH: Arm authorization (0 = disabled)

mavproxy.py --master=udp:127.0.0.1:14540 --cmd="
param set COM_RCL_EXCEPT 4
param set COM_RC_IN_MODE 4
param set NAV_RCL_ACT 0
param set COM_ARM_SWISBTN 0
sleep 1
exit
"

echo "[SUCCESS] PX4 configured for offboard mode!"
echo ""
echo "Key parameters set:"
echo "  - COM_RCL_EXCEPT=4 (Allow offboard when RC lost)"
echo "  - COM_RC_IN_MODE=4 (Disable RC requirement)"
echo "  - NAV_RCL_ACT=0 (No action on RC loss)"
echo "  - COM_ARM_SWISBTN=0 (Disable arm switch requirement)"
echo ""
echo "You can now run: ./run_circle_demo.sh"

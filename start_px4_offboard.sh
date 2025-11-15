#!/bin/bash

echo "========================================="
echo "  PX4 SITL + Gazebo with Offboard Config"
echo "========================================="

# Set environment variable to pass extra parameters to PX4
export PX4_SIM_SPEED_FACTOR=1

cd ~/PX4-Autopilot

# Create a custom rcS script that sets parameters before starting
cat > /tmp/px4_custom_rcS <<'RCEOF'
#!/bin/sh

# Original init
. ${R}etc/init.d-posix/rcS

# Set offboard-friendly parameters
param set COM_RCL_EXCEPT 4
param set COM_RC_IN_MODE 4  
param set NAV_RCL_ACT 0
param set COM_ARM_SWISBTN 0
param set COM_OBS_AVOID 0
param set COM_ARM_AUTH 0

echo "✓ Offboard parameters configured!"
RCEOF

chmod +x /tmp/px4_custom_rcS

echo "[INFO] Starting PX4 SITL with Gazebo (x500) and offboard configuration..."
echo "[INFO] Gazebo window will open - wait for the drone to appear"
echo ""

# Start PX4 with custom init script
PX4_SYS_AUTOSTART=4001 make px4_sitl gz_x500

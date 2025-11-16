#!/bin/bash

# DDS QoS Configuration Script
# Source this before running ROS2 nodes to fix payload size issues

WORKSPACE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

export FASTRTPS_DEFAULT_PROFILES_FILE="${WORKSPACE_ROOT}/src/attack_drone/qos_profile.xml"
export RMW_FASTRTPS_USE_QOS_FROM_XML=1

echo "✓ DDS QoS profile configured:"
echo "  Profile: $FASTRTPS_DEFAULT_PROFILES_FILE"

if [ -f "$FASTRTPS_DEFAULT_PROFILES_FILE" ]; then
    echo "  Status: File found ✓"
else
    echo "  Status: File NOT found ✗"
    echo "  Please check the file path!"
fi

#!/bin/bash

echo "======================================"
echo "  ROS2 Topic Diagnostic Check"
echo "======================================"
echo ""

echo "Checking for PX4 px4_1 topics:"
echo ""

echo "Vehicle Status (px4_1):"
ros2 topic info /px4_1/fmu/out/vehicle_status --verbose 2>/dev/null || echo "  ✗ NOT FOUND"
echo ""

echo "Vehicle Local Position (px4_1):"
ros2 topic info /px4_1/fmu/out/vehicle_local_position --verbose 2>/dev/null || echo "  ✗ NOT FOUND"
echo ""

echo "All px4_1 topics:"
ros2 topic list | grep "px4_1"
echo ""

echo "======================================"
echo "  Attempting to echo vehicle_status"
echo "======================================"
echo "Listening for 2 seconds..."
timeout 2 ros2 topic echo /px4_1/fmu/out/vehicle_status --once 2>/dev/null || echo "  ✗ NO MESSAGES RECEIVED"
echo ""

echo "======================================"
echo "  MicroXRCE Agent Check"
echo "======================================"
echo "Checking if MicroXRCEAgent is running on port 8888:"
netstat -tulpn 2>/dev/null | grep 8888 || lsof -i :8888 2>/dev/null || echo "  ✗ No process found on port 8888"
echo ""

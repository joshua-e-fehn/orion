#!/bin/bash
# Quick script to launch predictor with auto-following camera

echo "========================================="
echo "  Orion Predictor with Auto-Follow Camera"
echo "========================================="
echo ""
echo "This will launch:"
echo "  1. Target TF publisher (for camera tracking)"
echo "  2. RViz with auto-follow configuration"
echo ""
echo "Make sure simulation and attacker are already running!"
echo ""

# Source the workspace
source install/setup.bash

# Launch TF publisher in background
echo "Starting target TF publisher..."
ros2 run orion_flight target_tf_publisher &
TF_PID=$!

# Wait a moment
sleep 1

# Launch RViz with config
echo "Launching RViz with auto-follow camera..."
rviz2 -d src/orion_flight/config/predictor_viz.rviz &
RVIZ_PID=$!

echo ""
echo "========================================="
echo "  Camera will AUTO-FOLLOW target drone!"
echo "========================================="
echo ""
echo "Controls:"
echo "  - Scroll wheel: Zoom in/out"
echo "  - Middle-click + drag: Pan camera"
echo "  - Right-click + drag: Rotate view"
echo ""
echo "Press Ctrl+C to stop all processes..."
echo ""

# Wait for user interrupt
trap "kill $TF_PID $RVIZ_PID 2>/dev/null; exit" INT TERM

wait

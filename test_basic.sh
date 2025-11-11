#!/bin/bash
# Basic test script for the autonomous drone simulation

set -e

echo "========================================="
echo "Autonomous Drone Simulation - Basic Tests"
echo "========================================="
echo ""

# Detect architecture
ARCH=$(uname -m)
if [ "$ARCH" = "x86_64" ]; then
    IMAGE_TAG="amd64"
elif [ "$ARCH" = "aarch64" ] || [ "$ARCH" = "arm64" ]; then
    IMAGE_TAG="arm64"
else
    echo "❌ Unsupported architecture: $ARCH"
    exit 1
fi

echo "📋 Architecture: $ARCH"
echo "🏷️  Image tag: $IMAGE_TAG"
echo ""

# Check if image exists
IMAGE_NAME="autonomous_drone_sim:$IMAGE_TAG"
if ! docker image inspect $IMAGE_NAME > /dev/null 2>&1; then
    echo "❌ Image $IMAGE_NAME not found"
    echo "Please build it first with: ./build_images.sh"
    exit 1
fi

echo "✅ Image $IMAGE_NAME found"
echo ""

# Test 1: Container starts
echo "Test 1: Container starts..."
if docker run --rm $IMAGE_NAME echo "Container works" > /dev/null 2>&1; then
    echo "✅ Test 1 passed: Container starts successfully"
else
    echo "❌ Test 1 failed: Container failed to start"
    exit 1
fi
echo ""

# Test 2: ROS2 is installed
echo "Test 2: ROS2 installation..."
ROS_VERSION=$(docker run --rm $IMAGE_NAME bash -c "source /opt/ros/humble/setup.bash && ros2 --version 2>/dev/null | head -n1")
if [ -n "$ROS_VERSION" ]; then
    echo "✅ Test 2 passed: $ROS_VERSION"
else
    echo "❌ Test 2 failed: ROS2 not found"
    exit 1
fi
echo ""

# Test 3: Workspace is built
echo "Test 3: ROS2 workspace..."
if docker run --rm $IMAGE_NAME bash -c "source /root/ros2_ws/install/setup.bash && ros2 pkg list 2>/dev/null | grep -q autonomous_drone"; then
    echo "✅ Test 3 passed: autonomous_drone package found"
else
    echo "❌ Test 3 failed: autonomous_drone package not found"
    exit 1
fi
echo ""

# Test 4: MAVROS is available
echo "Test 4: MAVROS installation..."
if docker run --rm $IMAGE_NAME bash -c "source /opt/ros/humble/setup.bash && ros2 pkg list 2>/dev/null | grep -q mavros"; then
    echo "✅ Test 4 passed: MAVROS packages found"
else
    echo "❌ Test 4 failed: MAVROS not found"
    exit 1
fi
echo ""

# Test 5: PX4 SITL is available
echo "Test 5: PX4 SITL binary..."
if docker run --rm $IMAGE_NAME test -f /root/PX4-Autopilot/build/px4_sitl_default/bin/px4; then
    echo "✅ Test 5 passed: PX4 SITL binary found"
else
    echo "❌ Test 5 failed: PX4 SITL binary not found"
    exit 1
fi
echo ""

# Test 6: Python scripts are executable
echo "Test 6: Python scripts..."
if docker run --rm $IMAGE_NAME test -x /root/ros2_ws/src/autonomous_drone/scripts/waypoint_navigator.py; then
    echo "✅ Test 6 passed: Python scripts are executable"
else
    echo "❌ Test 6 failed: Python scripts not executable"
    exit 1
fi
echo ""

echo "========================================="
echo "🎉 All basic tests passed!"
echo "========================================="
echo ""
echo "To run the full simulation:"
echo "  ./run_simulation.sh"
echo ""
echo "Or manually with:"
echo "  docker-compose up"
echo ""

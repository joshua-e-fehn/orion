#!/bin/bash

###############################################################################
# Autonomous Interceptor Flight Script
# 
# This script helps you run the interceptor drone demo
# which follows a leader drone in PX4 SITL with Gazebo simulation.
#
# Author: Orion ARM Team
# Date: November 16, 2025
###############################################################################

set -e  # Exit on error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Get the workspace root
WORKSPACE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}  Autonomous Interceptor Flight Demo${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""

# Function to print colored messages
print_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Check if workspace is built
if [ ! -d "${WORKSPACE_ROOT}/install" ]; then
    print_error "Workspace not built! Please build first:"
    echo "  cd ${WORKSPACE_ROOT}"
    echo "  colcon build"
    exit 1
fi

print_info "Workspace directory: ${WORKSPACE_ROOT}"
echo ""

# Source the workspace
print_info "Sourcing ROS2 workspace..."
source "${WORKSPACE_ROOT}/install/setup.bash"
print_success "Workspace sourced!"
echo ""

# Instructions for PX4 SITL
echo -e "${YELLOW}========================================${NC}"
echo -e "${YELLOW}  IMPORTANT: PX4 SITL Setup${NC}"
echo -e "${YELLOW}========================================${NC}"
echo ""
print_warning "Before running this script, you need to start PX4 SITL with Gazebo:"
echo ""
echo -e "  - Start the leader drone (px4_1) in Gazebo"
echo -e "  - Then start the interceptor drone (px4_2)"
echo -e "  - PX4 console should show 'pxh>'"
echo ""

# Ask user if PX4 is ready
read -p "$(echo -e ${YELLOW}Is PX4 SITL running with leader drone? [y/N]:${NC} )" -n 1 -r
echo ""
if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    print_warning "Please start PX4 SITL first, then run this script again."
    exit 0
fi

echo ""
print_success "Great! Starting the interceptor drone demo..."
echo ""

# Check for micro-ros-agent
print_info "Checking for micro-ros-agent..."
if command -v micro-ros-agent &> /dev/null; then
    print_success "micro-ros-agent found!"
    
    read -p "$(echo -e ${YELLOW}Start micro-ros-agent automatically? [Y/n]:${NC} )" -n 1 -r
    echo ""
    if [[ ! $REPLY =~ ^[Nn]$ ]]; then
        USE_AGENT="true"
    else
        USE_AGENT="false"
    fi
else
    print_warning "micro-ros-agent not found. Ensure PX4 uXRCE-DDS is configured correctly."
    USE_AGENT="false"
fi

echo ""
print_info "Launch Parameters (you can modify these):"
echo "  - Flight Height: 5.0 meters"
echo "  - Trail Length: 10 positions"
echo "  - Proportional gain (kp): 0.5"
echo ""

# Launch the interceptor node
print_info "Launching interceptor drone node (following px4_1)..."
echo ""

# NOTE:
# 1. The intercept node lives in the 'interceptor' package (not 'attack_drone').
# 2. flight_height is in NED (negative = up). Use -5.0 for 5m above origin.
# 3. kp is the proportional gain pulling px4_2 toward px4_1.

INTERCEPT_PACKAGE="interceptor"
INTERCEPT_EXEC="intercept_node"

# Sanity check: ensure executable exists
if ! ros2 pkg executables | grep -q "${INTERCEPT_PACKAGE} ${INTERCEPT_EXEC}"; then
    print_error "Executable ${INTERCEPT_EXEC} not found in package ${INTERCEPT_PACKAGE}. Did you build the workspace?"
    echo "Try: colcon build --packages-select ${INTERCEPT_PACKAGE}"
    exit 1
fi

if [ "$USE_AGENT" = "true" ]; then
    print_info "Starting micro-ROS agent was requested earlier (external). Running interceptor now..."
fi

# Launch intercept node with corrected parameters
ros2 run ${INTERCEPT_PACKAGE} ${INTERCEPT_EXEC} \
    --ros-args -p flight_height:=-5.0 -p trail_length:=10 -p kp:=0.5

echo ""
print_success "Interceptor drone demo completed!"

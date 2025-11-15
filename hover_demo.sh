#!/bin/bash

###############################################################################
# Autonomous Hover Flight Demonstration Script
# 
# This script helps you run the autonomous hover flight demo
# with RViz visualization for your PX4 drone in Gazebo simulation.
#
# Author: Orion ARM Team
# Date: November 15, 2025
###############################################################################

set -e  # Exit on error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Get the workspace root (script is in the workspace root)
WORKSPACE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}  Autonomous Hover Flight Demo${NC}"
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
echo -e "  ${GREEN}In a separate terminal, run:${NC}"
echo -e "    cd ~/PX4-Autopilot"
echo -e "    make px4_sitl gz_x500"
echo ""
echo -e "  ${GREEN}Or if using the classic Gazebo:${NC}"
echo -e "    cd ~/PX4-Autopilot"
echo -e "    make px4_sitl gazebo-classic"
echo ""
echo -e "  ${GREEN}Wait until you see:${NC}"
echo -e "    - Gazebo window opens with the drone"
echo -e "    - PX4 console shows: 'pxh>'"
echo ""

# Ask user if PX4 is ready
read -p "$(echo -e ${YELLOW}Is PX4 SITL running? [y/N]:${NC} )" -n 1 -r
echo ""
if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    print_warning "Please start PX4 SITL first, then run this script again."
    exit 0
fi

echo ""
print_success "Great! Starting the hover flight demo..."
echo ""

# Check if micro-ros-agent is needed
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
    print_warning "micro-ros-agent not found. Make sure PX4 uXRCE-DDS is configured correctly."
    USE_AGENT="false"
fi

echo ""
print_info "Launch Parameters (you can modify these):"
echo "  - Flight Height: 5.0 meters"
echo "  - Trail Length: 10 positions"
echo ""

# Launch the system
print_info "Launching hover flight with RViz..."
echo ""

# Don't use XML QoS - causes more problems than it solves
# The QoS is configured directly in the Python code now

if [ "$USE_AGENT" = "true" ]; then
    ros2 launch attack_drone hover.launch.py \
        flight_height:=5.0 \
        trail_length:=10 \
        use_micro_ros_agent:=true
else
    ros2 launch attack_drone hover.launch.py \
        flight_height:=5.0 \
        trail_length:=10 \
        use_micro_ros_agent:=false
fi

echo ""
print_success "Demo completed!"

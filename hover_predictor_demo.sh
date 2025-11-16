#!/bin/bash

###############################################################################
# Hover Demo with Target Prediction Visualization
# 
# This script launches the hover flight demo with CV or CA predictor
# to visualize predicted target trajectories in real-time.
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
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# Get the workspace root
WORKSPACE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo -e "${CYAN}========================================${NC}"
echo -e "${CYAN}  Hover Demo with Predictor${NC}"
echo -e "${CYAN}========================================${NC}"
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

# Check if PX4 is running
echo -e "${YELLOW}========================================${NC}"
echo -e "${YELLOW}  Pre-flight Checklist${NC}"
echo -e "${YELLOW}========================================${NC}"
echo ""
print_warning "Before running this demo, ensure PX4 SITL is running:"
echo ""
echo -e "  ${GREEN}In a separate terminal:${NC}"
echo -e "    cd ~/PX4-Autopilot"
echo -e "    make px4_sitl gz_x500"
echo ""
echo -e "  ${GREEN}Wait for:${NC}"
echo -e "    - Gazebo window with the drone"
echo -e "    - PX4 console showing 'pxh>'"
echo ""

# Ask user if PX4 is ready
echo -ne "${YELLOW}Is PX4 SITL running? [y/N]: ${NC}"
read -n 1 -r
echo ""
if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    print_warning "Please start PX4 SITL first, then run this script again."
    exit 0
fi

echo ""
print_success "Great! Proceeding with demo setup..."
echo ""

# Ask user which predictor to use
echo -e "${CYAN}========================================${NC}"
echo -e "${CYAN}  Select Predictor Type${NC}"
echo -e "${CYAN}========================================${NC}"
echo ""
echo "Available predictors:"
echo -e "  ${GREEN}1${NC} - CV (Constant Velocity) - Best for straight-line motion"
echo -e "  ${GREEN}2${NC} - CA (Constant Acceleration) - Best for maneuvering targets"
echo ""
echo -ne "${YELLOW}Select predictor [1-2] (default: 1): ${NC}"
read -n 1 -r
echo ""

PREDICTOR_TYPE="cv"
if [[ $REPLY =~ ^2$ ]]; then
    PREDICTOR_TYPE="ca"
    print_info "Using CA (Constant Acceleration) predictor"
else
    print_info "Using CV (Constant Velocity) predictor"
fi

echo ""

# Demo parameters
echo -e "${CYAN}========================================${NC}"
echo -e "${CYAN}  Demo Parameters${NC}"
echo -e "${CYAN}========================================${NC}"
echo ""
echo -e "  ${BLUE}Target Drone:${NC} px4_1 (main PX4 drone)"
echo -e "  ${BLUE}Prediction Horizons:${NC} 0.5s, 1.0s, 2.0s, 3.0s, 5.0s"
echo -e "  ${BLUE}Update Rate:${NC} 20 Hz"
echo -e "  ${BLUE}Predictor Type:${NC} $PREDICTOR_TYPE"
echo ""

# Ask if user wants to start micro-ros-agent
print_info "Checking for MicroXRCEAgent..."
if command -v MicroXRCEAgent &> /dev/null; then
    print_success "MicroXRCEAgent found!"
    
    echo -ne "${YELLOW}Start MicroXRCEAgent automatically? [Y/n]: ${NC}"
    read -n 1 -r
    echo ""
    if [[ ! $REPLY =~ ^[Nn]$ ]]; then
        USE_AGENT="true"
    else
        USE_AGENT="false"
    fi
else
    print_warning "MicroXRCEAgent not found."
    echo "  PX4 v1.14+ has built-in uXRCE-DDS, so this may not be needed."
    USE_AGENT="false"
fi

echo ""
echo -e "${CYAN}========================================${NC}"
echo -e "${CYAN}  Starting Demo${NC}"
echo -e "${CYAN}========================================${NC}"
echo ""

print_info "IMPORTANT: You need TWO things running:"
echo ""
echo -e "${GREEN}1. PX4 SITL (already checked) ✓${NC}"
echo -e "${GREEN}2. Hover Demo${NC} - Start this in a SEPARATE terminal:"
echo ""
echo -e "   ${BLUE}./hover_demo.sh${NC}"
echo ""
echo -e "Or manually:"
echo -e "   ${BLUE}ros2 launch attack_drone hover.launch.py${NC}"
echo ""
echo -ne "${YELLOW}Is the hover demo running? [y/N]: ${NC}"
read -n 1 -r
echo ""
if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    print_warning "Please start the hover demo first:"
    echo ""
    echo "  In a NEW terminal:"
    echo "    cd ~/Documents/Orion/orion_arm"
    echo "    source install/setup.bash"
    echo "    ./hover_demo.sh"
    echo ""
    echo "  Then run this script again."
    exit 0
fi

echo ""
print_info "Launching $PREDICTOR_TYPE predictor...
print_info "This will open RViz with visualization of:"
echo "  - Target drone position and trajectory"
echo "  - Predicted future positions at multiple horizons"
echo "  - Uncertainty ellipsoids (growing with time)"
echo ""

# Set DDS QoS profile to handle larger message sizes
export FASTRTPS_DEFAULT_PROFILES_FILE="${WORKSPACE_ROOT}/src/attack_drone/qos_profile.xml"
export RMW_FASTRTPS_USE_QOS_FROM_XML=1

# Launch the system
ros2 launch orion_flight hover_with_predictor.launch.py \
    predictor_type:=$PREDICTOR_TYPE \
    target_namespace:=px4_1 \
    prediction_update_rate:=20.0 \
    prediction_horizons:="[0.5, 1.0, 2.0, 3.0, 5.0]" \
    use_micro_ros_agent:=$USE_AGENT

echo ""
print_success "Demo completed!"

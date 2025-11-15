#!/bin/bash

###############################################################################
# Setup PX4 Dependencies Script
# 
# This script clones and sets up the required PX4 packages for the Orion
# autonomous drone project.
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

# Get the workspace root
WORKSPACE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}  PX4 Dependencies Setup${NC}"
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

# Check if src directory exists
if [ ! -d "${WORKSPACE_ROOT}/src" ]; then
    print_error "src/ directory not found!"
    exit 1
fi

cd "${WORKSPACE_ROOT}"

# Check if px4_msgs already exists
if [ -d "src/px4_msgs" ]; then
    print_warning "src/px4_msgs already exists"
    read -p "$(echo -e ${YELLOW}Remove and re-clone? [y/N]:${NC} )" -n 1 -r
    echo ""
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        print_info "Removing existing px4_msgs..."
        rm -rf src/px4_msgs
    else
        print_info "Keeping existing px4_msgs"
        PX4_MSGS_EXISTS=true
    fi
fi

# Check if px4_ros_com already exists
if [ -d "src/px4_ros_com" ]; then
    print_warning "src/px4_ros_com already exists"
    read -p "$(echo -e ${YELLOW}Remove and re-clone? [y/N]:${NC} )" -n 1 -r
    echo ""
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        print_info "Removing existing px4_ros_com..."
        rm -rf src/px4_ros_com
    else
        print_info "Keeping existing px4_ros_com"
        PX4_ROS_COM_EXISTS=true
    fi
fi

echo ""

# Clone px4_msgs if needed
if [ -z "$PX4_MSGS_EXISTS" ]; then
    print_info "Cloning px4_msgs from PX4 repository..."
    git clone https://github.com/PX4/px4_msgs.git src/px4_msgs
    
    print_info "Checking out release/1.14 branch..."
    cd src/px4_msgs
    git checkout release/1.14
    cd ../..
    
    print_success "px4_msgs cloned successfully!"
else
    print_info "Skipping px4_msgs clone (already exists)"
fi

echo ""

# Clone px4_ros_com if needed
if [ -z "$PX4_ROS_COM_EXISTS" ]; then
    print_info "Cloning px4_ros_com from PX4 repository..."
    git clone https://github.com/PX4/px4_ros_com.git src/px4_ros_com
    
    print_info "Checking out release/v1.14 branch..."
    cd src/px4_ros_com
    git checkout release/v1.14
    cd ../..
    
    print_success "px4_ros_com cloned successfully!"
else
    print_info "Skipping px4_ros_com clone (already exists)"
fi

echo ""
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}  Dependencies Setup Complete!${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""

print_info "Next steps:"
echo "  1. Install ROS2 dependencies:"
echo "     rosdep install --from-paths src --ignore-src -r -y"
echo ""
echo "  2. Build the workspace:"
echo "     colcon build --symlink-install"
echo ""
echo "  3. Source the workspace:"
echo "     source install/setup.bash"
echo ""
echo "  4. Run the demo:"
echo "     ./run_circle_demo.sh"
echo ""

print_success "Ready to build!"

#!/bin/bash

###############################################################################
# PX4 SITL Multi-Instance Simulation Startup Script
# 
# This script launches:
# 1. First PX4 instance (i=1) with Gazebo
# 2. Second PX4 instance (i=2) standalone
# 3. MicroXRCE agent for communication
#
# Each runs in a new terminal window
#
# Author: Orion ARM Team
# Date: November 15, 2025
###############################################################################

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

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

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}  PX4 SITL Multi-Instance Startup${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""

# Check if PX4-Autopilot exists
if [ ! -d ~/PX4-Autopilot ]; then
    print_error "PX4-Autopilot directory not found at ~/PX4-Autopilot"
    echo "Please clone PX4-Autopilot first:"
    echo "  git clone https://github.com/PX4/PX4-Autopilot.git ~/PX4-Autopilot"
    exit 1
fi

print_success "PX4-Autopilot found at ~/PX4-Autopilot"
echo ""

# Check if PX4 binary exists
if [ ! -f ~/PX4-Autopilot/build/px4_sitl_default/bin/px4 ]; then
    print_error "PX4 binary not found at ~/PX4-Autopilot/build/px4_sitl_default/bin/px4"
    echo "Please build PX4 first:"
    echo "  cd ~/PX4-Autopilot"
    echo "  make px4_sitl"
    exit 1
fi

print_success "PX4 binary found"
echo ""

# Function to launch in a new terminal or background
launch_in_terminal() {
    local title=$1
    local command=$2
    local no_cd=${3:-false}  # Optional parameter to skip cd
    local logfile="/tmp/${title}.log"
    
    print_info "Launching: $title"
    
    # Prepare the command with or without cd
    if [ "$no_cd" = "true" ]; then
        local full_cmd="$command"
    else
        local full_cmd="cd ~/PX4-Autopilot && $command"
    fi
    
    # Try terminal emulators, prioritizing gnome-terminal
    if command -v gnome-terminal &> /dev/null; then
        # Use gnome-terminal with proper environment handling
        gnome-terminal --title="$title" -- bash -c "$full_cmd; bash" 2>/dev/null &
    elif command -v tmux &> /dev/null; then
        # Use tmux if available
        tmux new-window -n "$title" "$full_cmd"
    elif command -v screen &> /dev/null; then
        # Use screen if available
        screen -dmS "$title" bash -c "$full_cmd"
    elif command -v xterm &> /dev/null; then
        xterm -title "$title" -e bash -c "$full_cmd; bash" &
    elif command -v konsole &> /dev/null; then
        konsole --title "$title" -e bash -c "$full_cmd; bash" &
    elif command -v xfce4-terminal &> /dev/null; then
        xfce4-terminal --title="$title" -e bash -c "$full_cmd; bash" &
    else
        print_warning "No terminal emulator found. Running in background."
        print_warning "Output saved to: $logfile"
        nohup bash -c "$full_cmd" > "$logfile" 2>&1 &
        print_info "  Background PID: $!"
    fi
    
    sleep 1
}

echo -e "${YELLOW}========================================${NC}"
echo -e "${YELLOW}  Starting PX4 Instances${NC}"
echo -e "${YELLOW}========================================${NC}"
echo ""

# Terminal 1: First PX4 instance with Gazebo (instance 1)
cmd1="PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=gz_x500 ./build/px4_sitl_default/bin/px4 -i 1"
launch_in_terminal "PX4_Instance_1_Gazebo" "$cmd1"

sleep 2

# Terminal 2: Second PX4 instance standalone (instance 2)
cmd2="PX4_GZ_STANDALONE=1 PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE=\"0,5\" PX4_SIM_MODEL=gz_x500 ./build/px4_sitl_default/bin/px4 -i 2"
launch_in_terminal "PX4_Instance_2_Standalone" "$cmd2"

sleep 2

# Terminal 3: MicroXRCE Agent
cmd3="export LD_LIBRARY_PATH=/usr/local/lib:\$LD_LIBRARY_PATH && MicroXRCEAgent udp4 -p 8888"
launch_in_terminal "MicroXRCEAgent" "$cmd3" "true"

echo ""
print_success "All instances launched in separate terminals!"
echo ""
echo -e "${YELLOW}Waiting for instances to initialize...${NC}"
sleep 5
echo ""

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}  PX4 SITL Setup Complete${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""
echo "You can now run the hover demo in another terminal:"
echo "  cd /home/victor-tipkemper/hackathon/orion"
echo "  source install/setup.bash"
echo "  ros2 launch attack_drone hover.launch.py"
echo ""
echo "Tip: To close all windows, press Ctrl+C or close each terminal manually."
echo ""

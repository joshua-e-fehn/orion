#!/bin/bash

###############################################################################
# Orion Quick Launch Script
# 
# Interactive script to easily launch simulation and drone behaviors
#
# Author: Orion ARM Team
# Date: November 15, 2025
###############################################################################

set -e

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m'

WORKSPACE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo -e "${BLUE}╔════════════════════════════════════════════════════════════════════╗${NC}"
echo -e "${BLUE}║                    Orion Quick Launch                              ║${NC}"
echo -e "${BLUE}╚════════════════════════════════════════════════════════════════════╝${NC}"
echo ""

# Source workspace
if [ ! -d "${WORKSPACE_ROOT}/install" ]; then
    echo -e "${RED}Error: Workspace not built!${NC}"
    echo "Please run: cd ${WORKSPACE_ROOT} && colcon build"
    exit 1
fi

source "${WORKSPACE_ROOT}/install/setup.bash"

echo -e "${GREEN}What would you like to launch?${NC}"
echo ""
echo "  1) Complete Simulation (2 drones + agents)"
echo "  2) Attacker Behavior (hover/circle)"
echo "  3) Interceptor System (predictor/planner)"
echo "  4) Full Demo (all in one)"
echo "  5) Exit"
echo ""
echo -ne "${YELLOW}Enter choice [1-5]: ${NC}"
read -r choice

case $choice in
    1)
        echo ""
        echo -e "${CYAN}Launching complete simulation...${NC}"
        echo ""
        ros2 launch orion_flight simulation.launch.py
        ;;
    
    2)
        echo ""
        echo -e "${GREEN}Select attacker behavior:${NC}"
        echo "  1) Hover (straight line)"
        echo "  2) Circle"
        echo ""
        echo -ne "${YELLOW}Enter choice [1-2]: ${NC}"
        read -r attacker_mode
        
        case $attacker_mode in
            1)
                echo ""
                echo -ne "${YELLOW}Flight height (meters) [default: 5.0]: ${NC}"
                read -r height
                height=${height:-5.0}
                
                echo -ne "${YELLOW}Hover duration (seconds) [default: 30.0]: ${NC}"
                read -r duration
                duration=${duration:-30.0}
                
                echo ""
                echo -e "${CYAN}Launching hover mode...${NC}"
                echo ""
                ros2 launch orion_flight attacker.launch.py \
                    mode:=hover \
                    flight_height:=${height} \
                    hover_duration:=${duration}
                ;;
            
            2)
                echo ""
                echo -ne "${YELLOW}Circle radius (meters) [default: 10.0]: ${NC}"
                read -r radius
                radius=${radius:-10.0}
                
                echo -ne "${YELLOW}Flight altitude (meters) [default: 5.0]: ${NC}"
                read -r altitude
                altitude=${altitude:-5.0}
                
                echo -ne "${YELLOW}Angular velocity (rad/s) [default: 0.5]: ${NC}"
                read -r angular_vel
                angular_vel=${angular_vel:-0.5}
                
                echo ""
                echo -e "${CYAN}Launching circle mode...${NC}"
                echo ""
                ros2 launch orion_flight attacker.launch.py \
                    mode:=circle \
                    radius:=${radius} \
                    altitude:=${altitude} \
                    angular_velocity:=${angular_vel}
                ;;
            
            *)
                echo -e "${RED}Invalid choice!${NC}"
                exit 1
                ;;
        esac
        ;;
    
    3)
        echo ""
        echo -e "${GREEN}Select operation mode:${NC}"
        echo "  1) Prediction Only"
        echo "  2) Planning Only"
        echo "  3) Full System (prediction + planning)"
        echo ""
        echo -ne "${YELLOW}Enter choice [1-3]: ${NC}"
        read -r mode_choice
        
        case $mode_choice in
            1) mode="prediction_only" ;;
            2) mode="planning_only" ;;
            3) mode="full" ;;
            *)
                echo -e "${RED}Invalid choice!${NC}"
                exit 1
                ;;
        esac
        
        # Predictor selection (if needed)
        if [ "$mode" == "prediction_only" ] || [ "$mode" == "full" ]; then
            echo ""
            echo -e "${GREEN}Select predictor:${NC}"
            echo "  1) CV (Constant Velocity) - Best for straight lines"
            echo "  2) CA (Constant Acceleration) - Best for maneuvers"
            echo ""
            echo -ne "${YELLOW}Enter choice [1-2]: ${NC}"
            read -r pred_choice
            
            case $pred_choice in
                1) predictor="cv" ;;
                2) predictor="ca" ;;
                *)
                    echo -e "${RED}Invalid choice!${NC}"
                    exit 1
                    ;;
            esac
        else
            predictor="cv"  # Default (not used in planning_only)
        fi
        
        # Planner selection (if needed)
        if [ "$mode" == "planning_only" ] || [ "$mode" == "full" ]; then
            echo ""
            echo -e "${GREEN}Select planner:${NC}"
            echo "  1) PP (Pure Pursuit)"
            echo ""
            echo -ne "${YELLOW}Enter choice [1]: ${NC}"
            read -r plan_choice
            
            case $plan_choice in
                1) planner="pp" ;;
                *)
                    echo -e "${RED}Invalid choice!${NC}"
                    exit 1
                    ;;
            esac
            
            echo ""
            echo -ne "${YELLOW}Lookahead distance (meters) [default: 3.0]: ${NC}"
            read -r lookahead
            lookahead=${lookahead:-3.0}
            
            echo -ne "${YELLOW}Max speed (m/s) [default: 5.0]: ${NC}"
            read -r max_speed
            max_speed=${max_speed:-5.0}
        else
            planner="pp"  # Default (not used in prediction_only)
            lookahead=3.0
            max_speed=5.0
        fi
        
        echo ""
        echo -e "${CYAN}Launching interceptor system...${NC}"
        echo -e "${CYAN}  Mode: ${mode}${NC}"
        if [ "$mode" != "planning_only" ]; then
            echo -e "${CYAN}  Predictor: ${predictor^^}${NC}"
        fi
        if [ "$mode" != "prediction_only" ]; then
            echo -e "${CYAN}  Planner: ${planner^^}${NC}"
            echo -e "${CYAN}  Lookahead: ${lookahead}m${NC}"
            echo -e "${CYAN}  Max Speed: ${max_speed}m/s${NC}"
        fi
        echo ""
        
        ros2 launch orion_flight interceptor.launch.py \
            mode:=${mode} \
            predictor:=${predictor} \
            planner:=${planner} \
            lookahead_distance:=${lookahead} \
            max_speed:=${max_speed}
        ;;
    
    4)
        echo ""
        echo -e "${CYAN}Full Demo Mode${NC}"
        echo ""
        echo -e "${YELLOW}This will launch a complete demonstration.${NC}"
        echo -e "${YELLOW}Please run the following commands in separate terminals:${NC}"
        echo ""
        echo -e "${GREEN}Terminal 1 (Simulation):${NC}"
        echo "  cd ~/Documents/Orion/orion_arm"
        echo "  source install/setup.bash"
        echo "  ros2 launch orion_flight simulation.launch.py"
        echo ""
        echo -e "${GREEN}Terminal 2 (Attacker - Circle):${NC}"
        echo "  cd ~/Documents/Orion/orion_arm"
        echo "  source install/setup.bash"
        echo "  ros2 launch orion_flight attacker.launch.py mode:=circle radius:=10.0"
        echo ""
        echo -e "${GREEN}Terminal 3 (Interceptor - Full System):${NC}"
        echo "  cd ~/Documents/Orion/orion_arm"
        echo "  source install/setup.bash"
        echo "  ros2 launch orion_flight interceptor.launch.py mode:=full predictor:=ca planner:=pp"
        echo ""
        echo -e "${YELLOW}Wait for each terminal to fully start before launching the next!${NC}"
        echo ""
        ;;
    
    5)
        echo ""
        echo -e "${GREEN}Goodbye!${NC}"
        exit 0
        ;;
    
    *)
        echo ""
        echo -e "${RED}Invalid choice!${NC}"
        exit 1
        ;;
esac

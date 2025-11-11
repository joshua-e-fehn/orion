#!/bin/bash
set -e

# Source ROS2 environment
source /opt/ros/${ROS_DISTRO}/setup.bash
source ${WORKSPACE}/install/setup.bash

# Set up PX4 environment
export PX4_HOME=/root/PX4-Autopilot
export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:/root/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models
export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:/root/PX4-Autopilot/build/px4_sitl_default/build_gazebo-classic

# Execute the command
exec "$@"

# Setting Up PX4 Dependencies

The `px4_msgs` and `px4_ros_com` packages are external dependencies from the official PX4 repositories. They are kept as separate git repositories (not part of this repo) to allow easy updates.

## Setup Instructions

After cloning this repository, you need to clone the PX4 dependencies:

```bash
cd ~/Documents/Orion/orion_arm

# Clone px4_msgs
git clone https://github.com/PX4/px4_msgs.git src/px4_msgs
cd src/px4_msgs
git checkout release/1.14
cd ../..

# Clone px4_ros_com  
git clone https://github.com/PX4/px4_ros_com.git src/px4_ros_com
cd src/px4_ros_com
git checkout release/v1.14  # Note: v1.14, not 1.14
cd ../..

# Build the workspace
colcon build --symlink-install
```

## Why This Approach?

- ✅ Always get latest updates from PX4
- ✅ Can track PX4 official releases
- ✅ Smaller orion repository size
- ✅ Clear separation between orion code and dependencies
- ✅ Easy to update PX4 packages independently

## Automated Setup Script

You can also use the provided setup script (if created):

```bash
./setup_dependencies.sh
```

## Package Information

- **px4_msgs**: https://github.com/PX4/px4_msgs
  - Contains all PX4 message definitions
  - Required for any PX4 ROS2 communication
  
- **px4_ros_com**: https://github.com/PX4/px4_ros_com
  - Contains PX4 ROS2 example code
  - Used as reference (you have `orion_flight` for actual flight)

## Updating PX4 Dependencies

To update to latest PX4 release:

```bash
cd src/px4_msgs
git pull
git checkout release/1.15  # or latest version

cd ../px4_ros_com
git pull  
git checkout release/v1.15  # or latest version

cd ../..
colcon build --symlink-install
```

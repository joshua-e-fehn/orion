#!/bin/bash
# Simple run script for autonomous drone simulation

set -e

# Detect architecture
ARCH=$(uname -m)
if [ "$ARCH" = "x86_64" ]; then
    IMAGE_TAG="amd64"
elif [ "$ARCH" = "aarch64" ] || [ "$ARCH" = "arm64" ]; then
    IMAGE_TAG="arm64"
else
    echo "Unsupported architecture: $ARCH"
    exit 1
fi

echo "Detected architecture: $ARCH"
echo "Using image: autonomous_drone_sim:$IMAGE_TAG"
echo ""

# Check if image exists
if ! docker image inspect autonomous_drone_sim:$IMAGE_TAG > /dev/null 2>&1; then
    echo "Image not found. Building..."
    docker buildx build --platform linux/$IMAGE_TAG -t autonomous_drone_sim:$IMAGE_TAG --load .
fi

# Enable X11 forwarding
echo "Enabling X11 forwarding..."
xhost +local:docker

echo ""
echo "Starting simulation container..."
echo "Press Ctrl+C to stop"
echo ""

# Run the container
docker run -it --rm \
    --privileged \
    --network host \
    -e DISPLAY=$DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v $HOME/.Xauthority:/root/.Xauthority:rw \
    autonomous_drone_sim:$IMAGE_TAG \
    bash -c "
        source /opt/ros/humble/setup.bash && \
        source /root/ros2_ws/install/setup.bash && \
        ros2 launch autonomous_drone simulation.launch.py
    "

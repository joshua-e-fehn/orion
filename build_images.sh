#!/bin/bash
# Build script for multi-architecture Docker images

set -e

echo "Building Docker images for ARM64 and AMD64 architectures..."

# Build for AMD64 (x86_64)
echo "Building for AMD64..."
docker buildx build --platform linux/amd64 -t autonomous_drone_sim:amd64 --load .

# Build for ARM64
echo "Building for ARM64..."
docker buildx build --platform linux/arm64 -t autonomous_drone_sim:arm64 --load .

echo "Build complete!"
echo ""
echo "Available images:"
docker images | grep autonomous_drone_sim

echo ""
echo "To run the simulation:"
echo "  AMD64: docker run -it --rm --privileged --network host -e DISPLAY=\$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix autonomous_drone_sim:amd64"
echo "  ARM64: docker run -it --rm --privileged --network host -e DISPLAY=\$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix autonomous_drone_sim:arm64"

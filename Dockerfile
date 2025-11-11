# Multi-architecture Dockerfile for ROS2 Humble with PX4, MAVROS, Gazebo, and RViz
# Supports both ARM64 and AMD64 architectures

ARG ROS_DISTRO=humble
FROM ros:${ROS_DISTRO}-ros-base

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
ENV WORKSPACE=/root/ros2_ws
ENV PX4_VERSION=v1.14.3

# Install essential tools and dependencies
RUN apt-get update && apt-get install -y \
    git \
    wget \
    curl \
    vim \
    nano \
    build-essential \
    cmake \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    lsb-release \
    gnupg2 \
    software-properties-common \
    && rm -rf /var/lib/apt/lists/*

# Install Gazebo Classic 11 (required for PX4)
RUN apt-get update && apt-get install -y \
    gazebo11 \
    libgazebo11-dev \
    && rm -rf /var/lib/apt/lists/*

# Install ROS2 packages
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-gazebo-ros-pkgs \
    ros-${ROS_DISTRO}-gazebo-ros2-control \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-rqt \
    ros-${ROS_DISTRO}-rqt-common-plugins \
    ros-${ROS_DISTRO}-geographic-msgs \
    ros-${ROS_DISTRO}-angles \
    ros-${ROS_DISTRO}-eigen3-cmake-module \
    ros-${ROS_DISTRO}-tf2-geometry-msgs \
    ros-${ROS_DISTRO}-tf2-eigen \
    && rm -rf /var/lib/apt/lists/*

# Install PX4 dependencies
RUN apt-get update && apt-get install -y \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-ugly \
    gstreamer1.0-libav \
    libeigen3-dev \
    libopencv-dev \
    libxml2-utils \
    libxml2-dev \
    protobuf-compiler \
    geographiclib-tools \
    libeigen3-dev \
    libgoogle-glog-dev \
    libgtest-dev \
    python3-empy \
    python3-toml \
    python3-numpy \
    python3-yaml \
    python3-dev \
    && rm -rf /var/lib/apt/lists/*

# Install GeographicLib datasets for MAVROS
RUN wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh && \
    chmod +x install_geographiclib_datasets.sh && \
    ./install_geographiclib_datasets.sh && \
    rm install_geographiclib_datasets.sh

# Clone PX4 Autopilot
WORKDIR /root
RUN git clone https://github.com/PX4/PX4-Autopilot.git --recursive --depth 1 --branch ${PX4_VERSION}
WORKDIR /root/PX4-Autopilot

# Build PX4 for SITL (Software In The Loop) with Gazebo Classic
RUN DONT_RUN=1 make px4_sitl_default gazebo-classic

# Create ROS2 workspace
WORKDIR ${WORKSPACE}/src

# Clone MAVROS
RUN git clone https://github.com/mavlink/mavros.git -b ros2 --depth 1

# Install MAVROS dependencies
WORKDIR ${WORKSPACE}
RUN apt-get update && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    rm -rf /var/lib/apt/lists/*

# Create the autonomous flight package
RUN mkdir -p ${WORKSPACE}/src/autonomous_drone

# Copy package files (will be created below)
COPY autonomous_drone/ ${WORKSPACE}/src/autonomous_drone/

# Build the workspace
WORKDIR ${WORKSPACE}
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    colcon build --symlink-install

# Setup environment
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc && \
    echo "source ${WORKSPACE}/install/setup.bash" >> /root/.bashrc && \
    echo "export PX4_HOME=/root/PX4-Autopilot" >> /root/.bashrc && \
    echo "export GAZEBO_MODEL_PATH=\${GAZEBO_MODEL_PATH}:/root/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models" >> /root/.bashrc && \
    echo "export GAZEBO_PLUGIN_PATH=\${GAZEBO_PLUGIN_PATH}:/root/PX4-Autopilot/build/px4_sitl_default/build_gazebo-classic" >> /root/.bashrc

# Create entrypoint script
COPY entrypoint.sh /root/entrypoint.sh
RUN chmod +x /root/entrypoint.sh

WORKDIR ${WORKSPACE}
ENTRYPOINT ["/root/entrypoint.sh"]
CMD ["bash"]

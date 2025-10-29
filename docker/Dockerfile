# ROS Melodic (Ubuntu 18.04) environment for DOFBOT simulation
FROM osrf/ros:melodic-desktop-full

# Prevent interactive prompts during installation
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=melodic

# Install essential dependencies
RUN apt-get update && apt-get install -y \
    # Build tools
    build-essential \
    cmake \
    git \
    python-catkin-tools \
    python-rosdep \
    python-rosinstall \
    python-rosinstall-generator \
    python-wstool \
    # MoveIt and Gazebo
    ros-melodic-moveit \
    ros-melodic-moveit-visual-tools \
    ros-melodic-gazebo-ros \
    ros-melodic-gazebo-ros-control \
    ros-melodic-gazebo-ros-pkgs \
    # Controllers
    ros-melodic-ros-control \
    ros-melodic-ros-controllers \
    ros-melodic-controller-manager \
    ros-melodic-joint-state-controller \
    ros-melodic-effort-controllers \
    ros-melodic-position-controllers \
    ros-melodic-velocity-controllers \
    # Kinematics and planning
    ros-melodic-kdl-parser \
    ros-melodic-kdl-conversions \
    ros-melodic-trac-ik-kinematics-plugin \
    # Vision and sensors
    ros-melodic-cv-bridge \
    ros-melodic-image-transport \
    ros-melodic-camera-info-manager \
    # Python dependencies
    python-pip \
    python-numpy \
    python-opencv \
    python-scipy \
    python-yaml \
    # Utilities
    wget \
    curl \
    vim \
    nano \
    && rm -rf /var/lib/apt/lists/*

# Install Python 2.7 packages for basic functionality
# Note: Skip pyyaml as it's already installed as system package
RUN pip install --upgrade pip && \
    pip install \
    opencv-python==4.2.0.32 \
    opencv-contrib-python==4.2.0.32 \
    numpy==1.16.6 \
    scipy==1.2.3 \
    matplotlib==2.2.5 \
    Pillow==6.2.2

# Note: YOLOv5 requires Python 3.6+, we'll add Python 3 support later
# PyYAML is already available as system package (python-yaml)

# Create workspace directory
RUN mkdir -p /root/dofbot_ws/src

# Setup ROS environment
RUN echo "source /opt/ros/melodic/setup.bash" >> /root/.bashrc
RUN echo "source /root/dofbot_ws/devel/setup.bash 2>/dev/null || true" >> /root/.bashrc

# Set working directory
WORKDIR /root/dofbot_ws

# Initialize rosdep
RUN rosdep update

# Expose ROS master port and Gazebo port
EXPOSE 11311 11345

# Default command
CMD ["/bin/bash"]

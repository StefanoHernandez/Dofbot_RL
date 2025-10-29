#!/bin/bash
# Setup script to build DOFBOT catkin workspace inside Docker container

set -e  # Exit on error

echo "=========================================="
echo "DOFBOT Workspace Setup"
echo "=========================================="

# Source ROS environment
source /opt/ros/melodic/setup.bash

# Navigate to workspace
cd /root/dofbot_ws

echo ""
echo "Step 1: Installing dependencies with rosdep..."
echo "----------------------------------------------"
# Install dependencies for all packages in src
rosdep install --from-paths src --ignore-src -r -y || true

echo ""
echo "Step 2: Building catkin workspace..."
echo "----------------------------------------------"
# Build workspace
catkin_make

echo ""
echo "Step 3: Sourcing workspace..."
echo "----------------------------------------------"
# Source the workspace
source devel/setup.bash

echo ""
echo "=========================================="
echo "Workspace setup complete!"
echo "=========================================="
echo ""
echo "Available packages:"
rospack list | grep dofbot

echo ""
echo "You can now launch simulations:"
echo "  - Gazebo only:       roslaunch dofbot_config gazebo.launch"
echo "  - Gazebo + MoveIt:   roslaunch dofbot_config demo_gazebo.launch"
echo "  - MoveIt (fake):     roslaunch dofbot_config demo.launch"
echo ""

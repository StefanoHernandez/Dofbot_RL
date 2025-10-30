#!/bin/bash
# Start ROS-MuJoCo visualization demo

echo "==================================="
echo "  DOFBOT ROS-MuJoCo Demo"
echo "==================================="
echo ""
echo "This will start:"
echo "  1. ROS + MoveIt (for planning)"
echo "  2. MuJoCo viewer (for visualization with real meshes)"
echo ""
echo "Instructions:"
echo "  - Use RViz to plan movements with MoveIt"
echo "  - See the movements in MuJoCo window in real-time"
echo ""
echo "Starting ROS-MuJoCo bridge..."
echo ""

# Source ROS workspace
source /opt/ros/noetic/setup.bash
source /root/dofbot_ws/devel/setup.bash

# Set MuJoCo rendering
export MUJOCO_GL=osmesa

# Start the bridge
python3 /root/rl_scripts/ros_mujoco_bridge.py /root/mujoco_models/dofbot.xml

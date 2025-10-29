#!/bin/bash
# Test script for DOFBOT simulation - Run inside Docker container

source /opt/ros/melodic/setup.bash
source /root/dofbot_ws/devel/setup.bash

echo "=========================================="
echo "DOFBOT Simulation Test Menu"
echo "=========================================="
echo ""
echo "Select test to run:"
echo "  1) Gazebo only (just spawn robot)"
echo "  2) Gazebo + MoveIt + RViz (full demo - RECOMMENDED)"
echo "  3) MoveIt with fake execution (no Gazebo)"
echo "  4) Test motion planning script"
echo "  5) Exit"
echo ""
read -p "Enter choice [1-5]: " choice

case $choice in
    1)
        echo ""
        echo "Launching Gazebo with DOFBOT..."
        echo "----------------------------------------------"
        echo "This will spawn the robot in an empty Gazebo world."
        echo "You should see the 6DOF arm with camera."
        echo ""
        roslaunch dofbot_config gazebo.launch
        ;;
    2)
        echo ""
        echo "Launching Gazebo + MoveIt + RViz (Full Demo)..."
        echo "----------------------------------------------"
        echo "This will launch:"
        echo "  - Gazebo simulation with robot"
        echo "  - MoveIt motion planning"
        echo "  - RViz visualization with MotionPlanning plugin"
        echo ""
        echo "In RViz you can:"
        echo "  - Drag the interactive marker to plan movements"
        echo "  - Click 'Plan' to compute trajectory"
        echo "  - Click 'Execute' to run on simulated robot"
        echo ""
        roslaunch dofbot_config demo_gazebo.launch
        ;;
    3)
        echo ""
        echo "Launching MoveIt with fake execution..."
        echo "----------------------------------------------"
        echo "This runs MoveIt without Gazebo (faster for testing)."
        echo ""
        roslaunch dofbot_config demo.launch
        ;;
    4)
        echo ""
        echo "Running motion planning test script..."
        echo "----------------------------------------------"
        echo "First, make sure you have launched option 2 in another terminal!"
        echo ""
        read -p "Have you launched demo_gazebo.launch? (y/n): " confirm
        if [ "$confirm" = "y" ]; then
            cd /root/dofbot_ws/src/dofbot_moveit/scripts
            python 02_motion_plan.py
        else
            echo "Please launch demo_gazebo.launch first, then run this again."
        fi
        ;;
    5)
        echo "Exiting..."
        exit 0
        ;;
    *)
        echo "Invalid choice. Please run script again."
        exit 1
        ;;
esac

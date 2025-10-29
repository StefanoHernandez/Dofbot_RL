# DOFBOT Simulation Environment - Docker Setup

This Docker environment provides a complete ROS Melodic setup for testing the DOFBOT robotic arm simulation with Gazebo, MoveIt, and RViz.

## Prerequisites

- Docker installed on your system
- Docker Compose installed
- X11 server running (for GUI applications)
- At least 4GB of free disk space

## Quick Start

### 1. Start the Environment

From the `docker` directory, run:

```bash
./start.sh
```

This script will:
- Configure X11 permissions for GUI applications
- Build the Docker image (first time only, ~5-10 minutes)
- Start the container in detached mode

### 2. Enter the Container

```bash
docker exec -it dofbot_simulation bash
```

You should now be inside the container at `/root/dofbot_ws`.

### 3. Build the Workspace

Inside the container, run the setup script:

```bash
./setup_workspace.sh
```

This will:
- Install ROS package dependencies
- Build the catkin workspace
- Source the workspace environment

**Expected output**: You should see packages like `dofbot_config`, `dofbot_info`, `dofbot_moveit`, etc.

### 4. Test the Simulation

Run the test menu:

```bash
./test_simulation.sh
```

Choose option **2** for the full demo (recommended for first test).

## What to Expect

### Option 2: Gazebo + MoveIt + RViz (Full Demo)

When you launch the full demo, you should see:

1. **Gazebo window**: 3D physics simulation
   - DOFBOT arm with 5 joints
   - Camera mounted on the arm
   - Empty world environment

2. **RViz window**: Visualization and planning interface
   - Robot model displayed
   - MotionPlanning plugin on the left panel
   - Interactive marker (colored ball) at end-effector

### Testing Motion Planning

In RViz:

1. **Drag the interactive marker** to a new position
2. Click **"Plan"** in the MotionPlanning panel
   - You should see a trajectory preview (ghost robot states)
3. Click **"Execute"** to run the motion
   - Watch the robot move in both RViz and Gazebo

### What This Tests

- ✅ URDF model loads correctly
- ✅ Gazebo physics simulation works
- ✅ MoveIt motion planning works
- ✅ Inverse kinematics (IK) solver works
- ✅ Trajectory execution on simulated robot
- ✅ Visualization in RViz

## Files Overview

```
docker/
├── Dockerfile              # ROS Melodic environment definition
├── docker-compose.yml      # Container configuration
├── start.sh               # Setup X11 and start container
├── setup_workspace.sh     # Build catkin workspace (run inside container)
├── test_simulation.sh     # Interactive test menu (run inside container)
└── README.md              # This file
```

## Mounted Volumes

The container mounts:
- `../Jetson-dofbot-Code (2)/dofbot_ws/src` → `/root/dofbot_ws/src` (read/write)
- `../Jetson-dofbot-Code (2)/Dofbot` → `/root/Dofbot` (read-only)
- `../dofbot-jetson_nano` → `/root/docs` (read-only, documentation)

Changes you make to the source files inside the container will persist on your host.

## Troubleshooting

### GUI windows not appearing

If Gazebo/RViz don't show up:

```bash
# On host, grant X11 access
xhost +local:docker

# Check DISPLAY variable inside container
echo $DISPLAY
```

### Build errors

If `catkin_make` fails:

```bash
# Clean workspace
cd /root/dofbot_ws
rm -rf build devel
catkin_make clean

# Reinstall dependencies
rosdep install --from-paths src --ignore-src -r -y

# Rebuild
catkin_make
```

### MoveIt planning fails

Check that:
- Robot is in a valid state (no self-collisions)
- Target pose is reachable
- Planning timeout is sufficient (increase in RViz)

### ROS connection issues

All ROS nodes run on the same host network, so `ROS_MASTER_URI` should work automatically.

## Manual Launch Commands

If you prefer not to use the test menu:

### Launch Gazebo only
```bash
roslaunch dofbot_config gazebo.launch
```

### Launch full demo (Gazebo + MoveIt + RViz)
```bash
roslaunch dofbot_config demo_gazebo.launch
```

### Launch MoveIt with fake execution (no Gazebo)
```bash
roslaunch dofbot_config demo.launch
```

## Next Steps

After confirming the simulation works:

1. **Test existing scripts**:
   - Motion planning: `/root/dofbot_ws/src/dofbot_moveit/scripts/02_motion_plan.py`
   - Kinematics service: Test IK/FK with `rosservice call /dofbot_kinemarics`

2. **Evaluate for RL project**:
   - MoveIt provides high-level motion planning (great for path planning)
   - Gazebo provides physics simulation (but slower than MuJoCo)
   - Need to add:
     - Object spawning in Gazebo (for pick-and-place)
     - Vision integration (YOLOv11)
     - RL training loop (TorchRL)
     - Reward computation
     - Sim2Real transfer capabilities

3. **Possible modifications**:
   - Switch from Gazebo to MuJoCo for faster simulation
   - Integrate YOLO11 instead of YOLOv5
   - Add randomized environments for RL
   - Create custom training loop with TorchRL

## Stopping the Environment

To stop and remove the container:

```bash
cd /path/to/DOFBOT/docker
docker-compose down
```

To stop and keep the container for later:

```bash
docker-compose stop
```

To restart a stopped container:

```bash
docker-compose start
```

## System Information

- **ROS Version**: ROS Melodic (ROS 1)
- **Ubuntu Version**: 18.04 (Bionic)
- **Gazebo Version**: 9.x
- **MoveIt Version**: 1.0.x
- **Python Version**: 2.7 (ROS Melodic default)

## Notes

- The workspace is rebuilt each time you run `setup_workspace.sh` - this is safe
- You can edit code on your host machine; changes are immediately visible in the container
- The container uses host networking for ROS communication
- X11 forwarding is used for GUI applications (Gazebo, RViz)

## Support

If you encounter issues:

1. Check this README's troubleshooting section
2. Review ROS/MoveIt logs: `roscd dofbot_config && cd ../..`
3. Verify all dependencies installed: `rosdep check --from-paths src --ignore-src`

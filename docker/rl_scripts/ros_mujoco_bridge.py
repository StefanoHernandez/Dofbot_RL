#!/usr/bin/env python3
"""
ROS-MuJoCo Bridge for DOFBOT
Subscribes to /joint_states from MoveIt and visualizes in MuJoCo
"""

import rospy
import mujoco
import mujoco.viewer
import numpy as np
from sensor_msgs.msg import JointState
import threading


class ROSMuJoCoBridge:
    def __init__(self, model_path):
        """Initialize ROS-MuJoCo bridge"""
        # Load MuJoCo model
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)

        # Joint mapping (ROS joint names to MuJoCo joint indices)
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']
        self.joint_indices = []

        # Find joint indices in MuJoCo model
        for joint_name in self.joint_names:
            try:
                idx = self.model.joint(joint_name).id
                self.joint_indices.append(idx)
                print(f"Mapped {joint_name} to MuJoCo joint index {idx}")
            except KeyError:
                print(f"Warning: Joint {joint_name} not found in MuJoCo model")

        # Current joint positions
        self.current_positions = np.zeros(len(self.joint_names))
        self.lock = threading.Lock()

        # Initialize ROS node
        rospy.init_node('ros_mujoco_bridge', anonymous=True)

        # Subscribe to joint states
        rospy.Subscriber('/joint_states', JointState, self.joint_state_callback)

        print("ROS-MuJoCo Bridge initialized!")
        print(f"Subscribed to /joint_states")
        print(f"Mapped {len(self.joint_indices)} joints")

    def joint_state_callback(self, msg):
        """Callback for /joint_states topic"""
        with self.lock:
            # Update joint positions based on incoming message
            for i, joint_name in enumerate(self.joint_names):
                if joint_name in msg.name:
                    idx = msg.name.index(joint_name)
                    self.current_positions[i] = msg.position[idx]

    def update_mujoco(self):
        """Update MuJoCo simulation with current joint positions"""
        with self.lock:
            for i, mj_idx in enumerate(self.joint_indices):
                # Set joint position in MuJoCo
                self.data.qpos[mj_idx] = self.current_positions[i]

        # Forward kinematics
        mujoco.mj_forward(self.model, self.data)

    def run_viewer(self):
        """Run MuJoCo viewer in a separate thread"""
        print("\nStarting MuJoCo viewer...")
        print("Controls:")
        print("  - Move robot in RViz with MoveIt")
        print("  - See real-time visualization in this MuJoCo window")
        print("  - Press ESC to close MuJoCo viewer")

        with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
            while viewer.is_running() and not rospy.is_shutdown():
                # Update MuJoCo with latest joint positions from ROS
                self.update_mujoco()

                # Sync viewer
                viewer.sync()

                # Sleep to match ROS rate
                rospy.sleep(0.01)  # 100 Hz


def main():
    import sys

    if len(sys.argv) < 2:
        print("Usage: ros_mujoco_bridge.py <path_to_mujoco_xml>")
        sys.exit(1)

    model_path = sys.argv[1]

    try:
        bridge = ROSMuJoCoBridge(model_path)
        bridge.run_viewer()
    except rospy.ROSInterruptException:
        print("\nShutting down ROS-MuJoCo bridge")
    except KeyboardInterrupt:
        print("\nShutting down ROS-MuJoCo bridge")


if __name__ == '__main__':
    main()

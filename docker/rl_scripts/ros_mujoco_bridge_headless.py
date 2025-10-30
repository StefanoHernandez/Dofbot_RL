#!/usr/bin/env python3
"""
ROS-MuJoCo Bridge for DOFBOT (Headless mode with video recording)
Subscribes to /joint_states from MoveIt and creates video of MuJoCo simulation
"""

import rospy
import mujoco
import numpy as np
from sensor_msgs.msg import JointState
import threading
import time
import os


class ROSMuJoCoBridge:
    def __init__(self, model_path, record_video=True):
        """Initialize ROS-MuJoCo bridge"""
        # Load MuJoCo model
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)

        # Create offscreen renderer
        self.renderer = mujoco.Renderer(self.model, height=720, width=1280)

        # Joint mapping
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']
        self.joint_indices = []

        # Find joint indices
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
        self.last_update_time = time.time()

        # Recording
        self.record_video = record_video
        self.frames = []
        self.max_frames = 1000  # Limit to avoid memory issues

        # Initialize ROS node
        rospy.init_node('ros_mujoco_bridge', anonymous=True)

        # Subscribe to joint states
        rospy.Subscriber('/joint_states', JointState, self.joint_state_callback)

        print("ROS-MuJoCo Bridge initialized!")
        print(f"Subscribed to /joint_states")
        print(f"Mapped {len(self.joint_indices)} joints")
        print(f"Recording: {record_video}")

    def joint_state_callback(self, msg):
        """Callback for /joint_states topic"""
        with self.lock:
            # Update joint positions
            for i, joint_name in enumerate(self.joint_names):
                if joint_name in msg.name:
                    idx = msg.name.index(joint_name)
                    self.current_positions[i] = msg.position[idx]

            self.last_update_time = time.time()

    def update_mujoco(self):
        """Update MuJoCo simulation with current joint positions"""
        with self.lock:
            for i, mj_idx in enumerate(self.joint_indices):
                self.data.qpos[mj_idx] = self.current_positions[i]

        # Forward kinematics
        mujoco.mj_forward(self.model, self.data)

    def render_frame(self):
        """Render current frame"""
        self.update_mujoco()
        self.renderer.update_scene(self.data)
        pixels = self.renderer.render()
        return pixels

    def run_headless(self, duration=30):
        """Run in headless mode, recording frames"""
        print(f"\nRunning MuJoCo simulation in headless mode...")
        print(f"Duration: {duration} seconds")
        print(f"Waiting for ROS messages...")
        print()

        rate = rospy.Rate(30)  # 30 FPS
        start_time = time.time()
        frame_count = 0

        while not rospy.is_shutdown() and (time.time() - start_time) < duration:
            # Render frame
            frame = self.render_frame()

            # Record if enabled
            if self.record_video and len(self.frames) < self.max_frames:
                self.frames.append(frame.copy())

            frame_count += 1

            # Print status
            if frame_count % 30 == 0:  # Every second
                elapsed = time.time() - start_time
                time_since_update = time.time() - self.last_update_time
                print(f"Time: {elapsed:.1f}s | Frames: {frame_count} | "
                      f"Last ROS update: {time_since_update:.2f}s ago | "
                      f"Joints: {self.current_positions}")

            rate.sleep()

        print(f"\nSimulation complete!")
        print(f"Total frames rendered: {frame_count}")
        print(f"Frames recorded: {len(self.frames)}")

        return frame_count

    def save_video(self, output_path="/root/rl_scripts/mujoco_output.mp4"):
        """Save recorded frames as video"""
        if not self.frames:
            print("No frames to save!")
            return

        try:
            import imageio
            print(f"\nSaving video to: {output_path}")
            imageio.mimsave(output_path, self.frames, fps=30)
            print(f"✓ Video saved successfully!")
            print(f"  Frames: {len(self.frames)}")
            print(f"  Duration: {len(self.frames)/30:.1f}s")
        except ImportError:
            print("imageio not installed. Saving first frame as image instead...")
            import matplotlib.pyplot as plt
            plt.imsave("/root/rl_scripts/mujoco_frame.png", self.frames[0])
            print("✓ First frame saved as mujoco_frame.png")

    def save_current_frame(self, output_path="/root/rl_scripts/mujoco_current.png"):
        """Save current frame as image"""
        frame = self.render_frame()

        try:
            import matplotlib.pyplot as plt
            plt.imsave(output_path, frame)
            print(f"✓ Frame saved to: {output_path}")
        except ImportError:
            # Fallback to imageio
            import imageio
            imageio.imwrite(output_path, frame)
            print(f"✓ Frame saved to: {output_path}")


def main():
    import sys

    if len(sys.argv) < 2:
        print("Usage: ros_mujoco_bridge_headless.py <path_to_mujoco_xml> [duration]")
        sys.exit(1)

    model_path = sys.argv[1]
    duration = int(sys.argv[2]) if len(sys.argv) > 2 else 30

    try:
        bridge = ROSMuJoCoBridge(model_path, record_video=True)

        # Run simulation
        bridge.run_headless(duration=duration)

        # Save video
        bridge.save_video()

        # Save final frame
        bridge.save_current_frame()

    except rospy.ROSInterruptException:
        print("\nShutting down ROS-MuJoCo bridge")
    except KeyboardInterrupt:
        print("\nShutting down ROS-MuJoCo bridge")


if __name__ == '__main__':
    main()

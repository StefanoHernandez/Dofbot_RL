#!/usr/bin/env python3
"""
Test MuJoCo viewer with different rendering options
"""

import os
import sys

# Try different OpenGL contexts
print("Testing MuJoCo viewer with different configurations...\n")

# Configuration 1: Default
print("=== Test 1: Default configuration ===")
try:
    import mujoco
    import mujoco.viewer

    model = mujoco.MjModel.from_xml_path('/root/mujoco_models/dofbot.xml')
    data = mujoco.MjData(model)

    print("✓ Model loaded successfully")
    print("Attempting to launch viewer...")

    # Set initial pose
    for i in range(model.nq):
        data.qpos[i] = 0.0

    mujoco.mj_forward(model, data)

    print("Starting viewer (will run for 5 seconds)...")
    print("If a window appears, MuJoCo viewer is working!")

    with mujoco.viewer.launch_passive(model, data) as viewer:
        import time
        start = time.time()
        while viewer.is_running() and (time.time() - start) < 5:
            viewer.sync()
            time.sleep(0.01)

    print("✓ Viewer test completed successfully!\n")

except Exception as e:
    print(f"✗ Viewer test failed: {e}\n")

# Configuration 2: Try with EGL
print("=== Test 2: With EGL backend ===")
try:
    os.environ['MUJOCO_GL'] = 'egl'
    import mujoco

    model = mujoco.MjModel.from_xml_path('/root/mujoco_models/dofbot.xml')
    data = mujoco.MjData(model)

    # Create renderer (headless)
    renderer = mujoco.Renderer(model, height=480, width=640)

    print("✓ EGL rendering works!")
    print("  (This mode doesn't open windows but can save images)\n")

except Exception as e:
    print(f"✗ EGL test failed: {e}\n")

print("=" * 50)
print("Summary:")
print("If Test 1 succeeded: MuJoCo viewer will work!")
print("If only Test 2 succeeded: Use headless mode with image/video saving")
print("=" * 50)

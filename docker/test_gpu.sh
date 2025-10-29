#!/bin/bash
# Test GPU availability inside Docker container
# Run this INSIDE the container after it starts

echo "=========================================="
echo "GPU Test inside DOFBOT Container"
echo "=========================================="
echo ""

# Check if nvidia-smi is available (it should be from the host)
echo "Test 1: nvidia-smi availability"
echo "----------------------------------------------"
if command -v nvidia-smi &> /dev/null; then
    nvidia-smi
    echo ""
    echo "✓ nvidia-smi works!"
else
    echo "✗ nvidia-smi not found"
    echo "GPU support may not be available"
    exit 1
fi

echo ""
echo "Test 2: CUDA environment variables"
echo "----------------------------------------------"
echo "NVIDIA_VISIBLE_DEVICES: $NVIDIA_VISIBLE_DEVICES"
echo "NVIDIA_DRIVER_CAPABILITIES: $NVIDIA_DRIVER_CAPABILITIES"

echo ""
echo "Test 3: GPU devices in /dev"
echo "----------------------------------------------"
ls -la /dev/nvidia* 2>/dev/null || echo "No NVIDIA devices found in /dev/"

echo ""
echo "Test 4: GPU libraries"
echo "----------------------------------------------"
ldconfig -p | grep -i nvidia | head -5 || echo "No NVIDIA libraries found"

echo ""
echo "=========================================="
echo "Summary"
echo "=========================================="
echo ""
echo "If nvidia-smi worked above, your GPU is available!"
echo ""
echo "For Gazebo to use GPU:"
echo "  1. Launch with: roslaunch dofbot_config demo_gazebo.launch"
echo "  2. Check Gazebo logs for GPU acceleration messages"
echo ""
echo "To verify GPU usage during simulation:"
echo "  Run 'nvidia-smi' in another terminal while Gazebo is running"
echo "  You should see 'gazebo' or 'gzclient' processes using GPU"
echo ""

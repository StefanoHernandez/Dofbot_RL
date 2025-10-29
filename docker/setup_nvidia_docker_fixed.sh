#!/bin/bash
# Setup NVIDIA Container Toolkit for Docker CLI daemon (FIXED VERSION)
# This enables GPU support in Docker containers

set -e

echo "=========================================="
echo "NVIDIA Docker Setup for CLI Daemon (v2)"
echo "=========================================="
echo ""

# Check if running as root
if [ "$EUID" -ne 0 ]; then
    echo "This script needs sudo privileges."
    echo "Please run with: sudo ./setup_nvidia_docker_fixed.sh"
    exit 1
fi

# Check if NVIDIA driver is installed
echo "Step 1: Checking NVIDIA driver..."
if ! command -v nvidia-smi &> /dev/null; then
    echo "ERROR: NVIDIA driver not found!"
    echo "Please install NVIDIA drivers first."
    exit 1
fi

echo "✓ NVIDIA driver found:"
nvidia-smi --query-gpu=name,driver_version --format=csv,noheader
echo ""

# Clean up old/broken configurations
echo "Step 2: Cleaning up old configurations..."
rm -f /etc/apt/sources.list.d/nvidia-container-toolkit.list
rm -f /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
echo "✓ Cleanup done"
echo ""

# Install NVIDIA Container Toolkit using official method
echo "Step 3: Installing NVIDIA Container Toolkit..."
echo ""

# Add NVIDIA GPG key
echo "Adding NVIDIA GPG key..."
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | \
    gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg

# Add generic repository (works for all Ubuntu/Debian)
echo "Adding NVIDIA repository..."
curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
    sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
    tee /etc/apt/sources.list.d/nvidia-container-toolkit.list

# Update package list
echo ""
echo "Updating package list..."
apt-get update

# Install toolkit
echo ""
echo "Installing nvidia-container-toolkit..."
apt-get install -y nvidia-container-toolkit

echo "✓ NVIDIA Container Toolkit installed"
echo ""

# Configure Docker daemon
echo "Step 4: Configuring Docker daemon..."
nvidia-ctk runtime configure --runtime=docker

echo "✓ Docker daemon configured"
echo ""

# Restart Docker service
echo "Step 5: Restarting Docker daemon..."
systemctl restart docker

echo "✓ Docker daemon restarted"
echo ""

# Test GPU support
echo "Step 6: Testing GPU support in Docker..."
echo ""
docker run --rm --gpus all nvidia/cuda:11.8.0-base-ubuntu22.04 nvidia-smi

if [ $? -eq 0 ]; then
    echo ""
    echo "=========================================="
    echo "✓ SUCCESS!"
    echo "=========================================="
    echo ""
    echo "NVIDIA Docker is now configured and working!"
    echo ""
    echo "You can now use GPU in your containers with:"
    echo "  docker run --gpus all ..."
    echo ""
    echo "Next step: Run ./start.sh to launch DOFBOT container"
    echo ""
else
    echo ""
    echo "=========================================="
    echo "✗ FAILED"
    echo "=========================================="
    echo ""
    echo "GPU test failed. Please check the error messages above."
    exit 1
fi

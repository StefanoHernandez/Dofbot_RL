#!/bin/bash
# Setup NVIDIA Container Toolkit for Docker CLI daemon
# This enables GPU support in Docker containers

set -e

echo "=========================================="
echo "NVIDIA Docker Setup for CLI Daemon"
echo "=========================================="
echo ""

# Check if running as root
if [ "$EUID" -ne 0 ]; then
    echo "This script needs sudo privileges."
    echo "Please run with: sudo ./setup_nvidia_docker.sh"
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

# Check if Docker Desktop is running (and stop it if needed)
echo "Step 2: Checking Docker Desktop..."
if systemctl is-active --quiet docker.service && pgrep -f "Docker Desktop" > /dev/null; then
    echo "WARNING: Docker Desktop is running!"
    echo "To use CLI daemon with GPU, you should:"
    echo "  1. Stop Docker Desktop"
    echo "  2. Use system Docker daemon instead"
    echo ""
    read -p "Do you want to continue anyway? (y/n): " choice
    if [ "$choice" != "y" ]; then
        echo "Aborting. Please stop Docker Desktop and run this script again."
        exit 1
    fi
fi

# Install NVIDIA Container Toolkit
echo "Step 3: Installing NVIDIA Container Toolkit..."
echo ""

# Add NVIDIA GPG key
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | \
    gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg

# Add repository
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -s -L https://nvidia.github.io/libnvidia-container/$distribution/libnvidia-container.list | \
    sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
    tee /etc/apt/sources.list.d/nvidia-container-toolkit.list

# Update package list
echo "Updating package list..."
apt-get update

# Install toolkit
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
    echo "Or in docker-compose with:"
    echo "  deploy:"
    echo "    resources:"
    echo "      reservations:"
    echo "        devices:"
    echo "          - driver: nvidia"
    echo "            count: all"
    echo "            capabilities: [gpu]"
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

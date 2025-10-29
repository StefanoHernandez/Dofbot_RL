#!/bin/bash
# Startup script to prepare X11 and launch Docker container

echo "=========================================="
echo "DOFBOT Docker Environment Startup"
echo "=========================================="

# Detect docker-compose command (V1 or V2)
if command -v docker-compose &> /dev/null; then
    COMPOSE_CMD="docker-compose"
elif docker compose version &> /dev/null; then
    COMPOSE_CMD="docker compose"
else
    echo "Error: Docker Compose not found!"
    echo "Please install Docker Compose: https://docs.docker.com/compose/install/"
    exit 1
fi

echo "Using: $COMPOSE_CMD"
echo ""

# Setup X11 for Docker
echo "Setting up X11 permissions for GUI applications..."
xhost +local:docker

# Create .Xauthority file if it doesn't exist
if [ ! -f /tmp/.docker.xauth ]; then
    touch /tmp/.docker.xauth
    xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f /tmp/.docker.xauth nmerge -
    chmod 644 /tmp/.docker.xauth
fi

echo "X11 setup complete."
echo ""

# Change to docker directory
cd "$(dirname "$0")"

# Check if we need to build
if [ ! "$(docker images -q dofbot_melodic:latest 2> /dev/null)" ]; then
    echo "Docker image not found. Building..."
    $COMPOSE_CMD build
    echo ""
fi

# Start container
echo "Starting DOFBOT simulation container..."
echo ""
$COMPOSE_CMD up -d

# Check if container started successfully
if [ $? -eq 0 ]; then
    echo "=========================================="
    echo "Container started successfully!"
    echo "=========================================="
    echo ""
    echo "To enter the container, run:"
    echo "  docker exec -it dofbot_simulation bash"
    echo ""
    echo "Once inside, setup the workspace:"
    echo "  ./setup_workspace.sh"
    echo ""
    echo "Then test the simulation:"
    echo "  ./test_simulation.sh"
    echo ""
    echo "To stop the container:"
    echo "  cd docker && $COMPOSE_CMD down"
    echo ""
else
    echo "Error: Failed to start container."
    exit 1
fi

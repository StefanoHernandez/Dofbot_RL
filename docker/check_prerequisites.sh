#!/bin/bash
# Check prerequisites before starting Docker environment

echo "=========================================="
echo "DOFBOT Docker Prerequisites Check"
echo "=========================================="
echo ""

ERRORS=0

# Check Docker
echo -n "Checking Docker... "
if command -v docker &> /dev/null; then
    echo "✓ OK ($(docker --version))"
else
    echo "✗ MISSING"
    echo "  Please install Docker: https://docs.docker.com/get-docker/"
    ERRORS=$((ERRORS + 1))
fi

# Check Docker Compose
echo -n "Checking Docker Compose... "
if command -v docker-compose &> /dev/null; then
    echo "✓ OK ($(docker-compose --version))"
elif docker compose version &> /dev/null; then
    echo "✓ OK ($(docker compose version))"
else
    echo "✗ MISSING"
    echo "  Please install Docker Compose: https://docs.docker.com/compose/install/"
    ERRORS=$((ERRORS + 1))
fi

# Check Docker daemon
echo -n "Checking Docker daemon... "
if docker info &> /dev/null; then
    echo "✓ Running"
else
    echo "✗ NOT RUNNING"
    echo "  Please start Docker daemon"
    ERRORS=$((ERRORS + 1))
fi

# Check X11 (for GUI)
echo -n "Checking X11 server... "
if [ -n "$DISPLAY" ]; then
    echo "✓ OK (DISPLAY=$DISPLAY)"
else
    echo "✗ DISPLAY not set"
    echo "  GUI applications may not work"
    ERRORS=$((ERRORS + 1))
fi

# Check disk space
echo -n "Checking disk space... "
AVAILABLE=$(df . | tail -1 | awk '{print $4}')
AVAILABLE_GB=$((AVAILABLE / 1024 / 1024))
if [ $AVAILABLE_GB -ge 4 ]; then
    echo "✓ OK (${AVAILABLE_GB} GB available)"
else
    echo "⚠ WARNING (only ${AVAILABLE_GB} GB available, need 4+ GB)"
    echo "  Docker image build may fail"
fi

# Check workspace directory
echo -n "Checking workspace directory... "
WORKSPACE_DIR="../Jetson-dofbot-Code (2)/dofbot_ws/src"
if [ -d "$WORKSPACE_DIR" ]; then
    PACKAGES=$(find "$WORKSPACE_DIR" -name "package.xml" | wc -l)
    echo "✓ OK ($PACKAGES ROS packages found)"
else
    echo "✗ NOT FOUND"
    echo "  Expected: $WORKSPACE_DIR"
    ERRORS=$((ERRORS + 1))
fi

# Check URDF file
echo -n "Checking URDF file... "
URDF_FILE="../Jetson-dofbot-Code (2)/dofbot_ws/src/dofbot_info/urdf/dofbot.urdf"
if [ -f "$URDF_FILE" ]; then
    echo "✓ OK"
else
    echo "✗ NOT FOUND"
    echo "  Expected: $URDF_FILE"
    ERRORS=$((ERRORS + 1))
fi

echo ""
echo "=========================================="
if [ $ERRORS -eq 0 ]; then
    echo "✓ All checks passed!"
    echo "=========================================="
    echo ""
    echo "Ready to start. Run:"
    echo "  ./start.sh"
    echo ""
    exit 0
else
    echo "✗ $ERRORS error(s) found"
    echo "=========================================="
    echo ""
    echo "Please fix the errors above before continuing."
    echo ""
    exit 1
fi

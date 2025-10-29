#!/bin/bash
# Fix script to install libdofbot_kinemarics.so in system path
# Run this INSIDE the container before building workspace

echo "=========================================="
echo "Fixing libdofbot_kinemarics.so location"
echo "=========================================="
echo ""

# Find the library files
LIB1="/root/dofbot_ws/src/dofbot_info/include/dofbot_info/libdofbot_kinemarics.so"
LIB2="/root/dofbot_ws/src/dofbot_moveit/include/dofbot_moveit/libdofbot_kinemarics.so"

echo "Step 1: Copying library to /usr/local/lib..."

if [ -f "$LIB1" ]; then
    cp "$LIB1" /usr/local/lib/
    echo "✓ Copied from dofbot_info"
elif [ -f "$LIB2" ]; then
    cp "$LIB2" /usr/local/lib/
    echo "✓ Copied from dofbot_moveit"
else
    echo "✗ Library not found in expected locations!"
    echo "Searching..."
    find /root/dofbot_ws -name "libdofbot_kinemarics.so" 2>/dev/null
    exit 1
fi

echo ""
echo "Step 2: Updating library cache..."
ldconfig

echo ""
echo "Step 3: Verifying library is found..."
ldconfig -p | grep dofbot_kinemarics

if [ $? -eq 0 ]; then
    echo ""
    echo "=========================================="
    echo "✓ Library fixed successfully!"
    echo "=========================================="
    echo ""
    echo "You can now run: ./setup_workspace.sh"
    echo ""
else
    echo ""
    echo "✗ Library still not found by linker"
    echo "Please check manually"
    exit 1
fi

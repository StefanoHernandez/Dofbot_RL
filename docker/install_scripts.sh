#!/bin/bash
# Install helper scripts into the container workspace
# Run this INSIDE the container after setup_workspace.sh

echo "Installing helper scripts to /root/dofbot_ws..."

# Copy scripts to workspace root
cp /root/dofbot_ws/src/../setup_workspace.sh /root/dofbot_ws/ 2>/dev/null || \
   echo "Note: setup_workspace.sh should already be in workspace"

cp /root/dofbot_ws/src/../test_simulation.sh /root/dofbot_ws/ 2>/dev/null || \
   echo "Note: test_simulation.sh should already be in workspace"

chmod +x /root/dofbot_ws/*.sh 2>/dev/null || true

echo "Done! Scripts are ready in /root/dofbot_ws/"
ls -la /root/dofbot_ws/*.sh 2>/dev/null || echo "Scripts not found - they should be mounted"

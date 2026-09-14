#!/bin/bash
# Bounce WallE_5 only. Camera and vision stay running.
set -e

echo "Stopping supervisor..."
sudo systemctl stop walle-supervisor.service 2>/dev/null || true
tmux kill-session -t walle 2>/dev/null || true
pkill -f WallE_5.py 2>/dev/null || true
sleep 1

echo "Installing unit (if you edited it)..."
sudo cp ~/my_vision_robot/deploy/walle-supervisor.service /etc/systemd/system/
sudo systemctl daemon-reload

echo "Starting supervisor..."
sudo systemctl start walle-supervisor.service

echo
systemctl is-active walle-supervisor.service
echo "  Run tail -f ~/my_vision_robot/logs/walle.log"
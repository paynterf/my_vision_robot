#!/bin/bash
# Bounce WallE_5 in a one-pane tmux session (real TTY for C/5).
# Camera/vision services are left running.
set -e

SESSION=walle
WALLE=~/my_vision_robot/software/scripts/WallE_5.py
ROS_SETUP=~/my_vision_robot/software/ros2/install/setup.bash

echo "Stopping systemd supervisor and any old WallE..."
sudo systemctl stop walle-supervisor.service 2>/dev/null || true
tmux kill-session -t "$SESSION" 2>/dev/null || true
pkill -f WallE_5.py 2>/dev/null || true
sleep 1

echo "Starting WallE in tmux session '$SESSION'..."
tmux new-session -d -s "$SESSION" -n walle \
  "bash -lc 'source /opt/ros/jazzy/setup.bash; source $ROS_SETUP; exec python3 -u $WALLE'"
sleep 2
tmux send-keys -t "$SESSION" Enter

echo
echo "  attach:     tmux attach -t $SESSION"
echo "  detach:     Ctrl-b d"
echo "  walle log:  tail -f ~/my_vision_robot/logs/walle.log"
echo "  telem log:  tail -f ~/my_vision_robot/logs/telemetry.log"
tmux ls
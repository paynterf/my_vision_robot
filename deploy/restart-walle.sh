#!/bin/bash
# Bounce WallE_5 in a one-pane tmux session (real TTY for C/5).
# Camera/vision services are left running.
set -euo pipefail

SESSION=walle
WALLE=~/my_vision_robot/software/scripts/WallE_5.py
JAZZY_SETUP=/opt/ros/jazzy/setup.bash
ROS_SETUP=~/my_vision_robot/software/ros2/install/setup.bash
STOP_TIMEOUT=8

ts() { date '+%H:%M:%S'; }
say() { echo "[$(ts)] $*"; }

for f in "$JAZZY_SETUP" "$ROS_SETUP" "$WALLE"; do
  if [[ ! -f $f ]]; then
    say "missing file: $f"
    exit 1
  fi
done
say "setup + WallE files ok"

if ! sudo -n true 2>/dev/null; then
  say "sudo needs a password (one-time for this login)"
  sudo -v
fi
say "sudo ok"

say "Stopping supervisor (max ${STOP_TIMEOUT}s, then SIGKILL)..."
sudo timeout "$STOP_TIMEOUT" systemctl stop walle-supervisor.service 2>/dev/null || true
sudo systemctl kill -s SIGKILL walle-supervisor.service 2>/dev/null || true
sudo systemctl reset-failed walle-supervisor.service 2>/dev/null || true
say "supervisor down (or ignored)"

say "Killing old WallE process (tmux session kept until replace)..."
pkill -f WallE_5.py 2>/dev/null || true
sleep 0.5

say "Replacing tmux session '$SESSION'..."
tmux kill-session -t "$SESSION" 2>/dev/null || true
tmux new-session -d -s "$SESSION" -n "$SESSION" \
  "bash -lc 'source $JAZZY_SETUP; source $ROS_SETUP; exec python3 -u $WALLE'"
sleep 0.5

echo
echo "  attach NOW: tmux attach -t $SESSION"
echo "  panic:      C  or  5"
echo "  detach:     Ctrl-b d"
echo "  walle log:  tail -f ~/my_vision_robot/logs/walle.log"
echo "  telem log:  tail -f ~/my_vision_robot/logs/telemetry.log"
tmux ls
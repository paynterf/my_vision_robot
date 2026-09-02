#!/bin/bash
# pane 0 = WallE_5  |  pane 1 = telemetry log
set -euo pipefail

SESSION=walle
ROOT=/home/pi/my_vision_robot
LOG="$ROOT/logs/telemetry.log"
SETUP="$ROOT/software/ros2/install/setup.bash"
WALLE="$ROOT/software/scripts/WallE_5.py"

mkdir -p "$ROOT/logs"
touch "$LOG"

if tmux has-session -t "$SESSION" 2>/dev/null; then
  exit 0
fi

tmux new-session -d -s "$SESSION" -n monitor \
  "bash -lc 'source \"$SETUP\"; python3 \"$WALLE\"; echo; echo WallE_5 exited.; exec bash'"

tmux split-window -v -t "$SESSION":monitor \
  "bash -lc 'echo --- telemetry.log ---; tail -n 50 -F \"$LOG\"'"

tmux select-layout -t "$SESSION":monitor even-vertical
tmux select-pane -t "$SESSION":monitor.0

exit 0
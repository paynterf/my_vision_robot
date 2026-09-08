#!/bin/bash
set -e
python3 -m py_compile /home/pi/my_vision_robot/software/scripts/WallE_5.py
tmux send-keys -t walle "5" Enter
sleep 0.3
tmux send-keys -t walle C-c
sleep 1
tmux send-keys -t walle "python3 /home/pi/my_vision_robot/software/scripts/WallE_5.py" Enter
echo "WallE_5 restarted in tmux session 'walle'"
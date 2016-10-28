#!/bin/bash

SESSION=$USER

tmux -2 new-session -d -s $SESSION
# Setup a window for tailing log files
tmux new-window -t $SESSION:0 -n 'basic'
tmux new-window -t $SESSION:1 -n 'camera'
tmux new-window -t $SESSION:2 -n 'whycon'
tmux new-window -t $SESSION:3 -n 'Moveit!'
tmux new-window -t $SESSION:4 -n 'Cleaning'

tmux select-window -t $SESSION:0
tmux split-window -v
tmux select-pane -t 0
tmux send-keys "cd ~/baxter_ws" C-m
tmux send-keys "./baxter.sh" C-m
tmux send-keys "rosrun interactive_face zoidberg.py" C-m
tmux resize-pane -U 30
tmux select-pane -t 1
tmux send-keys "cd ~/baxter_ws" C-m
tmux send-keys "./baxter.sh" C-m
tmux send-keys "rosrun baxter_interface joint_trajectory_action_server.py"

tmux select-window -t $SESSION:1
tmux split-window -v
tmux select-pane -t 0
tmux send-keys "cd ~/baxter_ws" C-m
tmux send-keys "./baxter.sh" C-m
tmux send-keys "roslaunch zoidbot_tools camera.launch"
tmux select-pane -t 1
tmux send-keys "cd ~/baxter_ws" C-m
tmux send-keys "./baxter.sh" C-m
tmux send-keys "rosrun openni_tracker openni_tracker _camera_frame_id:=xtion_camera_depth_frame"


tmux select-window -t $SESSION:2
tmux send-keys "cd ~/baxter_ws" C-m
tmux send-keys "./baxter.sh" C-m
tmux send-keys "roslaunch zoidbot_tools whycon.launch"

tmux select-window -t $SESSION:3
tmux send-keys "cd ~/baxter_ws" C-m
tmux send-keys "./baxter.sh" C-m
tmux send-keys "roslaunch baxter_moveit_config demo_baxter.launch"

tmux select-window -t $SESSION:4
tmux split-window -v
tmux select-pane -t 0
tmux send-keys "cd ~/baxter_ws" C-m
tmux send-keys "./baxter.sh" C-m
tmux send-keys "rosrun zoidbot_tools baxter_grab.py"
tmux select-pane -t 1
tmux send-keys "cd ~/baxter_ws" C-m
tmux send-keys "./baxter.sh" C-m
tmux send-keys "rosrun zoidbot_tools clean_table.py"

# Set default window
tmux select-window -t $SESSION:0

# Attach to session
tmux -2 attach-session -t $SESSION

tmux setw -g mode-mouse on
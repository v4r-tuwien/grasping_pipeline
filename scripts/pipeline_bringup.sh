#!/bin/bash

SESSION=VEREFINE



tmux -2 new-session -d -s $SESSION
tmux set -g mouse on

tmux new-window -t $SESSION:0 
tmux new-window -t $SESSION:1 

## with map setting
tmux select-window -t $SESSION:0
tmux split-window -h
tmux split-window -v
tmux select-pane -t 0
tmux split-window -v
tmux select-pane -t 0
tmux send-keys "source /home/v4r/moveit_ws/devel/setup.bash" C-m
tmux select-pane -t 1
tmux send-keys "source /home/v4r/Markus_L/devel/setup.bash" C-m
tmux select-pane -t 0
tmux send-keys "roslaunch hsrb_moveit_config move_group.launch"
tmux send-keys enter
tmux select-pane -t 1
tmux send-keys "roslaunch hsrb_grasping verefine_grasping.launch"
tmux send-keys enter
tmux select-pane -t 2
tmux send-keys "source /home/v4r/Markus_L/devel/setup.bash" C-m
tmux send-keys "roslaunch grasping_pipeline statemachine.launch"

tmux rename-window 'grasping'

# Attach to session
tmux -2 attach-session -t $SESSION



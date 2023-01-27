#!/bin/bash

SESSION=GRASPING_PIPELINE

tmux -2 new-session -d -s $SESSION
tmux set -g mouse on

tmux new-window -t $SESSION:0 
tmux new-window -t $SESSION:1 

## with map setting
tmux select-window -t $SESSION:0
tmux split-window -h
tmux select-pane -t 0
tmux send-keys "hsrb_mode" C-m
tmux select-pane -t 1
tmux send-keys "hsrb_mode" C-m
tmux select-pane -t 0
tmux send-keys "roslaunch hsrb_moveit_config move_group.launch"
tmux send-keys enter
tmux select-pane -t 1
tmux send-keys "hsrb_mode" C-m
tmux send-keys "roslaunch grasping_pipeline statemachine.launch"
tmux send-keys enter
tmux select-window -t $SESSION:1
tmux split-window -h
tmux select-pane -t 0
tmux send-keys "hsrb_mode" C-m
tmux send-keys "roslaunch haf_grasping haf_grasping_all.launch" C-m
tmux select-pane -t 1
tmux send-keys "hsrb_mode" C-m
tmux send-keys "roslaunch table_plane_extractor get_objects_on_table.launch" C-m
tmux select-window -t $SESSION:0
tmux select-pane -t 2

tmux rename-window 'grasping'

# Attach to session
tmux -2 attach-session -t $SESSION

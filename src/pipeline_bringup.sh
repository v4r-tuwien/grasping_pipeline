#!/bin/bash

SESSION=GRASPING_PIPELINE
FILE_PATH=${BASH_SOURCE[0]}

SCRIPT_DIR=$(dirname "$0")
echo "Script directory is: $SCRIPT_DIR"

tmux -2 new-session -d -s $SESSION
tmux set -g mouse on

tmux new-window -t $SESSION:1 

## with map setting
tmux select-window -t $SESSION:0
tmux split-window -h
tmux split-window -h


tmux select-pane -t 0
tmux send-keys "roslaunch grasping_pipeline grasping_pipeline_statemachine.launch"

tmux select-pane -t 1
tmux send-keys "hsrb_mode" C-m
tmux send-keys "source $SCRIPT_DIR/../../../devel/setup.bash" C-m
tmux send-keys "roslaunch grasping_pipeline grasping_pipeline_servers.launch"
tmux split-window -v
tmux select-pane -t 2
tmux send-keys "hsrb_mode" C-m
tmux send-keys "source $SCRIPT_DIR/../../../devel/setup.bash" C-m
tmux send-keys "roslaunch grasping_pipeline grasping_pipeline_params.launch"

tmux select-pane -t 3
tmux send-keys "hsrb_mode" C-m
tmux send-keys rv
tmux split-window -v
tmux select-pane -t 4
tmux send-keys "ssh v4r@hsrb.local" C-m
tmux send-keys "startup && table"

tmux select-window -t $SESSION:1

tmux select-pane -t 0
tmux send-keys "ssh v4r@hsrb.local" C-m
tmux send-keys "source ~/demos/devel/setup.bash"
tmux send-keys enter
tmux send-keys "roslaunch hsrb_moveit_config move_group.launch" C-m

tmux select-window -t $SESSION:0
tmux select-pane -t 2

tmux rename-window 'grasping'

# Attach to session
tmux -2 attach-session -t $SESSION

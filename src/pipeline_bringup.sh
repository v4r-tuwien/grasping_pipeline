#!/bin/bash

SESSION=GRASPING_PIPELINE

tmux -2 new-session -d -s $SESSION
tmux set -g mouse on

tmux new-window -t $SESSION:1 

## with map setting
tmux select-window -t $SESSION:0
tmux split-window -h

tmux select-pane -t 0
tmux send-keys "hsrb_mode" C-m
tmux send-keys rv

tmux select-pane -t 1
tmux send-keys "hsrb_mode" C-m
tmux send-keys "roslaunch grasping_pipeline grasping_pipeline_PC.launch"


tmux select-window -t $SESSION:1
tmux split-window -h

tmux select-pane -t 0
tmux send-keys "ssh v4r@hsrb.local" C-m
tmux send-keys "source ~/lexi/catkin_ws/devel/setup.bash"
tmux send-keys enter
tmux send-keys "roslaunch hsrb_moveit_config move_group.launch" C-m

tmux select-pane -t 1
tmux send-keys "ssh v4r@hsrb.local" C-m
tmux send-keys "source ~/lexi/catkin_ws/devel/setup.bash"
tmux send-keys enter
tmux send-keys "roslaunch grasping_pipeline grasping_pipeline_robot.launch" C-m

tmux select-window -t $SESSION:0
tmux select-pane -t 1

tmux rename-window 'grasping'

# Attach to session
tmux -2 attach-session -t $SESSION

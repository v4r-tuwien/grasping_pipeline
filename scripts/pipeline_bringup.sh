#!/bin/bash

SESSION=$USER



tmux -2 new-session -d -s $SESSION
tmux set -g mouse on
# Setup a window for tailing log files
tmux new-window -t $SESSION:0 -n 'launch'
tmux new-window -t $SESSION:1 -n 'grasping_pipeline'


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
tmux send-keys "roslaunch haf_grasping haf_grasping_all.launch"
tmux send-keys enter
tmux select-pane -t 2
tmux send-keys "source /home/v4r/Markus_L/devel/setup.bash" C-m
tmux send-keys "roslaunch grasping_pipeline statemachine.launch"
tmux select-pan -t 3
tmux send-keys "sshpass -p 'ubuntu' ssh ubuntu@hsrb-tk1.local" C-m
tmux send-keys "export ROS_MASTER_URI=http://hsrb.local:11311" C-m
tmux send-keys "export ROS_HOSTNAME=hsrb-tk1" C-m
tmux send-keys "roslaunch hsrb_darknet_tutorials default_model_demo.launch" C-m


# Creating viewpoint
tmux select-window -t $SESSION:1
tmux split-window -v
tmux split-window -h
tmux select-pane -t 
tmux select-window -t $SESSION:0
# Attach to session
tmux -2 attach-session -t $SESSION



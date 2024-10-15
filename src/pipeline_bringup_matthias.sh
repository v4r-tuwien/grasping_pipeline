#!/bin/bash

SESSION=GRASPING_PIPELINE
FILE_PATH=${BASH_SOURCE[0]}

ROBOT_IP="10.0.0.143"
ROS_MASTER_URI="http://$ROBOT_IP:11311/"
echo ROS_MASTER_URI
ROS_IP=$(hostname -I | grep -o '\b10\.[0-9]\+\.[0-9]\+\.[0-9]\+\b')


SCRIPT_DIR=$(dirname "$0")
echo "Script directory is: $SCRIPT_DIR"

tmux -2 new-session -d -s $SESSION
tmux set -g mouse on

tmux new-window -t $SESSION:1 

tmux select-window -t $SESSION:1

tmux select-pane -t 0
tmux send-keys "ssh v4r@$ROBOT_IP" C-m
tmux send-keys "export ROS_MASTER_URI=$ROS_MASTER_URI" C-m
tmux send-keys "export ROS_IP=$ROBOT_IP" C-m
tmux send-keys "source ~/demos/devel/setup.bash"
tmux send-keys enter
tmux send-keys "roslaunch hsrb_moveit_config move_group.launch" C-m

sleep 10

## with map setting
tmux select-window -t $SESSION:0
tmux split-window -h
tmux split-window -v


tmux select-pane -t 0
tmux send-keys "export ROS_MASTER_URI=$ROS_MASTER_URI" C-m
tmux send-keys "export ROS_IP=$ROS_IP" C-m
tmux send-keys "roslaunch grasping_pipeline grasping_pipeline_statemachine.launch" C-m

tmux select-pane -t 1
tmux send-keys "hsrb_mode" C-m
tmux send-keys "export ROS_MASTER_URI=$ROS_MASTER_URI" C-m
tmux send-keys "export ROS_IP=$ROS_IP" C-m
tmux send-keys rv

tmux select-pane -t 2
tmux send-keys "hsrb_mode" C-m
tmux send-keys "export ROS_MASTER_URI=$ROS_MASTER_URI" C-m
tmux send-keys "export ROS_IP=$ROS_IP" C-m
tmux send-keys "source $SCRIPT_DIR/../../../devel/setup.bash" C-m
tmux send-keys "roslaunch grasping_pipeline grasping_pipeline_params.launch && roslaunch grasping_pipeline grasping_pipeline_servers_matthias.launch" C-m

tmux select-window -t $SESSION:0
tmux select-pane -t 2
tmux send-keys "export ROS_MASTER_URI=$ROS_MASTER_URI" C-m
tmux send-keys "export ROS_IP=$ROS_IP" C-m

tmux rename-window 'grasping'

# Attach to session
tmux -2 attach-session -t $SESSION

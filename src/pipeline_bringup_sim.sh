#!/bin/bash
port=11312

SESSION=GRASPING_PIPELINE

tmux -2 new-session -d -s $SESSION
tmux set -g mouse on

tmux new-window -t $SESSION:1 

## with map setting
tmux select-window -t $SESSION:0
tmux split-window -h
tmux split-window -v

tmux select-pane -t 0
tmux send-keys "export ROS_MASTER_URI=http://$ROS_IP:$port" C-m
tmux send-keys "roslaunch grasping_pipeline grasping_pipeline_params.launch && roslaunch grasping_pipeline grasping_pipeline_PC.launch"

tmux select-pane -t 1
tmux send-keys "export ROS_MASTER_URI=http://$ROS_IP:$port" C-m
tmux send-keys rv

tmux select-pane -t 2
tmux send-keys "export ROS_MASTER_URI=http://$ROS_IP:$port" C-m
tmux send-keys "roslaunch grasping_pipeline grasping_pipeline_robot.launch"

tmux select-window -t $SESSION:1
tmux split-window -h
tmux split-window -v

tmux select-pane -t 0
tmux send-keys "export ROS_MASTER_URI=http://$ROS_IP:$port" C-m
tmux send-keys "roslaunch hsrb_moveit_config move_group.launch"
#tmux send-keys "roslaunch hsrb_moveit_config hsrb_demo_with_controller.launch" C-m

tmux select-pane -t 1
tmux send-keys "export ROS_MASTER_URI=http://$ROS_IP:$port" C-m
tmux send-keys "roscore -p $port" C-m

tmux select-pane -t 2
tmux send-keys "export ROS_MASTER_URI=http://$ROS_IP:$port" C-m
tmux send-keys "roslaunch hsrb_gazebo_launch hsrb_mock_home_world.launch"

tmux select-window -t $SESSION:0
tmux select-pane -t 0

tmux rename-window 'grasping'

# Attach to session
tmux -2 attach-session -t $SESSION

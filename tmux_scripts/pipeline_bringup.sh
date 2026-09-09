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
tmux send-keys "ros2 launch grasping_pipeline grasping_pipeline_statemachine.launch.py"

tmux select-pane -t 1
tmux send-keys "ros2 launch grasping_pipeline grasping_pipeline_server.launch.py"

tmux select-pane -t 2
tmux send-keys "ros2 run grasping_pipeline userinput_publisher"

tmux select-window -t $SESSION:1
## TODO fix this ip/name lookup
tmux send-keys "ssh v4r@10.0.0.143" C-m
tmux send-keys "docker exec -it docker.humble.robot.service /ros_entrypoint.sh /bin/bash" C-m
tmux send-keys "ros2 launch hsrb_moveit_config hsrb_demo.launch.py"

tmux rename-window 'grasping'
tmux select-window -t $SESSION:0
tmux select-pane -t 2

# Attach to session
tmux -2 attach-session -t $SESSION

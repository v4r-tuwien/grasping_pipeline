## Starting the pipeline
ssh to robot:
```
ssh v4r@hsrb.local
```
source the workspace:
```
source ~/Markus_L/devel/setup.bash
```
run the tmux bringup:
```
rosrun grasping_pipeline pipeline_bringup.sh
```
 - start the statemachine (top right pane of tmux)
 - ignore error messages about hsrb_description
 - to show commands, press s and then enter
 - bring the robot in a good position where it sees the table (press n)

#### important commands:
 -  g - grasp: run the whole pipeline: find grasppoint, execute grasp, move back, handover, go 		back to neutral
 -  f - find grasp: find grasp without executing it. robot is not moving. print result with p
		     grasp marker gets published on /grasp_marker
 -  e - execute grasp: execute the grasp that was last found. need to run find grasp first

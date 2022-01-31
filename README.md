# Grasping Pipeline
Grasping pipeline for grasping YCBV objects with VeRefine and unkown objects with HAF grasping. 

## Dependencies
- [HAF](https://github.com/davidfischinger/haf_grasping.git)
- [object_detector_msgs](https://github.com/v4r-tuwien/object_detector_msgs)
- [sasha_handover](https://github.com/v4r-tuwien/sasha_handover)
- [open3d_ros_helper](https://github.com/SeungBack/open3d-ros-helper.git)
- [transforms3d](https://github.com/matthew-brett/transforms3d)
- [open3d](http://www.open3d.org/)
- [ros_numpy](http://wiki.ros.org/ros_numpy)
- [hsrb_moveit](https://git.hsr.io/tmc/hsrb_moveit)
- [verefine_pipeline](https://github.com/v4r-tuwien/verefine_pipeline)
- [detectron2_ros](https://github.com/v4r-tuwien/detectron2_ros)

## Usage

### Start grasping pipeline
ssh into the robot and source the workspace. 
Then, you can run the tmux bringup:
``` 
rosrun grasping_pipeline pipeline_bringup.sh
```
* start the statemachine (top right pane of tmux)
* to show commands, press s and then enter

Then you can start grasping

important commands:
* g - grasp: run the whole pipeline: find grasppoint, execute grasp, move back, handover, go back to neutral
* f - find grasp: find grasp without executing it. robot is not moving (so bring it into good position first). print result with p
* e - execute grasp: execute the grasp that was last found. need to run find grasppoint at some point first
* c - config: settings to turn on/off map usage and grasp stability check


### Grasping YCB objects 
Method 2 is for grasping one of the YCB objects on the table with verefine as pose estimator.

If you want to grasp a specific object with verefine, use Method 3 and choose the object.

Make sure the [verefine pipeline](https://github.com/v4r-tuwien/verefine_pipeline) is running and communicating with the robot. 


### Grasping non-YCB objects with haf-grasping 
Only needed if you want to grasp non-YCB objects. (Method 1)
Grasps are always top-grasps.

Make sure the [detectron docker image](https://github.com/v4r-tuwien/detectron2_ros) is running and communicating with the robot.




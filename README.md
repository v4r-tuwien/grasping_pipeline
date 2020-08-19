# Grasping Pipeline
Grasping pipeline for grasping YCBV objects with VeRefine and unkown objects with HAF grasping. 

## Usage
### Start densefusion pipeline
 ``` bash
 xhost +local:'hostname'
 cd /home/v4r/Jeremy/projects/hsr-grasping/compose/densefusion_pipeline
 docker-compose up
 ```
### Localize the robot in RVIZ 
Only necessary if you use the map. By default, map usage is turned on. To change this, either change the setting in `/home/v4r/Markus_L/src/grasping_pipeline/config/standard.yaml`  on Sasha or change it during runtime in the statemachine terminal. Press 'c' to go to the config settings, and '2' to turn off the map usage.

If you want to use the map, make sure that the robots position is accurate in RVIZ.
To start RVIZ:
 ``` bash
 rviz -d /home/v4r/Markus_L/src/rviz/grasp_demo.rviz 
 ```

Then use 2D Pose Estimate

### Start grasping pipeline
ssh into robot:
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
* start the statemachine (top right pane of tmux)
* ignore error messages about hsrb_description
* to show commands, press s and then enter
* if you don't use the map, bring the robot in a good position where it sees the table (press n). 

Then you can start grasping :D


Method 2 is for grasping one of the YCB objects on the table with verefine as pose estimator.

If you want to grasp a specific object with verefine, use Method 3 and choose the object.

important commands:
* g - grasp: run the whole pipeline: find grasppoint, execute grasp, move back, handover, go back to neutral
* f - find grasp: find grasp without executing it. robot is not moving (so bring it into good position first). print result with p
* e - execute grasp: execute the grasp that was last found. need to run find grasppoint at some point first
* c - config: settings to turn on/off map usage and grasp stability check
### Grasping non-YCB objects with haf-grasping 
Only needed if you want to grasp non-YCB objects. (Method 1)
Grasps are always top-grasps.

Start detectron docker image on backpack laptop. 10.0.0.143 is the IP of Sasha.

in docker image, run: 
```
 export ROS_MASTER_URI=http://10.0.0.143:11311/
 source catkin_build_ws/devel/setup.bash

 rosrun detectron2_ros service.py -ct 0.1 -c /detectron2_repo/configs/Misc panoptic_fpn_R_101_dconv_cascade_gn_3x.yaml\
   -o MODEL.WEIGHTS detectron2://Misc/panoptic_fpn_R_101_dconv_cascade_gn_3x/139797668/model_final_be35db.pkl \
   -t /hsrb/head_rgbd_sensor/rgb/image_raw -v

```



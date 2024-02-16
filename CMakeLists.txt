cmake_minimum_required(VERSION 3.0.1)
project(grasping_pipeline)

find_package(catkin REQUIRED 
genmsg 
rospy
actionlib_msgs
geometry_msgs
hsrb_interface_py
haf_grasping
sensor_msgs
handover
object_detector_msgs
vision_msgs
table_plane_extractor
v4r_util
tmc_geometric_shapes_msgs
message_filters
grasping_pipeline_msgs
)

## Generate actions in the 'action' folder
 add_action_files(
   DIRECTORY action
   FILES
   CreateCollisionEnvironment.action
 )

  add_message_files(
  FILES
  CollisionObject.msg
)

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs  
   actionlib_msgs
   geometry_msgs
   object_detector_msgs
   grasping_pipeline
   vision_msgs
 )

catkin_package()
include_directories()


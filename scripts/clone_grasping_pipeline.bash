#!/bin/bash

# This script clones all repositories that are needed for getting the grasping pipeline running.
# You will need access to the private repositories from v4r.
# You also have to setup a ssh-key ->
# -> refer to https://docs.github.com/en/authentication/connecting-to-github-with-ssh

################################################################################

mkdir -p ./workspace/ros2_ws/src
cd ./workspace/ros2_ws/src
git clone -b ros2_humble https://github.com/BitstreamRider/grasping_pipeline.git
git clone -b ros2_humble https://github.com/BitstreamRider/v4r_util.git
git clone -b ros2_humble https://github.com/BitstreamRider/table_plane_extractor.git
git clone -b ros2_humble https://github.com/BitstreamRider/sasha_handover.git
git clone -b ros2_humble https://github.com/BitstreamRider/object_detector_msgs.git
git clone -b ros2_humble https://github.com/BitstreamRider/grasping_pipeline_msgs.git
git clone -b ros2_humble_bremen_wrapper https://github.com/BitstreamRider/haf_grasping.git
git clone -b ros2_jazzy https://gitlab.informatik.uni-bremen.de/robokudo/robokudo_msgs.git

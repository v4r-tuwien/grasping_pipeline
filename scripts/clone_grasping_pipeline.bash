#!/bin/bash

# This script clones all repositories that are needed for getting the grasping pipeline running.

################################################################################

git clone -b ros2_humble https://github.com/v4r-tuwien/grasping_pipeline.git
git clone -b ros2_humble https://github.com/v4r-tuwien/v4r_util.git
git clone -b ros2_humble https://github.com/v4r-tuwien/table_plane_extractor.git
git clone -b ros2_humble https://github.com/v4r-tuwien/sasha_handover.git
git clone -b ros2_humble https://github.com/v4r-tuwien/object_detector_msgs.git
git clone -b ros2_humble https://github.com/v4r-tuwien/grasping_pipeline_msgs.git
git clone -b ros2_humble_bremen_wrapper https://github.com/v4r-tuwien/haf_grasping.git
# the message definitions for ros2 jazzy and ros2 humble have not changed, this repo only includes message definitions
# so the jazzy branch can be built for humble
git clone -b ros2_jazzy https://gitlab.informatik.uni-bremen.de/robokudo/robokudo_msgs.git

#!/bin/bash

# This script clones all repositories that are needed for getting the grasping pipeline running.
# You will need access to the private repositories from v4r.
# You also have to setup a ssh-key ->
# -> refer to https://docs.github.com/en/authentication/connecting-to-github-with-ssh

################################################################################

git clone git@github.com:v4r-tuwien/hsrb_moveit.git
git clone https://github.com/v4r-tuwien/grasping_pipeline.git
git clone https://github.com/v4r-tuwien/haf_grasping.git
git clone https://github.com/v4r-tuwien/v4r_util.git
git clone https://github.com/v4r-tuwien/table_plane_extractor.git
git clone https://github.com/v4r-tuwien/sasha_handover.git
git clone https://github.com/v4r-tuwien/object_detector_msgs.git
git clone https://github.com/v4r-tuwien/grasping_pipeline_msgs.git
git clone https://gitlab.informatik.uni-bremen.de/robokudo/robokudo_msgs.git

cd haf_grasping/libsvm-3.12
make

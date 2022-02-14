#! /usr/bin/env python3

import numpy as np
import open3d as o3d
import copy
import random
import os
import rospy
from open3d_ros_helper import open3d_ros_helper as orh
from sensor_msgs.msg import PointCloud2

rospy.init_node("mapcloudpub")

cloud = o3d.io.read_point_cloud('/home/v4r/demos/src/grasping_pipeline/scripts/hsrb_result_pred_legend_21.ply')
#cloud = cloud.voxel_down_sample(0.2)
bbx = o3d.geometry.AxisAlignedBoundingBox([-100, -100, 0.05], [100, 100, 100])
cloud = cloud.crop(bbx)
rospc = orh.o3dpc_to_rospc(cloud, 'map') 

mappub = rospy.Publisher("/mapcloud", PointCloud2, queue_size=10, latch=True)
while not rospy.is_shutdown():
    print(rospc.header)
    rospc.header.stamp = rospy.Time.now()
    mappub.publish(rospc)
    rospy.sleep(1)

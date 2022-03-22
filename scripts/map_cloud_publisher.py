#!/usr/bin/env python3

import numpy as np
import open3d as o3d
import os
import rospy
from open3d_ros_helper import open3d_ros_helper as orh
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid


rospy.init_node("mapcloudpub")

mappub = rospy.Publisher("/mapcloud", PointCloud2, queue_size=10, latch=True)

dir_path = os.path.dirname(os.path.realpath(__file__))
reco_cloud_file = os.path.join(
    dir_path, os.pardir, 'config', 'hsrb_result_pred_legend_21.ply')
cloud = o3d.io.read_point_cloud(reco_cloud_file)

if len(cloud.points) > 0:
    print('3D reconstruction found.')
    bbx = o3d.geometry.AxisAlignedBoundingBox(
        [-100, -100, 0.05], [100, 100, 100])
    cloud = cloud.crop(bbx)
    rospc = orh.o3dpc_to_rospc(cloud, 'map')
    print('Publish 3D map')

else:
    print('No reconstruction found, will use 2D map.')
    map = rospy.wait_for_message(
        '/static_obstacle_map_ref', OccupancyGrid, timeout=10)
    image = np.array(map.data, dtype=np.float32).reshape(
        (map.info.width, map.info.height))
    coordinates = map.info.resolution*np.array(np.where(image > 99))
    coordinates = np.swapaxes(coordinates, 0, 1)
    z = 0.2*np.ones((np.shape(coordinates)[0], 1), dtype=np.float32)
    image = np.concatenate((coordinates, z), 1)
    cloudimage = np.zeros(image.shape)
    cloudimage[:, 0] = image[:, 1] + map.info.origin.position.y
    cloudimage[:, 1] = image[:, 0] + map.info.origin.position.x
    cloudimage[:, 2] = image[:, 2] + map.info.origin.position.z
    image = cloudimage
    cloud = o3d.cpu.pybind.utility.Vector3dVector(image)
    cloud = o3d.geometry.PointCloud(cloud)
    cloud.paint_uniform_color((1, 0, 0))
    rospc = orh.o3dpc_to_rospc(cloud, 'map')
    print('Publish 2D map as cloud')

while not rospy.is_shutdown():
    rospc.header.stamp = rospy.Time.now()
    mappub.publish(rospc)
    rospy.sleep(1)

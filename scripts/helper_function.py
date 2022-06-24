#!/usr/bin/python3

import rospy
import tf
import open3d

import numpy as np

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose, PointStamped, Point, PoseStamped
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from lib_cloud_conversion_between_Open3D_and_ROS import convertCloudFromOpen3dToRos, convertCloudFromRosToOpen3d

def setSphereMarker(point, id, color, frame, pub):
    # sphere marker for rviz
    marker = Marker()
    marker.header.frame_id = frame
    marker.header.stamp = rospy.Time()
    marker.id = id
    marker.pose.position.x = point.x
    marker.pose.position.y = point.y
    marker.pose.position.z = point.z
    marker.type = Marker.SPHERE
    marker.scale.x = 0.1
    marker.scale.y = 0.1
    marker.scale.z = 0.1
    
    r, g, b = color
    marker.color.a = 1.0
    marker.color.r = r
    marker.color.g = g
    marker.color.b = b

    pub.publish(marker)

def setArrowMarker(pose, id, color, frame, pub):
    # cube marker for visalization
    marker = Marker()
    marker.header.frame_id = frame
    marker.header.stamp = rospy.Time()
    marker.id = id
    marker.pose = pose
    marker.type = Marker.ARROW
    r, g, b = color
    marker.color.a = 1.0
    marker.color.r = r
    marker.color.g = g
    marker.color.b = b
    marker.scale.x = 0.1
    marker.scale.y = 0.1
    marker.scale.z = 0.1

    pub.publish(marker)

def setCubeMarker(pose, id, color, frame, plane_width, plane_height, pub):
    # cube marker for visalization
    marker = Marker()
    marker.header.frame_id = frame
    marker.header.stamp = rospy.Time()
    marker.id = id
    marker.pose.position.x = pose.position.x
    marker.pose.position.y = pose.position.y
    marker.pose.position.z = pose.position.z / 2
    marker.pose.orientation = pose.orientation
    marker.type = Marker.CUBE
    marker.scale.x = plane_height
    marker.scale.y = plane_width
    marker.scale.z = pose.position.z / 2
    r, g, b = color
    marker.color.a = 1.0
    marker.color.r = r
    marker.color.g = g
    marker.color.b = b

    pub.publish(marker)


def transform_point(target_frame, source_frame, point):
    # transform point from source frame to target frame
    target_pose = PointStamped()
    source_point = PointStamped()
    source_point.header.frame_id = source_frame
    source_point.header.stamp = rospy.Time(0)
    source_point.point = point

    listener = tf.TransformListener()
    listener.waitForTransform(
        source_frame, target_frame, rospy.Time(0), rospy.Duration(4.0))
    
    try:
        target_point = listener.transformPoint(target_frame, source_point)
    except tf.Exception:
        rospy.logerr("Transform failure")

    return target_point.point

def transform_pose(target_frame, source_frame, pose):
    # transform pose from source to target frame
    target_pose = PoseStamped()
    source_pose = PoseStamped()
    source_pose.header.frame_id = source_frame
    source_pose.header.stamp = rospy.Time(0)
    source_pose.pose = pose

    listener = tf.TransformListener()
    listener.waitForTransform(
        source_frame, target_frame, rospy.Time(0), rospy.Duration(4.0))

    try:
        target_pose = listener.transformPose(target_frame, source_pose)
    except tf.Exception:
        rospy.logerr("Transform failure")

    return target_pose

def transformPointCloud(cloud, target_frame, source_frame, tf_buffer):
    # transform whole point cloud from source to target frame
    transform = tf_buffer.lookup_transform(
        target_frame, source_frame, rospy.Time())
    transformedCloud = do_transform_cloud(cloud, transform)
    return transformedCloud

def transformPoseFormat(pose, format_str):
    if format_str == "tuple":
        new = Pose()
        new.position.x = pose.pos.x
        new.position.y = pose.pos.y
        new.position.z = pose.pos.z
        new.orientation.x = pose.ori.x
        new.orientation.y = pose.ori.y
        new.orientation.z = pose.ori.z
        new.orientation.w = pose.ori.w
        return new
    elif format_str == "pose":
        return ((pose.position.x, pose.position.y, pose.position.z), (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w))
    else:
        return None


def filter_open3d_cloud_by_direction(open3d_cloud, direction, lower, upper):
    # filter point cloud 

    index = 3
    if direction == 'x':
        index = 0
    elif direction == 'y':
        index = 1
    elif direction == 'z':
        index = 2
    else:
        return open3d_cloud

    points = np.asarray(open3d_cloud.points)[:,index]
    colors = np.asarray(open3d_cloud.colors)[:,index]

    mask = np.where((points >= lower) & (points <= upper))

    open3d_cloud.points = open3d.utility.Vector3dVector(np.asarray(open3d_cloud.points)[mask])
    open3d_cloud.colors = open3d.utility.Vector3dVector(np.asarray(open3d_cloud.colors)[mask])

    return open3d_cloud


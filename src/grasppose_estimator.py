#! /usr/bin/env python3
import sys
import os
from copy import deepcopy
import numpy as np
import yaml
from yaml.loader import SafeLoader
from math import pi
from sensor_msgs.msg import Image, CameraInfo
from v4r_util.depth_pcd import convert_ros_depth_img_to_pcd
from geometry_msgs.msg import PoseStamped
import PyKDL
import actionlib
import rospy
from grasping_pipeline_msgs.msg import (FindGrasppointAction,
                                   FindGrasppointResult)
from tf.transformations import (quaternion_about_axis, quaternion_from_matrix,
                                quaternion_multiply)
from tf_conversions import posemath
from visualization_msgs.msg import Marker

from grasp_annotator import GraspAnnotator

from v4r_util.bb import create_ros_bb_stamped

import time

class FindGrasppointServer:
    '''
    Computes the grasp_poses for the object to grasp based on annotated grasps.

    The FindGrasppointServer uses the grasp checker to calculate the grasp poses for the
    object, based on grasp-annotations and the estimated object poses. 
    The server returns the grasp poses, the bounding box of the
    object to grasp and the object name of the object to grasp.
    The server adds markers to the rviz visualization to show the grasp pose and the bounding box of
    the object.
    
    Attributes
    ----------
    models_metadata: dict
        The metadata of the known objects. Contains the bounding box information.
    server: actionlib.SimpleActionServer
        This action server. Computes grasp poses for the object to grasp.
    marker_pub: rospy.Publisher
        The publisher for the rviz markers.
    cam_info: sensor_msgs.msg.CameraInfo
        The camera info message containing the camera parameters.
    timeout: float
        The timeout duration for setting up the action servers and waiting for results.
    
    Parameters
    ----------
    object_to_grasp: str
        The name of the object to grasp. If specified, the server will grasp this object. If not
        specified (empty string), the server will grasp the closest object.
    object_poses: list of geometry_msgs.msg.Pose
        The poses of the objects detected by the pose estimator.
    depth: sensor_msgs.msg.Image
        The depth image of the scene.
    class_names: list of str
        The class names of the objects detected by the pose estimator.
    
    Returns
    -------
    grasping_pipeline_msgs.msg.FindGrasppointResult
        The result of the FindGrasppoint action server. Contains the grasp poses, the bounding
        box of the object and the object name.
    '''
    
    def __init__(self, model_dir):
        '''
        Initializes the FindGrasppointServer.

        Loads the model metadata from the models_metadata.yml file in the model_dir. This file 
        contains the bounding box information for known objects. The server then waits for the
        camera info message. After that it initializes the action server and the rviz marker
        publisher.
        Finally it starts the action server.
        
        Parameters
        ----------
        model_dir: str
            Path to the directory containing the model metadata file.
        '''
        with open(os.path.join(model_dir, "models_metadata.yml")) as f:
            self.models_metadata = yaml.load(f, Loader=SafeLoader)

        self.server = actionlib.SimpleActionServer(
            'find_grasppoint', FindGrasppointAction, self.execute, False)
        self.marker_pub = rospy.Publisher('/grasping_pipeline/grasp_marker', Marker, queue_size=10)
        self.server.start()

        rospy.logdebug('Waiting for camera info')
        self.cam_info = rospy.wait_for_message(rospy.get_param('/cam_info_topic'), CameraInfo)
        
        self.timeout = rospy.get_param('/grasping_pipeline/timeout_duration')
        self.grasp_annotator = GraspAnnotator()
      
        rospy.loginfo('Initializing FindGrasppointServer done')


    def execute(self, goal):
        '''
        Executes the FindGrasppoint action server.
        
        If an object to grasp is specified in the goal, the server will estimate the grasp pose for
        the specified object. Otherwise, it will estimate the grasp pose for the closest object.
        The server uses the grasp checker which uses grasp-annotations to calculate grasp poses from
        the estimated object pose.
        In the end the server will add markers to the rviz visualization to show the grasp pose and the
        bounding box of the object.
        
        Parameters
        ----------
        goal: grasping_pipeline_msgs.msg.FindGrasppointGoal
            The goal of the FindGrasppoint action server. Contains the object to grasp.
        
        Returns
        -------
        grasping_pipeline_msgs.msg.FindGrasppointResult
            The result of the FindGrasppoint action server. Contains the grasp poses, the bounding 
            box of the object and the object name.
        '''
        try:
            scene_cloud, scene_cloud_o3d = convert_ros_depth_img_to_pcd(
                goal.depth, 
                self.cam_info, 
                project_valid_depth_only=False)
            
            if goal.object_to_grasp != None and goal.object_to_grasp != '':
                param_object_to_grasp = goal.object_to_grasp
            elif rospy.has_param('/grasping_pipeline/object_to_grasp'):
                # Getting object to grasp from config file
                param_object_to_grasp = rospy.get_param('/grasping_pipeline/object_to_grasp')
            else:
                param_object_to_grasp = None
                
            # Check if object to grasp is specified and detected
            if param_object_to_grasp != None and param_object_to_grasp != '' and param_object_to_grasp != 'None':
                rospy.loginfo(f'Object to grasp specified. Will grasp specified object {param_object_to_grasp}')
                if param_object_to_grasp in goal.class_names:
                    object_idxs = [goal.class_names.index(param_object_to_grasp)]     # changed           
                else:
                    rospy.logwarn(f'Object to grasp ({param_object_to_grasp}) not detected. Grasping closest object.')
                    object_idxs = self.get_closest_objects(goal.object_poses)             
            else:
                rospy.loginfo('No object to grasp specified. Will grasp closest object')
                object_idxs = self.get_closest_objects(goal.object_poses)    # changed to return a list of object indices sorted by distance

            for object_idx in object_idxs:

                result = FindGrasppointResult()
                object_to_grasp = goal.object_poses[object_idx]
                object_name = goal.class_names[object_idx]
                object_to_grasp_stamped = PoseStamped(pose = object_to_grasp, header = goal.depth.header)

                rospy.loginfo(f"Annoting grasps for object {object_name}")

                rospy.logdebug('Generating grasp poses')

                grasp_poses = self.grasp_annotator.annotate(object_to_grasp_stamped, scene_cloud, object_name)
                
                object_bb_stamped = self.get_bb_for_known_objects(object_to_grasp_stamped, object_name, goal.depth.header.frame_id, goal.depth.header.stamp)

                if grasp_poses is None or len(grasp_poses) < 1:
                    rospy.logerr(f"No grasp pose found for object {object_name}")
                    continue
                
                result.grasp_poses = grasp_poses
                result.grasp_object_bb = object_bb_stamped
                result.grasp_object_name = object_name

                self.add_marker(grasp_poses[0])
                self.add_bb_marker(object_bb_stamped)
                self.server.set_succeeded(result)
                return

        except (ValueError, TimeoutError) as e:
            rospy.logerr(str(e))
            self.server.set_aborted(text=str(e))
        self.server.set_aborted()

    def transform_to_kdl(self, pose):
        '''
        Converts a geometry_msgs.msg.Pose to a PyKDL.Frame.
        
        Parameters
        ----------
        pose: geometry_msgs.msg.Pose
            The pose to convert.
        
        Returns
        -------
        PyKDL.Frame
            The converted pose.
        '''
        return PyKDL.Frame(PyKDL.Rotation.Quaternion(pose.orientation.x, pose.orientation.y,
                                                 pose.orientation.z, pose.orientation.w),
                       PyKDL.Vector(pose.position.x,
                                    pose.position.y,
                                    pose.position.z))

    def get_bb_for_known_objects(self, object_pose, object_name, frame_id, stamp):
        '''
        Generates a bounding box for known objects based on the object pose and object name.
        
        Loads the bounding box information for the object from the models_metadata.yml file.
        
        Parameters
        ----------
        object_pose: geometry_msgs.msg.PoseStamped
            The pose of the object.
        object_name: str
            The name of the object. Used to look up the bounding box information.
        frame_id: str
            The frame id of the object pose.
        stamp: rospy.Time
            The timestamp of the object pose.

        Returns
        -------
        grasping_pipeline_msgs.msg.BoundingBoxStamped
            The bounding box for the object with frame id and timestamp.
        '''
        dataset = rospy.get_param('/grasping_pipeline/dataset')
        metadata = self.models_metadata[dataset][object_name]
        center = metadata['center']
        extent = metadata['extent']
        rot_mat = metadata['rot']
        bb_stamped = create_ros_bb_stamped(center, extent, rot_mat, frame_id, stamp)
        t1 = self.transform_to_kdl(bb_stamped.center)
        t2 = self.transform_to_kdl(object_pose.pose)
        t_res = t2 * t1
        t_res_ros = posemath.toMsg(t_res)
        bb_stamped.center = t_res_ros
        return bb_stamped

    def get_closest_objects(self, object_poses):
        '''
        Returns a sorted list with indices of the closest objects. The closest object is at index 0.
        
        The closest object is determined by the distance to the camera. The distance is calculated
        as the euclidean distance from the camera to the object pose.
        
        Parameters
        ----------
        estimator_result: GenericImgProcAnnotatorResult
            The result of the pose estimator.
        
        Raises
        ------
        ValueError
            If no close object is found.
        
        Returns
        -------  
        list of int
            The indices of the closest objects in the estimator result.
        '''

        object_idx = []
        distances = []

        for i, pose in enumerate(object_poses):
            pose_pos = pose.position
            dist = pose_pos.x*pose_pos.x + pose_pos.y*pose_pos.y + pose_pos.z*pose_pos.z
            distances.append(dist)
            object_idx.append(i)
        
        distances_np = np.array(distances)
        sort_idx = np.argsort(distances_np)
        return np.array(object_idx)[sort_idx]

    
        
    def add_marker(self, pose_goal):
        """ 
        Adds a marker to the rviz visualization to show the grasp pose.
        
        The marker is an arrow pointing in the direction of the grasp pose. The arrow is red.
        The topic of the marker is '/grasping_pipeline/grasp_marker'. The marker is published
        in the namespace 'grasp_marker' with id 0.
        
        Parameters
        ----------
        pose_goal: geometry_msgs.msg.PoseStamped
            The pose of the grasp.
        """
        marker = Marker()
        marker.header.frame_id = pose_goal.header.frame_id
        marker.header.stamp = rospy.Time()
        marker.ns = 'grasp_marker'
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        q2 = [pose_goal.pose.orientation.w, pose_goal.pose.orientation.x,
              pose_goal.pose.orientation.y, pose_goal.pose.orientation.z]
        q = quaternion_about_axis(pi / 2, (0, 1, 0))
        q = quaternion_multiply(q, q2)

        marker.pose.orientation.w = q[0]
        marker.pose.orientation.x = q[1]
        marker.pose.orientation.y = q[2]
        marker.pose.orientation.z = q[3]
        marker.pose.position.x = pose_goal.pose.position.x
        marker.pose.position.y = pose_goal.pose.position.y
        marker.pose.position.z = pose_goal.pose.position.z

        marker.scale.x = 0.1
        marker.scale.y = 0.05
        marker.scale.z = 0.01

        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0
        marker.color.b = 0
        self.marker_pub.publish(marker)
        rospy.loginfo('grasp_marker')

    
    def add_bb_marker(self, object_bb_stamped):
        '''
        Adds a marker to the rviz visualization to show the bounding box of the object.

        The marker is a blue bounding box.
        The topic of the marker is '/grasping_pipeline/grasp_marker'. The marker is published
        in the namespace 'bb_marker' with id 0.

        Parameters
        ----------
        object_bb_stamped: grasping_pipeline_msgs.msg.BoundingBoxStamped
            The bounding box of the object.
        '''
        marker = Marker()
        marker.header.frame_id = object_bb_stamped.header.frame_id
        marker.header.stamp = rospy.Time()
        marker.ns = 'bb_marker'
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        marker.pose = deepcopy(object_bb_stamped.center)
        marker.scale = deepcopy(object_bb_stamped.size)

        marker.color.a = 0.5
        marker.color.r = 0
        marker.color.g = 0
        marker.color.b = 1.0
        self.marker_pub.publish(marker)

if __name__ == '__main__':
    rospy.init_node('find_grasppoint_server')
    if len(sys.argv) < 2:
        rospy.logerr('No model dir was specified!')
        sys.exit(-1)
    model_dir = sys.argv[1]
    server = FindGrasppointServer(model_dir)
    rospy.spin()

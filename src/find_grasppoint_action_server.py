#! /usr/bin/env python3
import sys
import os
from copy import deepcopy
import numpy as np
import yaml
from yaml.loader import SafeLoader
from math import pi
import message_filters
from sensor_msgs.msg import Image, CameraInfo
from v4r_util.depth_pcd import convert_ros_depth_img_to_pcd
from geometry_msgs.msg import PoseStamped
import ros_numpy
import PyKDL
import actionlib
import rospy
from grasping_pipeline_msgs.msg import (FindGrasppointAction,
                                   FindGrasppointResult)
from object_detector_msgs.srv import VisualizePoseEstimationRequest, VisualizePoseEstimation
from tf.transformations import (quaternion_about_axis, quaternion_from_matrix,
                                quaternion_multiply)
from tf_conversions import posemath
from visualization_msgs.msg import Marker
from robokudo_msgs.msg import GenericImgProcAnnotatorAction, GenericImgProcAnnotatorGoal
from sensor_msgs.msg import RegionOfInterest
from geometry_msgs.msg import Pose, Point, Quaternion

from grasp_checker import check_grasp_hsr
from v4r_util.util import get_minimum_oriented_bounding_box, o3d_bb_to_ros_bb_stamped, create_ros_bb_stamped

class FindGrasppointServer:
    '''
    Calls the pose estimator to get object poses and then calculates grasp poses for the detected objects.

    The server subscribes to the rgb and depth image topics. It then calls the pose estimator to get
    object poses. If the pose estimator detects unknown objects, the server will use the label image
    to get the bounding boxes, regions of interest and bb center-poses. The poses of the unknown objects
    are then overwritten with the center poses of their bounding boxes.
    If an object to grasp is specified in the goal, the server will grasp the specified object.
    Otherwise, it will grasp the closest object. The server then calculates the grasp poses for the
    specified or the closest object.
    In the case of unknown objects the server will call the unknown object grasp detector to get
    grasp poses for the unknown objects.
    In the case of known objects the server will call the grasp checker which uses grasp-annotations
    to calculate grasp poses.
    In the end the server will add markers to the rviz visualization to show the grasp pose and the
    bounding box of the object.
    
    Attributes
    ----------
    models_metadata: dict
        The metadata of the known objects. Contains the bounding box information.
    server: actionlib.SimpleActionServer
        The action server for the FindGrasppoint action.
    marker_pub: rospy.Publisher
        The publisher for the rviz markers.
    depth: sensor_msgs.msg.Image
        The most recent depth image from the camera.
    rgb: sensor_msgs.msg.Image
        The most recent rgb image from the camera.
    image_sub: ApproximateTimeSynchronizer
        The message filter for the depth and rgb image topics.
    cam_info: sensor_msgs.msg.CameraInfo
        The camera info message containing the camera parameters.
    timeout: float
        The timeout duration for setting up the action servers and waiting for results.
    pose_estimator: actionlib.SimpleActionClient
        The action client for the pose estimator.
    res_vis_service: rospy.ServiceProxy
        The service proxy for the result visualization service, which visualizes the pose estimation
        results in an image with object contours and names.
    unknown_object_grasp_detector: actionlib.SimpleActionClient
        The action client for the unknown object grasp detector, which calculates grasp poses for
        unknown objects based on a region of interest.
    
    Parameters
    ----------
    object_to_grasp: str
        The name of the object to grasp. If specified, the server will grasp this object. If not
        specified (empty string), the server will grasp the closest object.
    
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
        contains the bounding box information for known objects. Afterwards it subscribes to the
        rgb and depth imag topics. It then connects to the pose estimator action server and 
        (optionally) to the unknown object grasp detector action server.
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

        self.depth, self.rgb = None, None
        self.image_sub = self.__setup_image_subs(rospy.get_param('/depth_topic'), rospy.get_param('/rgb_topic'))
        
        rospy.logdebug('Waiting for camera info')
        self.cam_info = rospy.wait_for_message(rospy.get_param('/cam_info_topic'), CameraInfo)
        
        self.timeout = rospy.get_param('/pose_estimator/timeout_duration')
        self.pose_estimator = self.__setup_estimator(rospy.get_param('/pose_estimator/detector_topic'), self.timeout)
        res_vis_service_name = rospy.get_param('/pose_estimator/result_visualization_service_name')
        self.res_vis_service = rospy.ServiceProxy(res_vis_service_name, VisualizePoseEstimation)

        self.unknown_object_grasp_detector = None
        unknown_object_grasp_detector_topic = rospy.get_param('/pose_estimator/unknown_object_grasp_detector_topic', None)
        if unknown_object_grasp_detector_topic is not None:
            self.unknown_object_grasp_detector = self.__setup_unknown_object_grasp_detector(unknown_object_grasp_detector_topic, self.timeout)
        
        rospy.loginfo('Initializing FindGrasppointServer done')

    def __setup_image_subs(self, depth_topic, rgb_topic):
        '''
        Creates a message filter for the depth and rgb image topics to synchronize them.
        
        Parameters
        ----------
        depth_topic: str
            The topic of the depth image.
        rgb_topic: str
            The topic of the rgb image.
        
        Returns
        -------
        ApproximateTimeSynchronizer
            The message filter for the depth and rgb image topics.
        '''
        depth_sub = message_filters.Subscriber(depth_topic, Image)
        rgb_sub = message_filters.Subscriber(rgb_topic, Image)
        image_sub = message_filters.ApproximateTimeSynchronizer([depth_sub, rgb_sub], 5, 0.1)
        image_sub.registerCallback(self.image_callback)
        return image_sub
    
    def __setup_estimator(self, topic, timeout):
        '''
        Connects to the pose estimator action server.

        Parameters
        ----------
        topic: str
            The topic of the pose estimator action server.
        timeout: float
            The timeout duration in seconds.
        '''
        rospy.loginfo('Waiting for pose_estimator actionserver')
        pose_estimator = actionlib.SimpleActionClient(
            topic, 
            GenericImgProcAnnotatorAction)
        if not pose_estimator.wait_for_server(timeout=rospy.Duration(timeout)):
            rospy.logerr(f'Connection to pose_estimator \'{topic}\' timed out!')
            raise TimeoutError
        return pose_estimator
    
    def __setup_unknown_object_grasp_detector(self, topic, timeout):
        '''
        Connects to the unknown object grasp detector action server.
        
        Parameters
        ----------
        topic: str
            The topic of the unknown object grasp detector action server.
        timeout: float
            The timeout duration in seconds.
        '''
        rospy.loginfo('Waiting for unknown object grasp_detector')
        unknown_object_grasp_detector = actionlib.SimpleActionClient(
                topic, 
                GenericImgProcAnnotatorAction)
        if not unknown_object_grasp_detector.wait_for_server(timeout=rospy.Duration(timeout)):
            rospy.logerr(f'Connection to unknown_object_grasp_detector \'{topic}\' timed out!')
            raise TimeoutError
        return unknown_object_grasp_detector
        
    def execute(self, goal):
        '''
        Executes the FindGrasppoint action server.
        
        The server first calls the pose estimator to get object poses. It then performs a check
        to ensure that the results were passed correctly. If the pose estimator detected unknown
        objects, it will use the label image to get the bbs, regions of interest and bb center-poses. 
        The poses of the unknown objects are then overwritten with the center poses of their
        bounding boxes. 
        If an object to grasp is specified in the goal, the server will grasp the specified object.
        Otherwise, it will grasp the closest object. 
        The server then calculates the grasp poses for the specified or the closest object.
        In the case of unknown objects the server will call the unknown object grasp detector to get
        grasp poses for the unknown objects. 
        In the case of known objects the server will call the grasp checker which uses grasp-annotations
        to calculate grasp poses.
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
        while self.depth is None: 
            rospy.loginfo('Waiting for images')  
            rospy.sleep(0.1)
        rgb = self.rgb
        depth = self.depth
        estimator_goal = GenericImgProcAnnotatorGoal(rgb = rgb, depth = depth)
        estimator_result = self.get_estimator_result(estimator_goal)
        self.visualize_pose_estimation_result(
            rgb, 
            estimator_result.pose_results, 
            estimator_result.class_names
        )

        try:
            rospy.logdebug('Performing consistency checks')
            self.perform_estimator_results_consistency_checks(estimator_result)

            scene_cloud, scene_cloud_o3d = convert_ros_depth_img_to_pcd(
                depth, 
                self.cam_info, 
                project_valid_depth_only=False)

            if 'Unknown' in estimator_result.class_names:
                # We expect label image values to be in the same order as the lists for class_confidence, class_names, ...
                bbs, ROI_2d, poses = self.prepare_unknown_object_detection(estimator_result, scene_cloud_o3d)
                rospy.loginfo("Overwriting unknown object poses with information from label image")
                estimator_result.pose_results = self.overwrite_unknown_object_poses(estimator_result, poses)
                # fill bbs with pseudo entries so that we can use the object_idx that gets calculated afterwards
                filled_bbs = self.fill_bbs(estimator_result, bbs)

            if goal.object_to_grasp != None and goal.object_to_grasp != '':
                rospy.loginfo(f'Object to grasp specified. Will grasp specified object {goal.object_to_grasp}')
                object_idx = estimator_result.class_names.index(goal.object_to_grasp)
            else:
                rospy.loginfo('No object to grasp specified. Will grasp closest object')
                object_idx = self.get_closest_object(estimator_result)

            result = FindGrasppointResult()
            object_to_grasp = estimator_result.pose_results[object_idx]
            object_name = estimator_result.class_names[object_idx]
            object_to_grasp_stamped = PoseStamped(pose = object_to_grasp, header = depth.header)

            rospy.logdebug('Generating grasp poses')
            if object_name == 'Unknown':
                grasp_poses = self.call_unknown_obj_grasp_detector(rgb, depth, ROI_2d[object_idx])
                object_bb = filled_bbs[object_idx]
                object_bb_stamped = o3d_bb_to_ros_bb_stamped(object_bb, depth.header.frame_id, depth.header.stamp)
            else:
                grasp_poses = check_grasp_hsr(
                    object_to_grasp_stamped, scene_cloud, object_name, table_plane=None, visualize=True)
                object_bb_stamped = self.get_bb_for_known_objects(object_to_grasp_stamped, object_name, depth.header.frame_id, depth.header.stamp)

            if len(grasp_poses) < 1:
                raise ValueError('No grasp pose found')
            
            result.grasp_poses = grasp_poses
            result.grasp_object_bb = object_bb_stamped
            result.grasp_object_name = object_name

            self.add_marker(grasp_poses[0])
            self.add_bb_marker(object_bb_stamped)
            self.server.set_succeeded(result)

        except (ValueError, TimeoutError) as e:
            rospy.logerr(str(e))
            self.server.set_aborted(text=str(e))

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
        metadata = self.models_metadata[object_name]
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

    def fill_bbs(self, estimator_result, bbs):
        '''
        Creates a list of bounding boxes for unknown objects. Fill the list with 0 for known objects.
        
        The list is the same size as the list of detected objects. If an object is not an unknown
        object, the list entry is set to 0. So the list contains only bounding boxes for unknown
        objects.
        
        Parameters
        ----------
        estimator_result: GenericImgProcAnnotatorResult
            The result of the pose estimator.
        bbs: list of open3d.geometry.OrientedBoundingBox
            The list of bounding boxes for the unknown objects.

        Returns
        -------
        list of open3d.geometry.OrientedBoundingBox
            The list of bounding boxes for the unknown objects.
        '''
        j = 0
        new_bb_list = []
        for i, class_name in enumerate(estimator_result.class_names):
            if class_name == 'Unknown':
                new_bb_list.append(bbs[j])
                j += 1
            else:
                new_bb_list.append(0)
        return new_bb_list

    def overwrite_unknown_object_poses(self, estimator_result, poses):
        '''
        Overwrites the poses of unknown objects in the estimator result with the given poses.
        
        Parameters
        ----------
        estimator_result: GenericImgProcAnnotatorResult
            The result of the pose estimator.
        poses: list of geometry_msgs.msg.Pose
            The poses for the unknown objects.
        
        Returns
        -------
        list of geometry_msgs.msg.Pose
            The list of poses with the unknown object poses overwritten. The poses for known objects
            are kept. The list is the same size as the list of detected objects and has the same order.
        '''
        j = 0
        new_poses = []
        for i, class_name in enumerate(estimator_result.class_names):
            if class_name == 'Unknown':
                new_poses.append(poses[j])
                j += 1
            else:
                # if not unknown -> keep original pose
                new_poses.append(estimator_result.pose_results[i])
        return new_poses

    def call_unknown_obj_grasp_detector(self, rgb, depth, obj_ROI):
        '''
        Calls the unknown object grasp detector to get grasp poses for the detected unknown object.
        
        Parameters
        ----------
        rgb: sensor_msgs.msg.Image
            The rgb image.
        depth: sensor_msgs.msg.Image
            The depth image.
        obj_ROI: sensor_msgs.msg.RegionOfInterest
            The region of interest for the unknown object.
        
        Raises
        ------
        TimeoutError
            If the unknown object grasp detector doesn't return results before timing out.
        ValueError
            If no unknown object grasp detector is specified but unknown objects are detected.
        
        Returns
        -------
        list of geometry_msgs.msg.PoseStamped
            All detected grasp_poses for the unknown object.
        '''
        if self.unknown_object_grasp_detector is None:
            rospy.logerr("No unknown object grasp detector specified but detected unknown object(s)")
            raise ValueError("No unknown object grasp detector specified")
        grasp_detector_goal = GenericImgProcAnnotatorGoal(rgb = rgb, depth = depth)

        grasp_detector_goal.bb_detections = [obj_ROI]

        res = self.unknown_object_grasp_detector.send_goal(grasp_detector_goal)
        rospy.logdebug('Waiting for unknown object grasp detector results')

        succesful = self.unknown_object_grasp_detector.wait_for_result(
            rospy.Duration(self.timeout))
        if not succesful:
            raise TimeoutError("Unknown object grasp detector didn't return results before timing out!")
        
        grasp_poses = self.unknown_object_grasp_detector.get_result().pose_results
        grasp_poses_stamped = []
        for pose in grasp_poses:
            pose_stamped = PoseStamped()
            pose_stamped.header.stamp = rospy.Time.now()
            pose_stamped.header.frame_id = depth.header.frame_id
            pose_stamped.pose = pose
            grasp_poses_stamped.append(pose_stamped)

        return grasp_poses_stamped

    def get_closest_object(self, estimator_result):
        '''
        Returns the index of the closest object in the estimator result.
        
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
        int
            The index of the closest object in the estimator result.
        '''
        conf_threshold = float(rospy.get_param('/pose_estimator/class_confidence_threshold'))

        min_dist_squared = float("inf")
        cl_conf = estimator_result.class_confidences
        object_idx = None

        for i, pose in enumerate(estimator_result.pose_results):
            # skip objects that have low confidence
            if len(cl_conf) > 1 and cl_conf[i] < conf_threshold:
                continue

            pose_pos = pose.position
            dist_squared = pose_pos.x*pose_pos.x + pose_pos.y*pose_pos.y + pose_pos.z*pose_pos.z
            if dist_squared < min_dist_squared:
                min_dist_squared = dist_squared
                object_idx = i

        if object_idx is None:
            raise ValueError("No close object found")
        
        return object_idx

    def prepare_unknown_object_detection(self, estimator_result, scene_cloud_o3d):
        '''
        Prepares the unknown object detection by checking the label image and converting it to bounding boxes and poses.
        
        Parameters
        ----------
        estimator_result: GenericImgProcAnnotatorResult
            The result of the pose estimator.
        scene_cloud_o3d: open3d.geometry.PointCloud
            The scene point cloud.
        
        Raises
        ------
        ValueError
            If the label image is inconsistent.
        
        Returns
        -------
        list of open3d.geometry.OrientedBoundingBox
            The list of bounding boxes for the unknown objects.
        list of sensor_msgs.msg.RegionOfInterest
            The list of regions of interest for the unknown objects.
        list of geometry_msgs.msg.Pose
            The list of poses for the unknown objects. The poses are the centers of the bounding boxes.
        '''
        self.check_label_img(estimator_result.image)
        label_image = ros_numpy.numpify(estimator_result.image)
        o3d_bbs, ROI_2d = self.convert_label_img_to_bb(scene_cloud_o3d, label_image)
        if len(o3d_bbs) != estimator_result.class_names.count('Unknown'):
            rospy.logerr("Number of unique labels in label image inconsistent to number of detected unknown objects")
            raise ValueError('Label image inconsistent')
        poses = []
        for o3d_bb in o3d_bbs:
            center_point = o3d_bb.get_center()
            x, y, z = center_point[0], center_point[1], center_point[2]

            pos = Point(x=x, y=y, z=z)
            rot_mat = np.eye(4)
            rot_mat[:3, :3] = o3d_bb.R
            quat = quaternion_from_matrix(rot_mat)
            ros_quat = Quaternion(x = quat[0], y = quat[1], z = quat[2], w = quat[3])

            pose = Pose(position = pos, orientation = ros_quat)
            poses.append(pose)
        return o3d_bbs, ROI_2d, poses
    
    def get_estimator_result(self, estimator_goal):
        '''
        Calls the pose estimator action server and waits for the result.
        
        Parameters
        ----------
        estimator_goal: GenericImgProcAnnotatorGoal
            The goal to send to the pose estimator.
            
        Raises
        ------
        TimeoutError
            If the estimator doesn't return results before timing out.
        
        Returns
        -------
        GenericImgProcAnnotatorResult
            The result of the pose estimator.
        '''
        rospy.logdebug('Sending goal to estimator')
        self.pose_estimator.send_goal(estimator_goal)

        rospy.logdebug('Waiting for estimator results')
        goal_finished = self.pose_estimator.wait_for_result(rospy.Duration(self.timeout))
        if not goal_finished:
            rospy.logerr('Estimator didn\'t return results before timing out!')
            raise TimeoutError
        estimator_result = self.pose_estimator.get_result()
        rospy.loginfo(f'Detector detected {len(estimator_result.pose_results)} potential object poses.')

        return estimator_result

    def perform_estimator_results_consistency_checks(self, estimator_result):
        '''
        Checks the consistency of the pose estimator results.
        
        The consistency checks are:
        - Check if the pose estimator passed any poses.
        - Check if the pose estimator passed any model names.
        - Check if the number of poses and model names match.
        - Check if the number of class confidences and poses match (optional, only if class confidences are passed).
        
        Parameters
        ----------
        estimator_result: GenericImgProcAnnotatorResult
            The result of the pose estimator.
        
        Raises
        ------
        ValueError
            If any of the consistency checks fails.
        '''
        consistent = True
        if len(estimator_result.pose_results) < 1:
            rospy.logerr('pose_estimator did not pass any poses!')
            consistent = False
        if len(estimator_result.class_names) < 1:
            rospy.logerr('pose_estimator did not pass any model names!')
            consistent = False
        if len(estimator_result.pose_results) != len(estimator_result.class_names):
            rospy.logerr(f'Mismatch between list sizes: {len(estimator_result.pose_results) = },' +
                                    f'{len(estimator_result.class_names) = }')
            consistent = False
        if len(estimator_result.class_confidences) < 1:
            rospy.logwarn("No class confidences passed. Will not perform confidence thresholding")
        elif (len(estimator_result.pose_results) != len(estimator_result.class_confidences)):
            rospy.logerr(f'Mismatch between list sizes: {len(estimator_result.pose_results) = },' +
                                    f'{len(estimator_result.class_confidences) = }')
            consistent = False
        if not consistent:
            raise ValueError("Consistency check failed")

    def convert_label_img_to_bb(self, scene_pc, labels):
        '''
        Computes bounding boxes and regions of interest based on the label image and the scene point cloud.
        
        Parameters
        ----------
        scene_pc: open3d.geometry.PointCloud
            The scene point cloud.
        labels: numpy.ndarray
            The label image. Should be a signed image with -1 to indicate pixels without object. 
            Each object should have a unique label. The labels should be in the same order as the
            class names and poses in the pose estimator result. The encoding should be 8SC1, 16SC1 
            or 32SC1.

        Returns
        -------
        list of open3d.geometry.OrientedBoundingBox
            The list of bounding boxes for each object in the label image.
        list of sensor_msgs.msg.RegionOfInterest
            The list of regions of interest for each object in the label image.
        '''
        labels_unique = np.unique(labels)
        # get bounding box for each object
        bbs_o3d = []
        ROI_2d = []
        for label in labels_unique:
            if label == -1:
                continue
            roi = RegionOfInterest()
            obj_mask = (labels == label)
            rows = np.any(obj_mask, axis=1)
            cols = np.any(obj_mask, axis=0)
            rmin, rmax = np.where(rows)[0][[0, -1]]
            cmin, cmax = np.where(cols)[0][[0, -1]]
            roi = RegionOfInterest(x_offset = cmin, 
                                   y_offset=rmin, 
                                   width = cmax-cmin, 
                                   height = rmax-rmin)
            ROI_2d.append(roi)
            
            obj_indices = np.nonzero(obj_mask.flatten())
            obj_pcd = scene_pc.select_by_index(obj_indices[0])
            obj_bb_3d = get_minimum_oriented_bounding_box(obj_pcd)
            bbs_o3d.append(obj_bb_3d)
        return bbs_o3d, ROI_2d
    
    def check_label_img(self, label_img):
        '''
        Checks the consistency of the label image.
        
        The consistency checks are:
        - Check if the label image has a height and width greater than 0.
        - Check if the label image encoding is supported (8SC1, 16SC1 or 32SC1).

        Parameters
        ----------
        label_img: sensor_msgs.msg.Image
            The label image.
        
        Raises
        ------
        ValueError
            If the label image is inconsistent.
        '''
        isConsistent = True

        if (label_img.height < 1 or label_img.width < 1):
            rospy.logerr(f"No label image passed for unknown objects! {label_img.height = }, {label_img.width = }")
            isConsistent = False

        # Expect signed image with -1 to indicate pixels without object
        supported_encodings = ['8SC1', '16SC1', '32SC1']
        if(label_img.encoding not in supported_encodings):
            rospy.logerr(f"Encoding not supported: Got {label_img.encoding = } but has to be one off {supported_encodings}")
            isConsistent = False
        if not isConsistent:
            raise ValueError("Label image is not consistent!")

    def image_callback(self, depth, rgb):
        '''
        Callback for the depth and rgb image topics.
        
        Parameters
        ----------
        depth: sensor_msgs.msg.Image
            The depth image.
        rgb: sensor_msgs.msg.Image
            The rgb image.
        '''
        self.depth = depth
        self.rgb = rgb
        
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

    def visualize_pose_estimation_result(self, rgb, model_poses, model_names):
        '''
        Visualizes the pose estimation result using the pose estimator result visualization service.
        
        The service creates and publishes an image with a contour of the detected objects and their 
        object names.
        
        Parameters
        ----------
        rgb: sensor_msgs.msg.Image
            The rgb image.
        model_poses: list of geometry_msgs.msg.Pose
            The poses of the detected objects.
        model_names: list of str
            The names of the detected objects.
        '''
        request = VisualizePoseEstimationRequest()
        request.rgb_image = rgb
        request.model_poses = model_poses
        request.model_names = model_names
        self.res_vis_service(request)

if __name__ == '__main__':
    rospy.init_node('find_grasppoint_server')
    if len(sys.argv) < 2:
        rospy.logerr('No model dir was specified!')
        sys.exit(-1)
    model_dir = sys.argv[1]
    server = FindGrasppointServer(model_dir)
    rospy.spin()

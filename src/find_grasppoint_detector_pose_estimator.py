#! /usr/bin/env python3
import sys
import numpy as np
import yaml
from yaml.loader import SafeLoader
from math import asin, atan2, isnan, pi
import message_filters
from sensor_msgs.msg import Image, CameraInfo
from v4r_util.depth_pcd import convert_ros_depth_img_to_pcd
from geometry_msgs.msg import PoseStamped
import ros_numpy

import actionlib
import rospy
from grasping_pipeline.msg import (FindGrasppointAction,
                                   FindGrasppointResult)
from hsrb_interface import Robot
from tf.transformations import (quaternion_about_axis, quaternion_from_matrix,
                                quaternion_multiply, unit_vector)
from visualization_msgs.msg import Marker
from robokudo_msgs.msg import GenericImgProcAnnotatorAction, GenericImgProcAnnotatorGoal

from grasp_checker import check_grasp_hsr
from v4r_util.util import get_minimum_oriented_bounding_box, o3d_bb_list_to_ros_bb_arr


class FindGrasppointServer:
    # gets grasp point of object with highest confidence, if no confidences passed -> choosees closest instead
    #TODO add central visualization
    def __init__(self, cfg):
        self.cfg = cfg
        self.server = actionlib.SimpleActionServer(
            'find_grasppoint', FindGrasppointAction, self.execute, False)
        self.server.start()

        depth_topic = cfg['depth_topic']
        rgb_topic = cfg['rgb_topic']
        depth_sub = message_filters.Subscriber(depth_topic, Image)
        rgb_sub = message_filters.Subscriber(rgb_topic, Image)
        image_sub = message_filters.ApproximateTimeSynchronizer([depth_sub, rgb_sub], 5, 0.1)
        self.depth, self.rgb = None, None
        image_sub.registerCallback(self.image_callback)
        
        rospy.logdebug('Waiting for camera info')
        self.cam_info = rospy.wait_for_message(cfg['cam_info_topic'], CameraInfo)
        
        rospy.loginfo('Waiting for detector_pose_estimator actionserver')
        self.detector_pose_estimator = actionlib.SimpleActionClient(
            cfg['detector_pose_estimator']['topic'], 
            GenericImgProcAnnotatorAction)
        if not self.detector_pose_estimator.wait_for_server(timeout=rospy.Duration(10.0)):
            topic = cfg['detector_pose_estimator']['topic']
            rospy.logerr(f'Connection to detector_pose_estimator \'{topic}\' timed out!')

        self.unknown_object_grasp_detector = None
        if cfg['unknown_object_grasp_detector_topic'] not in ['None', None]:
            rospy.loginfo('Waiting for unknown object grasp_detector')
            self.unknown_object_grasp_detector = actionlib.SimpleActionClient(
                cfg['unknown_object_grasp_detector_topic'], 
                GenericImgProcAnnotatorAction)
            if not self.unknown_object_grasp_detector.wait_for_server(timeout=rospy.Duration(10.0)):
                topic = cfg['unknown_object_grasp_detector_topic']
                rospy.logerr(f'Connection to unknown_object_grasp_detector \'{topic}\' timed out!')
        
        rospy.loginfo('Initializing FindGrasppointServer done')

    def execute(self, goal):
        while self.depth is None: 
            rospy.loginfo('Waiting for images')  
            rospy.sleep(0.1)
        rgb = self.rgb
        depth = self.depth
        estimator_goal = GenericImgProcAnnotatorGoal()
        estimator_goal.rgb = rgb
        estimator_goal.depth = depth
        rospy.logdebug('Sending goal to estimator')
        self.detector_pose_estimator.send_goal(estimator_goal)

        rospy.logdebug('Waiting for estimator results')
        timeout_duration = float(self.cfg['detector_pose_estimator']['timeout_duration'])
        self.detector_pose_estimator.wait_for_result(rospy.Duration(timeout_duration))
        estimator_result = self.detector_pose_estimator.get_result()
        rospy.loginfo(f'Detector detected {len(estimator_result.pose_results)} potential object poses.')

        rospy.logdebug('Performing consistency checks')
        isConsistent = self.perform_estimator_results_consistency_checks(estimator_result)
        if not isConsistent:
            self.server.set_aborted(text="Consistency check failed")
            return

        scene_cloud, scene_cloud_o3d = convert_ros_depth_img_to_pcd(depth, self.cam_info)
        if 'Unknown' in estimator_result.class_names:
            isConsistent = self.check_label_img(estimator_result.image)
            if not isConsistent:
                self.server.set_aborted(text='Label image not correct')
                return
            label_image = ros_numpy.numpify(estimator_result.image)
            bbs = self.convert_label_img_to_bb(scene_cloud_o3d, label_image)
            if len(bbs) != estimator_result.class_names.count('Unknown'):
                rospy.logerr("Number of unique labels in label image inconsistent to number of detected unknown objects")
                self.server.set_aborted(text='Label image inconsistent')
                return
            ros_bbs = o3d_bb_list_to_ros_bb_arr(bbs, depth.header.frame_id, rospy.Time.now())
            j = 0
            for i, poses in enumerate(estimator_result.pose_results):
                if estimator_result.class_names[i] == 'Unknown':
                    estimator_result.pose_results[i] = ros_bbs.boxes[j].center
                    j -=- 1
        
        conf_threshold = float(cfg['class_confidence_threshold'])
        if len(estimator_result.class_confidences) < 1:
            rospy.logwarn('Detector did not pass confidences. Grasping closest object instead')
            min_dist_squared = float("inf")
            for i, pose in enumerate(estimator_result.pose_results):
                pose_pos = pose.position
                dist_squared = pose_pos.x*pose_pos.x + pose_pos.y*pose_pos.y + pose_pos.z*pose_pos.z
                if dist_squared < min_dist_squared:
                    min_dist_squared = dist_squared
                    object_idx = i
        else:
            confidences = estimator_result.class_confidences
            max_conf = max(confidences)
            if max_conf < conf_threshold:
                self.server.set_aborted(text='No object poses left after thresholding')
                return
            object_idx = confidences.index(max_conf)

        result = FindGrasppointResult()
        object_to_grasp = estimator_result.pose_results[object_idx]
        object_name = estimator_result.class_names[object_idx]
        object_to_grasp_stamped = PoseStamped()
        object_to_grasp_stamped.pose = object_to_grasp
        object_to_grasp_stamped.header = depth.header

        rospy.logdebug('Generating grasp poses')
        if object_name == 'Unknown':
            if self.unknown_object_grasp_detector is None:
                rospy.logerr("No unknown object grasp detector specified but detected unknown object(s)")
                self.server.set_aborted(text='No unknown object grasp detector specified')
                return
            else:
                grasp_detector_goal = GenericImgProcAnnotatorGoal()
                grasp_detector_goal.rgb = rgb
                grasp_detector_goal.depth = depth
                grasp_detector_goal.bb_detections = ros_bbs[object_idx]
                res = self.unknown_object_grasp_detector(grasp_detector_goal)
                rospy.logdebug('Waiting for unknown object grasp detector results')
                timeout_duration = float(self.cfg['unknown_object_grasp_detector_timeout'])
                self.unknown_object_grasp_detector.wait_for_result(rospy.Duration(timeout_duration))
                valid_grasp_poses = PoseStamped()
                valid_grasp_poses.header.stamp = rospy.Time.now()
                valid_grasp_poses.header.frame_id = depth.header.frame_id
                valid_grasp_poses.pose = self.detector_pose_estimator.get_result()
        else:
            valid_grasp_poses = check_grasp_hsr(
                object_to_grasp_stamped, scene_cloud, object_name, table_plane=None, visualize=True)
        if len(valid_grasp_poses) < 1:
            self.server.set_aborted(text='No grasp pose found')
            return

        result.grasp_poses = valid_grasp_poses

        self.add_marker(valid_grasp_poses[0])
        self.server.set_succeeded(result)

    def perform_estimator_results_consistency_checks(self, estimator_result):
        consistent = True
        if len(estimator_result.pose_results) < 1:
            rospy.logerr('Detector_pose_estimator did not pass any poses!')
            consistent = False
        if len(estimator_result.class_names) < 1:
            rospy.logerr('Detector_pose_estimator did not pass any model names!')
            consistent = False
        if len(estimator_result.pose_results) != len(estimator_result.class_names):
            rospy.logerr(f'Mismatch between list sizes: {len(estimator_result.pose_results) = },' +
                                    f'{len(estimator_result.class_names) = }')
            consistent = False
        # if len=0 -> will pick closest object instead
        if (len(estimator_result.class_confidences) > 0 and 
            (len(estimator_result.pose_results) != len(estimator_result.class_confidences))):
            
            rospy.logerr(f'Mismatch between list sizes: {len(estimator_result.pose_results) = },' +
                                    f'{len(estimator_result.class_confidences) = }')
            consistent = False
        return consistent

    def convert_label_img_to_bb(self, scene_pc, labels):
        labels_unique = np.unique(labels)
        print(f"{labels_unique = }")
        # get bounding box for each object
        bbs = []
        for label in labels_unique:
            if label == -1:
                continue
            obj_indices = np.nonzero((labels == label).flatten())
            print(f"{label = }, {obj_indices[0] = }")
            obj = scene_pc.select_by_index(obj_indices[0])
            obj_bb = get_minimum_oriented_bounding_box(obj)
            bbs.append(obj_bb)
        return bbs
    
    def check_label_img(self, label_img):
        isConsistent = True

        if (label_img.height < 1 or label_img.width < 1):
            rospy.logerr(f"No label image passed for unknown objects! {label_img.height = }, {label_img.width = }")
            isConsistent = False

        # Expect signed image with -1 to indicate pixels without object
        supported_encodings = ['8SC1', '16SC1', '32SC1']
        if(label_img.encoding not in supported_encodings):
            rospy.logerr(f"Encoding not supported: Got {label_img.encoding = } but has to be one off {supported_encodings}")
            isConsistent = False
    
        return isConsistent

    def image_callback(self, depth, rgb):
        self.depth = depth
        self.rgb = rgb

    def add_marker(self, pose_goal):
        """ publishes a grasp marker to /grasping_pipeline/grasp_marker

        Arguments:
            pose_goal {geometry_msgs.msg.PoseStamped} -- pose for the grasp marker
        """
        marker = Marker()
        marker.header.frame_id = pose_goal.header.frame_id
        marker.header.stamp = rospy.Time()
        marker.ns = 'grasp_marker'
        marker.id = 0
        marker.type = 0
        marker.action = 0

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


if __name__ == '__main__':
    rospy.init_node('find_grasppoint_server')
    if len(sys.argv) < 2:
        rospy.logerr('No yaml config file was specified!')
        sys.exit(-1)
    with open(sys.argv[1]) as f:
        cfg = yaml.load(f, Loader=SafeLoader)
    server = FindGrasppointServer(cfg)
    rospy.spin()

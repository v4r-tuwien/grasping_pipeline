#! /usr/bin/env python3
import sys
import numpy as np
import yaml
from yaml.loader import SafeLoader
from math import asin, atan2, isnan, pi
import message_filters
from sensor_msgs.msg import Image, CameraInfo
from v4r_util.depth_pcd import convert_np_label_img_to_ros_color_img, convert_ros_depth_img_to_pcd
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
from sensor_msgs.msg import RegionOfInterest
from geometry_msgs.msg import Pose, Point, Quaternion

from grasp_checker import check_grasp_hsr
from v4r_util.util import get_minimum_oriented_bounding_box, o3d_bb_list_to_ros_bb_arr


class FindGrasppointServer:
    #TODO add central visualization
    #TODO add 3D bb handling for known objects (model.json)
    def __init__(self, cfg):
        self.cfg = cfg
        self.server = actionlib.SimpleActionServer(
            'find_grasppoint', FindGrasppointAction, self.execute, False)
        self.marker_pub = rospy.Publisher('/grasping_pipeline/grasp_marker', Marker)
        self.server.start()

        self.depth, self.rgb = None, None
        self.image_sub = self.__setup_image_subs(cfg)
        
        rospy.logdebug('Waiting for camera info')
        self.cam_info = rospy.wait_for_message(cfg['cam_info_topic'], CameraInfo)
        
        self.detector_pose_estimator = self.__setup_detector(cfg)

        self.unknown_object_grasp_detector = None
        if cfg['unknown_object_grasp_detector_topic'] not in ['None', None]:
            self.unknown_object_grasp_detector = self.__setup_unknown_object_grasp_detector(cfg)
        
        rospy.loginfo('Initializing FindGrasppointServer done')

    def __setup_image_subs(self, cfg):
        depth_topic = cfg['depth_topic']
        rgb_topic = cfg['rgb_topic']
        depth_sub = message_filters.Subscriber(depth_topic, Image)
        rgb_sub = message_filters.Subscriber(rgb_topic, Image)
        image_sub = message_filters.ApproximateTimeSynchronizer([depth_sub, rgb_sub], 5, 0.1)
        image_sub.registerCallback(self.image_callback)
        return image_sub
    
    def __setup_detector(self, cfg):
        rospy.loginfo('Waiting for detector_pose_estimator actionserver')
        detector_pose_estimator = actionlib.SimpleActionClient(
            cfg['detector_pose_estimator']['topic'], 
            GenericImgProcAnnotatorAction)
        if not detector_pose_estimator.wait_for_server(timeout=rospy.Duration(10.0)):
            topic = cfg['detector_pose_estimator']['topic']
            rospy.logerr(f'Connection to detector_pose_estimator \'{topic}\' timed out!')
            raise TimeoutError
        return detector_pose_estimator
    
    def __setup_unknown_object_grasp_detector(self, cfg):
        rospy.loginfo('Waiting for unknown object grasp_detector')
        unknown_object_grasp_detector = actionlib.SimpleActionClient(
                cfg['unknown_object_grasp_detector_topic'], 
                GenericImgProcAnnotatorAction)
        if not unknown_object_grasp_detector.wait_for_server(timeout=rospy.Duration(10.0)):
            topic = cfg['unknown_object_grasp_detector_topic']
            rospy.logerr(f'Connection to unknown_object_grasp_detector \'{topic}\' timed out!')
            raise TimeoutError
        return unknown_object_grasp_detector

    def execute(self, goal):
        while self.depth is None: 
            rospy.loginfo('Waiting for images')  
            rospy.sleep(0.1)
        rgb = self.rgb
        depth = self.depth
        estimator_goal = GenericImgProcAnnotatorGoal(rgb = rgb, depth = depth)
        estimator_result = self.get_estimator_result(estimator_goal)

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
                unknown_obj_bbs_ros = o3d_bb_list_to_ros_bb_arr(bbs, frame_id=depth.header.frame_id, stamp=depth.header.stamp)

            object_idx = self.get_closest_object(estimator_result)

            result = FindGrasppointResult()
            object_to_grasp = estimator_result.pose_results[object_idx]
            object_name = estimator_result.class_names[object_idx]
            object_to_grasp_stamped = PoseStamped(pose = object_to_grasp, header = depth.header)

            rospy.logdebug('Generating grasp poses')
            if object_name == 'Unknown':
                grasp_poses = self.call_unknown_obj_grasp_detector(rgb, depth, ROI_2d[object_idx])
            else:
                grasp_poses = check_grasp_hsr(
                    object_to_grasp_stamped, scene_cloud, object_name, table_plane=None, visualize=True)
            if len(grasp_poses) < 1:
                raise ValueError('No grasp pose found')
            
            result.grasp_poses = grasp_poses
            #TODO: add 3D BB of known objects to result
            result.object_bbs = unknown_obj_bbs_ros

            self.add_marker(grasp_poses[0])
            self.server.set_succeeded(result)

        except (ValueError, TimeoutError) as e:
            rospy.logerr(str(e))
            self.server.set_aborted(text=str(e))

    def overwrite_unknown_object_poses(self, estimator_result, poses):
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
        if self.unknown_object_grasp_detector is None:
            rospy.logerr("No unknown object grasp detector specified but detected unknown object(s)")
            raise ValueError("No unknown object grasp detector specified")
        grasp_detector_goal = GenericImgProcAnnotatorGoal(rgb = rgb, depth = depth)

        grasp_detector_goal.bb_detections = [obj_ROI]

        res = self.unknown_object_grasp_detector.send_goal(grasp_detector_goal)
        rospy.logdebug('Waiting for unknown object grasp detector results')

        timeout_duration = float(self.cfg['unknown_object_grasp_detector_timeout'])
        succesful = self.unknown_object_grasp_detector.wait_for_result(
            rospy.Duration(timeout_duration))
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
        conf_threshold = float(self.cfg['class_confidence_threshold'])

        min_dist_squared = float("inf")
        cl_conf = estimator_result.class_confidences
        object_idx = None

        for i, pose in enumerate(estimator_result.pose_results):
            # skip objects that have low confidence
            if len(cl_conf) > 1 and cl_conf < conf_threshold:
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
        rospy.logdebug('Sending goal to estimator')
        self.detector_pose_estimator.send_goal(estimator_goal)

        rospy.logdebug('Waiting for estimator results')
        timeout_duration = float(self.cfg['detector_pose_estimator']['timeout_duration'])
        self.detector_pose_estimator.wait_for_result(rospy.Duration(timeout_duration))
        estimator_result = self.detector_pose_estimator.get_result()
        rospy.loginfo(f'Detector detected {len(estimator_result.pose_results)} potential object poses.')

        return estimator_result

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
        if len(estimator_result.class_confidences) < 1:
            rospy.logwarn("No class confidences passed. Will not perform confidence thresholding")
        elif (len(estimator_result.pose_results) != len(estimator_result.class_confidences)):
            rospy.logerr(f'Mismatch between list sizes: {len(estimator_result.pose_results) = },' +
                                    f'{len(estimator_result.class_confidences) = }')
            consistent = False
        if not consistent:
            raise ValueError("Consistency check failed")

    def convert_label_img_to_bb(self, scene_pc, labels):
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

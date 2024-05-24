#! /usr/bin/env python3
import numpy as np
from copy import deepcopy
import ros_numpy
from cv_bridge import CvBridge
import rospy
from actionlib import SimpleActionClient
from actionlib_msgs.msg import GoalStatus
from robokudo_msgs.msg import GenericImgProcAnnotatorAction, GenericImgProcAnnotatorGoal
from grasping_pipeline_msgs.srv import CallDirectGraspPoseEstimator, CallDirectGraspPoseEstimatorResponse
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from v4r_util.depth_pcd import convert_np_depth_img_to_o3d_pcd
from v4r_util.util import get_minimum_oriented_bounding_box, o3d_bb_to_ros_bb_stamped
from tf.transformations import (quaternion_about_axis, quaternion_multiply)

class DirectGraspposeEstimatorCaller:

    def __init__(self):
        self.bridge = CvBridge()
        self.srv = rospy.Service('call_direct_grasppose_estimator', CallDirectGraspPoseEstimator , self.execute)
        self.cam_info = rospy.wait_for_message(rospy.get_param('/cam_info_topic'), CameraInfo)
        self.marker_pub = rospy.Publisher('/grasping_pipeline/grasp_marker', Marker, queue_size=10)
    
    def execute(self, req):
        topic = rospy.get_param('/grasping_pipeline/grasppoint_estimator_topic')
        timeout = rospy.get_param('/grasping_pipeline/timeout_duration')

        grasppose_est = SimpleActionClient(topic, GenericImgProcAnnotatorAction)

        rospy.loginfo('Waiting for direct-grasppose-estimator server with topic: %s' % topic)
        if not grasppose_est.wait_for_server(timeout=rospy.Duration(timeout)):
            rospy.logerr(f'Connection to direct-grasppose_estimator \'{topic}\' timed out!')
            raise rospy.ServiceException
        rospy.loginfo('Connected to direct-grasppose-estimator server')

        bbs_3D = self.get_3D_bbs(req.depth, req.mask_detections, req.bb_detections)
        center_poses = self.get_bb_center_poses(bbs_3D)
        
        if req.object_to_grasp != None and req.object_to_grasp != '':
            if req.object_to_grasp in req.class_names:
                rospy.loginfo(f'Object to grasp specified. Will grasp specified object {req.object_to_grasp}')
                object_idx = req.class_names.index(req.object_to_grasp)               
            else:
                rospy.logwarn(f'Object to grasp {goal.object_to_grasp} not detected. Aborting')
                raise rospy.ServiceException
        else:
            rospy.loginfo('No object to grasp specified. Will grasp closest object')
            object_idx = self.get_closest_object(center_poses)
        
        object_mask, object_bb = [], []
        if len(req.mask_detections) > 0:
            object_mask = [req.mask_detections[object_idx]]
        if len(req.bb_detections) > 0:
            object_bb = [req.bb_detections[object_idx]]
        if len(object_mask) <= 0 and len(object_bb) <= 0:
            rospy.logerr('No mask or bb detections provided! Need at least either one to proceed!')
            raise rospy.ServiceException
        
        goal = GenericImgProcAnnotatorGoal(
            rgb=req.rgb, 
            depth=req.depth, 
            mask_detections=object_mask, 
            bb_detections=object_bb, 
            class_names=[req.class_names[object_idx]]
        )

        rospy.logdebug('Sending goal to direct-grasppose-estimator')
        grasppose_est.send_goal(goal)
        rospy.logdebug('Waiting for direct grasppose estimation results')
        goal_finished = grasppose_est.wait_for_result(rospy.Duration(timeout))
        if not goal_finished:
            rospy.logerr('Direct grasppose Estimator didn\'t return results before timing out!')
            raise rospy.ServiceException
        graspposes = grasppose_est.get_result()
        status = grasppose_est.get_state()

        if status != GoalStatus.SUCCEEDED or len(graspposes.pose_results) <= 0:
            rospy.logerr('Grasppose Estimator failed to estimate poses!')
            raise rospy.ServiceException
        rospy.loginfo(f'Estimated the grasppose of {len(graspposes.class_names)} objects.')

        assert len(graspposes.pose_results) == 1, 'Expected only one grasppose result, but got more than one!'

        res = CallDirectGraspPoseEstimatorResponse()
        res.grasp_object_name = req.class_names[object_idx]
        res.grasp_object_bb = bbs_3D[object_idx]
        res.grasp_poses = [PoseStamped(header=req.depth.header, pose=graspposes.pose_results[0])]
        self.add_bb_marker(res.grasp_object_bb)
        self.add_marker(res.grasp_poses[0])

        return res

    def get_bb_center_poses(self, bbs):
        center_poses = []
        for bb in bbs:
            center_poses.append(bb.center)
        return center_poses
    
    def get_3D_bbs(self, depth, masks, bbs_2d):
        bbs = []
        depth_np = ros_numpy.numpify(depth)
        if len(masks) > 0:
            for mask in masks:
                bb = self.get_bb_3D_from_mask(depth, depth_np, mask)
                bbs.append(bb)
        elif len(bbs_2d) > 0:
            for bb in bbs_2d:
                self.get_bb_3D(depth, depth_np, bb)
                bbs.append(bb)
        else:
            rospy.logerr('No masks or bbs provided to extract object depth values!')
            raise rospy.ServiceException
        return bbs
        
    def get_bb_3D_from_mask(self, depth, depth_np, mask):
        depth_img_obj = np.full_like(depth_np, np.nan, dtype=np.float32)
        mask = ros_numpy.numpify(mask)
        mask = mask != 0
        # only copy the object's depth values, rest stay NaN
        depth_img_obj[mask] = depth_np[mask]
        object_pcd = convert_np_depth_img_to_o3d_pcd(depth_img_obj, self.cam_info, project_valid_depth_only=True)
        obj_bb_o3d = get_minimum_oriented_bounding_box(object_pcd)
        return o3d_bb_to_ros_bb_stamped(obj_bb_o3d, depth.header.frame_id, depth.header.stamp)

    def get_bb_3D_from_bb(self, depth, depth_np, bb_2d):
        depth_img_obj = np.full_like(depth_np, np.nan)
        bb = bb_2d
        y, x = bb.y_offset, bb.x_offset
        h, w = bb.height, bb.width
        # only copy the object's depth values, rest stay NaN
        depth_img_obj[y:y+h, x:x+w] = depth_np[y:y+h, x:x+w]
        object_pcd = convert_np_depth_img_to_o3d_pcd(depth_img_obj, self.cam_info, project_valid_depth_only=True)
        obj_bb_o3d = get_minimum_oriented_bounding_box(object_pcd)
        return o3d_bb_to_ros_bb_stamped(obj_bb_o3d, depth.header.frame_id, depth.header.stamp)
    
    def get_closest_object(self, object_poses):
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
        min_dist_squared = float("inf")
        object_idx = None

        for i, pose in enumerate(object_poses):
            pose_pos = pose.position
            dist_squared = pose_pos.x*pose_pos.x + pose_pos.y*pose_pos.y + pose_pos.z*pose_pos.z
            if dist_squared < min_dist_squared:
                min_dist_squared = dist_squared
                object_idx = i

        if object_idx is None:
            raise ValueError("No close object found")
        
        return object_idx

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
        q = quaternion_about_axis(3.1415 / 2, (0, 1, 0))
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
    node = rospy.init_node('grasppose_estimator')
    est = DirectGraspposeEstimatorCaller()
    rospy.spin()

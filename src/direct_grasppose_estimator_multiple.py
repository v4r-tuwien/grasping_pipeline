#! /usr/bin/env python3
import numpy as np
from copy import deepcopy
import ros_numpy
from cv_bridge import CvBridge
import rospy
from actionlib import SimpleActionClient
from actionlib_msgs.msg import GoalStatus
from robokudo_msgs.msg import GenericImgProcAnnotatorAction, GenericImgProcAnnotatorGoal
from grasping_pipeline_msgs.srv import CallMultipleDirectGraspPoseEstimator, CallMultipleDirectGraspPoseEstimatorResponse
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from v4r_util.depth_pcd import convert_np_depth_img_to_o3d_pcd
from v4r_util.bb import get_minimum_oriented_bounding_box, o3d_bb_to_ros_bb_stamped
from tf.transformations import (quaternion_about_axis, quaternion_multiply)

class MultipleDirectGraspposeEstimatorCaller:
    '''Calls a service that directly estimates the grasppose.
    
    Calls a service that directly estimates the grasppose without needing an 
    object pose and annotations. The topic of the service is defined in the
    parameter '/grasping_pipeline/grasppoint_estimator_topic'. The results are 
    published as a marker in the rviz visualization with the topic 
    '/grasping_pipeline/grasp_marker'.

    Parameters
    ----------
    rgb: sensor_msgs.msg.Image
        The RGB image of the scene.
    depth: sensor_msgs.msg.Image
        The depth image of the scene.
    mask_detections: list of sensor_msgs.msg.Image
        The object masks.
    bb_detections: list of sensor_msgs.msg.RegionOfInterest
        The 2D bounding boxes of the objects.
    class_names: list of str
        The class names of the objects.
    
    Attributes
    ----------
    bridge: CvBridge
        The OpenCV bridge to convert images.
    srv: rospy.Service
        This service. Calls the direct grasppose estimator when called.
    cam_info: sensor_msgs.msg.CameraInfo
        The camera information/intrinsics.
    marker_pub: rospy.Publisher
        The publisher that publishes the markers in the rviz visualization.
    
    Raises
    ------
    rospy.ServiceException
        If the direct grasppose estimator service times out or fails to estimate the grasppose
        or if no mask or bounding box detections are provided.
    
    Returns
    -------
    grasp_poses: list of geometry_msgs.msg.PoseStamped
        The estimated graspposes.
    grasp_object_bbs: list of grasping_pipeline_msgs.msg.BoundingBoxStamped
        The bounding boxes of the objects to grasp.
    grasp_object_names: list of str
        The names of the objects to grasp.
    '''
    def __init__(self):
        self.bridge = CvBridge()
        self.srv = rospy.Service('call_multiple_direct_grasppose_estimator', CallMultipleDirectGraspPoseEstimator , self.execute)
        self.cam_info = rospy.wait_for_message(rospy.get_param('/cam_info_topic'), CameraInfo)
        self.marker_pub = rospy.Publisher('/grasping_pipeline/grasp_marker_array', MarkerArray, queue_size=10)
        self.marker_array = MarkerArray()
    
    def execute(self, req):
        '''
        Calls the direct grasppose estimator service and returns the grasppose.

        If no mask or bounding box detections are provided, the service will raise
        an error.

        Parameters
        ----------
        req: grasping_pipeline_msgs.srv.CallMultipleDirectGraspPoseEstimatorRequest
            The request to the service. Contains the RGB image, depth image, mask detections,
            bounding box detections, class names, and (optionally) the name of the object to grasp.

        Raises
        ------
        rospy.ServiceException
            If the direct grasppose estimator service times out or fails to estimate the grasppose
            or if no mask or bounding box detections are provided.
        
        Returns
        -------
        grasping_pipeline_msgs.srv.CallMultipleDirectGraspPoseEstimatorResponse
            The response of the service. Contains the names of the objects to grasp, the bounding boxes
            of the objects, and the estimated graspposes.
        '''
        topic = rospy.get_param('/grasping_pipeline/grasppoint_estimator_topic')
        timeout = rospy.get_param('/grasping_pipeline/timeout_duration')

        grasppose_est = SimpleActionClient(topic, GenericImgProcAnnotatorAction)

        rospy.loginfo('Waiting for direct-grasppose-estimator server with topic: %s' % topic)
        if not grasppose_est.wait_for_server(timeout=rospy.Duration(timeout)):
            rospy.logerr(f'Connection to direct-grasppose_estimator \'{topic}\' timed out!')
            raise rospy.ServiceException
        rospy.loginfo('Connected to direct-grasppose-estimator server')

        bbs_3D = self.get_3D_bbs(req.depth, req.mask_detections, req.bb_detections)

        res = CallMultipleDirectGraspPoseEstimatorResponse()
        for i, _ in enumerate(req.class_names):
            object_mask = []
            object_bb = []

            if len(req.mask_detections) > 0:
                object_mask = [req.mask_detections[i]]
            if len(req.bb_detections) > 0:
                object_bb = [req.bb_detections[i]]
            if len(object_mask) <= 0 and len(object_bb) <= 0:
                rospy.logerr(f'No mask or bb detections provided for object {req.class_names[i]}! Need at least either one to proceed!')
                raise rospy.ServiceException
        
            goal = GenericImgProcAnnotatorGoal(
                rgb=req.rgb, 
                depth=req.depth, 
                mask_detections=object_mask, 
                bb_detections=object_bb, 
                class_names=[req.class_names[i]]
            )

            grasppose_est.send_goal(goal)
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

            res.grasp_object_names.append(req.class_names[i])
            res.grasp_object_bbs.append(bbs_3D[i])
            res.grasp_poses.append(PoseStamped(header=req.depth.header, pose=graspposes.pose_results[0])) # TODO: For now we only save one (easier with the current message definition)
            self.add_bb_marker(res.grasp_object_bbs[-1], id=i)
            self.add_marker(res.grasp_poses[-1], id=i)

        self.marker_pub.publish(self.marker_array)

        return res
    
    def get_3D_bbs(self, depth, masks, bbs_2d):
        '''Extracts the 3D bounding boxes of the objects.

        If masks are provided, the 3D bounding boxes are extracted from the masks 
        and the depth image. Otherwise, if 2D bounding boxes are provided, the 3D
        bounding boxes are extracted from the 2D bounding boxes. If neither masks
        nor 2D bounding boxes are provided, an error is raised.

        Parameters
        ----------
        depth: sensor_msgs.msg.Image
            The depth image.
        masks: list of sensor_msgs.msg.Image
            The object masks.
        bbs_2d: list of sensor_msgs.msg.RegionOfInterest
            The 2D bounding boxes of the objects.
        '''
        bbs = []
        depth_np = ros_numpy.numpify(depth)
        if len(masks) > 0:
            for mask in masks:
                bb = self.get_bb_3D_from_mask(depth, depth_np, mask)
                bbs.append(bb)
        elif len(bbs_2d) > 0:
            for bb in bbs_2d:
                self.get_bb_3D_from_bb(depth, depth_np, bb)
                bbs.append(bb)
        else:
            rospy.logerr('No masks or bbs provided to extract object depth values!')
            raise rospy.ServiceException
        return bbs
        
    def get_bb_3D_from_mask(self, depth, depth_np, mask):
        '''Extracts the 3D bounding box of the object from the mask.

        Parameters
        ----------
        depth: sensor_msgs.msg.Image
            The depth image.
        depth_np: numpy.ndarray
            The depth image as a numpy array.
        mask: sensor_msgs.msg.Image
            The object mask.

        Returns
        -------
        grasping_pipeline_msgs.msg.BoundingBoxStamped
            The 3D bounding box of the object.
        '''
        depth_img_obj = np.full_like(depth_np, np.nan, dtype=np.float32)
        mask = ros_numpy.numpify(mask)
        mask = mask != 0
        # only copy the object's depth values, rest stay NaN
        depth_img_obj[mask] = depth_np[mask]
        object_pcd = convert_np_depth_img_to_o3d_pcd(depth_img_obj, self.cam_info, project_valid_depth_only=True)
        
        # do clustering to remove parts from the background
        labels = object_pcd.cluster_dbscan(eps=0.04, min_points=100, print_progress=False)
        labels_unique, counts = np.unique(labels, return_counts=True)
        max_label = labels_unique[np.argmax(counts)]
        object_pcd = object_pcd.select_by_index(np.where(labels == max_label)[0])
        
        obj_bb_o3d = get_minimum_oriented_bounding_box(object_pcd)
        return o3d_bb_to_ros_bb_stamped(obj_bb_o3d, depth.header.frame_id, depth.header.stamp)

    def get_bb_3D_from_bb(self, depth, depth_np, bb_2d):
        '''Extracts the 3D bounding box of the object from the 2D bounding box.

        Parameters
        ----------
        depth: sensor_msgs.msg.Image
            The depth image.
        depth_np: numpy.ndarray
            The depth image as a numpy array.
        bb_2d: sensor_msgs.msg.RegionOfInterest
            The 2D bounding box of the object.
        '''
        depth_img_obj = np.full_like(depth_np, np.nan)
        bb = bb_2d
        y, x = bb.y_offset, bb.x_offset
        h, w = bb.height, bb.width
        # only copy the object's depth values, rest stay NaN
        depth_img_obj[y:y+h, x:x+w] = depth_np[y:y+h, x:x+w]
        object_pcd = convert_np_depth_img_to_o3d_pcd(depth_img_obj, self.cam_info, project_valid_depth_only=True)
        
        # do clustering to remove parts from the background
        labels = object_pcd.cluster_dbscan(eps=0.04, min_points=100, print_progress=False)
        labels_unique, counts = np.unique(labels, return_counts=True)
        max_label = labels_unique[np.argmax(counts)]
        object_pcd = object_pcd.select_by_index(np.where(labels == max_label)[0])
        
        obj_bb_o3d = get_minimum_oriented_bounding_box(object_pcd)
        return o3d_bb_to_ros_bb_stamped(obj_bb_o3d, depth.header.frame_id, depth.header.stamp)


    def add_marker(self, pose_goal, id):
        """ 
        Adds a marker to the rviz visualization to show the grasp pose.
        
        The marker is an arrow pointing in the direction of the grasp pose. The arrow is red.
        The topic of the marker is '/grasping_pipeline/grasp_marker'.
        
        Parameters
        ----------
        pose_goal: geometry_msgs.msg.PoseStamped
            The pose of the grasp.
        id: int
            The id of the marker. Used to distinguish between different markers in rviz.
        """
        marker = Marker()
        marker.header.frame_id = pose_goal.header.frame_id
        marker.header.stamp = rospy.Time()
        marker.ns = 'grasp_marker_' + str(id)
        marker.id = id
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

        self.marker_array.markers.append(marker)

        rospy.loginfo('grasp_marker')

    
    def add_bb_marker(self, object_bb_stamped, id):
        '''
        Adds a marker to the rviz visualization to show the bounding box of the object.

        The marker is a blue bounding box.
        The topic of the marker is '/grasping_pipeline/grasp_marker'.

        Parameters
        ----------
        object_bb_stamped: grasping_pipeline_msgs.msg.BoundingBoxStamped
            The bounding box of the object.
        id: int
            The id of the marker. Should be unique for each object to avoid overwriting markers.
        '''
        marker = Marker()
        marker.header.frame_id = object_bb_stamped.header.frame_id
        marker.header.stamp = rospy.Time()
        marker.ns = 'bb_marker_' + str(id)
        marker.id = id
        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        marker.pose = deepcopy(object_bb_stamped.center)
        marker.scale = deepcopy(object_bb_stamped.size)

        marker.color.a = 0.5
        marker.color.r = 0
        marker.color.g = 0
        marker.color.b = 1.0
        
        self.marker_array.markers.append(marker)


if __name__ == '__main__':
    node = rospy.init_node('multiple_direct_grasppose_estimator')
    est = MultipleDirectGraspposeEstimatorCaller()
    rospy.spin()

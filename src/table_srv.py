#! /usr/bin/env python3

import rospy
import smach
from sensor_msgs.msg import PointCloud2
from grasping_pipeline_msgs.srv import TablePlaneExtractor
from v4r_util.tf2 import TF2Wrapper
from v4r_util.conversions import bounding_box_to_bounding_box_stamped, quat_to_rot_mat
from v4r_util.bb import align_bounding_box_rotation, ros_bb_to_o3d_bb, o3d_bb_to_ros_bb
from sensor_msgs.msg import CameraInfo
import numpy as np
import cv2
from sensor_msgs.msg import RegionOfInterest
from v4r_util.tf2 import TF2Wrapper
import ros_numpy
from grasping_pipeline_msgs.srv import FindTablePlanes, FindTablePlanesResponse
from grasping_pipeline_msgs.srv import RemoveNonTableObjects, RemoveNonTableObjectsResponse

class TableSrv():
    '''
    Finds the table planes in the point cloud using the TablePlaneExtractor service.
    
    Returns
    -------
    table_bbs: vision_msgs/BoundingBox3DArray
        The bounding boxes of the detected planes. The first one is considered the actual table.
    table_plane_equations: list of object_detector_msgs/Plane
        The plane equations of the detected planes.
    '''

    def __init__(self, enlarge_table_bb_to_floor=True):
        '''
        Initializes the FindTablePlanes state. Creates a TablePlaneExtractor service proxy.
        
        Parameters
        ----------
        enlarge_table_bb_to_floor: bool
            If True, the bounding boxes of the tables are enlarged to the floor. This is useful
            to prevent the robot from colliding with the table legs by creating a box that reaches
            the floor.
        '''
        rospy.init_node('table_srv')
        self.topic = rospy.get_param('/point_cloud_topic')
        self.table_extractor_srv_name = '/table_plane_extractor/get_planes'
        self.table_extractor = rospy.ServiceProxy(
            self.table_extractor_srv_name, TablePlaneExtractor)
        self.tf_wrapper = TF2Wrapper()
        self.enlarge_table_bb_to_floor = enlarge_table_bb_to_floor

        self.table_plane_service = rospy.Service('find_table_planes', FindTablePlanes, self.handle_find_table_planes)
        self.remove_non_table_objects_service = rospy.Service('remove_non_table_objects', RemoveNonTableObjects, self.handle_remove_non_table_objects)

        rospy.loginfo('Table services are ready.')

    def project_3d_bb_to_2d(self, bb_3d, camera_info):
        if camera_info.distortion_model != 'plumb_bob':
            raise ValueError('This function only supports plumb_bob distortion model.')
        camera_matrix = np.array(camera_info.K, np.float32).reshape(3, 3)
        dist_coeffs = np.array(camera_info.D, np.float32)
        
        # transform each corner of the 3D bounding box as a 3D point
        x_c = bb_3d.center.position.x
        y_c = bb_3d.center.position.y
        z_c = bb_3d.center.position.z
        rotmat = quat_to_rot_mat(bb_3d.center.orientation)
        width = rotmat @ np.array([bb_3d.size.x/2, 0, 0])
        height = rotmat @ np.array([0, bb_3d.size.y/2, 0])
        depth = rotmat @ np.array([0, 0, bb_3d.size.z/2])
        center = np.array([x_c, y_c, z_c])
        corners = np.array([center + width + height + depth,
                            center + width + height - depth,
                            center + width - height + depth,
                            center + width - height - depth,
                            center - width + height + depth,
                            center - width + height - depth,
                            center - width - height + depth,
                            center - width - height - depth])
        points_2d = cv2.projectPoints(corners, np.zeros(3), np.zeros(3), camera_matrix, dist_coeffs)[0].reshape(-1, 2)
                
        # create a mask based on the projected corner points
        mask = np.zeros((camera_info.height, camera_info.width), np.uint8)
        hull = cv2.convexHull(points_2d.astype(np.int32))
        cv2.fillConvexPoly(mask, hull, 1)
        
        # remove invalid points which are outside the image
        points_2d = points_2d[(points_2d[:, 0] >= 0) & (points_2d[:, 0] <= camera_info.width) & (points_2d[:, 1] >= 0) & (points_2d[:, 1] <= camera_info.height)]
        
        x_min = min(points_2d[:, 0])
        x_max = max(points_2d[:, 0])
        y_min = min(points_2d[:, 1])
        y_max = max(points_2d[:, 1])
        x_min = max(0, x_min)
        x_max = min(camera_info.width, x_max)
        y_min = max(0, y_min)
        y_max = min(camera_info.height, y_max)

        bb_2d = RegionOfInterest()
        bb_2d.x_offset = int(x_min)
        bb_2d.y_offset = int(y_min)
        bb_2d.width = int(x_max - x_min)
        bb_2d.height = int(y_max - y_min)
        return bb_2d, mask
    
    def filter_bb_on_table(self, bbs, table_bb):
        idxs_to_keep = []
        for i, bb in enumerate(bbs):
            overlap_area = self.calculate_overlap(bb, table_bb)
            detection_area = bb.width * bb.height
            if overlap_area / detection_area > 0.5:
                idxs_to_keep.append(i)
        return idxs_to_keep
            
    def calculate_overlap(self, bb1, bb2):
        '''
        Calculates the overlap of two bounding boxes.
        
        Parameters
        ----------
        bb1: sensor_msgs/RegionOfInterest
            The first bounding box.
        bb2: sensor_msgs/RegionOfInterest
            The second bounding box.
        '''
        x1 = max(bb1.x_offset, bb2.x_offset)
        y1 = max(bb1.y_offset, bb2.y_offset)
        x2 = min(bb1.x_offset + bb1.width, bb2.x_offset + bb2.width)
        y2 = min(bb1.y_offset + bb1.height, bb2.y_offset + bb2.height)
        w = max(0, x2 - x1)
        h = max(0, y2 - y1)
        return w * h
    
    def bb_to_mask(self, bb, width, height):
        '''
        Converts a bounding box to a mask.
        
        Parameters
        ----------
        bb: sensor_msgs/RegionOfInterest
            The bounding box.
        width: int
            The width of the mask. Generally the width of the image.
        height: int
            The height of the mask. Generally the height of the image.
        
        Returns
        -------
        np.array
            The mask.
        '''
        mask = np.zeros((height, width), np.uint8)
        mask[bb.y_offset:bb.y_offset+bb.height, bb.x_offset:bb.x_offset+bb.width] = 1
        return mask        
    
    def filter_masks_on_table(self, masks, table_mask):
        idxs_to_keep = []
        for i, mask in enumerate(masks):
            intersection = mask & table_mask
            area = np.sum(intersection)
            total_area = np.sum(mask)
            if area / total_area > 0.5:
                idxs_to_keep.append(i)
        return idxs_to_keep
        
    def filter_table_detection(self, class_names):
        table_idxs = []
        for i, class_name in enumerate(class_names):
            if 'table' in class_name:
                table_idxs.append(i)
        return table_idxs

    def handle_remove_non_table_objects(self, req):
        cam_info_topic = rospy.get_param('/cam_info_topic')
        camera_info = rospy.wait_for_message(cam_info_topic, CameraInfo)

        if len(req.table_bbs.boxes) == 0:
            rospy.logwarn('No table bounding box detected.')
            return 'failed'
        
        if len(req.bb_detections) == 0 and len(req.mask_detections) == 0:
            return 'succeeded'
        
        table_bb = req.table_bbs.boxes[0]
        table_bb = bounding_box_to_bounding_box_stamped(table_bb, req.table_bbs.header.frame_id, req.table_bbs.header.stamp)
        table_bb_camera_frame = self.tf_wrapper.transform_bounding_box(table_bb, camera_info.header.frame_id)
        table_bb_2d, table_mask = self.project_3d_bb_to_2d(table_bb_camera_frame, camera_info)

        mask_detections = req.mask_detections
        bb_detections = req.bb_detections
        class_names = req.class_names

        table_idxs = self.filter_table_detection(class_names)
        # use masks if available, otherwise use bounding boxes
        if len(mask_detections) != 0:
            if len(mask_detections) != len(class_names):
                raise ValueError(f'The number of masks and class names must be the same. {len(mask_detections) = }, {len(class_names) = }')
            np_masks = [ros_numpy.numpify(mask) for mask in mask_detections]
            idxs_to_keep = self.filter_masks_on_table(np_masks, table_mask)
        elif len(bb_detections) != 0:
            if len(bb_detections) != len(class_names):
                raise ValueError(f'The number of bounding boxes and class names must be the same. {len(bb_detections) = }, {len(class_names) = }')
            idxs_to_keep = self.filter_bb_on_table(bb_detections, table_bb_2d)
        
        # remove table detections
        idxs_to_keep = set(idxs_to_keep) - set(table_idxs)
        
        # only keep the objects that are on the table
        objects_left = len(idxs_to_keep)
        class_names = [class_names[i] for i in idxs_to_keep]
        if len(mask_detections) != 0:
            mask_detections = [mask_detections[i] for i in idxs_to_keep]
        if len(bb_detections) != 0:
            bb_detections = [bb_detections[i] for i in idxs_to_keep]

        res = RemoveNonTableObjectsResponse()
        res.bb_detections = bb_detections
        res.mask_detections = mask_detections
        res.class_names = class_names
        return res
    
    def handle_find_table_planes(self, req):
        '''
        Service handler for finding table planes in the point cloud.

        Waits for a point cloud and calls the TablePlaneExtractor service.
        '''
        rospy.loginfo('Executing state FIND_TABLE_PLANES. Waiting for point cloud.')
        cloud = rospy.wait_for_message(self.topic, PointCloud2, timeout=15)
        rospy.loginfo('Received point cloud. Waiting for table plane extractor service.')
        rospy.wait_for_service(self.table_extractor_srv_name)
        rospy.loginfo('Service available. Calling table plane extractor.')
        
        response = self.table_extractor(cloud)
        boxes = response.plane_bounding_boxes
        
        if self.enlarge_table_bb_to_floor:
            transform_to_base = False
            if boxes.header.frame_id != 'base_link':
                transform_to_base = True

            new_boxes = []
            for ros_bb in boxes.boxes:
                if transform_to_base:
                    ros_bb = bounding_box_to_bounding_box_stamped(ros_bb, boxes.header.frame_id , rospy.Time.now())
                    ros_bb = self.tf_wrapper.transform_bounding_box(ros_bb, 'base_link')
                
                aligned_bb_o3d = align_bounding_box_rotation(ros_bb_to_o3d_bb(ros_bb))
                ros_bb = o3d_bb_to_ros_bb(aligned_bb_o3d)

                center = ros_bb.center.position
                old_center_z = center.z
                size = ros_bb.size
                center.z = (center.z + size.z/2)/2
                size.x = size.x + 0.04
                size.y = size.y + 0.04
                size.z = old_center_z + size.z/2 - 0.02
                new_boxes.append(ros_bb)

            response.plane_bounding_boxes.boxes = new_boxes
            response.plane_bounding_boxes.header.frame_id = 'base_link'

        res = FindTablePlanesResponse()
        res.table_bbs = response.plane_bounding_boxes
        res.table_plane_equations = response.planes
        return res
    
if __name__ == '__main__':
    table_srv = TableSrv()
    rospy.spin()
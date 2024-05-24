#! /usr/bin/env python3
import numpy as np
import cv2
from cv_bridge import CvBridge
import rospy
import matplotlib.pyplot as plt
from actionlib import SimpleActionClient
from robokudo_msgs.msg import GenericImgProcAnnotatorAction, GenericImgProcAnnotatorGoal
from grasping_pipeline_msgs.srv import CallObjectDetector, CallObjectDetectorResponse
from sensor_msgs.msg import Image, RegionOfInterest

class CallObjectDetectorService:

    def __init__(self):
        self.bridge = CvBridge()
        self.srv = rospy.Service('call_object_detector', CallObjectDetector , self.execute)
        self.label_image_pub = rospy.Publisher('/grasping_pipeline/obj_det_label_image', Image, queue_size=1)
        self.bb_image_pub = rospy.Publisher('/grasping_pipeline/obj_det_bb_image', Image, queue_size=1)
        rospy.loginfo('Known Object Detector Service initialized')
    
    def execute(self, req):
        topic = rospy.get_param('/grasping_pipeline/object_detector_topic')
        timeout = rospy.get_param('/grasping_pipeline/timeout_duration')

        obj_det = SimpleActionClient(topic, GenericImgProcAnnotatorAction)

        rospy.loginfo('Waiting for object detector server with topic: %s' % topic)
        if not obj_det.wait_for_server(timeout=rospy.Duration(timeout)):
            rospy.logerr(f'Connection to object detector \'{topic}\' timed out!')
            raise rospy.ServiceException
        rospy.loginfo('Connected to object detector server')

        goal = GenericImgProcAnnotatorGoal()
        goal.rgb = req.rgb
        goal.depth = req.depth

        rospy.logdebug('Sending goal to object detector')
        obj_det.send_goal(goal)
        rospy.logdebug('Waiting for object detection results')
        goal_finished = obj_det.wait_for_result(rospy.Duration(timeout))
        if not goal_finished:
            rospy.logerr('Object Detector didn\'t return results before timing out!')
            raise rospy.ServiceException
        detection_result = obj_det.get_result()

        if not detection_result.success or len(detection_result.class_names) <= 0:
            rospy.logerr('Object Detector failed to detect objects!')
            raise rospy.ServiceException
        rospy.loginfo(f'Detected {len(detection_result.class_names)} objects.')

        np_rgb = self.bridge.imgmsg_to_cv2(req.rgb)

        valid_label_image = check_label_img(detection_result.image)
        if valid_label_image:
            label_image_np = self.bridge.imgmsg_to_cv2(detection_result.image)
            label_image_np = visualize_label_image(np_rgb, label_image_np)
            self.label_image_pub.publish(self.bridge.cv2_to_imgmsg(label_image_np))

            if len(detection_result.bounding_boxes) <= 0:
                bbs = self.convert_label_img_to_2D_BB(label_image_np)
                detection_result.bounding_boxes = bbs

        if not valid_label_image and len(detection_result.bounding_boxes) <= 0:
            rospy.logerr('No valid label image and no bounding boxes detected! Need at least one of them!')
            raise rospy.ServiceException

        bb_image_np = visualize_rois(np_rgb, detection_result.bounding_boxes)
        self.bb_image_pub.publish(self.bridge.cv2_to_imgmsg(bb_image_np))

        res = CallObjectDetectorResponse()
        if valid_label_image:
            res.mask_detections = self.split_label_image_into_masks_ros(detection_result.image)
        res.bb_detections = detection_result.bounding_boxes
        res.class_names = detection_result.class_names
        res.class_confidences = detection_result.class_confidences
        
        return res
    
    def convert_label_img_to_2D_BB(self, label_img):
        labels_unique = np.unique(label_img)
        ROI_2d = []
        for label in labels_unique:
            if label == -1:
                continue
            roi = RegionOfInterest()
            obj_mask = (label_img == label)
            rows = np.any(obj_mask, axis=1)
            cols = np.any(obj_mask, axis=0)
            rmin, rmax = np.where(rows)[0][[0, -1]]
            cmin, cmax = np.where(cols)[0][[0, -1]]
            roi = RegionOfInterest(x_offset = cmin, 
                                   y_offset=rmin, 
                                   width = cmax-cmin, 
                                   height = rmax-rmin)
            ROI_2d.append(roi)
        return ROI_2d
    
    def split_label_image_into_masks_ros(self, label_image):
        label_image_np = self.bridge.imgmsg_to_cv2(label_image)
        masks_np = self.split_label_image_into_masks_np(label_image_np)
        masks_ros = []
        for mask_np in masks_np:
            mask_ros = self.bridge.cv2_to_imgmsg(mask_np)
            masks_ros.append(mask_ros)
        return masks_ros

    def split_label_image_into_masks_np(self, label_image):
        unique_labels = np.unique(label_image)
        masks = []
        for label in unique_labels:
            if label == -1:
                continue
            mask = (label_image == label).astype(np.uint8)
            masks.append(mask)
        return masks
        
def visualize_rois(image, rois):
    image_copy = image.copy()
    for roi in rois:
        x1 = roi.x_offset
        y1 = roi.y_offset
        x2 = roi.x_offset + roi.width
        y2 = roi.y_offset + roi.height
        cv2.rectangle(image_copy, (x1, y1), (x2, y2), (0, 255, 0), 2)
    return image_copy

def visualize_label_image(image, label_image):
    image_copy = image.copy()
    cmap = plt.get_cmap('magma')
    unique_labels = np.unique(label_image)
    for label in unique_labels:
        if label == -1:
            continue
        mask = label_image == label
        color = np.array(cmap(label/(len(unique_labels)-1))[:3]) * 255
        image_copy[mask] = color
    return image_copy

def check_label_img(label_img):
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
    if (label_img.height < 1 or label_img.width < 1):
        rospy.loginfo("No label image passed!")
        return False

    # Expect signed image with -1 to indicate pixels without object
    supported_encodings = ['8SC1', '16SC1', '32SC1']
    if(label_img.encoding not in supported_encodings):
        rospy.logerr(f"Label Image Encoding not supported: Got {label_img.encoding = } but has to be one off {supported_encodings}")
        return False
    return True

if __name__ == '__main__':
    node = rospy.init_node('object_detector')
    obj_det = CallObjectDetectorService()
    rospy.spin()

    
#! /usr/bin/env python3
import numpy as np
import cv2
import rospy
import ros_numpy
import matplotlib.pyplot as plt
from actionlib import SimpleActionClient
from actionlib_msgs.msg import GoalStatus
from robokudo_msgs.msg import GenericImgProcAnnotatorAction, GenericImgProcAnnotatorGoal
from grasping_pipeline_msgs.srv import CallObjectDetector, CallObjectDetectorResponse
from sensor_msgs.msg import Image, RegionOfInterest

class CallObjectDetectorService:
    '''Service that calls the object detector and returns the detected objects.

    This service calls the object detector server and returns the detected objects.
    It also visualizes the detected objects and publishes the visualization images.

    Parameters
    ----------
    rgb : sensor_msgs.msg.Image
        RGB image of the scene
    depth : sensor_msgs.msg.Image
        Depth image of the scene

    Attributes
    ----------
    srv : rospy.Service
        This service. Calls the object detector when called
    label_image_pub : rospy.Publisher
        Publisher for the label image visualization. This is only published if the 
        object detector returns a valid label image. If the object detector doesn't return a
        valid label image, the label image is generated from the bounding boxes. 
        The label image is visualized by coloring each object with a unique color. 
        The topic is '/grasping_pipeline/obj_det_label_image'.
    bb_image_pub : rospy.Publisher
        Publisher for the bounding box visualization. The bounding boxes are visualized by drawing
        a rectangle around each object. The topic is '/grasping_pipeline/obj_det_bb_image'.

    Other Parameters
    ----------------
    topic : str
        Name of the object detector topic. Loaded from the 'grasping_pipeline/object_detector_topic' parameter
    timeout : int
        Timeout duration for the object detector. Loaded from the 'grasping_pipeline/timeout_duration' parameter

    Returns
    -------
    mask_detections : List[sensor_msgs.msg.Image]
        List of masks for each detected object. Each mask is a binary image where the object
        is white and the background is black.
    bb_detections : List[sensor_msgs.msg.RegionOfInterest]
        List of bounding boxes for each detected object. Each bounding box is a rectangle
        that encloses the object.
    class_names : List[str]
        List of class names for each detected object.
    class_confidences : List[float]
        List of confidences for each detected object.
    '''

    def __init__(self):
        self.srv = rospy.Service('call_object_detector', CallObjectDetector , self.execute)
        self.label_image_pub = rospy.Publisher('/grasping_pipeline/obj_det_label_image', Image, queue_size=1, latch=True)
        self.bb_image_pub = rospy.Publisher('/grasping_pipeline/obj_det_bb_image', Image, queue_size=1, latch=True)
        rospy.loginfo('Known Object Detector Service initialized')
    
    def execute(self, req):
        '''Calls the object detector and returns the detected objects.

        This function calls the object detector server and returns the detected objects.
        It also visualizes the detected objects and publishes the visualization images.
        It converts the label image from the object detector to masks if the label image is valid.
        If no bounding boxes were passed by the object detector, it converts the label image to bounding boxes.

        Parameters
        ----------
        req : grasping_pipeline_msgs.srv.CallObjectDetectorRequest
            Request containing the RGB and Depth images

        Raises
        ------
        rospy.ServiceException
            If the object detector fails to detect objects or times out or if neither a valid label image
            nor bounding boxes were returned by the object detector.
        
        Returns
        -------
        grasping_pipeline_msgs.srv.CallObjectDetectorResponse
            Response containing the detected objects. The response contains the masks, bounding boxes,
            class names and confidences for the detected objects.
        '''
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
        server_state = obj_det.get_state()
        
        if server_state != GoalStatus.SUCCEEDED or len(detection_result.class_names) <= 0:
            rospy.logwarn('Object Detector failed to detect objects!')
            # return empty response if no objects were detected
            return CallObjectDetectorResponse()
        rospy.loginfo(f'Detected {len(detection_result.class_names)} objects.')

        np_rgb = ros_numpy.numpify(req.rgb)

        valid_label_image = check_label_img(detection_result.image)
        if valid_label_image:
            label_image_np = ros_numpy.numpify(detection_result.image)
            label_image_vis = visualize_label_image(np_rgb, label_image_np)
            self.label_image_pub.publish(ros_numpy.msgify(Image, label_image_vis, encoding='rgb8'))

            if len(detection_result.bounding_boxes) <= 0:
                bbs = self.convert_label_img_to_2D_BB(label_image_np)
                detection_result.bounding_boxes = bbs

        if not valid_label_image and len(detection_result.bounding_boxes) <= 0:
            rospy.logerr('No valid label image and no bounding boxes detected! Need at least one of them!')
            raise rospy.ServiceException

        bb_image_np = visualize_rois(np_rgb, detection_result.bounding_boxes, detection_result.class_names)
        self.bb_image_pub.publish(ros_numpy.msgify(Image, bb_image_np, encoding='rgb8'))

        res = CallObjectDetectorResponse()
        if valid_label_image:
            res.mask_detections = self.split_label_image_into_masks_ros(detection_result.image)
        res.bb_detections = detection_result.bounding_boxes
        res.class_names = detection_result.class_names
        res.class_confidences = detection_result.class_confidences

        rospy.set_param('/grasping_pipeline/object_detection/class_names', res.class_names)
        
        return res
    
    def convert_label_img_to_2D_BB(self, label_img):
        ''' Converts the label image to 2D bounding boxes.

        Converts the label image to 2D bounding boxes. The bounding boxes are calculated by 
        finding the minimumand maximum row and column indices of each object in the label image.

        Parameters
        ----------
        label_img : np.ndarray
            The label image. Each object in the label image has to have a unique label.
        
        Returns
        -------
        List[sensor_msgs.msg.RegionOfInterest]
            List of bounding boxes for each detected object.
        '''
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
        ''' Splits the label image into masks.

        Splits the label image into masks. Each mask is a binary image where the object
        is white and the background is black.

        Parameters
        ----------
        label_image : sensor_msgs.msg.Image
            The label image. Each object in the label image has to have a unique label.
        
        Returns
        -------
        List[sensor_msgs.msg.Image]
            List of masks for each detected object. The background pixels have a value of
            0 and the object pixels have a value of != 0.
        '''
        label_image_np = ros_numpy.numpify(label_image)
        masks_np = self.split_label_image_into_masks_np(label_image_np)
        masks_ros = []
        for mask_np in masks_np:
            mask_ros = ros_numpy.msgify(Image, mask_np, encoding='8UC1')
            masks_ros.append(mask_ros)
        return masks_ros

    def split_label_image_into_masks_np(self, label_image):
        '''Splits the label image into masks.

        Splits the label image into masks. Each mask is a binary image where the object
        is white and the background is black.

        Parameters
        ----------
        label_image : np.ndarray
            The label image. Each object in the label image has to have a unique label.
        
        Returns
        -------
        List[np.ndarray]
            List of masks for each detected object. The background pixels have a value of
            0 and the object pixels have a value of != 0.
        '''
        unique_labels = np.unique(label_image)
        masks = []
        for label in unique_labels:
            if label == -1:
                continue
            mask = (label_image == label).astype(np.uint8)
            masks.append(mask)
        return masks
        
def visualize_rois(image, rois, class_names, scale=2):
    '''Visualizes the bounding boxes on the image.

    Visualizes the bounding boxes on the image. The bounding boxes are visualized by drawing
    a rectangle around each object. The class name of each object is also displayed above the bounding box.

    Parameters
    ----------
    image : np.ndarray
        The image on which the bounding boxes are visualized.
    rois : List[sensor_msgs.msg.RegionOfInterest]
        List of bounding boxes for each detected object.
    class_names : List[str]
        List of class names for each detected object.
    scale : int
        Scale factor for the image. The image is resized by this factor before visualizing the bounding boxes. This is used to make the labeled text more readable (since it has more pixels).

    Returns
    -------
    np.ndarray
        The image with the bounding boxes visualized.
    '''
    num_classes = len(class_names)
    image_copy = image.copy()
    image_copy = cv2.resize(image_copy, (image_copy.shape[1]*scale, image_copy.shape[0]*scale))
    
    for i, (roi, class_name) in enumerate(zip(rois, class_names)):
        
        blue = 255 / num_classes * i
        green = 255 - blue
        red = 0
        color = (blue, green, red)
        color = (0, 0, 0)

        x1 = roi.x_offset * scale
        y1 = roi.y_offset * scale
        x2 = (roi.x_offset + roi.width) * scale
        y2 = (roi.y_offset + roi.height) * scale
        cv2.rectangle(image_copy, (x1, y1), (x2, y2), color, 2)
        cv2.putText(image_copy, class_name, (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5*scale, color, 2)
    return image_copy

def visualize_label_image(image, label_image):
    '''Visualizes the label image.

    Visualizes the label image. The label image is visualized by coloring each object with a unique color.
    The color is determined by the label of the object. The background pixels are taken from the original image.

    Parameters
    ----------
    image : np.ndarray
        The original image. Used to get the background pixels.
    label_image : np.ndarray
        The label image. The labels are used to color the objects.
    
    Returns
    -------
    np.ndarray
        The image with the labels visualized.
    '''
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

    
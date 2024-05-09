import os
import numpy as np
import cv2
from cv_bridge import CvBridge
import rospy
import matplotlib.pyplot as plt
from actionlib import SimpleActionClient
from robokudo_msgs.msg import GenericImgProcAnnotatorAction, GenericImgProcAnnotatorGoal

topic = '/object_detector/yolov5'
data_dir = os.path.dirname(__file__)


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

def plot_images(images, titles=['Original', 'BBs', 'Masks']):
    fig, ax = plt.subplots(1, len(images))
    for i, image in enumerate(images):
        ax[i].title.set_text(titles[i])
        ax[i].imshow(image, interpolation='nearest', aspect='equal')
    plt.show()

if __name__ == '__main__':
    node = rospy.init_node('test_object_detector')

    bridge = CvBridge()
    image_path = os.path.join(data_dir,'img_01.png')
    image = cv2.imread(os.path.join(data_dir,'data/img_01.png'))
    ros_image = bridge.cv2_to_imgmsg(image, encoding='rgb8')
    
    client = SimpleActionClient(topic, GenericImgProcAnnotatorAction)
    client.wait_for_server()
    
    goal = GenericImgProcAnnotatorGoal()
    goal.rgb = ros_image
    
    client.send_goal(goal)
    client.wait_for_result()
    result = client.get_result()
    
    roi_image = visualize_rois(image, result.bounding_boxes)
    mask_image = visualize_label_image(image, bridge.imgmsg_to_cv2(result.image))
    plot_images([image, roi_image, mask_image])
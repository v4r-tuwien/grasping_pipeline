import rospy
from actionlib import SimpleActionServer
from sensor_msgs.msg import RegionOfInterest
from cv_bridge import CvBridge, CvBridgeError
from robokudo_msgs.msg import GenericImgProcAnnotatorResult, GenericImgProcAnnotatorAction
import numpy as np

model_name = "yolov100"

class MODEL_ROS_WRAPPER():
    def __init__(self):
        self.bridge = CvBridge()
        self.server = SimpleActionServer(f'/object_detector/{model_name}', GenericImgProcAnnotatorAction, self.service_call, False)
        self.model = MYMODEL()
        self.server.start()

    def service_call(self, goal):
        rgb = goal.rgb
        width, height = rgb.width, rgb.height

        try:
            image = self.bridge.imgmsg_to_cv2(rgb, "rgb8")
        except CvBridgeError as e:
            print(e)

        ros_detections = self.inference(image, rgb.header) 

        if ros_detections.success:
            self.server.set_succeeded(ros_detections)
        else:
            self.server.set_aborted(ros_detections)

    def inference(self, image, rgb_header):
        height, width, channels = image.shape

        results = self.model.inference(image)  # predict on an image
        confidences = []
        bboxes = []
        class_names = []
        label_image = np.full((height, width), -1, np.int16)
        idx = 0
        for box, mask, class_name, confidence in results:
            class_names.append(class_name)
            
            bb = RegionOfInterest()
            xmin = int(box[0])
            ymin = int(box[1])
            xmax = int(box[2])
            ymax = int(box[3])
            bb.x_offset = xmin
            bb.y_offset = ymin
            bb.height = ymax - ymin
            bb.width = xmax - xmin
            bb.do_rectify = False
            bboxes.append(bb)
            
            label_image[mask > 0] = idx

            confidences.append(confidence)
            idx += 1

        if len(class_names) > 0:
            mask_image = self.bridge.cv2_to_imgmsg(label_image, encoding="16SC1", header=rgb_header)
            server_result = GenericImgProcAnnotatorResult()
            server_result.success = True
            server_result.bounding_boxes = bboxes
            server_result.class_names = class_names
            server_result.class_confidences = confidences
            server_result.image = mask_image
        else:
            server_result = GenericImgProcAnnotatorResult()
            server_result.success = False

        return server_result 
    
if __name__ == "__main__":

    try:
        rospy.init_node(f'{model_name.upper()}')
        MODEL_ROS_WRAPPER()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
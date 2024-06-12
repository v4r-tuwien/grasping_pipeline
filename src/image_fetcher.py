#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from message_filters import ApproximateTimeSynchronizer, Subscriber
from grasping_pipeline_msgs.srv import FetchImages, FetchImagesResponse

class SynchronizedImageFetcher:
    '''
    This class provides a service that fetches synchronized RGB and Depth images from the robot.
    Instead of subscribing to the RGB and Depth topics directly, it uses ApproximateTimeSynchronizer
    to ensure that the images are captured at the same time. Additionally, it unregisters the subscribers
    after the images are captured to save bandwidth.

    Attributes
    ----------
    service : rospy.Service
        This service. Fetches synchronized images when called
    rgb_image : sensor_msgs.msg.Image
        RGB image captured by the robot
    depth_image : sensor_msgs.msg.Image
        Depth image captured by the robot
    
    Other Parameters
    ----------------
    rgb_topic : str
        Name of the RGB image topic. Loaded from the 'rgb_topic' parameter
    depth_topic : str
        Name of the Depth image topic. Loaded from the 'depth_topic' parameter
    
    Returns
    -------
    rgb : sensor_msgs.msg.Image
        RGB image captured by the robot
    depth : sensor_msgs.msg.Image
        Depth image captured by the robot
    '''
    def __init__(self):
        rospy.init_node('synchronized_image_fetcher')
        self.service = rospy.Service('fetch_synchronized_images', FetchImages, self.fetch)
        self.rgb_image = None
        self.depth_image = None

    def fetch(self, req):
        '''Fetches synchronized RGB and Depth images from the robot.

        After the images are captured, the service unregisters the subscribers to save bandwidth.

        Returns
        -------
        FetchImagesResponse
            Response containing the synchronized RGB and Depth images
        '''
        rgb_topic = rospy.get_param('/rgb_topic')
        depth_topic = rospy.get_param('/depth_topic')
        rgb_sub = Subscriber(rgb_topic, Image)
        depth_sub = Subscriber(depth_topic, Image)

        ats = ApproximateTimeSynchronizer([rgb_sub, depth_sub], queue_size=5, slop=0.1)
        ats.registerCallback(self.callback)

        rospy.loginfo('Waiting for synchronized images...')
        while self.rgb_image is None or self.depth_image is None:
            rospy.sleep(0.05)
        
        rospy.loginfo('Synchronized Images captured!')

        # Unregister subscribers to save bandwidth
        rgb_sub.sub.unregister()
        depth_sub.sub.unregister()

        response = FetchImagesResponse(self.rgb_image, self.depth_image)

        # Reset stored images for next request
        self.rgb_image = None
        self.depth_image = None

        return response

    def callback(self, rgb_msg, depth_msg):
        '''
        Callback function for ApproximateTimeSynchronizer. Stores the synchronized RGB and Depth images.
        '''
        self.rgb_image = rgb_msg
        self.depth_image = depth_msg

if __name__ == "__main__":
    fetcher = SynchronizedImageFetcher()
    rospy.spin()
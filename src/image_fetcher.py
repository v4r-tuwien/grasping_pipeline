#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from message_filters import ApproximateTimeSynchronizer, Subscriber
from grasping_pipeline_msgs.srv import FetchImages, FetchImagesResponse

class SynchronizedImageFetcher:
    def __init__(self):
        rospy.init_node('synchronized_image_fetcher')
        self.service = rospy.Service('fetch_synchronized_images', FetchImages, self.fetch)
        self.rgb_image = None
        self.depth_image = None

    def fetch(self, req):
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
        self.rgb_image = rgb_msg
        self.depth_image = depth_msg

if __name__ == "__main__":
    fetcher = SynchronizedImageFetcher()
    rospy.spin()
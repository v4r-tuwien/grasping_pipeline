#!/usr/bin/env python3

from hsr_wrapper import HSR_wrapper
import rospy
from std_msgs.msg import String

class SimpleSpeechOutputServer:
    def __init__(self):
        self.hsr_wrapper = HSR_wrapper()
        self.speech_output = rospy.get_param("/speech_output", default='english')

        if self.speech_output == 'german':
            self.tts_coqui_publisher = rospy.Publisher('/tts_request', String, queue_size=0)
        self.tts_subscriber = rospy.Subscriber(
            '/sasha_say', String, self._tts_cb)

    def _tts_cb(self, data):
        message = data.data
        if self.speech_output == 'english':
            self.hsr_wrapper.tts_say(message)
        elif self.speech_output == 'german':
            self.tts_coqui_publisher.publish(data)

if __name__ == '__main__':
    rospy.init_node('simple_speech_output_server')
    rospy.loginfo('Simple Speech Output Server started')
    handover = SimpleSpeechOutputServer()
    rospy.spin()
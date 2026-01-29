#! /usr/bin/env python3

import rospy
from hsrb_interface import Robot, geometry
from grasping_pipeline_msgs.srv import TextToSpeech, TextToSpeechResponse

class TTSService():

    def __init__(self):
        rospy.init_node('tts_srv')
        self.robot = Robot()
        self.tts = self.robot.try_get('default_tts')
        self.tts.language = self.tts.ENGLISH

        self.srv = rospy.Service('tts_srv', TextToSpeech, self.handle_request)
        rospy.loginfo('Text to speech service is ready')

    def handle_request(self, req):
        self.tts.say(req.input)

        res = TextToSpeechResponse()
        res.result = True
        return res
    
if __name__ == '__main__':
    srv = TTSService()
    rospy.spin()
        
#! /usr/bin/env python

import rospy
import actionlib

from grasping_pipeline.msg import FindGrasppointAction, FindGrasppointGoal

if __name__ == '__main__':
    rospy.init_node('find_grasppoint_client')
    client = actionlib.SimpleActionClient('find_grasppoint', FindGrasppointAction)
    client.wait_for_server()
    
    goal = FindGrasppointGoal()
    goal.method = 1
    # Fill in the goal here
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration.from_sec(25.0))
    state = client.get_state()
    if state == 3:
        print client.get_result()
    elif state == 4:
        print 'Goal aborted'
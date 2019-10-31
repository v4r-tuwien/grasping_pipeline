#!/usr/bin/env python
# Copyright (C) 2016 Toyota Motor Corporation
import actionlib
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, PoseStamped, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import rospy
import tf.transformations

from hsrb_interface import Robot
from math import pi
from enum import Enum

import smach_ros
import smach
import random
import copy

from grasping_pipeline.msg import FindGrasppointAction, FindGrasppointGoal, ExecuteGraspAction, ExecuteGraspGoal



# Enum for states
class States(Enum):
    GRASP = 1
    OPEN = 2
    RETURN_TO_NEUTRAL = 3
    PRINT_GRASP_POSE = 4
    QUIT = 5
    SHOW_COMMANDS = 6
    FIND_GRASP = 7
    EXECUTE_GRASP = 8


# Mapping of states to characters
states_keys = {States.GRASP: 'g',
               States.OPEN: 'o',
               States.RETURN_TO_NEUTRAL: 'n',
               States.PRINT_GRASP_POSE: 'p',
               States.QUIT: 'q',
               States.SHOW_COMMANDS: 's',
               States.FIND_GRASP: 'f',
               States.EXECUTE_GRASP: 'e'}


class UserInput(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['quitting', 'neutral', 'grasping', 'opening', 'find_grasp', 'execute_grasp'],
                                    input_keys=['found_grasp_pose'],
                                    output_keys=['find_grasppoint_method'])

    def execute(self, userdata):
        rospy.loginfo('Executing state UserInput')
        self.print_help()
        while not rospy.is_shutdown():
            while True:
                user_input = raw_input('CMD> ')
                if len(user_input) == 1:
                    break
                print('Please enter only one character')
            char_in = user_input.lower()
            # Quit
            if char_in is None or char_in == states_keys[States.QUIT]:
                rospy.loginfo('Quitting')
                return 'quitting'
            # Grasp
            elif char_in == states_keys[States.GRASP]:
                rospy.loginfo('Grasping object')
                print('Choose a method for grasppoint calculation')
                print('\t1 - yolo + haf_grasping')
                while True:
                    user_input = raw_input('CMD> ')
                    if len(user_input) == 1:
                        break
                    print('Please enter only one character')
                char_in = user_input.lower()
                if char_in == '1':
                    userdata.find_grasppoint_method = 1
                    return 'grasping'
                else:
                    userdata.find_grasppoint_method = 0
                    print ('Not a valid method')
                    self.print_help()
            # Open gripper
            elif char_in == states_keys[States.OPEN]:
                rospy.loginfo('Opening gripper')
                return 'opening'
            # Return to neutral position
            elif char_in == states_keys[States.RETURN_TO_NEUTRAL]:
                rospy.loginfo('Returning robot to neutral position')
                return 'neutral'
            # Print the grasp_pose
            elif char_in == states_keys[States.PRINT_GRASP_POSE]:
                rospy.loginfo('Grasp pose:')
                try:
                    print (userdata.found_grasp_pose)
                except:
                    print ('No grasp pose found yet. Have you executed "FIND_GRASP" before?')
            # Show commands
            elif char_in == states_keys[States.SHOW_COMMANDS]:
                rospy.loginfo('Showing state machine commands')
                self.print_help()
            # Find grasp
            elif char_in == states_keys[States.FIND_GRASP]:
                rospy.loginfo('Finding a grasp pose')
                print('Choose a method for grasppoint calculation')
                print('\t1 - yolo + haf_grasping')
                while True:
                    user_input = raw_input('CMD> ')
                    if len(user_input) == 1:
                        break
                    print('Please enter only one character')
                char_in = user_input.lower()
                if char_in == '1':
                    userdata.find_grasppoint_method = 1
                    return 'find_grasp'
                else:
                    userdata.find_grasppoint_method = 0
                    print ('Not a valid method')
                    self.print_help()
            elif char_in == states_keys[States.EXECUTE_GRASP]:
                rospy.loginfo('Execute last found grasp')
                try:
                    print (userdata.found_grasp_pose)
                    return 'execute_grasp'
                except:
                    print ('No grasp pose found yet. Have you executed "FIND_GRASP" before?')
                    self.print_help()
            # Unrecognized command
            else:
                rospy.logwarn('Unrecognized command %s', char_in)

    def print_help(self):
        for name, member in States.__members__.items():
            print(states_keys[member] + ' - ' + name)


class GoToNeutral(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])

        #Robot initialization
        self.robot = Robot()
        self.base = self.robot.try_get('omni_base')
        self.tts = self.robot.try_get('default_tts')
        self.whole_body = self.robot.try_get('whole_body')

        self.move_client = actionlib.SimpleActionClient('/move_base/move', MoveBaseAction)

    def execute(self, userdata):
        rospy.loginfo('Executing state GoToNeutral')
        self.whole_body.move_to_neutral()
        self.whole_body.move_to_joint_positions({'arm_roll_joint':pi/2})
        move_goal = MoveBaseGoal()
        move_goal.target_pose.header.frame_id='map'
        move_goal.target_pose.pose.position.x = 1.1
        move_goal.target_pose.pose.position.y = -0.75
        move_goal.target_pose.pose.orientation.z = 1.0
        self.move_client.wait_for_server()
        self.move_client.send_goal(move_goal)
        self.move_client.wait_for_result()
        self.whole_body.gaze_point((0.7, 0.1, 0.4))
        return 'succeeded'


class Opening(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])

        #Robot initialization
        self.robot = Robot()
        self.gripper = self.robot.try_get('gripper')

    def execute(self, userdata):
        rospy.loginfo('Executing state Opening')
        self.gripper.command(1.0)
        return 'succeeded'


def main():
    rospy.init_node('grasping_statemachine')

    sm = smach.StateMachine(outcomes=['end'])
    with sm:
        smach.StateMachine.add('USER_INPUT', \
                               UserInput(), \
                               transitions={'quitting':'end', 
                                            'neutral':'GO_TO_NEUTRAL',
                                            'grasping':'FIND_GRASPPOINT',
                                            'opening':'OPENING',
                                            'find_grasp':'ONLY_FIND_GRASPPOINT',
                                            'execute_grasp':'EXECUTE_GRASP'})

        smach.StateMachine.add('GO_TO_NEUTRAL',
                                GoToNeutral(), \
                                transitions={'succeeded':'USER_INPUT'})

        smach.StateMachine.add('OPENING',
                                Opening(), \
                                transitions={'succeeded':'USER_INPUT'})
        smach.StateMachine.add('FIND_GRASPPOINT', \
                                smach_ros.SimpleActionState('find_grasppoint', FindGrasppointAction,
                                                            goal_slots = ['method'],
                                                            result_slots = ['grasp_pose']),
                                transitions={'succeeded':'EXECUTE_GRASP', 
                                            'preempted':'USER_INPUT',
                                            'aborted':'USER_INPUT'},
                                remapping={ 'method':'find_grasppoint_method', 
                                            'grasp_pose':'found_grasp_pose'})
        smach.StateMachine.add('ONLY_FIND_GRASPPOINT', \
                                smach_ros.SimpleActionState('find_grasppoint', FindGrasppointAction,
                                                            goal_slots = ['method'],
                                                            result_slots = ['grasp_pose']),
                                transitions={'succeeded':'USER_INPUT', 
                                            'preempted':'USER_INPUT',
                                            'aborted':'USER_INPUT'},
                                remapping={ 'method':'find_grasppoint_method', 
                                            'grasp_pose':'found_grasp_pose'})
        smach.StateMachine.add('EXECUTE_GRASP',
                                smach_ros.SimpleActionState('execute_grasp', ExecuteGraspAction,
                                                            goal_slots=['grasp_pose']),
                                transitions={'succeeded':'USER_INPUT', 
                                            'preempted':'USER_INPUT',
                                            'aborted':'USER_INPUT'},
                                remapping={'grasp_pose':'found_grasp_pose'})



    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    #Execute state machine
    outcome = sm.execute()

    # Wait for ctrl-c to stop the application
    #rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()

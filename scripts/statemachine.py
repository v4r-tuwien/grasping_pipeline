#!/usr/bin/env python
# Copyright (C) 2016 Toyota Motor Corporation
import copy
import random
from enum import Enum
from math import pi

import actionlib
import rospy
import smach
import smach_ros
import tf.transformations
import tf
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, PoseStamped, Quaternion
from hsrb_interface import Robot
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

from grasping_pipeline.msg import (ExecuteGraspAction, ExecuteGraspGoal,
                                   FindGrasppointAction, FindGrasppointGoal)
from handover.msg import HandoverAction, HandoverGoal

# neutral joint positions
neutral_joint_positions ={'arm_flex_joint': 0.0,
            'arm_lift_joint': 0.0,
            'arm_roll_joint': 1.570,
            'hand_motor_joint': 1.0,
            'head_pan_joint': 0.0,
            'head_tilt_joint': -0.75,
            'wrist_flex_joint': -1.57,
            'wrist_roll_joint': 0.0} 

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
    PRINT_USERDATA = 9
    CONFIG = 10
    CLEAN_TABLE = 11

# Mapping of states to characters
states_keys = {States.GRASP: 'g',
               States.OPEN: 'o',
               States.RETURN_TO_NEUTRAL: 'n',
               States.PRINT_GRASP_POSE: 'p',
               States.QUIT: 'q',
               States.SHOW_COMMANDS: 's',
               States.FIND_GRASP: 'f',
               States.EXECUTE_GRASP: 'e',
               States.PRINT_USERDATA: 'u',
               States.CONFIG: 'c',
               States.CLEAN_TABLE: 't'}

# Mapping of numbers to object names
objects_keys = {
                1: "002_master_chef_can",
                2: "003_cracker_box",
                3: "005_tomato_soup_can",
                4: "006_mustard_bottle",
                5: "009_gelatin_box",
                6: "010_potted_meat_can",
                7: '011_banana',
                8: '021_bleach_cleanser',
                9: '024_bowl',
                10: '025_mug',
#                11: '035_power_drill',
                11: '061_foam_brick'}


objects_display_names= {1: "Coffee Master Chef",
                        2: "Cracker Cheezit",
                        3: "Tomato Soup",
                        4: "Mustard",
                        5: "Gelatin Jello",
                        6: "Potted Meat Spam",
                        7: 'Banana',
                        8: 'Bleach Cleanser Soft Scrub',
                        9: 'Bowl',
                        10: 'Mug',
#                        11: 'Power Drill',
                        11: 'Foam Brick'}


class UserInput(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['quitting', 'neutral', 'grasping', 'opening', 
                                            'find_grasp', 'execute_grasp', 'go_to_table'],
                                    input_keys=['found_grasp_pose', 'grasp_height'],
                                    output_keys=['find_grasppoint_method', 'objects_to_find',
                                    'grasp_height', 'safety_distance', 'params', 'find_grasppoint_tries'])
        #config initial settings
        self.use_map = rospy.get_param("/use_map")
        self.grasp_check = rospy.get_param("/grasp_check")
        #grasp_params for unknown object method
        self.grasp_height_unknown = rospy.get_param("/grasp_height_unknown", default=0.07)
        self.safety_distance_unknown = rospy.get_param("safety_distance_unknown", default=0.14)

        #grasp params for known object method
        self.grasp_height_known = rospy.get_param("/grasp_height_known", default=0.0)
        self.safety_distance_known = rospy.get_param("safety_distance_unknown", default=0.08)

        self.clean_table = False
        self.params = { 'use_map': self.use_map,
                        'grasp_check': self.grasp_check,
                        'clean_table': self.clean_table,
                        'grasp_height_unknown': self.grasp_height_unknown,
                        'safety_distance_unknown': self.safety_distance_unknown,
                        'grasp_height_known': self.grasp_height_known,
                        'safety_distance_known': self.safety_distance_known}

    def execute(self, userdata):
        rospy.loginfo('Executing state UserInput')
        objects_to_find = objects_keys.values()
        objects_to_find.extend(['teddy bear', 'banana', 'bottle', 'sports ball', 'dog'])
        userdata.objects_to_find = objects_to_find
        self.print_help()
        while not rospy.is_shutdown():
            #write config to userdata
            self.params['clean_table'] = False
            userdata.params = self.params
            userdata.find_grasppoint_tries = 0

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
                print('\t1 - unknown object')
                print('\t2 - known object')
                print('\t3 - choose known object')

                while True:
                    user_input = raw_input('CMD> ')
                    if len(user_input) == 1:
                        break
                    print('Please enter only one character')
                char_in = user_input.lower()
                if char_in == '1':
                    userdata.find_grasppoint_method = 1
                    userdata.grasp_height = self.grasp_height_unknown
                    userdata.safety_distance = self.safety_distance_unknown
                    if self.use_map:
                        return 'go_to_table'
                    return 'grasping'
                elif char_in == '2':
                    userdata.find_grasppoint_method = 2
                    userdata.grasp_height = self.grasp_height_known
                    userdata.safety_distance = self.safety_distance_known
                    if self.use_map:
                        return 'go_to_table'
                    return 'grasping'
                elif char_in == '3':
                    userdata.find_grasppoint_method = 2
                    userdata.grasp_height = self.grasp_height_known
                    userdata.safety_distance = self.safety_distance_known
                    self.print_objects()
                    while True:
                        user_input = raw_input('CMD> ')
                        if user_input.isdigit():
                            if int(user_input) < len(objects_keys)+1: 
                                print('chosen number is {}'.format(user_input))
                                userdata.objects_to_find = [objects_keys[int(user_input)]]
                                if self.use_map:
                                    return 'go_to_table'                    
                                return 'grasping'
                        if user_input == 'q':
                            self.print_help()
                            break
                        print('Please enter a valid number')

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
                print('\t1 - unknown object')
                print('\t2 - known object')
                print('\t3 - choose known object')
                print('\t4 - known object with HAF')

                while True:
                    user_input = raw_input('CMD> ')
                    if len(user_input) == 1:
                        break
                    print('Please enter only one character')
                char_in = user_input.lower()
                if char_in == '1':
                    userdata.find_grasppoint_method = 1
                    userdata.grasp_height = self.grasp_height_unknown
                    userdata.safety_distance = self.safety_distance_unknown
                    return 'find_grasp'
                elif char_in == '2':
                    userdata.find_grasppoint_method = 2
                    userdata.grasp_height = self.grasp_height_known
                    userdata.safety_distance = self.safety_distance_known
                    return 'find_grasp'
                elif char_in == '3':
                    userdata.find_grasppoint_method = 2
                    userdata.grasp_height = self.grasp_height_known
                    userdata.safety_distance = self.safety_distance_known
                    self.print_objects()
                    while True:
                        user_input = raw_input('CMD> ')
                        if user_input.isdigit():
                            if int(user_input) < len(objects_keys)+1: 
                                userdata.objects_to_find = [objects_keys[int(user_input)]]
                                return 'find_grasp'
                        if user_input == 'q':
                            self.print_help()
                            break
                        print('Please enter a valid number')
                elif char_in == '4':
                    userdata.find_grasppoint_method = 3
                    userdata.grasp_height = self.grasp_height_unknown
                    userdata.safety_distance = self.safety_distance_unknown
                    return 'find_grasp'

                else:
                    userdata.find_grasppoint_method = 0
                    print ('Not a valid method')
                    self.print_help()

            #execute last found grasp
            elif char_in == states_keys[States.EXECUTE_GRASP]:
                rospy.loginfo('Execute last found grasp')
                try:
                    return 'execute_grasp'
                except:
                    print ('No grasp pose found yet. Have you executed "FIND_GRASP" before?')
                    self.print_help()

            #print userdata
            elif char_in == states_keys[States.PRINT_USERDATA]:
                for input_key in self._input_keys:
                    if input_key in userdata.keys():
                        print('userdata.{}: {}'.format(input_key, userdata[input_key]))
                        pose_goal = userdata.found_grasp_pose
                        q = [pose_goal.pose.orientation.x, pose_goal.pose.orientation.y, pose_goal.pose.orientation.z, pose_goal.pose.orientation.w]
                        br = tf.TransformBroadcaster()
                        br.sendTransform((pose_goal.pose.position.x, pose_goal.pose.position.y, pose_goal.pose.position.z),
                                q,
                                rospy.Time.now(),
                                'grasp_pose',
                                pose_goal.header.frame_id)                
                print(self.params)
                self.print_help()
            
            #set config
            elif char_in == states_keys[States.CONFIG]:
                print('Choose if the map should be used')
                print('\t1 - use map')
                print('\t2 - no map')
                print('\t3 - do grasp stability check')
                print('\t4 - no grasp stability check')
                while True:
                    user_input = raw_input('CMD> ')
                    if len(user_input) == 1:
                        break
                    print('Please enter only one character')
                char_in = user_input.lower()
                if char_in == '1':
                    self.params['use_map'] = True
                elif char_in == '2':
                    self.params['use_map'] = False
                elif char_in == '3':
                    self.params['grasp_check'] = True
                elif char_in == '4':
                    self.params['grasp_check'] = False
                else:
                    print('No valid option.')
                    self.print_help()

            # clean the table
            elif char_in == states_keys[States.CLEAN_TABLE]:
                rospy.loginfo('Clean the table')
                self.params['clean_table'] = True
                userdata.params = self.params

                userdata.find_grasppoint_method = 2
                userdata.grasp_height = self.grasp_height_known
                userdata.safety_distance = self.safety_distance_unknown
                if self.use_map:
                    return 'go_to_table'
                return 'grasping'

            # Unrecognized command
            else:
                rospy.logwarn('Unrecognized command %s', char_in)

    def print_help(self):
        for name, member in States.__members__.items():
            print(states_keys[member] + ' - ' + name)
    
    def print_objects(self):
        for i in range(len(objects_display_names)):
            print('{}'.format(i+1) + ' - ' + objects_display_names[i+1])


class GoToNeutral(smach.State):
    def __init__(self):
        smach.State.__init__(self,input_keys=['params'], outcomes=['succeeded', 'continuing'])
        #Robot initialization
        self.robot = Robot()
        self.base = self.robot.try_get('omni_base')
        self.tts = self.robot.try_get('default_tts')
        self.whole_body = self.robot.try_get('whole_body')

    def execute(self, userdata):
        rospy.loginfo('Executing state GoToNeutral')
        self.whole_body.move_to_neutral()
        self.whole_body.move_to_joint_positions({'arm_roll_joint':pi/2})
        self.whole_body.gaze_point((0.7, 0.05, 0.4))
        if userdata.params.get('clean_table'):
            return 'continuing'
        return 'succeeded'


class GoBackAndNeutral(smach.State):
    def __init__(self):
        smach.State.__init__(self,input_keys=['params'], 
                            output_keys=['find_grasppoint_tries', 'find_grasppoint_method', 'grasp_height', 'safety_distance'], 
                            outcomes=['succeeded'])
        #Robot initialization
        self.robot = Robot()
        self.base = self.robot.try_get('omni_base')
        self.tts = self.robot.try_get('default_tts')
        self.whole_body = self.robot.try_get('whole_body')
        self.move_client = actionlib.SimpleActionClient('/move_base/move', MoveBaseAction)

    
    def execute(self, userdata):
        rospy.loginfo('Executing state GoToNeutral')
        userdata.find_grasppoint_tries = 0
        userdata.find_grasppoint_method = 2
        userdata.grasp_height = userdata.params.get('grasp_height_known')
        userdata.safety_distance = userdata.params.get('safety_distance_known')
        if userdata.params.get('use_map'):
            move_goal = MoveBaseGoal()
            move_goal.target_pose.header.frame_id='map'
            move_goal.target_pose.pose.position.x = 1.15
            move_goal.target_pose.pose.position.y = -0.7
            move_goal.target_pose.pose.orientation.z = 0.707
            move_goal.target_pose.pose.orientation.w = 0.707
            self.move_client.wait_for_server()
            self.move_client.send_goal(move_goal)
            result = self.move_client.wait_for_result()
        else:
            self.base.go_rel(-0.1,0,0)
        self.whole_body.move_to_neutral()
        if userdata.params.get('grasp_check'):
            self.whole_body.move_to_joint_positions({'wrist_roll_joint':0.7854})
            self.whole_body.move_to_joint_positions({'wrist_roll_joint':-0.7854})
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


class GoToTable(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'])
        self.move_client = actionlib.SimpleActionClient('/move_base/move', MoveBaseAction)
        self.robot = Robot()
        self.whole_body = self.robot.try_get('whole_body')

    def execute(self, userdata):
        move_goal = MoveBaseGoal()
        move_goal.target_pose.header.frame_id='map'
        move_goal.target_pose.pose.position.x = 1.25
        move_goal.target_pose.pose.position.y = -0.75
        move_goal.target_pose.pose.orientation.z = 1.0
        self.move_client.wait_for_server()
        self.move_client.send_goal(move_goal)
        result = self.move_client.wait_for_result()

        if result:
            #go to neutral
            self.whole_body.move_to_joint_positions(neutral_joint_positions)
            rospy.sleep(1.0)
            
            vel = self.whole_body.joint_velocities
            while all(abs(i)>0.05 for i in vel.values()):
                vel = self.whole_body.joint_velocities
            rospy.sleep(2)

            return 'succeeded'
        else: 
            return 'aborted'

class NoGrasppointFound(smach.State):
    def __init__(self):
        smach.State.__init__(self,output_keys=['find_grasppoint_tries', 'find_grasppoint_method', 'grasp_height', 'safety_distance'], 
                            input_keys=['params', 'find_grasppoint_tries'], 
                            outcomes=['find_grasp', 'user_input'])
    
    def execute(self, userdata):
        rospy.sleep(0.1)
        userdata.find_grasppoint_tries = userdata.find_grasppoint_tries + 1

        if userdata.find_grasppoint_tries > 4:
            return 'user_input'
        if userdata.find_grasppoint_tries > 3:
            userdata.find_grasppoint_method = 1
            userdata.grasp_height = userdata.params.get('grasp_height_unknown')
            userdata.safety_distance = userdata.params.get('safety_distance_unknown')
        if userdata.params.get('clean_table'):
            return 'find_grasp'
        else:
            return 'user_input'


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
                                            'execute_grasp':'EXECUTE_GRASP',
                                            'go_to_table':'GO_TO_TABLE'})
        
        smach.StateMachine.add('GO_TO_TABLE',
                                GoToTable(), \
                                transitions={'succeeded':'FIND_GRASPPOINT',
                                            'aborted':'USER_INPUT'})


        smach.StateMachine.add('GO_TO_NEUTRAL',
                                GoToNeutral(), \
                                transitions={'succeeded':'USER_INPUT',
                                            'continuing':'GO_TO_TABLE'})

        smach.StateMachine.add('OPENING',
                                Opening(), \
                                transitions={'succeeded':'USER_INPUT'})

        smach.StateMachine.add('FIND_GRASPPOINT', \
                                smach_ros.SimpleActionState('find_grasppoint', FindGrasppointAction,
                                                            goal_slots = ['method', 'object_names', 'grasp_height'],
                                                            result_slots = ['grasp_pose']),
                                transitions={'succeeded':'EXECUTE_GRASP', 
                                            'preempted':'USER_INPUT',
                                            'aborted':'NO_GRASPPOINT_FOUND'},
                                remapping={ 'method':'find_grasppoint_method', 
                                            'grasp_pose':'found_grasp_pose',
                                            'object_names':'objects_to_find',})

        smach.StateMachine.add('NO_GRASPPOINT_FOUND', 
                                NoGrasppointFound(),
                                transitions={'find_grasp':'FIND_GRASPPOINT',
                                            'user_input':'USER_INPUT'})

        smach.StateMachine.add('ONLY_FIND_GRASPPOINT', \
                                smach_ros.SimpleActionState('find_grasppoint', FindGrasppointAction,
                                                            goal_slots = ['method', 'object_names', 'grasp_height'],
                                                            result_slots = ['grasp_pose']),
                                transitions={'succeeded':'USER_INPUT', 
                                            'preempted':'USER_INPUT',
                                            'aborted':'USER_INPUT'},
                                remapping={ 'method':'find_grasppoint_method', 
                                            'grasp_pose':'found_grasp_pose',
                                            'object_names':'objects_to_find'})

        smach.StateMachine.add('EXECUTE_GRASP',
                                smach_ros.SimpleActionState('execute_grasp', ExecuteGraspAction,
                                                            goal_slots=['grasp_pose', 'grasp_height', 'safety_distance']),
                                transitions={'succeeded':'NEUTRAL_BEFORE_HANDOVER', 
                                            'preempted':'USER_INPUT',
                                            'aborted':'GO_TO_NEUTRAL'},
                                remapping={'grasp_pose':'found_grasp_pose'})

        smach.StateMachine.add('NEUTRAL_BEFORE_HANDOVER',
                                GoBackAndNeutral(), 
                                transitions={'succeeded':'HANDOVER'})

        smach.StateMachine.add('HANDOVER', smach_ros.SimpleActionState('/handover', HandoverAction),
                                transitions={'succeeded':'GO_TO_NEUTRAL', 
                                            'preempted':'USER_INPUT',
                                            'aborted':'USER_INPUT'})




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

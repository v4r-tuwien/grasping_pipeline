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
               States.CONFIG: 'c'}

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
                11: '035_power_drill',
                12: '061_foam_brick'}

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
                        11: 'Power Drill',
                        12: 'Foam Brick'}


class UserInput(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['quitting', 'neutral', 'grasping', 'opening', 
                                            'find_grasp', 'execute_grasp', 'go_to_table'],
                                    input_keys=['found_grasp_pose', 'grasp_height', 'approach_vector_x', 
                                                'approach_vector_y', 'approach_vector_z'],
                                    output_keys=['find_grasppoint_method', 'objects_to_find',
                                    'grasp_height', 'safety_distance', 'use_map', 'grasp_check'])
        #config initial settings
        self.use_map = True
        self.grasp_check = True

    def execute(self, userdata):
        rospy.loginfo('Executing state UserInput')
        userdata.objects_to_find = ['apple', 'sports ball', 'orange', 'bottle', 
                                    '005_tomato_soup_can', '006_mustard_bottle']
        self.print_help()
        while not rospy.is_shutdown():
            #write config to userdata
            userdata.use_map = self.use_map
            userdata.grasp_check = self.grasp_check

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
                print('\t2 - verefine pose estimation')
                print('\t3 - choose object verefine')

                while True:
                    user_input = raw_input('CMD> ')
                    if len(user_input) == 1:
                        break
                    print('Please enter only one character')
                char_in = user_input.lower()
                if char_in == '1':
                    userdata.find_grasppoint_method = 1
                    userdata.grasp_height = 0.05
                    userdata.safety_distance = 0.14
                    if self.use_map:
                        return 'go_to_table'
                    return 'grasping'
                elif char_in == '2':
                    userdata.find_grasppoint_method = 2
                    userdata.grasp_height = 0.0
                    userdata.safety_distance = 0.05
                    if self.use_map:
                        return 'go_to_table'
                    return 'grasping'
                elif char_in == '3':
                    userdata.find_grasppoint_method = 2
                    userdata.grasp_height = 0.0
                    userdata.safety_distance = 0.05
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
                    print (userdata.approach_vector_x)
                    print (userdata.approach_vector_y)
                    print (userdata.approach_vector_z)

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
                print('\t2 - verefine pose estimation')
                print('\t3 - choose object verefine')

                while True:
                    user_input = raw_input('CMD> ')
                    if len(user_input) == 1:
                        break
                    print('Please enter only one character')
                char_in = user_input.lower()
                if char_in == '1':
                    userdata.find_grasppoint_method = 1
                    userdata.grasp_height = 0.05
                    userdata.safety_distance = 0.14
                    return 'find_grasp'
                elif char_in == '2':
                    userdata.find_grasppoint_method = 2
                    userdata.grasp_height = 0.0
                    userdata.safety_distance = 0.05
                    return 'find_grasp'
                elif char_in == '3':
                    userdata.find_grasppoint_method = 2
                    userdata.grasp_height = 0.0
                    userdata.safety_distance = 0.05
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
                        #q = tf.transformations.quaternion_about_axis(pi, (1,0,0))
                        #q = tf.transformations.quaternion_multiply(q, q2)
                        br = tf.TransformBroadcaster()
                        br.sendTransform((pose_goal.pose.position.x, pose_goal.pose.position.y, pose_goal.pose.position.z),
                                q,
                                rospy.Time.now(),
                                'grasp_pose',
                                pose_goal.header.frame_id)                
                print('use_map: {}'.format(self.use_map))
                print('grasp_check: {}'.format(self.grasp_check))
                self.print_help()
            
            #set config
            elif char_in == states_keys[States.CONFIG]:
                print('Choose if the map should be used')
                print('\t1 - use map')
                print('\t2 - no map')
                print('\t3 - do grasp stability check')
                print('\t4 - grasp stability check')
                while True:
                    user_input = raw_input('CMD> ')
                    if len(user_input) == 1:
                        break
                    print('Please enter only one character')
                char_in = user_input.lower()
                if char_in == '1':
                    self.use_map = True
                elif char_in == '2':
                    self.use_map = False
                elif char_in == '3':
                    self.grasp_check = True
                elif char_in == '4':
                    self.grasp_check = False
                else:
                    print('No valid option.')
                    self.print_help()

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
        smach.State.__init__(self, outcomes=['succeeded'])
        #Robot initialization
        self.robot = Robot()
        self.base = self.robot.try_get('omni_base')
        self.tts = self.robot.try_get('default_tts')
        self.whole_body = self.robot.try_get('whole_body')

    def execute(self, userdata):
        rospy.loginfo('Executing state GoToNeutral')
        self.whole_body.move_to_neutral()
        self.whole_body.move_to_joint_positions({'arm_roll_joint':pi/2})
        self.whole_body.gaze_point((0.7, 0.1, 0.4))
        return 'succeeded'


class GoBackAndNeutral(smach.State):
    def __init__(self):
        smach.State.__init__(self,input_keys=['use_map', 'grasp_check'], outcomes=['succeeded'])
        #Robot initialization
        self.robot = Robot()
        self.base = self.robot.try_get('omni_base')
        self.tts = self.robot.try_get('default_tts')
        self.whole_body = self.robot.try_get('whole_body')
        self.move_client = actionlib.SimpleActionClient('/move_base/move', MoveBaseAction)

    
    def execute(self, userdata):
        rospy.loginfo('Executing state GoToNeutral')
        if userdata.use_map:
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
        if userdata.grasp_check:
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
            #self.whole_body.move_to_neutral()
            #self.whole_body.move_to_joint_positions({'arm_roll_joint':pi/2})
            #self.whole_body.gaze_point((0.7, 0.1, 0.4))
            rospy.sleep(1.0)
            
            vel = self.whole_body.joint_velocities
            while all(abs(i)>0.05 for i in vel.values()):
                vel = self.whole_body.joint_velocities
            rospy.sleep(1.0)

            return 'succeeded'
        else: 
            return 'aborted'


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
                                transitions={'succeeded':'USER_INPUT'})

        smach.StateMachine.add('OPENING',
                                Opening(), \
                                transitions={'succeeded':'USER_INPUT'})

        smach.StateMachine.add('FIND_GRASPPOINT', \
                                smach_ros.SimpleActionState('find_grasppoint', FindGrasppointAction,
                                                            goal_slots = ['method', 'object_names'],
                                                            result_slots = ['grasp_pose', 'approach_vector_x', 'approach_vector_y', 'approach_vector_z']),
                                transitions={'succeeded':'EXECUTE_GRASP', 
                                            'preempted':'USER_INPUT',
                                            'aborted':'USER_INPUT'},
                                remapping={ 'method':'find_grasppoint_method', 
                                            'grasp_pose':'found_grasp_pose',
                                            'approach_vector_x' : 'approach_vector_x',
                                            'approach_vector_y' : 'approach_vector_y',
                                            'approach_vector_z' : 'approach_vector_z',
                                            'grasp_height':'grasp_height',
                                            'safety_height':'safety_height',
                                            'object_names':'objects_to_find',
                                            'use_map':'use_map',
                                            'grasp_check':'grasp_check'})

        smach.StateMachine.add('ONLY_FIND_GRASPPOINT', \
                                smach_ros.SimpleActionState('find_grasppoint', FindGrasppointAction,
                                                            goal_slots = ['method', 'object_names'],
                                                            result_slots = ['grasp_pose', 'approach_vector_x', 'approach_vector_y', 'approach_vector_z']),
                                transitions={'succeeded':'USER_INPUT', 
                                            'preempted':'USER_INPUT',
                                            'aborted':'USER_INPUT'},
                                remapping={ 'method':'find_grasppoint_method', 
                                            'grasp_pose':'found_grasp_pose',
                                            'approach_vector_x' : 'approach_vector_x',
                                            'approach_vector_y' : 'approach_vector_y',
                                            'approach_vector_z' : 'approach_vector_z',
                                            'object_names':'objects_to_find'})

        smach.StateMachine.add('EXECUTE_GRASP',
                                smach_ros.SimpleActionState('execute_grasp', ExecuteGraspAction,
                                                            goal_slots=['grasp_pose', 'grasp_height', 'safety_distance', 'approach_vector_x', 'approach_vector_y', 'approach_vector_z']),
                                transitions={'succeeded':'NEUTRAL_BEFORE_HANDOVER', 
                                            'preempted':'USER_INPUT',
                                            'aborted':'USER_INPUT'},
                                remapping={'grasp_pose':'found_grasp_pose',
                                            'grasp_height':'grasp_height',
                                            'approach_vector_x' : 'approach_vector_x',
                                            'approach_vector_y' : 'approach_vector_y',
                                            'approach_vector_z' : 'approach_vector_z',
                                            'safety_distance':'safety_distance',
                                            'use_map' : 'use_map',
                                            'grasp_check':'grasp_check'})

        smach.StateMachine.add('NEUTRAL_BEFORE_HANDOVER',
                                GoBackAndNeutral(), 
                                transitions={'succeeded':'HANDOVER'},
                                remapping={ 'use_map' : 'use_map',
                                            'grasp_check':'grasp_check'})

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

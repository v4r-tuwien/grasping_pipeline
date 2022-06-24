#!/usr/bin/env python3
# Copyright (C) 2016 Toyota Motor Corporation
from enum import Enum, IntEnum
from math import pi
from tkinter import TOP

import actionlib
import rospy
import smach
import smach_ros
import tf
from actionlib_msgs.msg import GoalStatus
from grasping_pipeline.msg import (ExecuteGraspAction, ExecuteGraspGoal,
                                   FindGrasppointAction, FindGrasppointGoal)
from handover.msg import HandoverAction
from hsrb_interface import Robot, geometry, collision_world
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from helper_function import *
from sensor_msgs.msg import PointCloud2
from placement.msg import *
from lib_cloud_conversion_between_Open3D_and_ROS import convertCloudFromOpen3dToRos, convertCloudFromRosToOpen3d
from tmc_placement_area_detector.srv import *
from tmc_geometric_shapes_msgs.msg import Shape
from tmc_geometric_shapes_msgs.srv import CacheMeshData, GetMeshData
from geometry_msgs.msg import Point, Vector3, Pose
import open3d

# neutral joint positions
neutral_joint_positions = {'arm_flex_joint': 0.0,
                           'arm_lift_joint': 0.0,
                           'arm_roll_joint': 1.570,
                           'hand_motor_joint': 1.0,
                           'head_pan_joint': 0.0,
                           'head_tilt_joint': -0.75,
                           'wrist_flex_joint': -1.57,
                           'wrist_roll_joint': 0.0}

neutral_joint_positions_4 = {'arm_flex_joint': 0.0,
                           'arm_lift_joint': 0.0,
                           'arm_roll_joint': 1.570,
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
    MOVE_TO_TABLE = 12
    DETECT_PLACEMENT_AREA = 13


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
               States.CLEAN_TABLE: 't',
               States.MOVE_TO_TABLE: 'm',
               States.DETECT_PLACEMENT_AREA: 'd'}

class PlaneMethod(IntEnum):
    RANSAC = 0

class CollisionMethod(IntEnum):
    ADD = 0
    REMOVE = 1
    ATTACH = 2
    DETACH = 3

class GraspMode(IntEnum):
    NORMAL = 0
    INVERT = 1
    TOP = 2

class SortMethod(IntEnum):
    MIDDLE = 0
    EDGE = 1
    NEAR = 2
    FAR = 3


class UserInput(smach.State):
    """ Initial smach state for the statemachine
    Serves as simple user interface in the terminal
    Captures input from the terimal and sets userdata
    Sets outcomes depending on the input

    Userdata:
        Input keys:
            found_grasp_poses {geometry_msgs/PoseStamped[]}:
                    result from find_grasppoint action

        Output keys:
            find_grasppoint_method {int}:
                    method identifier, 1-4
                    1 - detectron and HAF grasping
                    2 - verefine pipeline
                    3 - verefine pipeline and HAF
                    4 - pyrapose pipeline

            objects_to_find {list of str}: object names that are considered
                                           in find_grasppoint action

            params {dic}: additional parameters
                use_map: When True, robot moves to fixed points
                         on the map, otherwise only relative movement
                grasp_check: When True, robot twists the hand
                             after grasp to ensure a stable grasp
                clean_table: When True, robot is in clear table routine,
                             grasps and handovers are performed until
                             the table is empty

            find_grasppoint_tries {int}: how often find_grasppoint was
                                         called in a grasp run, relevant
                                         for table clearing routine

    Outcomes:
        quitting: input "q", the statemachine will transition to
                  it's final outcome and end the program.

        neutral: input "n", transition to GO_TO_NEUTRAL State,
                 robot will assume a fixed position staring down

        grasping: input "g", transition to FIND_GRASPPOINT State,
                  robot will search a grasppoint and execute it
                  expects a second input to choose method
                  This outcome is only possible when the map
                  is not used (self.use_map == False)
                  or input "t", Clean table routine that will not
                  go back to this state after an succesful attempt
                  and rather grasp objects until the table is empty

        opening: input "o", transition to OPENING State,
                 will open the gripper

        find_grasp: input "f", transition to ONLY_FIND_GRASPPOINT State,
                    no grasp execution after finding a grasppoint
                    expects a second input to choose method

        execute_grasp: input "e", transition to EXECUTE_GRASP State,
                       only possible if ONLY_FIND_GRASPPOINT State was
                       executed before

        go_to_table: input "g" or "t", transition to GO_TO_TABLE State,
                     robot will drive to the table before searching a
                     grasppoint and executing it
    """

    def __init__(self):
        smach.State.__init__(self, outcomes=['quitting', 'neutral', 'grasping', 'opening',
                                             'find_grasp', 'execute_grasp', 'go_to_table', 'detect_placement_area'],
                             input_keys=['found_grasp_poses'],
                             output_keys=['find_grasppoint_method', 'objects_to_find',
                                          'params', 'find_grasppoint_tries'])
        # config initial settings
        self.use_map = rospy.get_param("/use_map")
        self.grasp_check = rospy.get_param("/grasp_check")
        self.clean_table = False
        self.params = {'use_map': self.use_map,
                       'grasp_check': self.grasp_check,
                       'clean_table': self.clean_table}
        self.objects_keys = rospy.get_param("/objects_keys")
        self.objects_display_names = rospy.get_param("/objects_display_names")

    def execute(self, userdata):
        rospy.loginfo('Executing state UserInput')

        objects_to_find = list(self.objects_keys.values())
        objects_to_find.extend(
            ['teddy bear', 'banana', 'bottle', 'sports ball', 'dog', '000002'])
        userdata.objects_to_find = objects_to_find
        self.print_help()
        while not rospy.is_shutdown():
            # write config to userdata
            self.params['clean_table'] = False
            userdata.params = self.params
            userdata.find_grasppoint_tries = 0

            while True:
                user_input = input('CMD> ')
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
                print('\t4 - known object with HAF')
                print('\t5 - PyraPose known objects')
                print('\t6 - PyraPose choose object')

                while True:
                    user_input = input('CMD> ')
                    if len(user_input) == 1:
                        break
                    print('Please enter only one character')
                char_in = user_input.lower()
                if char_in == '1':
                    userdata.find_grasppoint_method = 1
                    if self.use_map:
                        return 'go_to_table'
                    return 'grasping'
                elif char_in == '2':
                    userdata.find_grasppoint_method = 2
                    if self.use_map:
                        return 'go_to_table'
                    return 'grasping'
                elif char_in == '3':
                    userdata.find_grasppoint_method = 2
                    self.print_objects()
                    while True:
                        user_input = input('CMD> ')
                        if user_input.isdigit():
                            if int(user_input) < len(self.objects_keys) + 1:
                                print('chosen number is {}'.format(user_input))
                                userdata.objects_to_find = [
                                    self.objects_keys[user_input]]
                                if self.use_map:
                                    return 'go_to_table'
                                return 'grasping'
                        if user_input == 'q':
                            self.print_help()
                            break
                        print('Please enter a valid number')
                elif char_in == '4':
                    userdata.find_grasppoint_method = 3
                    if self.use_map:
                        return 'go_to_table'
                    return 'grasping'
                elif char_in == '5':
                    userdata.find_grasppoint_method = 4
                    if self.use_map:
                        return 'go_to_table'
                    return 'grasping'
                elif char_in == '6':
                    userdata.find_grasppoint_method = 4
                    self.print_objects()
                    while True:
                        user_input = input('CMD> ')
                        if user_input.isdigit():
                            if int(user_input) < len(self.objects_keys) + 1:
                                print('chosen number is {}'.format(user_input))
                                userdata.objects_to_find = [
                                    self.objects_keys[user_input]]
                                if self.use_map:
                                    return 'go_to_table'
                                return 'grasping'
                        if user_input == 'q':
                            self.print_help()
                            break
                        print('Please enter a valid number')
                else:
                    userdata.find_grasppoint_method = 0
                    print('Not a valid method')
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
                    print(userdata.found_grasp_poses)

                except BaseException:
                    print(
                        'No grasp pose found yet. Have you executed "FIND_GRASP" before?')
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
                print('\t5 - PyraPose known objects')
                print('\t6 - PyraPose choose object')

                while True:
                    user_input = input('CMD> ')
                    if len(user_input) == 1:
                        break
                    print('Please enter only one character')
                char_in = user_input.lower()
                if char_in == '1':
                    userdata.find_grasppoint_method = 1
                    return 'find_grasp'
                elif char_in == '2':
                    userdata.find_grasppoint_method = 2
                    return 'find_grasp'
                elif char_in == '3':
                    userdata.find_grasppoint_method = 2
                    self.print_objects()
                    while True:
                        user_input = input('CMD> ')
                        if user_input.isdigit():
                            if int(user_input) < len(self.objects_keys) + 1:
                                userdata.objects_to_find = [
                                    self.objects_keys[user_input]]
                                return 'find_grasp'
                        if user_input == 'q':
                            self.print_help()
                            break
                        print('Please enter a valid number')
                elif char_in == '4':
                    userdata.find_grasppoint_method = 3
                    return 'find_grasp'
                elif char_in == '5':
                    userdata.find_grasppoint_method = 4
                    return 'find_grasp'
                elif char_in == '6':
                    userdata.find_grasppoint_method = 4
                    self.print_objects()
                    while True:
                        user_input = input('CMD> ')
                        if user_input.isdigit():
                            if int(user_input) < len(self.objects_keys) + 1:
                                userdata.objects_to_find = [
                                    self.objects_keys[user_input]]
                                return 'find_grasp'
                        if user_input == 'q':
                            self.print_help()
                            break
                        print('Please enter a valid number')
                else:
                    userdata.find_grasppoint_method = 0
                    print('Not a valid method')
                    self.print_help()

            # execute last found grasp
            elif char_in == states_keys[States.EXECUTE_GRASP]:
                rospy.loginfo('Execute last found grasp')
                try:
                    return 'execute_grasp'
                except BaseException:
                    print(
                        'No grasp pose found yet. Have you executed "FIND_GRASP" before?')
                    self.print_help()

            # print userdata
            elif char_in == states_keys[States.PRINT_USERDATA]:
                for input_key in self._input_keys:
                    if input_key in userdata.keys():
                        print('userdata.{}: {}'.format(
                            input_key, userdata[input_key]))
                        pose_goal = userdata.found_grasp_poses
                        q = [pose_goal.pose.orientation.x, pose_goal.pose.orientation.y,
                             pose_goal.pose.orientation.z, pose_goal.pose.orientation.w]
                        br = tf.TransformBroadcaster()
                        br.sendTransform((pose_goal.pose.position.x,
                                          pose_goal.pose.position.y,
                                          pose_goal.pose.position.z),
                                         q,
                                         rospy.Time.now(),
                                         'grasp_poses',
                                         pose_goal.header.frame_id)
                print(self.params)
                self.print_help()

            # set config
            elif char_in == states_keys[States.CONFIG]:
                print('Choose if the map should be used')
                print('\t1 - use map')
                print('\t2 - no map')
                print('\t3 - do grasp stability check')
                print('\t4 - no grasp stability check')
                while True:
                    user_input = input('CMD> ')
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
                if self.use_map:
                    return 'go_to_table'
                return 'grasping'


            elif char_in == states_keys[States.MOVE_TO_TABLE]:
                rospy.loginfo('Move to table')
                return 'go_to_table'

            elif char_in == states_keys[States.DETECT_PLACEMENT_AREA]:
                rospy.loginfo('Detect Placement Area')
                return 'detect_placement_area'

            # Unrecognized command
            else:
                rospy.logwarn('Unrecognized command %s', char_in)

    def print_help(self):
        """ prints states_keys and their names
        e.g.:
        states_keys = {States.EXAMPLE: 'a',
                    States.ANOTHER_ONE: 'b'}
        output:
               a - EXAMPLE
               b - ANOTHER_ONE
                ...
        """
        for name, member in States.__members__.items():
            print(states_keys[member] + ' - ' + name)

    def print_objects(self):
        """ prints object names and their identifiers
        defined in config/standard.yaml
        """
        for i in range(len(self.objects_display_names)):
            print('{} - {}'.format(i + 1,
                                   self.objects_display_names[str(i + 1)]))


class GoToNeutral(smach.State):
    """ Smach state that will move the robots joints to a
    predefined position, gazing at a fixed point.

    Userdata:
        Input Keys:
            params {dic}: additional parameters
                use_map: When True, robot moves to fixed points
                         on the map, otherwise only relative movement
                grasp_check: When True, robot twists the hand
                             after grasp to ensure a stable grasp
                clean_table: When True, robot is in clear table routine,
                             grasps and handovers are performed until
                             the table is empty

    Outcomes:
        succeeded: transition to USER_INPUT State
        continuing: in the clear table routine,
                    transition to GO_TO_TABLE State
    """

    def __init__(self):
        smach.State.__init__(self, input_keys=['params'], outcomes=[
                             'succeeded', 'continuing'])
        # Robot initialization
        self.robot = Robot()
        self.base = self.robot.try_get('omni_base')
        self.tts = self.robot.try_get('default_tts')
        self.whole_body = self.robot.try_get('whole_body')

    def execute(self, userdata):
        rospy.loginfo('Executing state GoToNeutral')
        self.whole_body.move_to_neutral()
        self.whole_body.move_to_joint_positions({'arm_roll_joint': pi / 2})
        self.whole_body.gaze_point((0.8, 0.05, 0.4))

        if userdata.params.get('clean_table'):
            return 'continuing'
        return 'succeeded'


class GoBackAndNeutral(smach.State):
    """ Smach state that will move the robot either backwards (use_map is False)
    or to a predefined position on the map (use_map is True)
    Also moves the joints to the neutral position
    Grasp check (turning the wrist) will be done here if enabled

    Userdata:
        Input Keys:
            params {dic}: additional parameters
                use_map: When True, robot moves to fixed points
                         on the map, otherwise only relative movement
                grasp_check: When True, robot twists the hand
                             after grasp to ensure a stable grasp
                clean_table: When True, robot is in clear table routine,
                             grasps and handovers are performed until
                             the table is empty
        Output keys:
            find_grasppoint_method {int}:
                    method identifier, 1-4
                    1 - detectron and HAF grasping
                    2 - verefine pipeline
                    3 - verefine pipeline and HAF
                    4 - pyrapose pipeline

            find_grasppoint_tries {int}: how often find_grasppoint was
                                         called in a grasp run, relevant
                                         for table clearing routine

    Outcomes:
        succeeded: transition to USER_INPUT State
        continuing: in the clear table routine,
                    transition to GO_TO_TABLE State
    """

    def __init__(self):
        smach.State.__init__(self, input_keys=['params'],
                             output_keys=['find_grasppoint_tries',
                                          'find_grasppoint_method'],
                             outcomes=['succeeded'])
        # Robot initialization
        self.robot = Robot()
        self.base = self.robot.try_get('omni_base')
        self.tts = self.robot.try_get('default_tts')
        self.whole_body = self.robot.try_get('whole_body')
        self.move_client = actionlib.SimpleActionClient(
            '/move_base/move', MoveBaseAction)

    def execute(self, userdata):
        rospy.loginfo('Executing state GoToNeutral')
        userdata.find_grasppoint_tries = 0
        # TODO check if and why that is necessary?
        userdata.find_grasppoint_method = 2
        if userdata.params.get('use_map'):
            move_goal = MoveBaseGoal()
            move_goal.target_pose.header.frame_id = 'map'
            move_goal.target_pose.pose.position.x = 0.4
            move_goal.target_pose.pose.position.y = 0.0
            move_goal.target_pose.pose.orientation.z = 0.707
            move_goal.target_pose.pose.orientation.w = 0.707
            self.move_client.wait_for_server()
            self.move_client.send_goal(move_goal)
            result = self.move_client.wait_for_result()
        else:
            self.base.go_rel(-0.1, 0, 0)
        self.whole_body.move_to_neutral()
        if userdata.params.get('grasp_check'):
            self.whole_body.move_to_joint_positions(
                {'wrist_roll_joint': 0.7854})
            self.whole_body.move_to_joint_positions(
                {'wrist_roll_joint': -0.7854})
        return 'succeeded'


class Opening(smach.State):
    """ Opens the robots gripper

    Outcomes:
        succeeded: transitions to USER_INPUT State
    """

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])

        # Robot initialization
        self.robot = Robot()
        self.gripper = self.robot.try_get('gripper')

    def execute(self, userdata):
        rospy.loginfo('Executing state Opening')
        self.gripper.command(1.0)
        return 'succeeded'


class GoToWaypoint1(smach.State):
    """ robot moves to a fixed position near the table
    using a move_base action goal
    also moves joints to neutral position

    Outcomes:
        succeeded: transitions to FIND_GRASPPOINT State
        aborted: transitions to USER_INPUT State
    """

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'])
        self.move_client = actionlib.SimpleActionClient(
            '/move_base/move', MoveBaseAction)
        self.robot = Robot()
        self.whole_body = self.robot.try_get('whole_body')

    def execute(self, userdata):
        move_goal = MoveBaseGoal()
        move_goal.target_pose.header.frame_id = 'map'
        move_goal.target_pose.pose.position.x = 0.778
        move_goal.target_pose.pose.position.y = -0.011
        move_goal.target_pose.pose.orientation.z = 0.7157
        move_goal.target_pose.pose.orientation.w = 0.698
        self.move_client.wait_for_server()
        self.move_client.send_goal(move_goal)
        result = self.move_client.wait_for_result()

        if result:
            # go to neutral
            # TODO use this command in other neutral position moves too
            self.whole_body.move_to_joint_positions(neutral_joint_positions)
            rospy.sleep(1.0)

            vel = self.whole_body.joint_velocities
            while all(abs(i) > 0.05 for i in vel.values()):
                vel = self.whole_body.joint_velocities
            rospy.sleep(2)

            return 'succeeded'
        else:
            return 'aborted'

class GoToWaypoint2(smach.State):
    """ robot moves to a fixed position near the table
    using a move_base action goal
    also moves joints to neutral position

    Outcomes:
        succeeded: transitions to FIND_GRASPPOINT State
        aborted: transitions to USER_INPUT State
    """

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'])
        self.move_client = actionlib.SimpleActionClient(
            '/move_base/move', MoveBaseAction)
        self.robot = Robot()
        self.whole_body = self.robot.try_get('whole_body')
        self.robot = Robot()
        self.gripper = self.robot.try_get('gripper')
           	    	
    def execute(self, userdata):
        move_goal = MoveBaseGoal()
        move_goal.target_pose.header.frame_id = 'map'
        move_goal.target_pose.pose.position.x = -0.241
        move_goal.target_pose.pose.position.y = 2.046
        move_goal.target_pose.pose.orientation.z = 0.7
        move_goal.target_pose.pose.orientation.w = 0.714
        self.move_client.wait_for_server()
        self.move_client.send_goal(move_goal)
        result = self.move_client.wait_for_result()
        
        if result:
            # go to neutral
            # TODO use this command in other neutral position moves too
            neutral_joint_positions = {'arm_flex_joint': 0.0,
                               'arm_lift_joint': 0.1,
                               'arm_roll_joint': 1.570,
                               'wrist_flex_joint': -1.57,
                               'wrist_roll_joint': 0.0}
            self.whole_body.move_to_joint_positions(neutral_joint_positions)
            rospy.sleep(1.0)
            self.whole_body.gaze_point((0.7, 0.1, 0.76))

            vel = self.whole_body.joint_velocities
            while all(abs(i) > 0.05 for i in vel.values()):
                vel = self.whole_body.joint_velocities
            rospy.sleep(2)
            return 'succeeded'
        else:
            return 'aborted'

class GoToWaypoint3(smach.State):
    """ robot moves to a fixed position near the table
    using a move_base action goal
    also moves joints to neutral position

    Outcomes:
        succeeded: transitions to FIND_GRASPPOINT State
        aborted: transitions to USER_INPUT State
    """

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'])
        self.move_client = actionlib.SimpleActionClient(
            '/move_base/move', MoveBaseAction)
        self.robot = Robot()
        self.whole_body = self.robot.try_get('whole_body')

    def execute(self, userdata):
        move_goal = MoveBaseGoal()
        move_goal.target_pose.header.frame_id = 'map'
        move_goal.target_pose.pose.position.x = -0.217
        move_goal.target_pose.pose.position.y = 0.37
        move_goal.target_pose.pose.orientation.z = 1
        move_goal.target_pose.pose.orientation.w = 0.05699
        self.move_client.wait_for_server()
        self.move_client.send_goal(move_goal)
        result = self.move_client.wait_for_result()

        if result:
            # go to neutral
            # TODO use this command in other neutral position moves too
            self.whole_body.move_to_joint_positions(neutral_joint_positions)
            rospy.sleep(1.0)

            vel = self.whole_body.joint_velocities
            while all(abs(i) > 0.05 for i in vel.values()):
                vel = self.whole_body.joint_velocities
            rospy.sleep(2)

            return 'succeeded'
        else:
            return 'aborted'

class GoToWaypoint4(smach.State):
    """ robot moves to a fixed position near the table
    using a move_base action goal
    also moves joints to neutral position

    Outcomes:
        succeeded: transitions to FIND_GRASPPOINT State
        aborted: transitions to USER_INPUT State
    """

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'])
        self.move_client = actionlib.SimpleActionClient(
            '/move_base/move', MoveBaseAction)
        self.robot = Robot()
        self.whole_body = self.robot.try_get('whole_body')

    def execute(self, userdata):
        move_goal = MoveBaseGoal()
        move_goal.target_pose.header.frame_id = 'map'
        move_goal.target_pose.pose.position.x = -0.259
        move_goal.target_pose.pose.position.y = 0.9146
        move_goal.target_pose.pose.orientation.z = 0.00823
        move_goal.target_pose.pose.orientation.w = 1
        self.move_client.wait_for_server()
        self.move_client.send_goal(move_goal)
        result = self.move_client.wait_for_result()

        if result:
            # go to neutral
            # TODO use this command in other neutral position moves too
            self.whole_body.move_to_joint_positions(neutral_joint_positions_4)
            rospy.sleep(1.0)

            vel = self.whole_body.joint_velocities
            while all(abs(i) > 0.05 for i in vel.values()):
                vel = self.whole_body.joint_velocities
            rospy.sleep(2)

            return 'succeeded'
        else:
            return 'aborted'


class NoGrasppointFound(smach.State):
    """ Smach state that will move the robot either backwards (use_map is False)
    or to a predefined position on the map (use_map is True)
    Also moves the joints to the neutral position
    Grasp check (turning the wrist) will be done here if enabled

    Userdata:
        Input Keys:
            params {dic}: additional parameters
                use_map: When True, robot moves to fixed points
                         on the map, otherwise only relative movement
                grasp_check: When True, robot twists the hand
                             after grasp to ensure a stable grasp
                clean_table: When True, robot is in clear table routine,
                             grasps and handovers are performed until
                             the table is empty

            find_grasppoint_tries {int}: how often find_grasppoint was
                                         called in a grasp run, relevant
                                         for table clearing routine
        Output keys:
            find_grasppoint_method {int}:
                    method identifier, 1-4
                    1 - detectron and HAF grasping
                    2 - verefine pipeline
                    3 - verefine pipeline and HAF
                    4 - pyrapose pipeline

            find_grasppoint_tries {int}: how often find_grasppoint was
                                         called in a grasp run, relevant
                                         for table clearing routine

    Outcomes:
        find_grasp: transition to FIND_GRASPPOINT State
        user_input: transition to USER_INPUT State,
                    ends clear table routine
    """

    def __init__(self):
        smach.State.__init__(self, output_keys=['find_grasppoint_tries', 'find_grasppoint_method'],
                             input_keys=['params', 'find_grasppoint_tries'],
                             outcomes=['find_grasp', 'user_input'])

    def execute(self, userdata):
        rospy.sleep(0.1)
        userdata.find_grasppoint_tries = userdata.find_grasppoint_tries + 1

        if userdata.find_grasppoint_tries > 4:
            return 'user_input'
        if userdata.find_grasppoint_tries > 3:
            userdata.find_grasppoint_method = 1
        if userdata.params.get('clean_table'):
            return 'find_grasp'
        else:
            return 'user_input'


class PlaceAtTable2(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'], input_keys=['grasped_pose'])
        self.robot = Robot()
        self.whole_body = self.robot.try_get('whole_body')
        self.tf = tf.TransformListener()
        self.robot = Robot()
        self.gripper = self.robot.try_get('gripper')
        self.collision_world = self.robot.try_get('global_collision_world')
        self.collision_world.remove_all()
           	    	
    def execute(self, userdata):

        pose = userdata.grasped_pose
        t = self.tf.getLatestCommonTime(
            '/map', pose.header.frame_id)
        pose.header.stamp = t
        pose = self.tf.transformPose('/map', pose)
        print(pose)

        safety = 0.07

        x = 2.98
        y = 1
        z = 0.833
        quat = [pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w]
        euler = tf.transformations.euler_from_quaternion(quat)

        table_pose = geometry.pose(x=2.98, y=0.6, z=0.76/2)
        self.collision_world.add_box(x=0.26, y=2, z=0.76, pose=table_pose, frame_id='map')  
        rospy.sleep(0.5)
        self.whole_body.move_end_effector_pose(geometry.pose(x + 0.05, y, z + safety, euler[0], euler[1], euler[2]), 'map')


        wrist_flex = self.whole_body.joint_positions.get('wrist_flex_joint')
        lift = self.whole_body.joint_positions.get('arm_lift_joint')

        self.whole_body.move_to_joint_positions({'wrist_flex_joint':wrist_flex - 0.4})

        self.whole_body.move_to_joint_positions({'arm_lift_joint':lift - 0.02})

        self.gripper.command(1.0)

        self.whole_body.move_end_effector_pose(geometry.pose(x - 2 * safety, y, z, euler[0], euler[1], euler[2]), 'map')
        
        return 'succeeded'

# class Placement(smach.State):
#     def __init__(self):
#         smach.State.__init__(self, outcomes=['succeeded', 'aborted'], input_keys=['grasped_pose'])
#         # init robot
#         self.robot = Robot()
#         self.gripper = self.robot.try_get('gripper')
#         self.whole_body = self.robot.try_get('whole_body')
#         self.omni = self.robot.try_get('omni_base')

#         # read parameter from startup.yaml
#         self.point_cloud_topic = rospy.get_param("/point_cloud_topic")
#         self.global_frame = rospy.get_param("/global_frame")
#         self.placement_object_dimensions = rospy.get_param("/placement_object_dimensions")
#         self.placement_resolution = rospy.get_param("/placement_resolution")
#         self.allow_placement = rospy.get_param("/allow_placement")
#         self.placement_grasp = rospy.get_param("/placement_grasp")
#         self.placement_steps = rospy.get_param("/placement_steps")
#         self.placement_sort = rospy.get_param("/placement_sort")
#         self.point_cloud_topic = rospy.get_param("/point_cloud_topic")
#         self.global_frame = rospy.get_param("/global_frame")
#         self.camera_frame = rospy.get_param("/camera_frame")
#         self.file_name = rospy.get_param("/file_name")
#         self.use_mesh = rospy.get_param("/use_mesh")
           	    	
#     def execute(self, userdata):

#         # init action servers
#         calcGripperPose_client = actionlib.SimpleActionClient('PlacementCalcGripperPose', PlacementCalcGripperPoseAction)
#         calcGripperPose_client.wait_for_server()

#         calculateAndSortPoses_client = actionlib.SimpleActionClient('PlacementCalculateAndSortPoses', PlacementCalculateAndSortPosesAction)
#         calculateAndSortPoses_client.wait_for_server()

#         collisionEnvironment_client = actionlib.SimpleActionClient('PlacementCollisionEnvironment', PlacementCollisionEnvironmentAction)
#         collisionEnvironment_client.wait_for_server()

#         detectPlane_client = actionlib.SimpleActionClient('PlacementDetectPlane', PlacementDetectPlaneAction)
#         detectPlane_client.wait_for_server()

#         placeAndMoveAway_client = actionlib.SimpleActionClient('PlacementPlaceAndMoveAway', PlacementPlaceAndMoveAwayAction)
#         placeAndMoveAway_client.wait_for_server()

#         calcGripperPose_goal = PlacementCalcGripperPoseGoal()
#         calculateAndSortPoses_goal = PlacementCalculateAndSortPosesGoal()
#         collisionEnvironment_goal = PlacementCollisionEnvironmentGoal()
#         detectPlane_goal = PlacementDetectPlaneGoal()
#         placeAndMoveAway_goal = PlacementPlaceAndMoveAwayGoal()

#         # move to init position
#         # move hand to left side to get a free view
#         neutral_joint_positions ={'arm_flex_joint': 0.0,
#                 'arm_lift_joint': 0.0,
#                 'arm_roll_joint': 1.570,
#                 'hand_motor_joint': 1.0,
#                 'head_pan_joint': 0.0,
#                 'head_tilt_joint': -0.75,
#                 'wrist_flex_joint': -1.57,
#                 'wrist_roll_joint': 0.0} 
#         self.whole_body.move_to_joint_positions(neutral_joint_positions)
#         self.whole_body.gaze_point((0.7, 0.1, 0.4))

#         # detect plane
#         rospy.loginfo("Step 1/5: Detect plane")

#         # wait for point cloud
#         cloud = rospy.wait_for_message(self.point_cloud_topic, PointCloud2)

#         detectPlane_goal.pcl = cloud
#         detectPlane_goal.target_frame = self.global_frame
#         detectPlane_goal.plane_method = PlaneMethod.RANSAC

#         detectPlane_client.send_goal(detectPlane_goal)
#         detectPlane_client.wait_for_result()
#         detectPlane_result = detectPlane_client.get_result()

#         if detectPlane_result.error == "No Error":
#             planeParams = detectPlane_result.planeParams
#         else:
#             print(detectPlane_result.error)
#             exit(1)

#         # create collision environment
#         rospy.loginfo("Step 2/5: Create Collision Environment")

#         pose = self.omni.get_pose()

#         # walls
#         collisionObject_list = list()

#         # collisionObject_list include pose and not pose stamped -change
        
#         # left wall
#         collisionObject = CollisionObject()
#         left_pose = Pose()
#         left_pose.position.y = pose.pos.y + 1.3
#         left_pose.position.x = pose.pos.x
#         left_pose.position.z = planeParams.center_pose.position.z / 2
#         left_pose.orientation.w = pose.ori.w
#         collisionObject.pose = left_pose
#         collisionObject.size_x = 3
#         collisionObject.size_y = 0.01
#         collisionObject.size_z = planeParams.center_pose.position.z
#         collisionObject.name = "wall_left"
#         collisionObject.frame = self.global_frame
#         collisionObject.method = CollisionMethod.ADD
#         collisionObject_list.append(collisionObject)

#         # right wall
#         collisionObject = CollisionObject()
#         right_pose = Pose()
#         right_pose.position.y = pose.pos.y - 1.3
#         right_pose.position.x = pose.pos.x
#         right_pose.position.z = planeParams.center_pose.position.z / 2
#         right_pose.orientation.w = pose.ori.w
#         collisionObject.pose = right_pose
#         collisionObject.size_x = 3
#         collisionObject.size_y = 0.01
#         collisionObject.size_z = planeParams.center_pose.position.z
#         collisionObject.name = "wall_right"
#         collisionObject.frame = self.global_frame
#         collisionObject.method = CollisionMethod.ADD
#         collisionObject_list.append(collisionObject)

#         # behind wall
#         collisionObject = CollisionObject()
#         behind_pose = Pose()
#         behind_pose.position.x = pose.pos.x + 1.2
#         behind_pose.position.y = pose.pos.y
#         behind_pose.position.z = planeParams.center_pose.position.z / 2
#         behind_pose.orientation.w = pose.ori.w
#         collisionObject.pose = behind_pose
#         collisionObject.size_x = 0.01
#         collisionObject.size_y = 3
#         collisionObject.size_z = planeParams.center_pose.position.z
#         collisionObject.name = "wall_behind"
#         collisionObject.frame = self.global_frame
#         collisionObject.method = CollisionMethod.ADD
#         collisionObject_list.append(collisionObject)

#         # front wall
#         collisionObject = CollisionObject()
#         front_pose = Pose()
#         front_pose.position.x = pose.pos.x - 1.2
#         front_pose.position.y = pose.pos.y
#         front_pose.position.z = planeParams.center_pose.position.z / 2
#         front_pose.orientation.w = pose.ori.w
#         collisionObject.pose = front_pose
#         collisionObject.size_x = 0.01
#         collisionObject.size_y = 3
#         collisionObject.size_z = planeParams.center_pose.position.z
#         collisionObject.name = "wall_front"
#         collisionObject.frame = self.global_frame
#         collisionObject.method = CollisionMethod.ADD
#         collisionObject_list.append(collisionObject)

#         # table
#         table_name = "table1"
#         collisionObject = CollisionObject()
#         table_pose = Pose()
#         table_pose.position.x = planeParams.center_pose.position.x
#         table_pose.position.y = planeParams.center_pose.position.y
#         table_pose.position.z = planeParams.center_pose.position.z / 2
#         table_pose.orientation.w = pose.ori.w
#         collisionObject.pose = table_pose
#         collisionObject.size_x = planeParams.height
#         collisionObject.size_y = planeParams.width
#         collisionObject.size_z = planeParams.center_pose.position.z
#         collisionObject.name = table_name
#         collisionObject.frame = self.global_frame
#         collisionObject.method = CollisionMethod.ADD
#         collisionObject_list.append(collisionObject)

#         # floor
#         collisionObject = CollisionObject()
#         floor_pose = Pose()
#         floor_pose.position.x = pose.pos.x
#         floor_pose.position.y = pose.pos.y
#         floor_pose.position.z = -0.1
#         floor_pose.orientation.w = pose.ori.w
#         collisionObject.pose = floor_pose
#         collisionObject.size_x = 15
#         collisionObject.size_y = 15
#         collisionObject.size_z = 0.1
#         collisionObject.name = "floor"
#         collisionObject.frame = self.global_frame
#         collisionObject.method = CollisionMethod.ADD
#         collisionObject_list.append(collisionObject)

#         # attach object in gripper
#         gripper_z_offset = 0.05
#         gripper_y_offset = 0.06
#         nx_dim, px_dim, z_dim = self.placement_object_dimensions
#         collisionObject = CollisionObject()
#         placement_box_pose = Pose()
#         placement_box_pose.orientation.w = 1.0
#         placement_box_pose.position.z = gripper_z_offset
#         # calculate z for different placement directions
#         if self.placement_grasp == GraspMode.NORMAL or self.placement_grasp == GraspMode.INVERT:
#             placement_box_pose.position.x = placement_box_pose.position.x - (nx_dim - px_dim) / 2 
#         placement_box_pose.position.z = placement_box_pose.position.z + z_dim / 2
#         distance = self.gripper.get_distance()
#         collisionObject.pose = placement_box_pose
#         collisionObject.size_x = nx_dim + px_dim
#         collisionObject.size_y = distance + gripper_y_offset
#         collisionObject.size_z = z_dim
#         collisionObject.name = "placement_object"
#         collisionObject.frame = "eef_link"
#         collisionObject.method = CollisionMethod.ATTACH
#         collisionObject_list.append(collisionObject)

#         # send goal
#         collisionEnvironment_goal.collisionObject_list = collisionObject_list
#         collisionEnvironment_client.send_goal(collisionEnvironment_goal)
#         collisionEnvironment_client.wait_for_result()
#         collisionEnvironment_result = collisionEnvironment_client.get_result()

#         if not collisionEnvironment_result.isDone:
#             print("Error while creating the collision environment")
#             exit(1)

#         # calculate center gripper pose
#         rospy.loginfo("Step 3/5: Calculate center pose for gripper")

#         objectDimension = ObjectDimension()
#         objectDimension.x_neg = nx_dim
#         objectDimension.x_pos = px_dim
#         objectDimension.z = z_dim

#         grasped_pose = userdata.grasped_pose
#         ori = grasped_pose.pose.orientation
#         euler = euler_from_quaternion(ori.x, ori.y, ori.z, ori.w)

#         if (euler[0] > 2.5):
#             self.placement_grasp = GraspMode.TOP
#         elif (euler[0] * euler[2] < 0):
#             self.placement_grasp = GraspMode.NORMAL
#         elif (euler[0] * euler[2] > 0):
#             self.placement_grasp = GraspMode.INVERT
#         else:
#             self.placement_grasp = GraspMode.NORMAL
        
#         calcGripperPose_goal.planeParams = planeParams
#         calcGripperPose_goal.objectDimension = objectDimension
#         calcGripperPose_goal.grasp_mode = self.placement_grasp
#         calcGripperPose_goal.target_frame = self.global_frame

#         calcGripperPose_client.send_goal(calcGripperPose_goal)
#         calcGripperPose_client.wait_for_result()
#         calcGripperPose_result = calcGripperPose_client.get_result()

#         # calc and sort all poses
#         rospy.loginfo("Step 4/5: Calculate and sort all poses for gripper")

#         calculateAndSortPoses_goal.sort_method = self.placement_sort
#         calculateAndSortPoses_goal.grasp_mode = self.placement_grasp
#         calculateAndSortPoses_goal.target_frame = self.global_frame
#         calculateAndSortPoses_goal.source_frame = self.camera_frame
#         calculateAndSortPoses_goal.planeParams = planeParams
#         calculateAndSortPoses_goal.objectDimension = objectDimension
#         calculateAndSortPoses_goal.center_pose = calcGripperPose_result.center_pose
#         calculateAndSortPoses_goal.pcl = cloud
#         calculateAndSortPoses_goal.placement_resolution = self.placement_resolution
#         calculateAndSortPoses_goal.placement_steps = self.placement_steps

#         calculateAndSortPoses_client.send_goal(calculateAndSortPoses_goal)
#         calculateAndSortPoses_client.wait_for_result()
#         calculateAndSortPoses_result = calculateAndSortPoses_client.get_result()

#         print(len(calculateAndSortPoses_result.pose_list))

#         # place and move away
#         rospy.loginfo("Step 5/5: Place object and move away")
#         placeAndMoveAway_goal.pose_list = calculateAndSortPoses_result.pose_list
#         placeAndMoveAway_goal.order = calculateAndSortPoses_result.order
#         placeAndMoveAway_goal.target_frame = self.global_frame
#         placeAndMoveAway_goal.support_surface_name = table_name

#         placeAndMoveAway_goal.name = "placement_object"

#         placeAndMoveAway_client.send_goal(placeAndMoveAway_goal)
#         placeAndMoveAway_client.wait_for_result()
#         placeAndMoveAway_result = placeAndMoveAway_client.get_result()

#         self.whole_body.move_to_neutral()

#         print(placeAndMoveAway_result.error)
        
#         return 'succeeded'

class AttachObject(smach.State):
    """ robot moves to a fixed position near the table
    using a move_base action goal
    also moves joints to neutral position

    Outcomes:
        succeeded: transitions to FIND_GRASPPOINT State
        aborted: transitions to USER_INPUT State
    """

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'])
        self.placement_object_dimensions = rospy.get_param("/placement_object_dimensions")
        self.robot = Robot()
        self.omni = self.robot.try_get('omni_base')

    def execute(self, userdata):
        collisionEnvironment_client = actionlib.SimpleActionClient('PlacementCollisionEnvironment', PlacementCollisionEnvironmentAction)
        collisionEnvironment_client.wait_for_server()
        collisionEnvironment_goal = PlacementCollisionEnvironmentGoal()

        
        # walls
        collisionObject_list = list()

        # collisionObject_list include pose and not pose stamped -change
        # attach object in gripper
        gripper_z_offset = 0.05
        gripper_y_offset = 0.06
        collisionObject = CollisionObject()
        placement_box_pose = Pose()
        placement_box_pose.orientation.w = 1.0
        placement_box_pose.position.z = gripper_z_offset
        collisionObject.pose = placement_box_pose
        collisionObject.size_x = 0.1
        collisionObject.size_y = 0.1
        collisionObject.size_z = 0.1
        collisionObject.name = "placement_object"
        collisionObject.frame = "eef_link"
        collisionObject.method = CollisionMethod.ATTACH
        collisionObject_list.append(collisionObject)

        pose = self.omni.get_pose()

        # floor
        collisionObject = CollisionObject()
        floor_pose = Pose()
        floor_pose.position.x = pose.pos.x
        floor_pose.position.y = pose.pos.y
        floor_pose.position.z = -0.1
        floor_pose.orientation.w = pose.ori.w
        collisionObject.pose = floor_pose
        collisionObject.size_x = 15
        collisionObject.size_y = 15
        collisionObject.size_z = 0.1
        collisionObject.name = "floor"
        collisionObject.frame = "map"
        collisionObject.method = CollisionMethod.ADD
        collisionObject_list.append(collisionObject)

        # send goal
        collisionEnvironment_goal.collisionObject_list = collisionObject_list
        collisionEnvironment_client.send_goal(collisionEnvironment_goal)
        collisionEnvironment_client.wait_for_result()
        collisionEnvironment_result = collisionEnvironment_client.get_result()

        if not collisionEnvironment_result.isDone:
            print("Error while creating the collision environment")
            return 'aborted'

        return 'succeeded'

class PlacementAreaDetector(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'], output_keys=['placement_area'], input_keys=['grasped_pose'])
        # init robot
        self.robot = Robot()
        self.gripper = self.robot.try_get('gripper')
        self.whole_body = self.robot.try_get('whole_body')
        self.omni = self.robot.try_get('omni_base')

        # read parameter from startup.yaml
        self.global_frame = rospy.get_param("/global_frame")
        self.placement_object_dimensions = rospy.get_param("/placement_object_dimensions")
        self.placement_sort = rospy.get_param("/placement_sort")
        self.global_frame = rospy.get_param("/global_frame")
           	    	
    def execute(self, userdata):

        # Detect Placement Area Service
        #string frame_id
        #geometry_msgs/Point target_point
        #geometry_msgs/Vector3 box_filter_range
        #geometry_msgs/Vector3 vertical_axis
        #float64 tilt_threshold
        #float64 distance_threshold
        #tmc_geometric_shapes_msgs/Shape object_shape
        #geometry_msgs/Pose object_to_surface
        #float64 surface_range

        grasped_pose = userdata.grasped_pose.pose
        frame = 'head_rgbd_sensor_rgb_frame'
        #print(userdata.grasped_pose)
        # grasped_pose = Pose()
        # grasped_pose.position.x = 0.189611
        # grasped_pose.position.y = 0.0962238
        # grasped_pose.position.z = 0.618759
        # grasped_pose.orientation.x = -0.457528
        # grasped_pose.orientation.y = 0.122534712
        # grasped_pose.orientation.z = 0.730142
        # grasped_pose.orientation.w = -0.492488698

        grasped_pose = transform_pose(self.global_frame , frame, grasped_pose).pose

        robot_pose = self.omni.get_pose()
        
        frame_id = self.global_frame 
        target_point = Point()
        target_point.x = robot_pose.pos.x
        target_point.y = robot_pose.pos.y
        target_point.z = 0.55

        box_filter_range = Vector3()
        box_filter_range.x = 4.0
        box_filter_range.y = 4.0
        box_filter_range.z = 1.0

        vertical_axis = Vector3()
        vertical_axis.x = 0.0
        vertical_axis.y = 0.0
        vertical_axis.z = 1.0

        tilt_threshold = np.pi * 60/180
        distance_threshold = 1.0

        object_shape = Shape()
        object_shape.type = Shape.MESH
        mesh = open3d.geometry.TriangleMesh.create_box(self.placement_object_dimensions[0], self.placement_object_dimensions[1], self.placement_object_dimensions[2])
        vertices_list = []
        for p in np.asarray(mesh.vertices):
            vertices_list.append(Point(p[0], p[1], p[2]))
        object_shape.triangles = np.asarray(mesh.triangles).flatten()
        object_shape.vertices = vertices_list

        object_to_surface = Pose()
        object_to_surface.position.x = 0
        object_to_surface.position.y = 0
        object_to_surface.position.z = 0
        object_to_surface.orientation.x = 0
        object_to_surface.orientation.y = 0
        object_to_surface.orientation.z = 0
        object_to_surface.orientation.w = 1

        surface_range = 1.0

        rospy.wait_for_service('detect_placement_area')

        try:
            detect_placement_area = rospy.ServiceProxy('detect_placement_area', DetectPlacementArea)
            response = detect_placement_area(frame_id, target_point, box_filter_range, vertical_axis, tilt_threshold, distance_threshold, object_shape, object_to_surface, surface_range)
        except rospy.ServiceException as e:
            print("DetectPlacmentAreaService call failed: %s" %e)

        userdata.placement_area = response.placement_area

        if response.error_code.val == 1:
            return 'succeeded'
        elif response.error_code.val == -1:
            print("ErrorCode: FAIL")
        elif response.error_code.val == -2:
            print("ErrorCode: INVALID_FRAME")
        elif response.error_code.val == -3:
            print("ErrorCode: NO_POINTCLOUD")
        elif response.error_code.val == -4:
            print("ErrorCode: NI_ACCEPTABLE_PLANE")
        elif response.error_code.val == -5:
            print("ErrorCode: INVALID_OBJECT_SURFACE")
        elif response.error_code.val == -1:
            print("ErrorCode: NON_POSITIVE")
        elif response.error_code.val == -1:
            print("ErrorCode: ZERO_VECTOR")
        
        return 'aborted'


def main():
    rospy.init_node('grasping_statemachine')

    sm = smach.StateMachine(outcomes=['end'])
    with sm:
        smach.StateMachine.add('USER_INPUT',
                               UserInput(),
                               transitions={'quitting': 'end',
                                            'neutral': 'GO_TO_NEUTRAL',
                                            'grasping': 'FIND_GRASPPOINT',
                                            'opening': 'OPENING',
                                            'find_grasp': 'ONLY_FIND_GRASPPOINT',
                                            'execute_grasp': 'EXECUTE_GRASP',
                                            'go_to_table': 'GO_TO_WAYPOINT_1',
                                            'detect_placement_area': 'DETECT_PLACEMENT_AREA'}) 

        smach.StateMachine.add('GO_TO_WAYPOINT_1',
                               GoToWaypoint1(),
                               transitions={'succeeded': 'USER_INPUT',
                                            'aborted': 'USER_INPUT'})
        
        smach.StateMachine.add('GO_TO_WAYPOINT_2',
                               GoToWaypoint2(),
                               transitions={'succeeded': 'DETECT_PLACEMENT_AREA',
                                            'aborted': 'USER_INPUT'})

        smach.StateMachine.add('GO_TO_WAYPOINT_3',
                        GoToWaypoint3(),
                        transitions={'succeeded': 'DETECT_PLACEMENT_AREA',
                                    'aborted': 'USER_INPUT'})

        smach.StateMachine.add('GO_TO_WAYPOINT_4',
                GoToWaypoint4(),
                transitions={'succeeded': 'ATTACH_OBJECT',
                            'aborted': 'USER_INPUT'})

        smach.StateMachine.add('ATTACH_OBJECT',
                AttachObject(),
                transitions={'succeeded': 'DETECT_PLACEMENT_AREA',
                            'aborted': 'USER_INPUT'})


        # smach.StateMachine.add('PLACE_OBJECT',
        #                        PlaceAtTable2(),
        #                        transitions={'succeeded': 'JUST_GO_TO_TABLE2',
        #                                     'aborted': 'USER_INPUT'})

        # smach.StateMachine.add('PLACE_OBJECT2',
        #                        Placement(),
        #                        transitions={'succeeded': 'JUST_GO_TO_TABLE2',
        #                                     'aborted': 'USER_INPUT'})

        smach.StateMachine.add('DETECT_PLACEMENT_AREA',
                               PlacementAreaDetector(),
                               transitions={'succeeded': 'PLACEMENT_PLACE',
                                            'aborted': 'USER_INPUT'})

        smach.StateMachine.add('PLACEMENT_PLACE',
                               smach_ros.SimpleActionState('placement_place', PlacementPlaceAction,
                                                           goal_slots=['placement_area', 'grasped_pose'],
                                                           result_slots=['error']),
                                transitions={'succeeded': 'USER_INPUT',
                                            'aborted': 'USER_INPUT',
                                            'preempted': 'USER_INPUT'})

        smach.StateMachine.add('GO_TO_NEUTRAL',
                               GoToNeutral(),
                               transitions={'succeeded': 'USER_INPUT',
                                            'continuing': 'GO_TO_WAYPOINT_1'})

        smach.StateMachine.add('OPENING',
                               Opening(),
                               transitions={'succeeded': 'USER_INPUT'})

        smach.StateMachine.add('FIND_GRASPPOINT',
                               smach_ros.SimpleActionState('find_grasppoint', FindGrasppointAction,
                                                           goal_slots=[
                                                               'method', 'object_names'],
                                                           result_slots=['grasp_poses']),
                               transitions={'succeeded': 'EXECUTE_GRASP',
                                            'preempted': 'USER_INPUT',
                                            'aborted': 'NO_GRASPPOINT_FOUND'},
                               remapping={'method': 'find_grasppoint_method',
                                          'grasp_poses': 'found_grasp_poses',
                                          'object_names': 'objects_to_find', })

        smach.StateMachine.add('NO_GRASPPOINT_FOUND',
                               NoGrasppointFound(),
                               transitions={'find_grasp': 'FIND_GRASPPOINT',
                                            'user_input': 'USER_INPUT'})

        smach.StateMachine.add('ONLY_FIND_GRASPPOINT',
                               smach_ros.SimpleActionState('find_grasppoint', FindGrasppointAction,
                                                           goal_slots=[
                                                               'method', 'object_names'],
                                                           result_slots=['grasp_poses']),
                               transitions={'succeeded': 'USER_INPUT',
                                            'preempted': 'USER_INPUT',
                                            'aborted': 'USER_INPUT'},
                               remapping={'method': 'find_grasppoint_method',
                                          'grasp_poses': 'found_grasp_poses',
                                          'object_names': 'objects_to_find'})

        smach.StateMachine.add('EXECUTE_GRASP',
                               smach_ros.SimpleActionState('execute_grasp', ExecuteGraspAction,
                                                           goal_slots=['grasp_poses'], 
                                                           result_slots=['grasped_pose']),
                               transitions={'succeeded': 'GO_TO_WAYPOINT_4', #GO_TO_TABLE2
                                            'preempted': 'USER_INPUT',
                                            'aborted': 'GO_TO_NEUTRAL'},
                               remapping={'grasp_poses': 'found_grasp_poses'})

        smach.StateMachine.add('NEUTRAL_BEFORE_HANDOVER',
                               GoBackAndNeutral(),
                               transitions={'succeeded': 'HANDOVER'})

        smach.StateMachine.add('HANDOVER', smach_ros.SimpleActionState('/handover', HandoverAction),
                               transitions={'succeeded': 'GO_TO_NEUTRAL',
                                            'preempted': 'USER_INPUT',
                                            'aborted': 'USER_INPUT'})

    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    # Execute state machine
    outcome = sm.execute()

    # Wait for ctrl-c to stop the application
    # rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()

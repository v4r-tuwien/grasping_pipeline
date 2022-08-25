#!/usr/bin/env python3
# Copyright (C) 2016 Toyota Motor Corporation
from enum import Enum
from math import pi

import actionlib
import rospy
import smach
import smach_ros
import tf
from actionlib_msgs.msg import GoalStatus
from grasping_pipeline.msg import (ExecuteGraspAction, ExecuteGraspGoal,
                                   FindGrasppointAction, FindGrasppointGoal)
from handover.msg import HandoverAction
from hsrb_interface import Robot
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

# neutral joint positions
neutral_joint_positions = {'arm_flex_joint': 0.0,
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
                                             'find_grasp', 'execute_grasp', 'go_to_table'],
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
            ['teddy bear', 'banana', 'bottle', 'sports ball', 'bowl', 'cup', 'book', 'transparent_canister', 'chair', 'baseball glove', 'handbag'])
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
                print('\t1 - Detectron unknown object')
                print('\t2 - Verefine known object')
                print('\t3 - Verefine choose known object')
                print('\t4 - Verefine known object with HAF')
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
                print('\t1 - Detectron unknown object')
                print('\t2 - Verefine known object')
                print('\t3 - Verefine choose known object')
                print('\t4 - Verefine known object with HAF')
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
            move_goal.target_pose.pose.position.x = 0.2
            move_goal.target_pose.pose.position.y = 0.1
            move_goal.target_pose.pose.orientation.z = 0.0
            move_goal.target_pose.pose.orientation.w = 1.0
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


class GoToTable(smach.State):
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
        move_goal.target_pose.pose.position.x = 0.5
        move_goal.target_pose.pose.position.y = 0.1
        move_goal.target_pose.pose.orientation.z = 0.0
        move_goal.target_pose.pose.orientation.w = 1.0
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
                                            'go_to_table': 'GO_TO_TABLE'})

        smach.StateMachine.add('GO_TO_TABLE',
                               GoToTable(),
                               transitions={'succeeded': 'FIND_GRASPPOINT',
                                            'aborted': 'USER_INPUT'})

        smach.StateMachine.add('GO_TO_NEUTRAL',
                               GoToNeutral(),
                               transitions={'succeeded': 'USER_INPUT',
                                            'continuing': 'GO_TO_TABLE'})

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
                                                           goal_slots=['grasp_poses']),
                               transitions={'succeeded': 'NEUTRAL_BEFORE_HANDOVER',
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

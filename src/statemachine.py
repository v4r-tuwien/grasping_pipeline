#! /usr/bin/env python3
import rospy
import smach
import smach_ros
from states.userinput import UserInput
from states.robot_control import GoToNeutral, GoBack, OpenGripper
from collision_environment import CreateCollisionObjects
from grasping_pipeline.msg import FindGrasppointAction, ExecuteGraspAction
from handover.msg import HandoverAction


def decision(userdata):
    userdata.method = 5
    userdata.object_names = ['unknown']
    return 'succeeded'


def create_statemachine():
    sm = smach.StateMachine(outcomes=['end'])
    smach.CBState
    with sm:
        smach.StateMachine.add('GO_TO_NEUTRAL', GoToNeutral(), transitions={
                               'succeeded': 'OPEN_GRIPPER'})
        smach.StateMachine.add('OPEN_GRIPPER', OpenGripper(), transitions={
                               'succeeded': 'FIND_GRASP_USERINPUT'})
        map = {'f': ['find_grasp', 'find grasp point'],
               'r': ['reset', 'reset state machine']}
        smach.StateMachine.add('FIND_GRASP_USERINPUT', UserInput(map), transitions={
                               'find_grasp': 'METHOD_DECISION', 'reset': 'GO_TO_NEUTRAL'})
        decider = smach.CBState(decision, output_keys=[
                                'method', 'object_names'], outcomes=['succeeded'])
        smach.StateMachine.add('METHOD_DECISION', decider, transitions={
                               'succeeded': 'FIND_GRASP'})
        find_grasp_actionstate = smach_ros.SimpleActionState('find_grasppoint', FindGrasppointAction, goal_slots=[
                                                             'method', 'object_names'], result_slots=['grasp_poses', 'object_bbs'])
        smach.StateMachine.add('FIND_GRASP', find_grasp_actionstate, transitions={
                               'succeeded': 'EXECUTE_GRASP_USERINPUT', 'aborted': 'FIND_GRASP_USERINPUT', 'preempted': 'FIND_GRASP_USERINPUT'})
        map = {'g': ['grasp', 'grasp object'], 't': ['retry', 'try again']}
        smach.StateMachine.add('EXECUTE_GRASP_USERINPUT', UserInput(map), transitions={
                               'grasp': 'CREATE_COLLISION_ENVIRONMENT', 'retry': 'FIND_GRASP'})
        smach.StateMachine.add('CREATE_COLLISION_ENVIRONMENT', CreateCollisionObjects(input_keys=['object_bbs']),
                               transitions={'succeeded': 'EXECUTE_GRASP', 'aborted': 'FIND_GRASP_USERINPUT'}, )
        execute_grasp_actionstate = smach_ros.SimpleActionState(
            'execute_grasp', ExecuteGraspAction, goal_slots=['grasp_poses'])
        smach.StateMachine.add('EXECUTE_GRASP', execute_grasp_actionstate, transitions={
                               'succeeded': 'GO_BACK_BEFORE_HANDOVER', 'aborted': 'GO_TO_NEUTRAL', 'preempted': 'GO_TO_NEUTRAL'})
        smach.StateMachine.add('GO_BACK_BEFORE_HANDOVER',
                               GoBack(),
                               transitions={'succeeded': 'HANDOVER'})
        smach.StateMachine.add('HANDOVER', smach_ros.SimpleActionState('/handover', HandoverAction),
                               transitions={'succeeded': 'GO_TO_NEUTRAL',
                                            'preempted': 'GO_TO_NEUTRAL',
                                            'aborted': 'GO_TO_NEUTRAL'})

    return sm


def create_statemachine2():
    sm = smach.StateMachine(outcomes=['end'])
    smach.CBState
    with sm:
        smach.StateMachine.add('GO_TO_NEUTRAL', GoToNeutral(), transitions={
                               'succeeded': 'CREATE_COLLISION_ENVIRONMENT'})
        smach.StateMachine.add('CREATE_COLLISION_ENVIRONMENT', CreateCollisionObjects(),
                               transitions={'succeeded': 'end', 'aborted': 'GO_TO_NEUTRAL'})

    return sm


if __name__ == '__main__':
    rospy.init_node('sasha_statemachine')
    sm = create_statemachine()

    try:
        # Create and start the introspection server
        sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
        sis.start()

        # Execute state machine
        outcome = sm.execute()

        # Wait for ctrl-c to stop the application
        # rospy.spin()
        sis.stop()
    except rospy.ROSInterruptException:
        pass

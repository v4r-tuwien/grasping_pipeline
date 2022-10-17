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


def create_statemachine(enable_userinput=True):
    #sm = smach.StateMachine(outcomes=['end'])
    seq = smach.Sequence(outcomes=['end'], connector_outcome='succeeded')

    with seq:
        smach.Sequence.add('GO_TO_NEUTRAL', GoToNeutral())

        smach.Sequence.add('OPEN_GRIPPER', OpenGripper())

        if enable_userinput:
            abort_state = 'FIND_GRASP_USERINPUT'
            map = {'f': ['succeeded', 'find grasp point'],
                   'r': ['reset', 'reset state machine']}
            smach.Sequence.add('FIND_GRASP_USERINPUT', UserInput(
                map), transitions={'reset': 'GO_TO_NEUTRAL'})
        else:
            abort_state = 'FIND_GRASP'

        decider = smach.CBState(decision, output_keys=[
                                'method', 'object_names'], outcomes=['succeeded'])
        smach.Sequence.add('METHOD_DECISION', decider)

        find_grasp_actionstate = smach_ros.SimpleActionState('find_grasppoint', FindGrasppointAction, goal_slots=[
                                                             'method', 'object_names'], result_slots=['grasp_poses', 'object_bbs'])
        smach.Sequence.add('FIND_GRASP', find_grasp_actionstate, transitions={
            'aborted': abort_state, 'preempted': abort_state})

        if enable_userinput:
            map = {'g': ['succeeded', 'grasp object'],
                   't': ['retry', 'try again']}
            smach.Sequence.add('EXECUTE_GRASP_USERINPUT', UserInput(
                map), transitions={'retry': 'FIND_GRASP'})

        smach.Sequence.add('CREATE_COLLISION_ENVIRONMENT', CreateCollisionObjects(input_keys=['object_bbs']),
                           transitions={'aborted': abort_state}, )

        execute_grasp_actionstate = smach_ros.SimpleActionState(
            'execute_grasp', ExecuteGraspAction, goal_slots=['grasp_poses'])

        smach.Sequence.add('EXECUTE_GRASP', execute_grasp_actionstate, transitions={
                           'aborted': 'GO_TO_NEUTRAL', 'preempted': 'GO_TO_NEUTRAL'})

        smach.Sequence.add('GO_BACK_BEFORE_HANDOVER',
                           GoBack())

        smach.Sequence.add('HANDOVER', smach_ros.SimpleActionState('/handover', HandoverAction),
                           transitions={'succeeded': 'GO_TO_NEUTRAL',
                                        'preempted': 'GO_TO_NEUTRAL',
                                        'aborted': 'GO_TO_NEUTRAL'})

    return seq


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
    sm = create_statemachine(enable_userinput=False)

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

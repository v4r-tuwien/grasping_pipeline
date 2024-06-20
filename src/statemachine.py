#! /usr/bin/env python3
import rospy
import smach
import smach_ros
from states.statemachine_components import get_robot_setup_sm, get_execute_grasp_sm, get_placement_sm, get_find_grasp_sm
from states.userinput import UserInput
from states.robot_control import GoToWaypoint, GoBack, GoToNeutral, CheckTopGrasp
from states.grasp_method_selector import GraspMethodSelector
from grasping_pipeline_msgs.msg import FindGrasppointAction
from handover.msg import HandoverAction

def create_statemachine(do_handover=True):
    sm = smach.StateMachine(outcomes=['end'])

    table_waypoint = GoToWaypoint(0.25, 0.41, 0)
    setup_sm = get_robot_setup_sm(table_waypoint)
    find_grasp_sm = get_find_grasp_sm()
    execute_grasp_sm = get_execute_grasp_sm(table_waypoint)
    placement_sm = get_placement_sm()
    with sm:
        smach.StateMachine.add('SETUP', setup_sm, transitions={'setup_succeeded': 'FIND_GRASP_USERINPUT'})

        map = {'f': ['succeeded', 'find grasp point']}
        smach.StateMachine.add('FIND_GRASP_USERINPUT', UserInput(
            map), transitions={'succeeded': 'FIND_GRASP'})

        smach.StateMachine.add('FIND_GRASP', find_grasp_sm, transitions={
            'end_find_grasp': 'EXECUTE_GRASP_USERINPUT', 'failed': 'SETUP'}, 
            remapping={'grasp_poses':'grasp_poses', 'grasp_object_bb':'grasp_object_bb', 'grasp_object_name':'grasp_object_name'})
        

        map = {'g': ['succeeded', 'grasp object'],
               't': ['retry', 'try again']}
        smach.StateMachine.add('EXECUTE_GRASP_USERINPUT', UserInput(
            map), transitions={'retry': 'FIND_GRASP', 'succeeded': 'EXECUTE_GRASP'})
        
        smach.StateMachine.add('EXECUTE_GRASP', execute_grasp_sm, transitions={
            'end_execute_grasp': 'CHECK_TOP_GRASP', 'failed_to_grasp': 'SETUP'})
        
        smach.StateMachine.add('CHECK_TOP_GRASP', CheckTopGrasp(), transitions={'top_grasp': 'HANDOVER', 'not_top_grasp': 'AFTER_GRASP_USERINPUT'})
        
        map = {'p': ['placement', 'place object'],
               'h': ['handover', 'handover object']}
        smach.StateMachine.add('AFTER_GRASP_USERINPUT', UserInput(
            map), transitions={'placement': 'PLACEMENT', 'handover': 'HANDOVER'})

        smach.StateMachine.add('HANDOVER', smach_ros.SimpleActionState('/handover', HandoverAction),
                                transitions={'succeeded': 'SETUP',
                                             'preempted': 'SETUP',
                                             'aborted': 'SETUP'})
        
        smach.StateMachine.add('PLACEMENT', placement_sm, transitions={'end_placement': 'RETREAT_AFTER_PLACEMENT', 'failed_to_place': 'GO_BACK_TO_TABLE'})
        
        smach.StateMachine.add('RETREAT_AFTER_PLACEMENT', GoBack(0.2), transitions={'succeeded': 'GO_TO_NEUTRAL_AFTER_PALCEMENT'})
        smach.StateMachine.add('GO_TO_NEUTRAL_AFTER_PALCEMENT', GoToNeutral(), transitions={'succeeded': 'SETUP'})

        smach.StateMachine.add('GO_BACK_TO_TABLE', table_waypoint, transitions={'succeeded': 'HANDOVER', 'aborted': 'GO_BACK_TO_TABLE'})

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

#! /usr/bin/env python3
import rospy
import smach
import smach_ros
from statemachine_components import get_robot_setup_sm, get_execute_grasp_sm, get_placement_sm, get_find_grasp_sm, get_object_detector_sm, get_pose_estimator_sm
from userinput import UserInput
from robot_control import GoToWaypoint, GoBack, GoToNeutral, CheckTopGrasp
from grasping_pipeline_msgs.msg import HandoverAction
from check_table_clean import CheckTableClean, RemoveNonTableObjects
from find_table_planes import FindTablePlanes

def create_statemachine(do_handover=True):
    sm = smach.StateMachine(outcomes=['end'])

    table_waypoint = GoToWaypoint(0.50, 0.4, 0)
    setup_sm = get_robot_setup_sm(table_waypoint)
    find_grasp_sm = get_find_grasp_sm()
    execute_grasp_sm = get_execute_grasp_sm(table_waypoint)
    placement_sm = get_placement_sm()
    single_grasp_sm = get_single_grasp_sm(table_waypoint, find_grasp_sm, execute_grasp_sm, placement_sm)
    clear_table_sm = get_clear_table_sm(table_waypoint, get_object_detector_sm(), get_pose_estimator_sm(), execute_grasp_sm, placement_sm, setup_sm)
    with sm:
        smach.StateMachine.add('SETUP', setup_sm, transitions={'setup_succeeded': 'DECIDE_PROCEDURE'})
        
        map = {'g': ['single_grasp', 'single grasp'], 't': ['clear_table', 'clear table']}
        smach.StateMachine.add('DECIDE_PROCEDURE', UserInput(
            map), transitions={'single_grasp': 'SINGLE_GRASP', 'clear_table': 'CLEAR_TABLE'})
        smach.StateMachine.add('SINGLE_GRASP', single_grasp_sm, transitions={'succeeded': 'SETUP', 'failed': 'SETUP'})
        smach.StateMachine.add('CLEAR_TABLE', clear_table_sm, transitions={'succeeded': 'SETUP'})

    return sm

def get_clear_table_sm(table_waypoint, object_detector_sm, pose_estimator_sm, execute_grasp_sm, placement_sm, setup_sm):
    sm = smach.StateMachine(outcomes=['succeeded'])
    with sm:
        smach.StateMachine.add('SETUP', setup_sm, transitions={'setup_succeeded': 'DETECT_OBJECTS'})
        smach.StateMachine.add('DETECT_OBJECTS', object_detector_sm, transitions={'failed': 'SETUP', 'succeeded': 'GET_TABLE_PLANES'})
        smach.StateMachine.add('GET_TABLE_PLANES', FindTablePlanes(enlarge_table_bb_to_floor=False), transitions={'succeeded': 'REMOVE_NON_TABLE_OBJECTS'})
        smach.StateMachine.add('REMOVE_NON_TABLE_OBJECTS', RemoveNonTableObjects(), transitions={'succeeded': 'CHECK_TABLE_CLEAN', 'failed': 'SETUP'})
        smach.StateMachine.add('CHECK_TABLE_CLEAN', CheckTableClean(), transitions={'clean': 'succeeded', 'not_clean': 'POSE_ESTIMATION'})
        smach.StateMachine.add('POSE_ESTIMATION', pose_estimator_sm, transitions={'failed': 'SETUP', 'succeeded': 'EXECUTE_GRASP'})
        smach.StateMachine.add('EXECUTE_GRASP', execute_grasp_sm, transitions={
                'end_execute_grasp': 'CHECK_TOP_GRASP', 'failed_to_grasp': 'SETUP'})
        
        #TODO test with combination of placement + handover, instead of only handover
        smach.StateMachine.add('CHECK_TOP_GRASP', CheckTopGrasp(), transitions={'top_grasp': 'HANDOVER', 'not_top_grasp': 'HANDOVER'})

        smach.StateMachine.add('HANDOVER', smach_ros.SimpleActionState('/handover', HandoverAction, goal_slots=['object_name']),
                                    transitions={'succeeded': 'SETUP',
                                                'preempted': 'SETUP',
                                                'aborted': 'SETUP'}, remapping={'object_name':'grasp_object_name'})
            
        smach.StateMachine.add('PLACEMENT', placement_sm, transitions={'end_placement': 'RETREAT_AFTER_PLACEMENT', 'failed_to_place': 'GO_BACK_TO_TABLE'})
            
        smach.StateMachine.add('RETREAT_AFTER_PLACEMENT', GoBack(0.2), transitions={'succeeded': 'GO_TO_NEUTRAL_AFTER_PLACEMENT', 'aborted': 'GO_TO_NEUTRAL_AFTER_PLACEMENT'})
        smach.StateMachine.add('GO_TO_NEUTRAL_AFTER_PLACEMENT', GoToNeutral(), transitions={'succeeded': 'SETUP'})

        smach.StateMachine.add('GO_BACK_TO_TABLE', table_waypoint, transitions={'succeeded': 'HANDOVER', 'aborted': 'GO_BACK_TO_TABLE'})
    return sm

def get_single_grasp_sm(table_waypoint, find_grasp_sm, execute_grasp_sm, placement_sm):
    sm = smach.StateMachine(outcomes=['failed', 'succeeded'])
    with sm:
        smach.StateMachine.add('FIND_GRASP', find_grasp_sm, transitions={
                'end_find_grasp': 'EXECUTE_GRASP_USERINPUT', 'failed': 'failed'})

        map = {'g': ['succeeded', 'grasp object'],
                't': ['retry', 'try again']}
        smach.StateMachine.add('EXECUTE_GRASP_USERINPUT', UserInput(
                map), transitions={'retry': 'failed', 'succeeded': 'EXECUTE_GRASP'})
            
        smach.StateMachine.add('EXECUTE_GRASP', execute_grasp_sm, transitions={
                'end_execute_grasp': 'CHECK_TOP_GRASP', 'failed_to_grasp': 'failed'})
            
        smach.StateMachine.add('CHECK_TOP_GRASP', CheckTopGrasp(), transitions={'top_grasp': 'HANDOVER', 'not_top_grasp': 'AFTER_GRASP_USERINPUT'})
            
        map = {'p': ['placement', 'place object'],
                'h': ['handover', 'handover object']}
        smach.StateMachine.add('AFTER_GRASP_USERINPUT', UserInput(
                map), transitions={'placement': 'PLACEMENT', 'handover': 'HANDOVER'})

        smach.StateMachine.add('HANDOVER', smach_ros.SimpleActionState('/handover', HandoverAction, goal_slots=['object_name']),
                                    transitions={'succeeded': 'succeeded',
                                                'preempted': 'succeeded',
                                                'aborted': 'succeeded'}, remapping={'object_name':'grasp_object_name'})
            
        smach.StateMachine.add('PLACEMENT', placement_sm, transitions={'end_placement': 'RETREAT_AFTER_PLACEMENT', 'failed_to_place': 'GO_BACK_TO_TABLE'})
            
        smach.StateMachine.add('RETREAT_AFTER_PLACEMENT', GoBack(0.2), transitions={'succeeded': 'GO_TO_NEUTRAL_AFTER_PLACEMENT', 'aborted': 'GO_TO_NEUTRAL_AFTER_PLACEMENT'})
        smach.StateMachine.add('GO_TO_NEUTRAL_AFTER_PLACEMENT', GoToNeutral(), transitions={'succeeded': 'succeeded'})

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

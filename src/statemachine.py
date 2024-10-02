#! /usr/bin/env python3
import rospy
import smach
import smach_ros
from statemachine_components import get_robot_setup_sm, get_execute_grasp_sm, get_placement_sm, get_find_grasp_sm, get_obj_detection_sm
from userinput import UserInput
from robot_control import GoToWaypoint, GoBack, GoToNeutral, CheckTopGrasp, CheckVerifyGrasp, VerifyGrasp, CheckObjectDetection
from grasp_method_selector import GraspMethodSelector
from grasping_pipeline_msgs.msg import FindGrasppointAction
from handover.msg import HandoverAction

def create_statemachine(do_handover=True):
    sm = smach.StateMachine(outcomes=['end'])

    table_waypoint = GoToWaypoint(0.25, 0.41, 0)
    setup_sm = get_robot_setup_sm(table_waypoint)
    obj_detection_sm = get_obj_detection_sm()
    find_grasp_sm = get_find_grasp_sm()
    execute_grasp_sm = get_execute_grasp_sm(table_waypoint)
    placement_sm = get_placement_sm()
    with sm:
        # State machine for robot setup
        smach.StateMachine.add('SETUP', setup_sm, transitions={'setup_succeeded': 'FIND_GRASP_USERINPUT'})

        # States for user input after setup
        map = {'f': ['succeeded', 'find grasp point']}
        smach.StateMachine.add('FIND_GRASP_USERINPUT', UserInput(
            map), transitions={'succeeded': 'OBJECT_DETECTION'})
        
        # State machine for getting the rgb and depth image and calling the object detector
        # Returns rgb, depth, bb_detections, mask_detections, class_names, class_confidences
        smach.StateMachine.add('OBJECT_DETECTION', obj_detection_sm, transitions={
            'succeeded': 'CHECK_OBJECT_DETECTION', 'aborted': 'SETUP', 'failed': 'SETUP'}, 
            remapping={'rgb':'rgb', 'depth':'depth', 'bb_detections':'bb_detections', 'mask_detections':'mask_detections', 'class_names':'class_names', 'class_confidences':'class_confidences'})

        # State for checking if object detection was successful (TODO: Only temporary solution)
        # Problem: Object detection is allowed to not detect object when the grasp is getting verified, but not here (table has no more objects on it)
        smach.StateMachine.add('CHECK_OBJECT_DETECTION', CheckObjectDetection(), transitions={'succeeded': 'FIND_GRASP', 'failed': 'SETUP'})

        # State machine for finding the grasp point
        # Returns grasp_poses, grasp_object_bb, grasp_object_name
        smach.StateMachine.add('FIND_GRASP', find_grasp_sm, transitions={
            'end_find_grasp': 'EXECUTE_GRASP_USERINPUT', 'failed': 'SETUP'}, 
            remapping={'grasp_poses':'grasp_poses', 'grasp_object_bb':'grasp_object_bb', 'grasp_object_name':'grasp_object_name'})
        
        # States for user input after finding the grasp point
        map = {'g': ['succeeded', 'grasp object'],
               't': ['retry', 'try again']}
        smach.StateMachine.add('EXECUTE_GRASP_USERINPUT', UserInput(
            map), transitions={'retry': 'OBJECT_DETECTION', 'succeeded': 'EXECUTE_GRASP'})
        
        # State machine for executing the grasp
        # Returns placement_surface_to_wrist, top_grasp, verify_grasp
        smach.StateMachine.add('EXECUTE_GRASP', execute_grasp_sm, transitions={
            'end_execute_grasp': 'CHECK_VERIFY_GRASP', 'failed_to_grasp': 'SETUP'})
        
        # States for checking if the grasp needs to be verified (verify_grasp = True) (e.g. the gripper is closed)
        smach.StateMachine.add('CHECK_VERIFY_GRASP', CheckVerifyGrasp(), transitions={'succeeded': 'CHECK_TOP_GRASP', 'verify_grasp': 'VERIFY_GRASP_OBJECT_DETECTION'})

        smach.StateMachine.add('VERIFY_GRASP_OBJECT_DETECTION', obj_detection_sm, transitions={
            'succeeded': 'VERIFY_GRASP_COMPARE_OBJECT_NAMES', 'aborted': 'SETUP', 'failed': 'SETUP'}, 
            remapping={'rgb':'_', 'depth':'_', 'bb_detections':'bb_detections_new', 'mask_detections':'mask_detections_new', 'class_names':'class_names_new', 'class_confidences':'_'})

        smach.StateMachine.add('VERIFY_GRASP_COMPARE_OBJECT_NAMES', VerifyGrasp(), transitions={'succeeded': 'CHECK_TOP_GRASP', 'failed': 'SETUP'})
        
        # State for checking if the grasp is a top grasp (top_grasp = True). If the grasp is a top grasp the object is handed over to the user.
        smach.StateMachine.add('CHECK_TOP_GRASP', CheckTopGrasp(), transitions={'top_grasp': 'HANDOVER', 'not_top_grasp': 'AFTER_GRASP_USERINPUT'})
        
        # States for user input after the grasp
        map = {'p': ['placement', 'place object'],
               'h': ['handover', 'handover object']}
        smach.StateMachine.add('AFTER_GRASP_USERINPUT', UserInput(
            map), transitions={'placement': 'PLACEMENT', 'handover': 'HANDOVER'})

        # State for handing over the object to the user (either because it's a top grasp or the user requested it)
        smach.StateMachine.add('HANDOVER', smach_ros.SimpleActionState('/handover', HandoverAction),
                                transitions={'succeeded': 'SETUP',
                                             'preempted': 'SETUP',
                                             'aborted': 'SETUP'})
        
        # State machine for placing the object (only if the user requested it, grasp is not a top grasp)
        smach.StateMachine.add('PLACEMENT', placement_sm, transitions={'end_placement': 'RETREAT_AFTER_PLACEMENT', 'failed_to_place': 'GO_BACK_TO_TABLE'})
        
        # States for retreating after placing the object
        smach.StateMachine.add('RETREAT_AFTER_PLACEMENT', GoBack(0.2), transitions={'succeeded': 'GO_TO_NEUTRAL_AFTER_PLACEMENT', 'aborted': 'GO_TO_NEUTRAL_AFTER_PLACEMENT'})
        smach.StateMachine.add('GO_TO_NEUTRAL_AFTER_PLACEMENT', GoToNeutral(), transitions={'succeeded': 'SETUP'})

        # States for going back to the table
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

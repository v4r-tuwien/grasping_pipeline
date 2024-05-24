#! /usr/bin/env python3
import rospy
import smach
import smach_ros
from states.statemachine_components import get_robot_setup_sm, get_execute_grasp_sm, get_placement_sm
from states.userinput import UserInput
from states.robot_control import GoToWaypoint, GoBack
from states.grasp_method_selector import GraspMethodSelector
from grasping_pipeline_msgs.msg import FindGrasppointAction
from handover.msg import HandoverAction
from grasping_pipeline_msgs.srv import FetchImages, CallObjectDetector, CallPoseEstimator, CallDirectGraspPoseEstimator


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
            'end_execute_grasp': 'AFTER_GRASP_USERINPUT', 'failed_to_grasp': 'SETUP'})
        
        map = {'p': ['placement', 'place object'],
               'h': ['handover', 'handover object']}
        smach.StateMachine.add('AFTER_GRASP_USERINPUT', UserInput(
            map), transitions={'placement': 'PLACEMENT', 'handover': 'HANDOVER'})

        smach.StateMachine.add('HANDOVER', smach_ros.SimpleActionState('/handover', HandoverAction),
                                transitions={'succeeded': 'SETUP',
                                             'preempted': 'SETUP',
                                             'aborted': 'SETUP'})
        
        smach.StateMachine.add('PLACEMENT', placement_sm, transitions={'end_placement': 'RETREAT_AFTER_PLACEMENT', 'failed_to_place': 'AFTER_GRASP_USERINPUT'})
        smach.StateMachine.add('RETREAT_AFTER_PLACEMENT', GoBack(0.2), transitions={'succeeded': 'SETUP'})
    return sm

def get_find_grasp_sm():
    find_grasp_sm = smach.StateMachine(outcomes=['failed', 'end_find_grasp'], output_keys=['grasp_poses', 'grasp_object_bb', 'grasp_object_name'])
    with find_grasp_sm:
        image_fetcher_service = smach_ros.ServiceState('fetch_synchronized_images', FetchImages, response_slots=['rgb', 'depth'])
        smach.StateMachine.add('IMAGE_FETCHER', image_fetcher_service, transitions={'succeeded': 'CALL_OBJECT_DETECTOR', 'aborted': 'failed', 'preempted': 'failed'})
        
        call_object_detector_service = smach_ros.ServiceState(
            'call_object_detector', 
            CallObjectDetector, 
            request_slots=['rgb', 'depth'], 
            response_slots=['bb_detections', 'mask_detections', 'class_names', 'class_confidences'])
        smach.StateMachine.add('CALL_OBJECT_DETECTOR', call_object_detector_service, transitions={'succeeded': 'SELECT_GRASP_METHOD', 'aborted': 'failed', 'preempted': 'failed'})

        smach.StateMachine.add("SELECT_GRASP_METHOD", GraspMethodSelector(), transitions={'pose_based_grasp': 'CALL_POSE_ESTIMATOR', 'direct_grasp': 'CALL_DIRECT_GRASP_POSE_ESTIMATOR'})
        
        call_pose_estimator_service = smach_ros.ServiceState(
            'call_pose_estimator', 
            CallPoseEstimator, 
            request_slots=['rgb', 'depth', 'bb_detections', 'mask_detections', 'class_names', 'class_confidences'], 
            response_slots=['pose_results', 'class_names', 'class_confidences'])
        smach.StateMachine.add(
            'CALL_POSE_ESTIMATOR', 
            call_pose_estimator_service, 
            transitions={'succeeded': 'FIND_GRASP', 'aborted': 'failed', 'preempted': 'failed'},
            remapping={'pose_results':'object_poses'})
        
        find_grasp_actionstate = smach_ros.SimpleActionState(
            'find_grasppoint', 
            FindGrasppointAction, 
            goal_slots=['depth', 'class_names', 'object_poses'],
            result_slots=['grasp_poses', 'grasp_object_bb', 'grasp_object_name'])
        smach.StateMachine.add('FIND_GRASP', find_grasp_actionstate, transitions={
            'aborted': 'failed', 'preempted': 'failed', 'succeeded': 'end_find_grasp'})
        
        direct_grasp_pose_estimator_service = smach_ros.ServiceState(
            'call_direct_grasppose_estimator', 
            CallDirectGraspPoseEstimator, 
            request_slots=['rgb', 'depth', 'bb_detections', 'mask_detections', 'class_names'], 
            response_slots=['grasp_poses', 'grasp_object_bb', 'grasp_object_name'])
        smach.StateMachine.add(
            'CALL_DIRECT_GRASP_POSE_ESTIMATOR',
            direct_grasp_pose_estimator_service,
            transitions={'succeeded': 'end_find_grasp', 'aborted': 'failed', 'preempted': 'failed'},
        )

        return find_grasp_sm

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

import smach
import smach_ros
from states.robot_control import GoToNeutral, OpenGripper, GoToWaypoint, GoToAndLookAtPlacementArea, GoBack
from states.find_table_planes import FindTablePlanes
from states.collision_environment import CollisionEnvironment
from grasping_pipeline_msgs.msg import ExecuteGraspAction, PlaceAction
from states.grasp_method_selector import GraspMethodSelector
from grasping_pipeline_msgs.msg import FindGrasppointAction
from grasping_pipeline_msgs.srv import FetchImages, CallObjectDetector, CallPoseEstimator, CallDirectGraspPoseEstimator

def get_robot_setup_sm(setup_waypoint):
    '''
    Returns a state machine that performs a setup for the robot and brings it into a well defined state.
    '''
    seq = smach.Sequence(outcomes=['setup_succeeded'], connector_outcome='succeeded', input_keys=[], output_keys=[])
    with seq:
        smach.Sequence.add('GO_TO_TABLE', setup_waypoint, transitions={'aborted': 'GO_TO_TABLE'})
        smach.Sequence.add('GO_TO_NEUTRAL', GoToNeutral())
        smach.Sequence.add('OPEN_GRIPPER', OpenGripper(), transitions={'succeeded': 'setup_succeeded'})
    return seq

    
def get_execute_grasp_sm(after_grasp_waypoint):
    '''
    Returns a state machine that performs all steps necessary to execute a grasp.
    '''
    seq = smach.Sequence(outcomes=['end_execute_grasp', 'failed_to_grasp'],
                         connector_outcome='succeeded', 
                         input_keys=['grasp_object_bb', 'grasp_poses'],
                         output_keys=['placement_surface_to_wrist'])

    table_waypoint = GoToWaypoint(0.25, 0.41, 0)

    with seq:
        smach.Sequence.add('FIND_TABLE_PLANES', FindTablePlanes())

        smach.Sequence.add('ADD_COLLISION_OBJECTS', CollisionEnvironment())

        execute_grasp_actionstate = smach_ros.SimpleActionState(
            'execute_grasp', ExecuteGraspAction, goal_slots=['grasp_poses', 'grasp_object_name_moveit', 'table_plane_equations'], result_slots=['placement_surface_to_wrist']) 

        smach.Sequence.add('EXECUTE_GRASP', execute_grasp_actionstate, transitions={
                            'aborted': 'failed_to_grasp', 'preempted': 'failed_to_grasp'})

        smach.Sequence.add('RETREAT_AFTER_GRASP', GoBack(0.2))

        smach.Sequence.add('GO_TO_NEUTRAL_AFTER_GRASP', GoToNeutral())

        smach.Sequence.add('GO_BACK_TO_TABLE', after_grasp_waypoint, 
                            transitions={'succeeded': 'end_execute_grasp', 'aborted': 'GO_BACK_TO_TABLE'})
    return seq

def get_placement_sm():
    '''
    Returns a state machine that performs all steps necessary to place an object.'''
    sm = smach.Sequence(outcomes=['end_placement', 'failed_to_place'], connector_outcome='succeeded',input_keys=['grasp_object_name', 'placement_surface_to_wrist'], output_keys=[])
    with sm:
        smach.Sequence.add('GO_TO_AND_LOOK_AT_PLACEMENT_AREA', GoToAndLookAtPlacementArea(), transitions={'aborted': 'failed_to_place'})
        smach.Sequence.add('FIND_TABLE_PLANES_PLACEMENT', FindTablePlanes())
        smach.Sequence.add('PLACEMENT_PLACE',
                        smach_ros.SimpleActionState('place_object', PlaceAction,
                                                    goal_slots=[
                                                        'placement_area_bb', 'table_plane_equations', 'table_bbs', 'placement_surface_to_wrist']),
                        transitions={'succeeded': 'end_placement',
                                    'aborted': 'failed_to_place',
                                    'preempted': 'failed_to_place'})
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
#!/usr/bin/env python3
import rospy
import actionlib
import smach
import smach_ros
import json
from grasping_pipeline_msgs.msg import FindGrasppointAction
from robot_control import GoToWaypoint
from grasping_pipeline_msgs.msg import HandoverAction
from robot_llm.msg import RobotLLMAction, RobotLLMResult
from statemachine_components import get_robot_setup_sm, get_execute_grasp_sm, get_placement_sm, get_find_grasp_sm
from grasping_pipeline_msgs.srv import FetchImages, CallObjectDetector, CallPoseEstimator, CallDirectGraspPoseEstimator

def get_object_detection_sm():
    object_detection_sm = smach.StateMachine(outcomes=['detection_failed', 'success'], output_keys=['class_names'])
    with object_detection_sm:
        image_fetcher_service = smach_ros.ServiceState('fetch_synchronized_images', FetchImages, response_slots=['rgb', 'depth'])
        smach.StateMachine.add('IMAGE_FETCHER', image_fetcher_service, transitions={'succeeded': 'CALL_OBJECT_DETECTOR', 'aborted': 'detection_failed', 'preempted': 'detection_failed'})
        
        call_object_detector_service = smach_ros.ServiceState(
            'call_object_detector', 
            CallObjectDetector, 
            request_slots=['rgb', 'depth'], 
            response_slots=['bb_detections', 'mask_detections', 'class_names', 'class_confidences'])
        smach.StateMachine.add('CALL_OBJECT_DETECTOR', call_object_detector_service, transitions={'succeeded': 'success', 'aborted': 'detection_failed', 'preempted': 'detection_failed'})
    return object_detection_sm

def create_statemachine(do_handover=True):
    table_waypoint = GoToWaypoint(0.25, 0.0, 0)
    robot_setup_sm = get_robot_setup_sm(table_waypoint)
    execute_grasp_sm = get_execute_grasp_sm(table_waypoint)
    placement_sm = get_placement_sm()
    find_grasp_sm = get_find_grasp_sm()
    
    grasp_handover_sm = smach.StateMachine(outcomes=['success', 'object_not_found'])
    with grasp_handover_sm:
        smach.StateMachine.add('SETUP_ROBOT', robot_setup_sm, transitions={'setup_succeeded': 'FIND_GRASP'})
        
        smach.StateMachine.add('FIND_GRASP', find_grasp_sm, transitions={
            'end_find_grasp': 'EXECUTE_GRASP', 'failed': 'object_not_found'}, 
            remapping={'grasp_poses':'grasp_poses', 'grasp_object_bb':'grasp_object_bb', 'grasp_object_name':'grasp_object_name'})

        smach.StateMachine.add('EXECUTE_GRASP', execute_grasp_sm, transitions={
                        'end_execute_grasp': 'HANDOVER', 'failed_to_grasp': 'SETUP_ROBOT'})
        smach.StateMachine.add('HANDOVER', smach_ros.SimpleActionState('/handover', HandoverAction),
                            transitions={'succeeded': 'SETUP_ROBOT_AFTER_HANDOVER',
                                        'preempted': 'SETUP_ROBOT_AFTER_HANDOVER',
                                        'aborted': 'SETUP_ROBOT_AFTER_HANDOVER'})
        smach.StateMachine.add('SETUP_ROBOT_AFTER_HANDOVER', robot_setup_sm, transitions={'setup_succeeded': 'success'})

    grasp_place_sm = smach.StateMachine(outcomes=['success', 'object_not_found'])
    with grasp_place_sm:
            smach.StateMachine.add('SETUP_ROBOT', robot_setup_sm, transitions={'setup_succeeded': 'FIND_GRASP'})
            smach.StateMachine.add('FIND_GRASP', find_grasp_sm, transitions={
            'end_find_grasp': 'EXECUTE_GRASP', 'failed': 'object_not_found'}, 
            remapping={'grasp_poses':'grasp_poses', 'grasp_object_bb':'grasp_object_bb', 'grasp_object_name':'grasp_object_name'})

            smach.StateMachine.add('EXECUTE_GRASP', execute_grasp_sm, transitions={
                            'end_execute_grasp': 'PLACE_OBJECT', 'failed_to_grasp': 'SETUP_ROBOT'})
            smach.StateMachine.add('PLACE_OBJECT', placement_sm, transitions={'end_placement': 'SETUP_ROBOT_AFTER_PLACEMENT', 'failed_to_place': 'PLACE_OBJECT'})
            smach.StateMachine.add('SETUP_ROBOT_AFTER_PLACEMENT', robot_setup_sm, transitions={'setup_succeeded': 'success'})
    object_detection_sm = get_object_detection_sm()
    return grasp_handover_sm, grasp_place_sm, object_detection_sm

class LLM_Wrapper:
    def __init__(self):
        rospy.loginfo('Setting up LLM Wrapper')
        self.action_server = actionlib.SimpleActionServer('/robot_llm', RobotLLMAction, self.execute, auto_start = False)

        rospy.loginfo('Setting up state machines')
        grasp_handover_sm, grasp_place_sm, object_detection_sm = create_statemachine()
        self.behaviours = {'handover': grasp_handover_sm, 'placement': grasp_place_sm, 'detection': object_detection_sm}

        self.action_server.start()
        rospy.loginfo('LLM Wrapper is ready')

    def execute(self, goal):
        rospy.logerr('Executing goal: ' + str(goal))
        behaviour = goal.task
        sm = self.behaviours[behaviour]
        sm.userdata.object_to_grasp = goal.object_name
        rospy.loginfo('Executing behaviour: ' + behaviour)
        rospy.loginfo('Userdata: ' + str(sm.userdata))
        rospy.loginfo('userdata object_to_grasp: ' + sm.userdata.object_to_grasp)
        ud = smach.UserData()
        result = self.behaviours[behaviour].execute(parent_ud=ud)
        ud['status'] = result
        ud_dict = {}
        for key in ud.keys():
            ud_dict[key] = ud[key]
        json_str = json.dumps(ud_dict)
        rospy.loginfo('Behaviour executed. Returning Control')
        res = RobotLLMResult(result=json_str)
        #TODO pass aborted when object not found
        self.action_server.set_succeeded(res)



if __name__ == '__main__':
    rospy.init_node('llm_wrapper')

    llm_wrapper = LLM_Wrapper()
    goal = {'behaviour': 'grasp_handover', 'object_name': '003_cracker_box'}
    # llm_wrapper.execute(goal)

    while not rospy.is_shutdown():
        rospy.spin()
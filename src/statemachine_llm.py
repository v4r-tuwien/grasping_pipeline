#!/usr/bin/env python3
import rospy
import actionlib
import smach
import smach_ros
from grasping_pipeline_msgs.msg import FindGrasppointAction
from states.robot_control import GoToWaypoint
from handover.msg import HandoverAction
from robot_llm.msg import RobotLLMAction, RobotLLMResult
from states.statemachine_components import get_robot_setup_sm, get_execute_grasp_sm, get_placement_sm    

def create_statemachine(do_handover=True):
    table_waypoint = GoToWaypoint(0.25, 0.41, 0)
    robot_setup_sm = get_robot_setup_sm(table_waypoint)
    execute_grasp_sm = get_execute_grasp_sm(table_waypoint)
    placement_sm = get_placement_sm()
    
    grasp_handover_sm = smach.StateMachine(outcomes=['end_behaviour', 'object_not_found'])
    with grasp_handover_sm:
        smach.StateMachine.add('SETUP_ROBOT', robot_setup_sm, transitions={'setup_succeeded': 'FIND_GRASP'})

        find_grasp_actionstate = smach_ros.SimpleActionState('find_grasppoint',
                                                             FindGrasppointAction, 
                                                             goal_slots=['object_to_grasp'],
                                                             result_slots=['grasp_poses', 'grasp_object_bb', 'grasp_object_name'])
        smach.StateMachine.add('FIND_GRASP', find_grasp_actionstate, transitions={'succeeded': 'EXECUTE_GRASP',
            'aborted': 'object_not_found', 'preempted': 'object_not_found'})

        smach.StateMachine.add('EXECUTE_GRASP', execute_grasp_sm, transitions={
                        'end_execute_grasp': 'HANDOVER', 'failed_to_grasp': 'SETUP_ROBOT'})
        smach.StateMachine.add('HANDOVER', smach_ros.SimpleActionState('/handover', HandoverAction),
                            transitions={'succeeded': 'SETUP_ROBOT_AFTER_HANDOVER',
                                        'preempted': 'SETUP_ROBOT_AFTER_HANDOVER',
                                        'aborted': 'SETUP_ROBOT_AFTER_HANDOVER'})
        smach.StateMachine.add('SETUP_ROBOT_AFTER_HANDOVER', robot_setup_sm, transitions={'setup_succeeded': 'end_behaviour'})

    grasp_place_sm = smach.StateMachine(outcomes=['end_behaviour', 'object_not_found'])
    with grasp_place_sm:
            smach.StateMachine.add('SETUP_ROBOT', robot_setup_sm, transitions={'setup_succeeded': 'FIND_GRASP'})

            find_grasp_actionstate = smach_ros.SimpleActionState('find_grasppoint', 
                                                                 FindGrasppointAction, 
                                                                 goal_slots=['object_to_grasp'],
                                                                 result_slots=['grasp_poses', 'grasp_object_bb', 'grasp_object_name'])
            smach.StateMachine.add('FIND_GRASP', find_grasp_actionstate, transitions={'succeeded': 'EXECUTE_GRASP',
                'aborted': 'object_not_found', 'preempted': 'object_not_found'})

            smach.StateMachine.add('EXECUTE_GRASP', execute_grasp_sm, transitions={
                            'end_execute_grasp': 'PLACE_OBJECT', 'failed_to_grasp': 'SETUP_ROBOT'})
            smach.StateMachine.add('PLACE_OBJECT', placement_sm, transitions={'end_placement': 'SETUP_ROBOT_AFTER_PLACEMENT', 'failed_to_place': 'PLACE_OBJECT'})
            smach.StateMachine.add('SETUP_ROBOT_AFTER_PLACEMENT', robot_setup_sm, transitions={'setup_succeeded': 'end_behaviour'})
    return grasp_handover_sm, grasp_place_sm

class LLM_Wrapper:
    def __init__(self):
        rospy.loginfo('Setting up LLM Wrapper')
        self.action_server = actionlib.SimpleActionServer('/robot_llm', RobotLLMAction, self.execute, auto_start = False)

        rospy.loginfo('Setting up state machines')
        grasp_handover_sm, grasp_place_sm = create_statemachine()
        self.behaviours = {'handover': grasp_handover_sm, 'placement': grasp_place_sm}

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

        result = self.behaviours[behaviour].execute()
        rospy.loginfo('Behaviour executed. Returning Control')
        res = RobotLLMResult(result=result)
        #TODO pass aborted when object not found
        self.action_server.set_succeeded(res)



if __name__ == '__main__':
    rospy.init_node('llm_wrapper')

    llm_wrapper = LLM_Wrapper()
    goal = {'behaviour': 'grasp_handover', 'object_name': '003_cracker_box'}
    # llm_wrapper.execute(goal)

    while not rospy.is_shutdown():
        rospy.spin()
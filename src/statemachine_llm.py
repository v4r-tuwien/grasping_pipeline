#!/usr/bin/env python3
import rospy
import actionlib
import smach
import smach_ros
from collision_environment import CollisionEnvironment
from states.robot_control import GoToNeutral, OpenGripper, GoToWaypoint, GoToAndLookAtPlacementArea, GoBack
from states.find_table_planes import FindTablePlanes
from grasping_pipeline_msgs.msg import FindGrasppointAction, ExecuteGraspAction
from handover.msg import HandoverAction
from grasping_pipeline_msgs.msg import PlaceAction
LLMAction = PlaceAction

def get_robot_setup_sm():
    seq = smach.Sequence(outcomes=['setup_succeeded'], connector_outcome='succeeded', input_keys=[], output_keys=[])
    table_waypoint = GoToWaypoint(0.25, 0.41, 0)
    with seq:
        smach.Sequence.add('GO_TO_TABLE', table_waypoint, transitions={'aborted': 'GO_TO_TABLE'})
        smach.Sequence.add('GO_TO_NEUTRAL', GoToNeutral())
        smach.Sequence.add('OPEN_GRIPPER', OpenGripper(), transitions={'succeeded': 'setup_succeeded'})
    return seq

    
def get_execute_grasp_sm():
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

        smach.Sequence.add('GO_BACK_TO_TABLE', table_waypoint, 
                            transitions={'succeeded': 'end_execute_grasp', 'aborted': 'GO_BACK_TO_TABLE'})
    return seq

def get_placement_sm():
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
    

def create_statemachine(do_handover=True):
    robot_setup_sm = get_robot_setup_sm()
    execute_grasp_sm = get_execute_grasp_sm()
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
                            transitions={'succeeded': 'end_behaviour',
                                        'preempted': 'end_behaviour',
                                        'aborted': 'end_behaviour'})

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
            smach.StateMachine.add('PLACE_OBJECT', placement_sm, transitions={'end_placement': 'end_behaviour', 'failed_to_place': 'SETUP_ROBOT'})
    return grasp_handover_sm, grasp_place_sm

class LLM_Wrapper:
    def __init__(self):
        rospy.loginfo('Setting up LLM Wrapper')
        self.action_server = actionlib.SimpleActionServer('/llm', LLMAction, self.execute, auto_start = False)

        rospy.loginfo('Setting up state machines')
        grasp_handover_sm, grasp_place_sm = create_statemachine()
        self.behaviours = {'grasp_handover': grasp_handover_sm, 'grasp_place': grasp_place_sm}

        self.action_server.start()
        rospy.loginfo('LLM Wrapper is ready')

    def execute(self, goal):
        rospy.logerr('Executing goal: ' + str(goal))
        behaviour = goal['behaviour']
        sm = self.behaviours[behaviour]
        sm.userdata.object_to_grasp = goal['object_name']
        rospy.loginfo('Executing behaviour: ' + behaviour)
        rospy.loginfo('Userdata: ' + str(sm.userdata))
        rospy.loginfo('userdata object_to_grasp: ' + sm.userdata.object_to_grasp)

        self.behaviours[behaviour].execute()
        rospy.loginfo('Behaviour executed. Returning Control')


if __name__ == '__main__':
    rospy.init_node('llm_wrapper')

    llm_wrapper = LLM_Wrapper()
    goal = {'behaviour': 'grasp_handover', 'object_name': '003_cracker_box'}
    llm_wrapper.execute(goal)

    while not rospy.is_shutdown():
        rospy.spin()
#! /usr/bin/env python3
import rclpy
from rclpy.executors import SingleThreadedExecutor, MultiThreadedExecutor
import threading
import yasmin
from grasping_pipeline.statemachine_components import create_goal_cb, get_robot_setup_sm, get_execute_grasp_sm, get_placement_sm, get_find_grasp_sm, get_object_detector_sm, get_pose_estimator_sm
from grasping_pipeline.userinput import UserInput
from grasping_pipeline.robot_control import GoToWaypoint, GoBack, GoToNeutral, CheckTopGrasp
from grasping_pipeline_msgs.action import Handover
from grasping_pipeline.check_table_clean import CheckTableClean, RemoveNonTableObjects
from grasping_pipeline.find_table_planes import FindTablePlanes
import yasmin_ros
from grasping_pipeline.moveit_wrapper import MoveitWrapper
from v4r_util.tf2 import TF2Wrapper

def create_statemachine(node, do_handover=True):
    sm = yasmin.StateMachine(outcomes=['end'])    
    tf_wrapper = TF2Wrapper(node)
    moveit_wrapper = MoveitWrapper(tf_wrapper, node)

    # table_waypoint = GoToWaypoint(node, 0.53, 0.42, 0, "map")
    table_waypoint = GoToWaypoint(node, 0.33, 0.42, 0, "map")
    setup_sm = get_robot_setup_sm(table_waypoint, node)
    find_grasp_sm = get_find_grasp_sm(node)
    execute_grasp_sm = get_execute_grasp_sm(table_waypoint, node, moveit_wrapper)
    placement_sm = get_placement_sm(node)
    single_grasp_sm = get_single_grasp_sm(table_waypoint, find_grasp_sm, execute_grasp_sm, placement_sm, node)
    clear_table_sm = get_clear_table_sm(table_waypoint, get_object_detector_sm(node), get_pose_estimator_sm(node), execute_grasp_sm, placement_sm, setup_sm, node)
    sm.add_state('SETUP', setup_sm, transitions={'setup_succeeded': 'DECIDE_PROCEDURE'})
        
    map = {'g': ['single_grasp', 'single grasp'], 't': ['clear_table', 'clear table']}
    sm.add_state('DECIDE_PROCEDURE', UserInput(node, map), transitions={'single_grasp': 'SINGLE_GRASP', 'clear_table': 'CLEAR_TABLE'})
    sm.add_state('SINGLE_GRASP', single_grasp_sm, transitions={'succeeded': 'SETUP', 'failed': 'SETUP'})
    sm.add_state('CLEAR_TABLE', clear_table_sm, transitions={'succeeded': 'SETUP'})

    return sm

def get_clear_table_sm(table_waypoint, object_detector_sm, pose_estimator_sm, execute_grasp_sm, placement_sm, setup_sm, node):
    sm = yasmin.StateMachine(outcomes=['succeeded'])

    sm.add_state('SETUP', setup_sm, transitions={'setup_succeeded': 'DETECT_OBJECTS'})
    sm.add_state('DETECT_OBJECTS', object_detector_sm, transitions={'failed': 'SETUP', 'succeeded': 'GET_TABLE_PLANES'})
    sm.add_state('GET_TABLE_PLANES', FindTablePlanes(node, enlarge_table_bb_to_floor=False), transitions={'succeeded': 'REMOVE_NON_TABLE_OBJECTS'})
    sm.add_state('REMOVE_NON_TABLE_OBJECTS', RemoveNonTableObjects(node), transitions={'succeeded': 'CHECK_TABLE_CLEAN', 'failed': 'SETUP'})
    sm.add_state('CHECK_TABLE_CLEAN', CheckTableClean(), transitions={'clean': 'succeeded', 'not_clean': 'POSE_ESTIMATION'})
    sm.add_state('POSE_ESTIMATION', pose_estimator_sm, transitions={'failed': 'SETUP', 'succeeded': 'EXECUTE_GRASP'})
    sm.add_state('EXECUTE_GRASP', execute_grasp_sm, transitions={
            'end_execute_grasp': 'CHECK_TOP_GRASP', 'failed_to_grasp': 'SETUP'})
    
    #TODO test with combination of placement + handover, instead of only handover
    sm.add_state('CHECK_TOP_GRASP', CheckTopGrasp(node), transitions={'top_grasp': 'HANDOVER', 'not_top_grasp': 'HANDOVER'})
    handover_action = yasmin_ros.ActionState(Handover, '/handover', create_goal_handler=create_goal_cb(Handover, ['object_name']))
    sm.add_state(
        'HANDOVER', 
        handover_action,
        transitions={'succeeded': 'SETUP','aborted': 'SETUP'},
        remappings={'object_name':'grasp_object_name'}
    )
        
    # sm.add_state('PLACEMENT', placement_sm, transitions={'end_placement': 'RETREAT_AFTER_PLACEMENT', 'failed_to_place': 'GO_BACK_TO_TABLE'})
        
    # sm.add_state('RETREAT_AFTER_PLACEMENT', GoBack(node,0.2), transitions={'succeeded': 'GO_TO_NEUTRAL_AFTER_PLACEMENT', 'aborted': 'GO_TO_NEUTRAL_AFTER_PLACEMENT'})
    # sm.add_state('GO_TO_NEUTRAL_AFTER_PLACEMENT', GoToNeutral(node), transitions={'succeeded': 'SETUP'})

    # sm.add_state('GO_BACK_TO_TABLE', table_waypoint, transitions={'succeeded': 'HANDOVER', 'aborted': 'GO_BACK_TO_TABLE'})
    
    return sm

def get_single_grasp_sm(table_waypoint, find_grasp_sm, execute_grasp_sm, placement_sm, node):
    sm = yasmin.StateMachine(outcomes=['failed', 'succeeded'])
    sm.add_state('FIND_GRASP', find_grasp_sm, transitions={
            'end_find_grasp': 'EXECUTE_GRASP_USERINPUT', 'failed': 'failed'})

    map = {'g': ['succeeded', 'grasp object'],
            't': ['retry', 'try again']}
    sm.add_state('EXECUTE_GRASP_USERINPUT', UserInput(node, map), transitions={'retry': 'failed', 'succeeded': 'EXECUTE_GRASP'})
        
    sm.add_state('EXECUTE_GRASP', execute_grasp_sm, transitions={
            'end_execute_grasp': 'CHECK_TOP_GRASP', 'failed_to_grasp': 'failed'})
        
    sm.add_state('CHECK_TOP_GRASP', CheckTopGrasp(node), transitions={'top_grasp': 'HANDOVER', 'not_top_grasp': 'AFTER_GRASP_USERINPUT'})
        
    map = {'p': ['placement', 'place object'],
            'h': ['handover', 'handover object']}
    sm.add_state('AFTER_GRASP_USERINPUT', UserInput(node, map), transitions={'placement': 'PLACEMENT', 'handover': 'HANDOVER'})

    handover_action = yasmin_ros.ActionState(Handover, '/handover', create_goal_handler=create_goal_cb(Handover, ['object_name']))
    sm.add_state(
        'HANDOVER',
        handover_action,
        transitions={'succeeded': 'succeeded','aborted': 'succeeded',}, 
        remappings={'object_name':'grasp_object_name'}
    )
        
    sm.add_state('PLACEMENT', placement_sm, transitions={'end_placement': 'RETREAT_AFTER_PLACEMENT', 'failed_to_place': 'GO_BACK_TO_TABLE'})
        
    sm.add_state('RETREAT_AFTER_PLACEMENT', GoBack(node, 0.2), transitions={'succeeded': 'GO_TO_NEUTRAL_AFTER_PLACEMENT', 'aborted': 'GO_TO_NEUTRAL_AFTER_PLACEMENT'})
    sm.add_state('GO_TO_NEUTRAL_AFTER_PLACEMENT', GoToNeutral(node), transitions={'succeeded': 'succeeded'})

    sm.add_state('GO_BACK_TO_TABLE', table_waypoint, transitions={'succeeded': 'HANDOVER', 'aborted': 'GO_BACK_TO_TABLE'})
    return sm

def main():

    rclpy.init()

    node = rclpy.create_node("sasha_statemachine", automatically_declare_parameters_from_overrides=True)
    # executor = MultiThreadedExecutor()
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    # Run executor in background thread
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    blackboard = yasmin.Blackboard()

    sm = create_statemachine(node)

    outcome = sm(blackboard)

    node.get_logger().info(f"State machine finished with outcome: {outcome}")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

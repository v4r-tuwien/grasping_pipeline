#! /usr/bin/env python3
import rospy
import smach
import smach_ros
from collision_environment import CollisionEnvironment
from states.userinput import UserInput
from states.robot_control import GoToNeutral, GoBack, OpenGripper, GoToWaypoint, MoveToJointPositions
from states.find_table_planes import FindTablePlanes
from grasping_pipeline_msgs.msg import FindGrasppointAction, ExecuteGraspAction
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Pose
from handover.msg import HandoverAction
from states.placement_area_detector import PlacementAreaDetector
from placement.msg import PlacementPlaceAndMoveAwayAction
from grasping_pipeline_msgs.msg import PlaceAction


def create_statemachine(enable_userinput=True, do_handover=True):
    seq = smach.Sequence(outcomes=['end'], connector_outcome='succeeded')

    table_waypoint = GoToWaypoint(0.25, 0.41, 0)
    shelf_waypoint = GoToWaypoint(x=0.7, y=1.1, phi_degree=90.0)
    with seq:
        smach.Sequence.add('GO_TO_TABLE', table_waypoint, transitions={'aborted': 'GO_TO_TABLE'})
        smach.Sequence.add('GO_TO_NEUTRAL', GoToNeutral())
        smach.Sequence.add('OPEN_GRIPPER', OpenGripper())

        if enable_userinput:
            abort_state = 'FIND_GRASP_USERINPUT'
            map = {'f': ['succeeded', 'find grasp point'],
                   'r': ['reset', 'reset state machine']}
            smach.Sequence.add('FIND_GRASP_USERINPUT', UserInput(
                map), transitions={'reset': 'GO_TO_TABLE'})
        else:
            abort_state = 'FIND_GRASP'

        find_grasp_actionstate = smach_ros.SimpleActionState('find_grasppoint', FindGrasppointAction, result_slots=['grasp_poses', 'grasp_object_bb'])
        smach.Sequence.add('FIND_GRASP', find_grasp_actionstate, transitions={
            'aborted': abort_state, 'preempted': abort_state})

        if enable_userinput:
            map = {'g': ['succeeded', 'grasp object'],
                   't': ['retry', 'try again']}
            smach.Sequence.add('EXECUTE_GRASP_USERINPUT', UserInput(
                map), transitions={'retry': 'FIND_GRASP'})
        smach.Sequence.add('FIND_TABLE_PLANES', FindTablePlanes())

        smach.Sequence.add('ADD_COLLISION_OBJECTS', CollisionEnvironment())

        execute_grasp_actionstate = smach_ros.SimpleActionState(
            'execute_grasp', ExecuteGraspAction, goal_slots=['grasp_poses', 'grasp_object_name', 'table_plane_equations'], result_slots=['placement_surface_to_wrist']) 

        smach.Sequence.add('EXECUTE_GRASP', execute_grasp_actionstate, transitions={
                           'aborted': 'GO_TO_NEUTRAL', 'preempted': 'GO_TO_NEUTRAL'})
        smach.Sequence.add('RETREAT_AFTER_GRASP', table_waypoint, 
                           transitions={'aborted': 'GO_TO_NEUTRAL'})
        smach.Sequence.add('GO_TO_NEUTRAL_AFTER_GRASP', GoToNeutral())

        if do_handover:
            smach.Sequence.add('HANDOVER', smach_ros.SimpleActionState('/handover', HandoverAction),
                               transitions={'succeeded': 'GO_TO_NEUTRAL',
                                            'preempted': 'GO_TO_NEUTRAL',
                                            'aborted': 'GO_TO_NEUTRAL'})
        else:
            # smach.Sequence.add('MOVE_HAND_AWAY_FROM_CAM', MoveToJointPositions({'arm_roll_joint': 3.1415 / 2}))
            # make_sasha_tall_joints = {
            #     'arm_lift_joint': 0.6,  
            #     'head_pan_joint':0, 
            #     'head_tilt_joint':-0.7,
            #     'arm_flex_joint':-2.6}
            # smach.Sequence.add('MAKE_SASHA_TALL', MoveToJointPositions(make_sasha_tall_joints))
            # smach.Sequence.add('GO_TO_SHELF', shelf_waypoint, transitions={'aborted': 'GO_TO_NEUTRAL'})
            #TODO save: list of tables with: center point of plane, waypoint for robot and sasha joints for table
            # then => drive to waypoint and change joints depending on which table the object should be placed on
            # then detect table plane, use those detected bb to check whether center point is inside bb => if yes => found placement table
            # then in placement area detector: use base_link as frame_id, transform detected table bb to base_link and align,
            # then use prealigned object (aligned at timepoint of grasping) => use those x/y/z coordinates for size
            if enable_userinput:
                map = {'g': ['succeeded', 'place object']}
                smach.Sequence.add('PLACE_OBJECT_USER_INPUT', UserInput(
                    map))
            smach.Sequence.add('FIND_TABLE_PLANES_PLACEMENT', FindTablePlanes())
            smach.Sequence.add('DETECT_PLACEMENT_AREA', PlacementAreaDetector(
            ), transitions={'aborted': 'GO_TO_NEUTRAL'})
            smach.Sequence.add('PLACEMENT_PLACE',
                            smach_ros.SimpleActionState('place_object', PlaceAction,
                                                        goal_slots=[
                                                            'placement_areas', 'table_plane_equations', 'table_bbs', 'placement_surface_to_wrist']),
                            transitions={'succeeded': 'GO_TO_NEUTRAL',
                                        'aborted': 'DETECT_PLACEMENT_AREA',
                                        'preempted': 'DETECT_PLACEMENT_AREA'})
    return seq


if __name__ == '__main__':
    rospy.init_node('sasha_statemachine')
    sm = create_statemachine(enable_userinput=True, do_handover=False)
    # sm = test_placement()


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

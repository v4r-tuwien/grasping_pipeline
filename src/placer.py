#!/usr/bin/python3

import rospy
import actionlib
import moveit_commander
import tf2_ros
import sys

import numpy as np

from hsrb_interface import Robot, geometry
from placement.msg import PlaneParams, ObjectDimension, CollisionObject, PlacementPlaceAndMoveAwayAction, PlacementPlaceAndMoveAwayResult
from grasping_pipeline.msg import PlaceAction, PlaceResult
from visualization_msgs.msg import Marker
from std_srvs.srv import Empty
from moveit_msgs.msg import PlanningScene, DisplayTrajectory, MotionPlanResponse
from v4r_util.util import transform_pose, transformPoseFormat


class PlaceAndMoveAway():
    def __init__(self):
        # robot control
        self.robot = Robot()
        self.whole_body = self.robot.try_get('whole_body')
        self.gripper = self.robot.get('gripper')
        self.omni = self.robot.get('omni_base')

        # moveit
        moveit_commander.roscpp_initialize(sys.argv)
        self.robotCommander = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        rospy.sleep(2.0)
        self.group_name = 'whole_body'
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)

        self.planning_frame = self.move_group.get_planning_frame()  # odom
        self.eef_link = self.move_group.get_end_effector_link()  # hand_palm_link

        # self.move_group.set_workspace()
        self.move_group.allow_replanning(True)
        self.move_group.set_start_state_to_current_state()
        self.move_group.set_num_planning_attempts(3)
        self.move_group.set_goal_position_tolerance(0.02)

        # server init
        self.server = actionlib.SimpleActionServer(
            'Placer', PlaceAction, self.execute, False)

        # tf buffer for tf transform
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self._pubPlanningScene = rospy.Publisher(
            'planning_scene', PlanningScene, queue_size=100)
        rospy.wait_for_service('/get_planning_scene', 10.0)

        self.global_frame = rospy.get_param("/global_frame")

        self.server.start()

    def execute(self, goal):
        rospy.loginfo('Execute ActionServer Placer')

        self.result = PlaceResult()
        self.frame = goal.grasped_pose.header.frame_id
        orientation = goal.grasped_pose.pose.orientation

        placementAreas = goal.placement_area

        is_finished = False

        counter = 0

        self.move_group.set_support_surface_name('TablePlane0')

        # try poses and execute if possible
        pose_list = []
        pub = rospy.Publisher(
            '/placement/placement_marker', Marker, queue_size=10, latch=True)
        for area in placementAreas:
            counter += 1
            area.center.position.z += 0.02
            goal_pose = transform_pose(
                self.planning_frame, self.global_frame, area.center)
            goal_pose.pose.orientation = orientation
            pose_list.append(goal_pose)
            marker = Marker()
            marker.header.frame_id = self.frame
            marker.header.stamp = rospy.Time()
            marker.id = counter
            marker.pose = goal_pose.pose
            marker.type = Marker.ARROW
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0
            marker.scale.x = 0.1
            marker.scale.y = 0.05
            marker.scale.z = 0.01

            pub.publish(marker)

        place = self.move_group.place('placement_object', pose_list)
        if place:
            is_finished = True

            #check and correct
            #pose_eef = transformPoseFormat(self.whole_body.get_end_effector_pose(), "tuple")
            #position_tolerance = pose_eef.position.z - pose_list[index].position.z
            # if position_tolerance > 0.02:
            #    pose_eef.position.z -= position_tolerance
            #    self.whole_body.move_end_effector_pose(transformPoseFormat(pose_eef,"pose"))
        else:
            self.move_group.stop()
            self.move_group.clear_pose_targets()

        rospy.sleep(3)

        # execute the moveit planned trajectory backwards
        if is_finished:
            # open gripper
            distance = self.gripper.get_distance()
            self.gripper.set_distance(distance + 0.04)

            # get planned path from placement
            self.msg = rospy.wait_for_message(
                "/move_group/display_planned_path", DisplayTrajectory)

            # move back
            pose_rob = transformPoseFormat(self.omni.get_pose(), "tuple")
            pose_rob_trans = transform_pose(
                self.eef_link, self.global_frame, pose_rob)
            pose_rob_trans.pose.position.z -= 0.15
            # if np.abs(position_tolerance) > 0.02:
            #     pose_eef = transformPoseFormat(self.whole_body.get_end_effector_pose(), "tuple")
            #     pose_eef.position.z += position_tolerance
            #     self.whole_body.move_end_effector_pose(transformPoseFormat(pose_eef,"pose"))

            try:
                self.omni.go_pose(transformPoseFormat(
                    pose_rob_trans.pose, "pose"), 100.0, self.eef_link)
            except:
                pass

            # calculate reverse trajectory
            trajectory = self.msg.trajectory[0]
            reverse_plan = MotionPlanResponse()
            reverse_plan.trajectory.joint_trajectory.header.stamp = rospy.Time.now()
            reverse_plan.trajectory.joint_trajectory.header.frame_id = trajectory.joint_trajectory.header.frame_id
            reverse_plan.trajectory.joint_trajectory.joint_names = trajectory.joint_trajectory.joint_names

            trajectory_step = len(trajectory.joint_trajectory.points) - 1
            while (trajectory_step >= 0):
                reverse_plan.trajectory.joint_trajectory.points.append(
                    trajectory.joint_trajectory.points[trajectory_step])
                reverse_plan.trajectory.multi_dof_joint_trajectory.points.append(
                    trajectory.multi_dof_joint_trajectory.points[trajectory_step])
                trajectory_step -= 1

            plan = self.move_group.retime_trajectory(
                self.robotCommander.get_current_state(), reverse_plan.trajectory, 0.8)

            self.move_group.execute(plan, wait=True)

            self.move_group.stop()
            self.move_group.clear_pose_targets()

            self.server.set_succeeded(self.result)
        else:
            rospy.logerr("Motion failed")
            self.server.set_aborted(self.result)


if __name__ == '__main__':
    rospy.init_node('Execute_ActionServer_PlaceAndMoveAway')
    server = PlaceAndMoveAway()
    rospy.spin()

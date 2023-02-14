#! /usr/bin/env python3

import rospy
import actionlib

import moveit_commander
import sys
import geometry_msgs.msg
import tf.transformations
import tf


from grasping_pipeline.msg import ExecuteGraspAction, ExecuteGraspActionResult
from hsrb_interface import Robot, geometry
from visualization_msgs.msg import Marker
from math import pi
import numpy as np

from placement.msg import *
from geometry_msgs.msg import Pose


class ExecuteGraspServer:
    def __init__(self):
        self.robot = Robot()
        self.whole_body = self.robot.try_get('whole_body')
        self.gripper = self.robot.get('gripper')
        self.omni_base = self.robot.try_get('omni_base')
        self.tf = tf.TransformListener()

        self.move_group = self.moveit_init()
        self.server = actionlib.SimpleActionServer(
            'execute_grasp', ExecuteGraspAction, self.execute, False)

        self.server.start()

    def moveit_init(self):
        """ Initializes MoveIt, sets workspace and creates collision environment

        Returns:
            MoveGroupCommander -- MoveIt interface
        """
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot_cmd = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        rospy.sleep(1)
        self.group_name = "whole_body"
        move_group = moveit_commander.MoveGroupCommander(
            self.group_name, wait_for_servers=10.0)
        eef_link = move_group.get_end_effector_link()
        self.group_names = self.robot_cmd.get_group_names()

        t = self.tf.getLatestCommonTime('/odom', '/base_link')
        transform = self.tf.lookupTransform('/odom', '/base_link', t)
        move_group.set_workspace(
            (-1.5 + transform[0][0], -1.5 + transform[0][1], -1, 1.5 + transform[0][0], 1.5 + transform[0][1], 3))

        move_group.allow_replanning(True)
        self.scene.remove_attached_object(eef_link)
        self.scene.remove_world_object()
        move_group.clear_pose_targets()
        move_group.set_planner_id('RRTstarkConfigDefault')

        return move_group

    def execute(self, goal):
        res = ExecuteGraspActionResult()

        plan_found = False
        for grasp_pose in goal.grasp_poses:
            if grasp_pose.header.frame_id == "":
                rospy.loginfo('Not a valid goal. Aborted execution!')
                self.server.set_aborted()
                return
            initial_grasp_pose = grasp_pose

            # add safety_distance to grasp_pose
            q = [grasp_pose.pose.orientation.x, grasp_pose.pose.orientation.y,
                 grasp_pose.pose.orientation.z, grasp_pose.pose.orientation.w]

            approach_vector = qv_mult(q, [0, 0, -1])
            print(approach_vector)
            safety_distance = + \
                rospy.get_param("/safety_distance", default=0.08)
            grasp_pose.pose.position.x = grasp_pose.pose.position.x + \
                safety_distance * approach_vector[0]
            grasp_pose.pose.position.y = grasp_pose.pose.position.y + \
                safety_distance * approach_vector[1]
            grasp_pose.pose.position.z = grasp_pose.pose.position.z + \
                safety_distance * approach_vector[2]

            t = self.tf.getLatestCommonTime(
                '/odom', grasp_pose.header.frame_id)
            grasp_pose.header.stamp = t
            grasp_pose = self.tf.transformPose('/odom', grasp_pose)
            self.add_marker(grasp_pose)
            self.move_group.set_pose_target(grasp_pose)
            plan = self.move_group.plan()[1]
            if len(plan.joint_trajectory.points) > 0:
                plan_found = True
                break

        # abort if no plan is found
        if not plan_found:
            rospy.logerr('no grasp found')
            self.move_group.stop()
            self.move_group.clear_pose_targets()
            res.result.success = False
            self.server.set_aborted(res.result)
            return

        self.move_group.go(wait=True)
        rospy.sleep(0.5)

        self.whole_body.move_end_effector_by_line((0, 0, 1), safety_distance)

        self.gripper.apply_force(0.30)

        # move 5cm in z direction
        pose_vec, pose_quat = self.whole_body.get_end_effector_pose(
            'base_link')
        new_pose_vec = geometry.Vector3(
            pose_vec.x, pose_vec.y, pose_vec.z + 0.05)
        new_pose = geometry.Pose(new_pose_vec, pose_quat)
        self.whole_body.move_end_effector_pose(new_pose)

        self.whole_body.move_end_effector_by_line((0, 0, 1), -safety_distance)
        rospy.sleep(1)
        self.omni_base.go_rel(-0.2, 0.0, 0.0, 10)
        self.whole_body.move_to_neutral()

        # check if object is in gripper
        self.gripper.apply_force(0.50)
        if self.gripper.get_distance() > -0.004:
            res.result.success = True
            res.result.grasped_pose = initial_grasp_pose
            self.server.set_succeeded(res.result)

        else:
            res.result.success = False
            rospy.logerr('grasping failed')
            self.gripper.command(1.0)
            self.server.set_aborted()

    def add_marker(self, pose_goal):
        """ publishes a grasp marker to /grasping_pipeline/grasp_marker

        Arguments:
            pose_goal {geometry_msgs.msg.PoseStamped} -- pose for the grasp marker
        """
        br = tf.TransformBroadcaster()
        br.sendTransform((pose_goal.pose.position.x, pose_goal.pose.position.y, pose_goal.pose.position.z),
                         [pose_goal.pose.orientation.x, pose_goal.pose.orientation.y,
                             pose_goal.pose.orientation.z, pose_goal.pose.orientation.w],
                         rospy.Time.now(),
                         'grasp_pose_execute',
                         pose_goal.header.frame_id)

        marker_pub = rospy.Publisher(
            '/grasp_marker_2', Marker, queue_size=10, latch=True)
        marker = Marker()
        marker.header.frame_id = pose_goal.header.frame_id
        marker.header.stamp = rospy.Time()
        marker.ns = 'grasp_marker'
        marker.id = 0
        marker.type = 0
        marker.action = 0

        q2 = [pose_goal.pose.orientation.w, pose_goal.pose.orientation.x,
              pose_goal.pose.orientation.y, pose_goal.pose.orientation.z]
        q = tf.transformations.quaternion_about_axis(pi / 2, (0, 1, 0))
        q = tf.transformations.quaternion_multiply(q, q2)

        marker.pose.orientation.w = q[0]
        marker.pose.orientation.x = q[1]
        marker.pose.orientation.y = q[2]
        marker.pose.orientation.z = q[3]
        marker.pose.position.x = pose_goal.pose.position.x
        marker.pose.position.y = pose_goal.pose.position.y
        marker.pose.position.z = pose_goal.pose.position.z

        marker.scale.x = 0.1
        marker.scale.y = 0.05
        marker.scale.z = 0.01

        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker_pub.publish(marker)
        rospy.loginfo('grasp_marker')


def qv_mult(q, v):
    """ Rotating the vector v by quaternion q
    Arguments:
        q {list of float} -- Quaternion w,x,y,z
        v {list} -- Vector x,y,z

    Returns:
        numpy array -- rotated vector
    """
    rot_mat = tf.transformations.quaternion_matrix(q)[:3, :3]
    v = np.array(v)
    return rot_mat.dot(v)


if __name__ == '__main__':
    rospy.init_node('execute_grasp_server')
    server = ExecuteGraspServer()
    rospy.spin()

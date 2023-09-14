#! /usr/bin/env python3


import copy
import rospy
import actionlib

import moveit_commander
import sys
import geometry_msgs.msg
import tf.transformations
import tf


from grasping_pipeline_msgs.msg import ExecuteGraspAction, ExecuteGraspActionResult
from hsrb_interface import Robot, geometry
from v4r_util.tf2 import TF2Wrapper
from visualization_msgs.msg import Marker
from math import pi
import numpy as np
from libhsrmoveit import LibHSRMoveit

from placement.msg import *
from geometry_msgs.msg import Pose

def get_libraries():
    libtf = TF2Wrapper()
    rospy.logerr("got tf")
    libmoveit = LibHSRMoveit(libtf)
    rospy.logerr("got moveit")
    libs = {'tf': libtf, 'moveit': libmoveit}
    return libs

class ExecuteGraspServer:
    def __init__(self):
        self.lib = get_libraries() 
        
        self.server = actionlib.SimpleActionServer(
            'execute_grasp', ExecuteGraspAction, self.execute, False)
        self.server.start()
        rospy.logerr("init")

    def execute(self, goal):
        res = ExecuteGraspActionResult()
        # self.robot = Robot()
        # self.whole_body = self.robot.try_get('whole_body')
        #TODO get original robot pose to reset after succesful grasp or probably just move to waypoint
        
        rospy.logerr(goal.grasp_poses)
        for grasp_pose in goal.grasp_poses:
            # assumes static scene, i.e robot didn't move since grasp pose was found
            grasp_pose.header.stamp = rospy.Time.now()
            grasp_pose = self.lib['tf'].transform_pose('odom', grasp_pose)
            self.add_marker(grasp_pose, 3, 1, 0, 0)
            rospy.sleep(0.1)
            approach_pose = copy.deepcopy(grasp_pose)
            q = [grasp_pose.pose.orientation.x, grasp_pose.pose.orientation.y,
                 grasp_pose.pose.orientation.z, grasp_pose.pose.orientation.w]

            approach_vector = qv_mult(q, [0, 0, -1])

            safety_distance = 0.15
            approach_pose.pose.position.x = approach_pose.pose.position.x + \
                safety_distance * approach_vector[0] 
            approach_pose.pose.position.y = approach_pose.pose.position.y + \
                safety_distance * approach_vector[1] 
            approach_pose.pose.position.z = approach_pose.pose.position.z + \
                safety_distance * approach_vector[2]
            
            approach_pose2 = copy.deepcopy(approach_pose)
            approach_pose2.pose.position.x = approach_pose2.pose.position.x - \
                safety_distance * approach_vector[0] * 0.3
            approach_pose2.pose.position.y = approach_pose2.pose.position.y - \
                safety_distance * approach_vector[1] * 0.3
            approach_pose2.pose.position.z = approach_pose2.pose.position.z - \
                safety_distance * approach_vector[2] * 0.3
            self.add_marker(approach_pose2, 1, 0, 1, 0)

            success = self.lib['moveit'].whole_body_plan_and_go([approach_pose, approach_pose2])
            if not success:
                self.server.set_aborted()
                return 

            self.add_marker(grasp_pose, 2, 0, 0, 1)
            res = self.lib['moveit'].whole_body_IK_cartesian(
                [grasp_pose.pose.position.x],
                [grasp_pose.pose.position.y],
                [grasp_pose.pose.position.z],
                [grasp_pose.pose.orientation],
                eef_step = 0.01,
                fraction_th = 0.3,
                is_avoid_obstacle = False
            )
            rospy.sleep(0.1)
            self.lib['moveit'].gripper_grasp()
            #self.lib['moveit'].attach_object(goal.grasp_object_name)
            rospy.sleep(0.1)

            rospy.logerr(f"{res = } is call res")
            res = self.lib['moveit'].whole_body_IK_cartesian(
                [grasp_pose.pose.position.x],
                [grasp_pose.pose.position.y],
                [grasp_pose.pose.position.z + 0.05],
                [grasp_pose.pose.orientation],
                eef_step = 0.01,
                fraction_th = 0.3,
                is_avoid_obstacle = False
            )
            if not res:
                self.server.set_aborted()
                

            #TODO if all res succesful than do this and report success
            
            #TODO attach object
            # self.lib['moveit'].add_box()
            #TODO detach object
            #TODO remove those shitty lib dicts to get actual hints for coding
            
        self.server.set_succeeded(res)
        

    def add_marker(self, pose_goal, id=0, r=0, g=1, b=0):
        """ publishes a grasp marker to /grasping_pipeline/grasp_marker

        Arguments:
            pose_goal {geometry_msgs.msg.PoseStamped} -- pose for the grasp marker
        """
        rospy.logerr(f"{id = }, {r = }, {g = }, {b = }")
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
        marker.header.stamp = pose_goal.header.stamp
        marker.ns = 'grasp_marker'
        marker.id = id
        marker.type = Marker.ARROW
        marker.action = Marker.ADD

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
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
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

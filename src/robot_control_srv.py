#! /usr/bin/env python3

from math import pi
import numpy as np
from tf.transformations import quaternion_about_axis 
import rospy
import actionlib
from moveit_wrapper import MoveitWrapper
from v4r_util.tf2 import TF2Wrapper
from geometry_msgs.msg import PointStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from hsrb_interface import Robot
from hsrb_interface.exceptions import MobileBaseError
from grasping_pipeline_msgs.msg import BoundingBox3DStamped
from grasping_pipeline_msgs.srv import GoBack, GoBackResponse
from grasping_pipeline_msgs.srv import GoToWaypoint, GoToWaypointResponse
from grasping_pipeline_msgs.srv import MoveToJointPositions, MoveToJointPositionsResponse
from std_srvs.srv import SetBool, SetBoolResponse

class RobotControl():

    def __init__(self):
        rospy.init_node('robot_control_node')

        self.robot = Robot()
        self.whole_body = self.robot.try_get('whole_body')
        self.base = self.robot.try_get('omni_base')
        self.gripper = self.robot.try_get('gripper')

        self.move_client = actionlib.SimpleActionClient('/move_base/move', MoveBaseAction)

        self.neutral_srv = rospy.Service('/robot_control/go_to_neutral', SetBool, self.handle_go_to_neutral)
        self.go_back_srv = rospy.Service('/robot_control/go_back', GoBack, self.handle_go_back)
        self.go_to_waypoint_srv = rospy.Service('/robot_control/go_to_waypoint', GoToWaypoint, self.handle_go_to_waypoint)
        self.move_to_joint_positions_srv = rospy.Service('/robot_control/move_to_joint_positions', MoveToJointPositions, self.handle_move_to_joint_positions)
        self.open_gripper_srv = rospy.Service('/robot_control/open_gripper', SetBool, self.handle_open_gripper)

        rospy.loginfo("Robot Control Services are ready.")
        

    def handle_go_to_neutral(self, req):
        rospy.loginfo('Executing state GoToNeutral')
        joint_positions = {
            'arm_flex_joint': 0, 
            'arm_lift_joint': 0, 
            'arm_roll_joint': pi/2, 
            'head_pan_joint': 0, 
            'head_tilt_joint': -0.675, 
            'wrist_flex_joint': -pi/2, 
            'wrist_roll_joint': 0
            }
        self.whole_body.move_to_joint_positions(joint_positions)

        return SetBoolResponse(success=True)

    def handle_go_back(self, req):
        try:
            self.base.go_rel(-req.distance, 0, 0, timeout=20.0)
        except MobileBaseError as e:
            rospy.logerr(e)
            return GoBackResponse(success=False)
        return GoBackResponse(success=True)
    
    def handle_go_to_waypoint(self, req):
        move_goal = MoveBaseGoal()
        
        move_goal.target_pose.header.frame_id = req.frame_id
        move_goal.target_pose.pose.position.x = req.x
        move_goal.target_pose.pose.position.y = req.y
        quat = quaternion_about_axis(req.phi_deg * pi/180, (0, 0, 1))
        move_goal.target_pose.pose.orientation.x = quat[0]
        move_goal.target_pose.pose.orientation.y = quat[1]
        move_goal.target_pose.pose.orientation.z = quat[2]
        move_goal.target_pose.pose.orientation.w = quat[3]
        
        rospy.loginfo("Waiting for move client server!")
        finished = self.move_client.wait_for_server(rospy.Duration(req.timeout))
        
        if finished:
            self.move_client.send_goal(move_goal)
            rospy.loginfo("Waiting for result")
            finished = self.move_client.wait_for_result(rospy.Duration(req.timeout))

            if finished:
                # wait for robot to settle down
                rospy.sleep(1.0)
                return GoToWaypointResponse(success=True)
            else:
                rospy.logerr("Move server execution timed out!")
                self.move_client.cancel_all_goals()
                return GoToWaypointResponse(success=False)
        else:
            rospy.logerr("Could not connect to move server!")
            return GoToWaypointResponse(success=False)
    
    def handle_move_to_joint_positions(self, req):
        rospy.loginfo('Executing state MoveToJointPositions')
        joint_position_dict = {}
        for joint_name, position in zip(req.joint_names, req.joint_positions):
            joint_position_dict[joint_name] = position
        self.whole_body.move_to_joint_positions(joint_position_dict)

        return MoveToJointPositionsResponse(success=True)
    
    def handle_open_gripper(self, req):
        rospy.loginfo('Executing state OpenGripper')
        self.gripper.command(1.0)
        return SetBoolResponse(success=True)
    
if __name__ == "__main__":
    robot_control = RobotControl()
    rospy.spin()
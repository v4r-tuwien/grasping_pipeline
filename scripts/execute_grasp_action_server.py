#! /usr/bin/env python

import rospy
import actionlib

import moveit_commander
import moveit_msgs.msg
import sys
import geometry_msgs.msg

from grasping_pipeline.msg import ExecuteGraspAction, ExecuteGraspActionResult
from hsrb_interface import Robot

class ExecuteGraspServer:
  def __init__(self):
    self.server = actionlib.SimpleActionServer('execute_grasp', ExecuteGraspAction, self.execute, False)
    self.server.start()
    self.robot = Robot()
    self.whole_body = self.robot.try_get('whole_body')
    self.gripper = self.robot.get('gripper')
  
  def moveit_init(self):
    moveit_commander.roscpp_initialize(sys.argv)
    self.robot_cmd = moveit_commander.RobotCommander()
    self.scene = moveit_commander.PlanningSceneInterface()
    self.group_name = "whole_body"
    move_group = moveit_commander.MoveGroupCommander(self.group_name)
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                moveit_msgs.msg.DisplayTrajectory,
                                                queue_size=20)
    self.planning_frame = move_group.get_planning_frame()
    self.eef_link = move_group.get_end_effector_link()
    self.group_names = self.robot_cmd.get_group_names()   
    

    move_group.set_workspace((-3,-3,-1,3,3,5))
    move_group.allow_replanning(True)
    move_group.set_num_planning_attempts(5)
    return move_group

  def execute(self, goal):
    res = ExecuteGraspActionResult()

    if goal.grasp_pose.header.frame_id == "":
      rospy.loginfo('Not a valid goal. Aborted execution!')
      self.server.set_aborted()
      return
    #open gripper
    self.gripper.command(1.0)
    #add grasp_height and safety_distance to grasp_pose
    grasp_pose_1 = goal.grasp_pose
    grasp_pose_1.pose.position.z = goal.grasp_pose.pose.position.z + goal.safety_distance
    move_group = self.moveit_init()
    self.create_collision_environment()
    move_group.set_pose_target(grasp_pose_1)
    plan = move_group.plan()
    move_group.go()
    move_group.stop()
    move_group.clear_pose_targets()
    rospy.sleep(0.5)
    if len(plan.joint_trajectory.points) > 0 :
      if goal.safety_distance>goal.grasp_height:
        self.whole_body.move_end_effector_by_line((0,0,1), goal.safety_distance-goal.grasp_height)  
          
      self.gripper.apply_force(0.50)
      self.whole_body.move_end_effector_by_line((0,0,1), -goal.safety_distance)      
      self.whole_body.move_to_neutral()
      #self.gripper.command(1.0)
    else:
      print ('no grasp found')
      move_group.stop()
      move_group.clear_pose_targets()
      res.result.success = False
      self.server.set_succeeded(res.result)
      return

    if self.gripper.get_distance()>0.01:
      res.result.success = True
    else:
      res.result.success = False
    self.server.set_succeeded(res.result)

  def add_box(self, name, position_x = 0, position_y = 0, position_z = 0, size_x = 0.1, size_y = 0.1, size_z = 0.1):
    rospy.sleep(0.2)
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "map"
    box_pose.pose.orientation.w = 1.0
    box_pose.pose.position.x = position_x 
    box_pose.pose.position.y = position_y
    box_pose.pose.position.z = position_z
    box_name = name
    self.scene.add_box(box_name, box_pose, size=(size_x, size_y, size_z))

  def create_collision_environment(self):
    #self.add_box('table', 0.39, -0.765, 0.225, 0.55, 0.55, 0.35)
    #self.add_box('table', 0.39, -0.765, 0.225, 0.52, 0.52, 0.45)    
    self.add_box('cupboard', 1.4, 1.1, 1, 2.5, 1, 2)
    self.add_box('desk', -1.5, -0.9, 0.4, 0.8, 1.8, 0.8)
    self.add_box('drawer', 0.2, -2, 0.253, 0.8, 0.44, 0.56)
    self.add_box('cupboard_2', 2.08, -1.23, 0.6, 0.6, 1.9, 1.2)
    self.add_box('floor', 0,0,-0.1,5,5,0.1)

if __name__ == '__main__':
  rospy.init_node('execute_grasp_server')
  server = ExecuteGraspServer()
  rospy.spin()
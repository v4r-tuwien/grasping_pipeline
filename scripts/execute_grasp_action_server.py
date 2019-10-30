 #! /usr/bin/env python

import rospy
import actionlib

import moveit_commander
import moveit_msgs.msg
import sys

from grasping_pipeline.msg import ExecuteGraspAction, ExecuteGraspActionResult
import markus_grasp_test
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
    ## method 1 uses yolo and haf grasping  
    move_group = self.moveit_init()
    #markus.create_collision_environment()
    print (goal)
    move_group.set_pose_target(goal.grasp_pose)
    plan = move_group.plan()
    print (goal.grasp_pose.pose.position.z)

    if goal.grasp_pose.pose.position.z > 0.55:
      move_group.go()
      move_group.stop()
      move_group.clear_pose_targets()
      rospy.sleep(0.5)

      if len(plan.joint_trajectory.points) > 0 :
        self.gripper.apply_force(0.50)
        self.whole_body.move_end_effector_by_line((0,0,1), -0.07)
        self.whole_body.move_to_neutral()
        #self.gripper.command(1.0)

    else:
      print ('no grasp found')
      move_group.stop()
      move_group.clear_pose_targets()
    
    self.server.set_succeeded()


if __name__ == '__main__':
  rospy.init_node('execute_grasp_server')
  server = ExecuteGraspServer()
  rospy.spin()
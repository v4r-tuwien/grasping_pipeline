#! /usr/bin/env python

import rospy
import actionlib

import moveit_commander
import moveit_msgs.msg
import sys
import geometry_msgs.msg
import tf.transformations
import tf


from grasping_pipeline.msg import ExecuteGraspAction, ExecuteGraspActionResult
from hsrb_interface import Robot, geometry
from std_srvs.srv import Empty
from visualization_msgs.msg import Marker
from math import pi


class ExecuteGraspServer:
  def __init__(self):
    self.robot = Robot()
    self.whole_body = self.robot.try_get('whole_body')
    self.gripper = self.robot.get('gripper')
    self.tf = tf.TransformListener()
    self.use_map = rospy.get_param('/use_map', False)

    self.move_group = self.moveit_init()
    self.server = actionlib.SimpleActionServer('execute_grasp', ExecuteGraspAction, self.execute, False)
    self.clear_octomap = rospy.ServiceProxy('/clear_octomap', Empty)

    self.server.start()
  
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
    

    t = self.tf.getLatestCommonTime('/odom', '/base_link')
    transform = self.tf.lookupTransform('/odom', '/base_link', t)
    move_group.set_workspace((-1.5+transform[0][0],-1.5+transform[0][1],-1,1.5+transform[0][0],1.5+transform[0][1],3))
    
    move_group.allow_replanning(True)
    move_group.set_num_planning_attempts(5)
    self.create_collision_environment()

    return move_group

  def execute(self, goal):
    res = ExecuteGraspActionResult()
    self.create_collision_environment()

    self.clear_octomap()

    if goal.grasp_pose.header.frame_id == "":
      rospy.loginfo('Not a valid goal. Aborted execution!')
      self.server.set_aborted()
      return
    #open gripper
    self.gripper.command(1.0)
    #add grasp_height and safety_distance to grasp_pose
    grasp_pose_1 = goal.grasp_pose
    print grasp_pose_1
    print goal.approach_vector_x
    print goal.approach_vector_y
    print goal.approach_vector_z
    grasp_pose_1.pose.position.x = goal.grasp_pose.pose.position.x + goal.safety_distance*goal.approach_vector_x
    grasp_pose_1.pose.position.y = goal.grasp_pose.pose.position.y + goal.safety_distance*goal.approach_vector_y
    grasp_pose_1.pose.position.z = goal.grasp_pose.pose.position.z + goal.safety_distance*goal.approach_vector_z

    t = self.tf.getLatestCommonTime('/odom', grasp_pose_1.header.frame_id)
    grasp_pose_1.header.stamp = t
    grasp_pose_1 = self.tf.transformPose('/odom', grasp_pose_1)
    self.add_marker(grasp_pose_1)
    rospy.sleep(1) #TODO maybe not important
    self.move_group.set_pose_target(grasp_pose_1)
    plan = self.move_group.plan()
    #TODO sometimes the robot executes the plan, but plan_found is false
    if len(plan.joint_trajectory.points)>0:
      plan_found = True
    else:
      plan_found = False
    self.move_group.go(wait=True)
    #self.move_group.stop()
    #self.move_group.clear_pose_targets()
    rospy.sleep(0.5)
    if plan_found:
      if goal.safety_distance>goal.grasp_height:
        self.whole_body.move_end_effector_by_line((0,0,1), goal.safety_distance-goal.grasp_height)  
          
      self.gripper.apply_force(0.30)

      #move 5cm in z direction 
      pose_vec, pose_quat = self.whole_body.get_end_effector_pose('base_link')
      new_pose_vec = geometry.Vector3(pose_vec.x, pose_vec.y, pose_vec.z + 0.05)
      new_pose = geometry.Pose(new_pose_vec, pose_quat)
      self.whole_body.move_end_effector_pose(new_pose)

      self.whole_body.move_end_effector_by_line((0,0,1), -goal.safety_distance)      
      self.whole_body.move_to_neutral()
      #self.gripper.command(1.0)
    else:
      rospy.logerr('no grasp found')
      self.move_group.stop()
      self.move_group.clear_pose_targets()
      res.result.success = False
      self.server.set_aborted(res.result)
      return

    self.gripper.apply_force(0.50)
    if self.gripper.get_distance()>-0.004:
      res.result.success = True
      self.server.set_succeeded(res.result)

    else:
      res.result.success = False
      rospy.logerr('grasping failed')
      self.server.set_aborted()

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
    if self.use_map:
      self.add_box('table', 0.39, -0.765, 0.175, 0.52, 0.52, 0.35)
      self.add_box('cupboard', 1.4, 1.1, 1, 2.5, 1, 2)
      self.add_box('desk', -1.5, -0.9, 0.4, 0.8, 1.8, 0.8)
      self.add_box('drawer', 0.2, -2, 0.253, 0.8, 0.44, 0.56)
      self.add_box('cupboard_2', 2.08, -1.23, 0.6, 0.6, 1.9, 1.2)
    self.add_box('floor', 0, 0,-0.1,15,15,0.1)

  def add_marker(self, pose_goal):

    br = tf.TransformBroadcaster()
    br.sendTransform((pose_goal.pose.position.x, pose_goal.pose.position.y, pose_goal.pose.position.z),
    [pose_goal.pose.orientation.x, pose_goal.pose.orientation.y, pose_goal.pose.orientation.z, pose_goal.pose.orientation.w],
      rospy.Time.now(),
      'grasp_pose_execute',
      pose_goal.header.frame_id)

    marker_pub = rospy.Publisher('/grasp_marker_2', Marker, queue_size=10, latch=True)
    marker = Marker()
    marker.header.frame_id = pose_goal.header.frame_id
    marker.header.stamp = rospy.Time()
    marker.ns = 'grasp_marker'
    marker.id = 0
    marker.type = 0
    marker.action = 0

    q2 = [pose_goal.pose.orientation.w, pose_goal.pose.orientation.x, pose_goal.pose.orientation.y, pose_goal.pose.orientation.z]
    q = tf.transformations.quaternion_about_axis(pi/2, (0,1,0))
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


if __name__ == '__main__':
  rospy.init_node('execute_grasp_server')
  server = ExecuteGraspServer()
  rospy.spin()
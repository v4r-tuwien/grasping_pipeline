#! /usr/bin/env python

import rospy
import actionlib

from grasping_pipeline.msg import FindGrasppointAction, FindGrasppointActionResult
from hsrb_interface import Robot
import tf
from tf.transformations import quaternion_from_euler
from haf_grasping.msg import CalcGraspPointsServerActionResult, CalcGraspPointsServerActionGoal, CalcGraspPointsServerAction
from sensor_msgs.msg import PointCloud2
import geometry_msgs.msg
import sensor_msgs.point_cloud2 as pc2
from math import pi, atan2, asin, isnan
from visualization_msgs.msg import Marker
from tmc_vision_msgs.msg import DetectionArray, Detection


class FindGrasppointServer:
  def __init__(self):
    self.server = actionlib.SimpleActionServer('find_grasppoint', FindGrasppointAction, self.execute, False)
    self.server.start()
    self.robot = Robot()
    self.base = self.robot.try_get('omni_base')
    self.whole_body = self.robot.try_get('whole_body')
    self.gripper = self.robot.get('gripper')
    self.Transformer = tf.TransformListener(True, rospy.Duration(10))
    self.grasp_client = actionlib.SimpleActionClient('/calc_grasppoints_svm_action_server', CalcGraspPointsServerAction)
    self.yolo_detection_sub = rospy.Subscriber('/yolo2_node/detections', DetectionArray, self.yolo_detection_cb)

  def execute(self, goal):
    ## method 1 uses yolo and haf grasping
    if goal.method == 1:
      result = FindGrasppointActionResult().result
      rough_grasp_object_center = self.get_grasp_object_center_yolo()
      if rough_grasp_object_center is -1:
        self.server.set_aborted()
      else:
        self.call_haf_grasping(rough_grasp_object_center)
        grasp_pose = self.convert_grasp_result_for_moveit()
        result.grasp_pose = grasp_pose
        self.add_marker(grasp_pose)
        self.server.set_succeeded(result)
    else:
      rospy.loginfo('Method not implemented')
      self.server.set_aborted()

  def pointcloud_cb(self, data):
    self.cloud = data
  
  def yolo_detection_cb(self, data):
    self.yolo_detection = data
  
  def get_grasp_object_center_yolo(self):
    rospy.wait_for_message('/yolo2_node/detections', DetectionArray)
    detection = self.yolo_detection
    chosen_object = Detection()
    chosen_object.label.confidence = 0
    #use detection with biggest confidence 
    for i in range(len(detection.detections)):
        name = detection.detections[i].label.name
        if name =='sports ball' or name == 'apple' or name == 'cake':
            if detection.detections[i].label.confidence > chosen_object.label.confidence:
                chosen_object = detection.detections[i]
    if chosen_object.label.confidence == 0:
      return -1
            
    image_x = int(chosen_object.x)
    image_y = int(chosen_object.y)
    self.object_name = chosen_object.label.name
    pointcloud_sub = rospy.Subscriber('/hsrb/head_rgbd_sensor/depth_registered/rectified_points', PointCloud2, self.pointcloud_cb )
    rospy.wait_for_message('/hsrb/head_rgbd_sensor/depth_registered/rectified_points', PointCloud2, timeout=15)
    rospy.sleep(0.5)
    self.my_cloud = self.cloud
    points = pc2.read_points_list(self.my_cloud, field_names=None, skip_nans=False)

    index = image_y*self.my_cloud.width+image_x

    center = points[index]
    rospy.loginfo('len of points'.format(len(points)))
    while isnan(center[0]):
        index = index+self.my_cloud.width
        rospy.loginfo('index = '.format(index))
        if index>len(points)-1:
          return -1
        center = points[index]
        

    self.Transformer.waitForTransform('/base_link', '/head_rgbd_sensor_link', rospy.Time(), rospy.Duration(4.0))
    pose_goal = geometry_msgs.msg.Pose()
    point = geometry_msgs.msg.PointStamped()
    point.point.x = center[0]
    point.point.y = center[1]
    point.point.z = center[2]
    point.header.frame_id = '/head_rgbd_sensor_link'
    point_transformed = self.Transformer.transformPoint('/base_link', point)
    return point_transformed

  def call_haf_grasping(self, search_center):
      grasp_goal = CalcGraspPointsServerActionGoal()
      grasp_goal.goal.graspinput.goal_frame_id = 'base_link'
      grasp_goal.goal.graspinput.grasp_area_center.x = search_center.point.x + 0.05
      grasp_goal.goal.graspinput.grasp_area_center.y = search_center.point.y
      grasp_goal.goal.graspinput.grasp_area_center.z = search_center.point.z
      grasp_goal.goal.graspinput.grasp_area_length_x = 32
      grasp_goal.goal.graspinput.grasp_area_length_y = 32
      grasp_goal.goal.graspinput.approach_vector.z = 1.0
      grasp_goal.goal.graspinput.input_pc = self.my_cloud
      grasp_goal.goal.graspinput.max_calculation_time = rospy.Time(5)
      grasp_goal.goal.graspinput.gripper_opening_width = 1.0
      self.grasp_client.wait_for_server()
      self.grasp_client.send_goal(grasp_goal.goal)
      self.grasp_client.wait_for_result()
      self.grasp_result = self.grasp_client.get_result()

  def convert_grasp_result_for_moveit(self):
      grasp_x = self.grasp_result.graspOutput.averagedGraspPoint.x
      grasp_y = self.grasp_result.graspOutput.averagedGraspPoint.y
      grasp_z = self.grasp_result.graspOutput.averagedGraspPoint.z

      dir_x = -self.grasp_result.graspOutput.approachVector.x
      dir_y = -self.grasp_result.graspOutput.approachVector.y
      dir_z = -self.grasp_result.graspOutput.approachVector.z
      pitch = asin(dir_y)
      yaw = atan2(dir_x, dir_z)
      roll = self.grasp_result.graspOutput.roll
      #q = quaternion_from_euler(-pitch, yaw ,roll-pi/4, 'sxyz')
      q = quaternion_from_euler(pi, 0, -roll+pi/2, 'sxyz')
      
      self.Transformer.waitForTransform('/odom', '/base_link', rospy.Time(), rospy.Duration(4.0))

      grasp_pose_bl = geometry_msgs.msg.PoseStamped()

      grasp_pose_bl.pose.orientation.x = q[0]
      grasp_pose_bl.pose.orientation.y = q[1]
      grasp_pose_bl.pose.orientation.z = q[2]
      grasp_pose_bl.pose.orientation.w = q[3]
      grasp_pose_bl.pose.position.x = grasp_x
      grasp_pose_bl.pose.position.y = grasp_y
      grasp_pose_bl.pose.position.z = grasp_z+0.06
      grasp_pose_bl.header.frame_id = '/base_link'

      grasp_pose = self.Transformer.transformPose('/odom', grasp_pose_bl)
      return grasp_pose

  def add_marker(self, pose_goal):
    marker_pub = rospy.Publisher('/grasp_marker', Marker, queue_size=10, latch=True)
    marker = Marker()
    marker.header.frame_id = pose_goal.header.frame_id
    marker.header.stamp = rospy.Time()
    marker.ns = 'grasp_marker'
    marker.id = 0
    marker.type = 2
    marker.action = 0
    marker.pose.orientation.x = pose_goal.pose.orientation.x
    marker.pose.orientation.y = pose_goal.pose.orientation.y
    marker.pose.orientation.z = pose_goal.pose.orientation.z      
    marker.pose.orientation.w = pose_goal.pose.orientation.w
    marker.pose.position.x = pose_goal.pose.position.x
    marker.pose.position.y = pose_goal.pose.position.y
    marker.pose.position.z = pose_goal.pose.position.z

    marker.scale.x = 0.01
    marker.scale.y = 0.05
    marker.scale.z = 0.1

    marker.color.a = 1.0
    marker.color.r = 1.0
    marker.color.g = 0
    marker.color.b = 0
    marker_pub.publish(marker)
    rospy.loginfo('grasp_marker')


if __name__ == '__main__':
  rospy.init_node('find_grasppoint_server')
  server = FindGrasppointServer()
  rospy.spin()
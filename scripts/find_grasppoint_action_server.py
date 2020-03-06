#! /usr/bin/env python

import rospy
import actionlib

from grasping_pipeline.msg import FindGrasppointAction, FindGrasppointActionResult
from hsrb_interface import Robot
import tf
from tf.transformations import quaternion_from_euler, quaternion_multiply, quaternion_matrix
from haf_grasping.msg import CalcGraspPointsServerActionResult, CalcGraspPointsServerActionGoal, CalcGraspPointsServerAction
from sensor_msgs.msg import PointCloud2
import geometry_msgs.msg
import sensor_msgs.point_cloud2 as pc2
from math import pi, atan2, asin, isnan, sqrt
from visualization_msgs.msg import Marker
from tmc_vision_msgs.msg import DetectionArray, Detection
from object_detector_msgs.srv import get_poses, start, stop
from object_detector_msgs.msg import Detections as DetectronDetections, Detection as DetectronDetection
import numpy as np

import actionlib
from hsrb_grasping.msg import EstimateGraspAction, EstimateGraspGoal, ExecuteGraspAction, ExecuteGraspGoal,\
    ExecutePlaceAction, ExecutePlaceGoal
from hsrb_grasping.srv import save_grasp, save_graspResponse, save_graspRequest
import smach


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
    rospy.loginfo('Initializing FindGrasppointServer done')    
    self.detectron_sub = rospy.Subscriber('/detectron2_service/detections', DetectronDetections, self.detectron_cb)
    rospy.loginfo('Initializing FindGrasppointServer done')

    self.verefine_get_poses = rospy.ServiceProxy('/hsr_grasping/get_poses', get_poses)
    self.start_detectron = rospy.ServiceProxy('/detectron2_service/start', start)
    self.stop_detectron = rospy.ServiceProxy('/detectron2_service/stop', stop)


    action_name = '/hsrb_grasping_estimate'
    self.grasp_estimation_client = actionlib.SimpleActionClient(action_name, EstimateGraspAction)
    rospy.loginfo('Waiting for the action: %s', action_name)
    self.grasp_estimation_client.wait_for_server()


  def execute(self, goal):
    if not goal.object_names:
      print ('use default object names')
      #TODO define default object names?
      # or add option to grasp everything 
      object_names = ['apple', 'sports ball', 'bottle', 'banana', 'car']  
    else:
      object_names = goal.object_names
    print (object_names)
    rospy.loginfo('Method Number: {}'.format(goal.method))
    result = FindGrasppointActionResult().result

    ## method 1 uses yolo and haf grasping
    if goal.method == 1:
      rospy.loginfo('Chosen Method is YOLO + HAF')
      #rough_grasp_object_center = self.get_grasp_object_center_yolo(object_names)
      rough_grasp_object_center = self.get_grasp_object_center_detectron(object_names)
      if rough_grasp_object_center is -1:
        self.server.set_aborted()
      else:
        self.call_haf_grasping(rough_grasp_object_center)
        grasp_pose = self.convert_haf_result_for_moveit()
        
        result.grasp_pose = grasp_pose
        norm = sqrt(self.approach_vector_x**2 + self.approach_vector_y**2 + self.approach_vector_z**2)
        
        result.approach_vector_x = self.approach_vector_x/norm
        result.approach_vector_y = self.approach_vector_y/norm
        result.approach_vector_z = self.approach_vector_z/norm
        
        #self.add_marker(grasp_pose)
        self.server.set_succeeded(result)

    ## method 2 uses verefine
    elif goal.method == 2:
      self.verefine_object_found = False
      rospy.loginfo('Chosen Method is VEREFINE')
      try:
        object_poses_result = self.verefine_get_poses()
      except:
        rospy.loginfo('Aborted: error when calling get_poses service.')
        self.server.set_aborted()
        return
      confidence = 0
      object_nr = 0
      for i in range(0,len(object_poses_result.poses)):
        print object_poses_result.poses[i].name
        if object_poses_result.poses[i].name in object_names: #TODO add all objects option
          if confidence < object_poses_result.poses[i].confidence:
            confidence = object_poses_result.poses[i].confidence
            object_nr = i
            self.verefine_object_found = True    
      if self.verefine_object_found:
        goal = EstimateGraspGoal()
        goal.object_pose = object_poses_result.poses[object_nr]
        # Sends the goal to the action server
        self.grasp_estimation_client.send_goal(goal)
        # Waits for the server to finish performing the action.
        succeeded = self.grasp_estimation_client.wait_for_result()
        if succeeded:
            succeeded = self.grasp_estimation_client.get_result().success

        # If succeeded, then return the grasp poses
        if succeeded:
            grasp_estimation = self.grasp_estimation_client.get_result()

            print grasp_estimation.grasp_pose
            grasp_pose = grasp_estimation.grasp_pose            

            q =  [grasp_pose.pose.orientation.x, grasp_pose.pose.orientation.y, 
                  grasp_pose.pose.orientation.z, grasp_pose.pose.orientation.w]
            
            approach_vector = qv_mult(q, [0,0,-1])

            #write result
            grasp_pose.pose.orientation.x = q[0]
            grasp_pose.pose.orientation.y = q[1]
            grasp_pose.pose.orientation.z = q[2]
            grasp_pose.pose.orientation.w = q[3]

            result.grasp_pose = grasp_pose
            result.approach_vector_x = approach_vector[0]
            result.approach_vector_y = approach_vector[1]
            result.approach_vector_z = approach_vector[2]

            self.add_marker(grasp_pose) 
            self.server.set_succeeded(result)

        else:
          rospy.loginfo('no grasp found')
          self.server.set_aborted()
      else:
        rospy.loginfo('no object pose found')
        self.server.set_aborted()
    else:
      rospy.loginfo('Method not implemented')
      self.server.set_aborted()

  def pointcloud_cb(self, data):
    self.cloud = data
  
  def yolo_detection_cb(self, data):
    self.yolo_detection = data
  
  def detectron_cb(self,data):
    self.detectron_detection = data

  def get_grasp_object_center_detectron(self, object_names):
    print self.start_detectron()
    detections = rospy.wait_for_message('/detectron2_service/detections', DetectronDetections, timeout=10)
    print 'detection received'
    self.stop_detectron()
    print 'stop detectron'
    chosen_object = DetectronDetection()
    chosen_object.bbox.ymax = 0
    for i in range(len(detections.detections)):
      name = detections.detections[i].name
      if name in object_names:
          if detections.detections[i].bbox.ymax > chosen_object.bbox.ymax:
            chosen_object = detections.detections[i]
    if chosen_object.score == 0:
      return -1
    image_x = chosen_object.bbox.xmin + (chosen_object.bbox.xmax-chosen_object.bbox.xmin)/2
    image_y = chosen_object.bbox.ymin + (chosen_object.bbox.ymax-chosen_object.bbox.ymin)/2
    self.object_name = chosen_object.name
    pointcloud_sub = rospy.Subscriber('/hsrb/head_rgbd_sensor/depth_registered/rectified_points', PointCloud2, self.pointcloud_cb )
    rospy.wait_for_message('/hsrb/head_rgbd_sensor/depth_registered/rectified_points', PointCloud2, timeout=15)
    rospy.sleep(0.5)
    self.my_cloud = self.cloud
    points = pc2.read_points_list(self.my_cloud, field_names=None, skip_nans=False)

    index = image_y*self.my_cloud.width+image_x

    center = points[index]
    rospy.loginfo('len of points {}'.format(len(points)))
    while isnan(center[0]):
        index = index+self.my_cloud.width
        rospy.loginfo('index = {}'.format(index))
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

  def get_grasp_object_center_yolo(self, object_names):
    rospy.wait_for_message('/yolo2_node/detections', DetectionArray)
    detection = self.yolo_detection
    chosen_object = Detection()
    chosen_object.label.confidence = 0
    #use detection with biggest confidence 
    for i in range(len(detection.detections)):
        name = detection.detections[i].label.name
        if name in object_names:
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
    rospy.loginfo('len of points {}'.format(len(points)))
    while isnan(center[0]):
        index = index+self.my_cloud.width
        rospy.loginfo('index = {}'.format(index))
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
      self.approach_vector_x = 0.0
      self.approach_vector_y = 0.0
      self.approach_vector_z = 1.0
      
      grasp_goal = CalcGraspPointsServerActionGoal()
      grasp_goal.goal.graspinput.goal_frame_id = 'base_link'
      grasp_goal.goal.graspinput.grasp_area_center.x = search_center.point.x + 0.05
      grasp_goal.goal.graspinput.grasp_area_center.y = search_center.point.y
      grasp_goal.goal.graspinput.grasp_area_center.z = search_center.point.z
      grasp_goal.goal.graspinput.grasp_area_length_x = 32
      grasp_goal.goal.graspinput.grasp_area_length_y = 32
      
      grasp_goal.goal.graspinput.approach_vector.x = self.approach_vector_x
      grasp_goal.goal.graspinput.approach_vector.y = self.approach_vector_y
      grasp_goal.goal.graspinput.approach_vector.z = self.approach_vector_z
      

      grasp_goal.goal.graspinput.input_pc = self.my_cloud
      grasp_goal.goal.graspinput.max_calculation_time = rospy.Time(5)
      grasp_goal.goal.graspinput.gripper_opening_width = 1.0
      self.grasp_client.wait_for_server()
      self.grasp_client.send_goal(grasp_goal.goal)
      self.grasp_client.wait_for_result()
      self.grasp_result = self.grasp_client.get_result()

  def convert_haf_result_for_moveit(self):
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
      #q = quaternion_from_euler(pi, 0, -roll+pi/2, 'sxyz')
      #q = quaternion_from_euler(2*yaw, pitch, -roll+pi/2, 'sxyz')
      ##top grasp
      #q = quaternion_from_euler(yaw, pitch, roll-pi/2, 'sxyz')
      ##front grasp
      #q = quaternion_from_euler(yaw-pi, pitch-pi/2, roll-pi/2, 'sxyz')

      ##front grasp
      #q = quaternion_from_euler(yaw, pitch+pi/2, roll+pi/2, 'sxyz')
      ##top grasp
      #q = quaternion_from_euler(yaw, -pitch, -roll+pi/2, 'sxyz')
      
      q = quaternion_from_euler(yaw, pitch, roll-pi, 'rxyz')

      if not (self.approach_vector_x == 0 and self.approach_vector_y == 0):
        q_rot = tf.transformations.quaternion_about_axis(pi/2, (0,0,1))
        q = tf.transformations.quaternion_multiply(q_rot, q)
      q_rot = tf.transformations.quaternion_about_axis(pi/2, (self.approach_vector_x,self.approach_vector_y,self.approach_vector_z))
      q = tf.transformations.quaternion_multiply(q_rot, q)


      self.Transformer.waitForTransform('/odom', '/base_link', rospy.Time(), rospy.Duration(4.0))

      grasp_pose_bl = geometry_msgs.msg.PoseStamped()

      grasp_pose_bl.pose.orientation.x = q[0]
      grasp_pose_bl.pose.orientation.y = q[1]
      grasp_pose_bl.pose.orientation.z = q[2]
      grasp_pose_bl.pose.orientation.w = q[3]
      grasp_pose_bl.pose.position.x = grasp_x
      grasp_pose_bl.pose.position.y = grasp_y
      grasp_pose_bl.pose.position.z = grasp_z
      grasp_pose_bl.header.frame_id = '/base_link'

      self.add_marker(grasp_pose_bl)

      grasp_pose = self.Transformer.transformPose('/odom', grasp_pose_bl)
      return grasp_pose

  def add_marker(self, pose_goal):
    # br = tf.TransformBroadcaster()
    # br.sendTransform((pose_goal.pose.position.x, pose_goal.pose.position.y, pose_goal.pose.position.z),
    # [pose_goal.pose.orientation.w, pose_goal.pose.orientation.x, pose_goal.pose.orientation.y, pose_goal.pose.orientation.z],
    #   rospy.Time.now(),
    #   'grasp_pose',
    #   pose_goal.header.frame_id)
    
    marker_pub = rospy.Publisher('/grasp_marker', Marker, queue_size=10, latch=True)
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

    #marker.pose.orientation.x = pose_goal.pose.orientation.x
    #marker.pose.orientation.y = pose_goal.pose.orientation.y
    #marker.pose.orientation.z = pose_goal.pose.orientation.z
    #marker.pose.orientation.w = pose_goal.pose.orientation.w
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
    marker.color.r = 1.0
    marker.color.g = 0
    marker.color.b = 0
    marker_pub.publish(marker)
    rospy.loginfo('grasp_marker')

# rotate vector v by quaternion q
def qv_mult(q, v):
    rot_mat = quaternion_matrix(q)[:3, :3]
    v = np.array(v)
    return rot_mat.dot(v)

if __name__ == '__main__':
  rospy.init_node('find_grasppoint_server')
  server = FindGrasppointServer()
  rospy.spin()
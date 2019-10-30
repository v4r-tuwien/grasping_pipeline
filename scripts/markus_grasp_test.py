#!/usr/bin/env python

import sys
import rospy
import geometry_msgs.msg
from std_msgs.msg import String
from std_srvs.srv import Empty
from sensor_msgs.msg import PointCloud2  
from haf_grasping.msg import CalcGraspPointsServerActionResult, CalcGraspPointsServerActionGoal, CalcGraspPointsServerAction
import moveit_commander
import moveit_msgs.msg
from math import pi, cos, sin, atan2, acos, sqrt, asin, isnan
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker, MarkerArray
from hsrb_interface import Robot
import geometry_msgs.msg
import tf
import actionlib
import yaml
import os
import rospkg
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionResult, MoveBaseActionFeedback
from tmc_vision_msgs.msg import DetectionArray, Detection
import sensor_msgs.point_cloud2 as pc2
import trajectory_msgs.msg



class Main(object):
    def __init__(self):
        self.robot = Robot()
        self.base = self.robot.try_get('omni_base')
        self.tts = self.robot.try_get('default_tts')
        self.whole_body = self.robot.try_get('whole_body')
        self.gripper = self.robot.get('gripper')

        self.tts.language = self.tts.ENGLISH
        self.Transformer = tf.TransformListener(True, rospy.Duration(10))

        self.grasp_client = actionlib.SimpleActionClient('/calc_grasppoints_svm_action_server', CalcGraspPointsServerAction)

       

        self.move_client = actionlib.SimpleActionClient('/move_base/move', MoveBaseAction)

        self.yolo_detection_sub = rospy.Subscriber('/yolo2_node/detections', DetectionArray, self.yolo_detection_cb)

    def pointcloud_cb(self, data):
        self.cloud = data
    
    def yolo_detection_cb(self, data):
        self.yolo_detection = data
    
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

    def add_marker(self, pose_goal):
        marker_pub = rospy.Publisher('/grasp_marker', Marker, queue_size=10)
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
        rospy.sleep(1.9)
        marker_pub.publish(marker)
        rospy.sleep(1.9)
    
    def add_box(self, name, position_x = 0, position_y = 0, position_z = 0, size_x = 0.1, size_y = 0.1, size_z = 0.1):
        rospy.sleep(0.1)
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "map"
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.x = position_x 
        box_pose.pose.position.y = position_y
        box_pose.pose.position.z = position_z
        box_name = name
        self.scene.add_box(box_name, box_pose, size=(size_x, size_y, size_z))

    def go_to_viewpoint(self):
        """
        read viewpoints from a cfg file, choose a viewpoint, move there and gaze down.
        """
        cfg_file = '/home/v4r/table_ws/src/table_mapping/conf/patrolling.yaml'
        readfile = open(cfg_file, 'r')
        str = yaml.load(readfile)
        readfile.close()        
        for i in range(len(str)):
            if  str[i][2].position.x > 0 and str[i][2].position.y>-0.2:
                move_goal = MoveBaseGoal()
                move_goal.target_pose.header.frame_id='map'
                move_goal.target_pose.pose = str[i][2]
                move_goal.target_pose.pose.orientation.w = 0.7071068
                move_goal.target_pose.pose.orientation.z = -0.7071068
                print (str[i][2])
                print ("moved to {}".format(str[i][0]))
                self.move_client.wait_for_server()
                self.move_client.send_goal(move_goal)
                self.move_client.wait_for_result()
        self.whole_body.gaze_point((0.7, 0.1, 0.4))
        self.whole_body.move_to_joint_positions({'arm_roll_joint':pi/2})

    def go_to_position(self):
        """go to a predefined position"""
        self.whole_body.move_to_joint_positions({'arm_roll_joint':pi/2})
        move_goal = MoveBaseGoal()
        move_goal.target_pose.header.frame_id='map'
        move_goal.target_pose.pose.position.x = 1.1
        move_goal.target_pose.pose.position.y = -0.75
        move_goal.target_pose.pose.orientation.z = 1.0
        self.move_client.wait_for_server()
        self.move_client.send_goal(move_goal)
        self.move_client.wait_for_result()
        self.whole_body.gaze_point((0.7, 0.1, 0.4))

    def create_collision_environment(self):
        #self.add_box('table', 0.39, -0.765, 0.225, 0.55, 0.55, 0.35)
        self.add_box('table', 0.39, -0.765, 0.225, 0.52, 0.52, 0.45)    
        self.add_box('cupboard', 1.4, 1.1, 1, 2.5, 1, 2)
        self.add_box('desk', -1.5, -0.9, 0.4, 0.8, 1.8, 0.8)
        self.add_box('drawer', 0.2, -2, 0.253, 0.8, 0.44, 0.56)
        self.add_box('cupboard_2', 2.08, -1.23, 0.6, 0.6, 1.9, 1.2)
        #self.add_box('floor', 0,0,-0.051,5,5,0.1)
    
    def get_grasp_object_center_yolo(self):
        rospy.wait_for_message('/yolo2_node/detections', DetectionArray)
        detection = self.yolo_detection
        chosen_object = Detection()
        chosen_object.label.confidence = 0
        #use detection with biggest confidence 
        for i in range(len(detection.detections)):
            name = detection.detections[i].label.name
            if name =='sports ball' or name == 'apple' or name == 'cake':
                print detection.detections[i].label.name
                print detection.detections[i].label.confidence
                if detection.detections[i].label.confidence > chosen_object.label.confidence:
                    chosen_object = detection.detections[i]
                
        print chosen_object
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
        while isnan(center[0]):
            index = index+self.my_cloud.width
            center = points[index]
            print index

        self.Transformer.waitForTransform('/base_link', '/head_rgbd_sensor_link', rospy.Time(), rospy.Duration(4.0))
        pose_goal = geometry_msgs.msg.Pose()
        point = geometry_msgs.msg.PointStamped()
        point.point.x = center[0]
        point.point.y = center[1]
        point.point.z = center[2]
        point.header.frame_id = '/head_rgbd_sensor_link'
        point_transformed = self.Transformer.transformPoint('/base_link', point)
        return point_transformed

    def add_grasp_object_sphere(self, center, radius):
        object_pose = geometry_msgs.msg.PoseStamped()
        object_pose.header.frame_id = "base_link"
        object_pose.pose.orientation.w = 1.0
        object_pose.pose.position.x = center.point.x+0.05
        object_pose.pose.position.y = center.point.y
        object_pose.pose.position.z = center.point.z
        self.scene.add_sphere(self.object_name, object_pose, radius)

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

    def openGripper(self, posture):
        posture.joint_names = ["hand_motor_joint"]
        posture.points.append(trajectory_msgs.msg.JointTrajectoryPoint())
        posture.points[0].positions.append(1.2)
        posture.points[0].time_from_start = rospy.Duration(0.5)
        posture.header.frame_id = "/odom"

    def closedGripper(self, posture):
        posture.joint_names.append("hand_motor_joint")
        posture.points.append(trajectory_msgs.msg.JointTrajectoryPoint())
        posture.points[0].positions.append(-0.75)
        posture.points[0].time_from_start = rospy.Duration(0.5)
        posture.header.frame_id = "/odom"

    def pick(self, move_group, grasp_pose):
        grasps = []
        grasps.append(moveit_msgs.msg.Grasp())
        grasps[0].id = 'grasp_id'
        grasps[0].grasp_pose = grasp_pose

        grasps[0].pre_grasp_approach.direction.header.frame_id = "/odom"
        grasps[0].pre_grasp_approach.direction.vector.z = 1.0
        grasps[0].pre_grasp_approach.min_distance = 0.095
        grasps[0].pre_grasp_approach.desired_distance = 0.115

        grasps[0].post_grasp_retreat.direction.header.frame_id = "/odom"
        grasps[0].post_grasp_retreat.direction.vector.z = 1.0
        grasps[0].post_grasp_retreat.min_distance = 0.075
        grasps[0].post_grasp_retreat.desired_distance = 0.125
        grasps[0].grasp_quality = 0.8
        grasps[0].allowed_touch_objects = ["table"]
        
        #for i in range(10):
        #    self.openGripper(grasps[i].pre_grasp_posture)
        #    self.closedGripper(grasps[i].grasp_posture)
        #    grasps.append(grasps[0])

        self.openGripper(grasps[0].pre_grasp_posture)
        self.closedGripper(grasps[0].grasp_posture)
        move_group.set_support_surface_name("table")

        #print grasps

        move_group.pick('grasp_object',grasps)

        rospy.spin()

    def state_cb(self, data):
        self.state = data

    def run(self):
        move_group = self.moveit_init()


        ###move go_to_joint_states for grasping and lowering
        #print move_group.get_joints()
        #rospy.Subscriber('/hsrb/joint_states', moveit_msgs.msg.RobotState, self.state_cb, queue_size=10, buff_size=10000)
        #rospy.wait_for_message('/hsrb/joint_states', moveit_msgs.msg.RobotState)
        #state = self.state
        #move_group.set_start_state(state)

        #self.robot_cmd.get_current_state()

        #rospy.spin()
        self.go_to_position()

        self.create_collision_environment()            

        rospy.wait_for_service('/clear_octomap')
        clear_octomap = rospy.ServiceProxy('/clear_octomap', Empty)
        clear_octomap()

        rough_grasp_object_center = self.get_grasp_object_center_yolo()
        
        self.add_grasp_object_sphere(rough_grasp_object_center, radius=0.05)
        
        #rospy.sleep(1.0)
        grasping_group = 'gripper'
        touch_links = self.robot_cmd.get_link_names(group=grasping_group)
        
        self.call_haf_grasping(rough_grasp_object_center)
        print self.grasp_result
        print 'I`ll try to grasp the {}'.format(self.object_name)

        grasp_pose = self.convert_grasp_result_for_moveit()
        
        #self.add_marker(grasp_pose)
        print grasp_pose

        move_group.set_pose_target(grasp_pose)
        plan = move_group.plan()

        #self.pick(move_group, grasp_pose)

        print grasp_pose.pose.position.z
        if grasp_pose.pose.position.z > 0.55:
            move_group.go()
            move_group.stop()
            move_group.clear_pose_targets()
            rospy.sleep(0.5)
            if len(plan.joint_trajectory.points) > 0 :
                self.gripper.apply_force(0.50)
                self.scene.attach_box(self.eef_link, self.object_name, touch_links=touch_links)
                self.whole_body.move_end_effector_by_line((0,0,1), -0.07)
                self.whole_body.move_to_neutral()
                self.scene.remove_attached_object(self.eef_link, name=self.object_name)
                self.tts.say(u'I took the {}'.format(self.object_name))
                #self.gripper.command(1.0)

        else:
            print 'no grasp found'
            move_group.stop()
            move_group.clear_pose_targets()
        self.scene.remove_world_object()
    

if __name__ == '__main__':
    rospy.init_node('markus_test')
    main = Main()
    main.run()


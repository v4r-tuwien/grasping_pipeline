#! /usr/bin/env python3

import rospy
import smach
from hsrb_interface import Robot
from geometry_msgs.msg import PoseStamped
from vision_msgs.msg import BoundingBox3D
from v4r_util.conversions import vector3_to_list
from moveit_wrapper import MoveitWrapper
from v4r_util.tf2 import TF2Wrapper


class CollisionEnvironment(smach.State):
    def __init__(self):
        smach.State.__init__(
            self, outcomes=['succeeded'], input_keys=['grasp_object_bb', 'table_bbs'], output_keys=['grasp_object_name'])
        self.tf_wrapper = TF2Wrapper()
        self.moveit_wrapper = MoveitWrapper(self.tf_wrapper)
        self.moveit_wrapper.detach_all_objects()
        self.add_floor_plane()
        rospy.loginfo('CollEnv init')
    
    def add_floor_plane(self):
        # Add floor plane to filter weird octomap points in floor that prevent the robot from moving because of 'collisions' with the floor
        base_pose = self.moveit_wrapper.get_current_pose('map')

        pos = base_pose.pose.position
        ori = base_pose.pose.orientation

        floor = BoundingBox3D()
        floor.center.position.x = pos.x
        floor.center.position.y = pos.y
        floor.center.position.z = -0.07
        floor.center.orientation.w = ori.w

        size = [15, 15, 0.1]

        self.moveit_wrapper.add_box('floor', 'map', floor.center, size)
        #TODO octomap clear afterwards to properly handle floor

    def execute(self, userdata):
        grasp_obj_bb = userdata.grasp_object_bb
        table_bb = userdata.table_bbs.boxes[0]
        self.moveit_wrapper.add_box('object', grasp_obj_bb.header.frame_id, grasp_obj_bb.center, vector3_to_list(grasp_obj_bb.size))
        rospy.sleep(0.1)
        self.moveit_wrapper.add_box('table', userdata.table_bbs.header.frame_id, table_bb.center, vector3_to_list(table_bb.size))

        userdata.grasp_object_name = 'object'
        
        return 'succeeded'


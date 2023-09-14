#! /usr/bin/env python3

import rospy
import smach
from hsrb_interface import Robot
from v4r_util.util import transform_bounding_box
from v4r_util.conversions import vector3_to_list
from vision_msgs.msg import BoundingBox3DArray
from object_detector_msgs.srv import addBoxes, addBoxesRequest
from libhsrmoveit import LibHSRMoveit
from v4r_util.tf2 import TF2Wrapper


class AddCollisionObjects(smach.State):

    def __init__(self):
        smach.State.__init__(
            self, outcomes=['succeeded'], input_keys=['grasp_object_bb', 'table_bbs'], output_keys=['support_surface_name', 'grasp_object_name'])
        self.libtf = TF2Wrapper()
        self.libmoveit = LibHSRMoveit(self.libtf)
        # self.moveit_add_boxes_srv_name = '/moveit/collisionenv/add_boxes'
        # self.moveit_add_boxes = rospy.ServiceProxy(self.moveit_add_boxes_srv_name, addBoxes)

    def execute(self, userdata):
        grasp_obj_bb = userdata.grasp_object_bb
        table_bb = userdata.table_bbs.boxes[0]
        self.libmoveit.add_box('obj', grasp_obj_bb.header.frame_id, grasp_obj_bb.center, vector3_to_list(grasp_obj_bb.size))
        rospy.sleep(0.1)
        self.libmoveit.add_box('table', userdata.table_bbs.header.frame_id, table_bb.center, vector3_to_list(table_bb.size))

        # table_bbs = userdata.table_bbs
        # grasp_obj_bb = userdata.grasp_object_bb

        # names = len(table_bbs.boxes) * ['Table'] + 1 * ['GraspObject']

        # collision_object_bbs = BoundingBox3DArray()
        # frame_id = table_bbs.header.frame_id
        # collision_object_bbs.header.frame_id = frame_id
        # collision_object_bbs.boxes = table_bbs.boxes

        # grasp_obj_bb = transform_bounding_box(grasp_obj_bb, grasp_obj_bb.header.frame_id, frame_id)
        # collision_object_bbs.boxes.append(grasp_obj_bb)

        # coll_objs = addBoxesRequest()
        # coll_objs.box_types = names
        # coll_objs.bounding_boxes = collision_object_bbs

        # rospy.wait_for_service(self.moveit_add_boxes_srv_name)

        # response = self.moveit_add_boxes(coll_objs)

        # for name in response.names:
        #     if 'GraspObject' in name:
        #         userdata.grasp_object_name = name
        
        # #TODO searches for support surface based on closest center point, this can lead to issues if box very big
        # # as distance is large, should somehow change this to check the closest dist to a boundary
        # min_dist = float('inf')
        # support_surface = None
        # for table_bb, name in zip(table_bbs.boxes, response.names):
        #     dist = ((grasp_obj_bb.center.position.x - table_bb.center.position.x) ** 2 + 
        #             (grasp_obj_bb.center.position.y - table_bb.center.position.y) ** 2 + 
        #             (grasp_obj_bb.center.position.z - table_bb.center.position.z) ** 2)
        #     if dist < min_dist:
        #         support_surface = name
        #         min_dist = dist

        # userdata.support_surface_name = support_surface
        userdata.support_surface_name = 'a'
        userdata.grasp_object_name = 'obj'
        
        return 'succeeded'


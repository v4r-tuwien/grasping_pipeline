#! /usr/bin/env python3
import sys
import numpy as np
import rospy
import moveit_commander
import tf2_ros
import yaml
from yaml.loader import SafeLoader
from geometry_msgs.msg import Pose, Vector3, Point, PoseStamped, TwistStamped
from vision_msgs.msg import BoundingBox3DArray, BoundingBox3D
from object_detector_msgs.srv import addBoxes, addBoxesResponse
from v4r_util.util import transform_bounding_box, ros_bb_to_o3d_bb, transform_pose
from std_srvs.srv import Empty
from object_detector_msgs.srv import addBoxes



class MoveitServer():
    def __init__(self, cfg):
        moveit_commander.roscpp_initialize(sys.argv)

        #self.server = actionlib.SimpleActionServer('moveit_server', MoveitAction, self.execute, False)
        self.robot = moveit_commander.RobotCommander()
        self.planning_scene = moveit_commander.PlanningSceneInterface(synchronous=True)
        
        self.group_name = cfg['whole_body_group_name']
        if not self.robot.has_group(self.group_name):
            rospy.logerr(f'Whole_body group name \'{self.group_name}\' doesn\'t exist')
            sys.exit(-1)
        wait_for_server_duration = cfg['wait_for_server_duration']
        self.whole_body = moveit_commander.MoveGroupCommander(self.group_name, wait_for_servers=wait_for_server_duration)
        self.planning_frame = self.whole_body.get_planning_frame()
        self.end_effector_link = self.whole_body.get_end_effector_link()
        
        self.whole_body.allow_looking(True)
        self.whole_body.allow_replanning(True)
        self.whole_body.set_workspace([-10.0, -10.0, -10.0, 10.0, 10.0, 10.0])
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)
        self.idx = 0

        rospy.wait_for_service('/get_planning_scene', 10.0)

        self.init_world(cfg)

        rospy.Service('/moveit/collisionenv/add_boxes', addBoxes, self.add_boxes)

    def init_world(self, cfg):
        self.planning_scene.remove_attached_object(self.end_effector_link)
        self.planning_scene.remove_world_object()
        self.whole_body.clear_pose_targets()
        rospy.sleep(1)

        clear_octomap = rospy.ServiceProxy('/clear_octomap', Empty)
        clear_octomap()
        
        #TODO rewrite auf tf2
        base_pose = transform_pose('map', 'base_link', Pose())
        pos = base_pose.pose.position
        ori = base_pose.pose.orientation
        
        # add floor and walls around the robot
        # this prevents the robot to plan some weird ass paths
        collision_object_bbs = BoundingBox3DArray()
        collision_object_bbs.header.stamp = rospy.get_rostime()
        collision_object_bbs.header.frame_id = 'map'

        ws_cfg = cfg['workspace']

        floor = BoundingBox3D()
        floor_size = ws_cfg['floor']['size']
        floor.center.position = Point(pos.x, pos.y, -0.07)
        floor.center.orientation.w = ori.w
        floor.size = Vector3(floor_size[0], floor_size[1], 0.1)
        collision_object_bbs.boxes.append(floor)

        front_wall = BoundingBox3D()
        front_wall_center_offset = ws_cfg['front_wall']['center_offset']
        front_wall_len = ws_cfg['front_wall']['length']
        front_wall.center.position.x = pos.x + front_wall_center_offset
        front_wall.center.position.y = pos.y
        front_wall.center.position.z = 0.05
        front_wall.center.orientation.w = ori.w
        front_wall.size.x = 0.01
        front_wall.size.y = front_wall_len
        front_wall.size.z = 0.1
        collision_object_bbs.boxes.append(front_wall)

        rear_wall = BoundingBox3D()
        #TODO einheitlich auf vector3 und point
        rear_wall_center_offset = ws_cfg['rear_wall']['center_offset']
        rear_wall_len = ws_cfg['rear_wall']['length']
        rear_wall.center.position.x = pos.x - rear_wall_center_offset
        rear_wall.center.position.y = pos.y
        rear_wall.center.position.z = 0.05
        rear_wall.center.orientation.w = ori.w
        rear_wall.size = Vector3(0.01, rear_wall_len, 0.1)
        collision_object_bbs.boxes.append(rear_wall)

        left_wall = BoundingBox3D()
        left_wall_center_offset = ws_cfg['left_wall']['center_offset']
        left_wall_len = ws_cfg['left_wall']['length']
        left_wall.center.position = Point(pos.x, pos.y +  left_wall_center_offset, 0.05)
        left_wall.center.orientation.w = ori.w
        left_wall.size = Vector3(left_wall_len, 0.01, 0.1)
        collision_object_bbs.boxes.append(left_wall)

        right_wall = BoundingBox3D()
        right_wall_center_offset = ws_cfg['right_wall']['center_offset']
        right_wall_len = ws_cfg['right_wall']['length']
        right_wall.center.position = Point(pos.x, pos.y - right_wall_center_offset, 0.05)
        right_wall.center.orientation.w = ori.w
        right_wall.size = Vector3(right_wall_len, 0.01, 0.1)
        collision_object_bbs.boxes.append(right_wall)

        names = ['floor', 'front_wall', 'rear_wall', 'left_wall', 'right_wall']
        world_boxes = addBoxes()
        world_boxes.box_types = names
        world_boxes.bounding_boxes = collision_object_bbs
        self.add_boxes(world_boxes)
    
    def get_known_object_names_in_roi(self, ros_bb):
        # object is in ROI if center point inside ROI
        o3d_bb = ros_bb_to_o3d_bb(ros_bb)
        aabb = o3d_bb.get_axis_aligned_bounding_box()
        min_bound = aabb.min_bound 
        max_bound = aabb.max_bound
        scene_object_map = self.planning_scene.get_objects()
        objects_in_roi = []
        for object_id in scene_object_map:
            object = scene_object_map[object_id]
            #TODO sometime if it ever gets relevant:
            # currently we only consider the 'main' pose attribute of the collision object
            # and ignore the primitive_poses/mesh_poses attribute which are defined relative to the main pose
            # message definition of planningscene object found in: moveit_msgs/CollisionObject
            position = object.pose.position
            if ((position.x > min_bound[0]) and (position.x < max_bound[0]) and
                (position.y > min_bound[1]) and (position.y < max_bound[1]) and
                (position.z > min_bound[2]) and (position.z < max_bound[2])):

                objects_in_roi.append(object_id)
        return objects_in_roi

    def add_boxes(self, request):
        names = []
        header = request.bounding_boxes.header
        for box, type in zip(request.bounding_boxes.boxes, request.box_types):
            # transform to planning frame
            #TODO change auf tf2
            box_transformed = transform_bounding_box(box, header.frame_id, self.planning_frame)
            # only add box if it hasn't been placed yet
            #TODO solution for objects that where slightly moved
            objects_in_roi = self.get_known_object_names_in_roi(box_transformed)
            print(f"{objects_in_roi = }")
            if len(objects_in_roi) == 0:
                name = type + str(self.idx)
                self.idx -=- 1
                pose = PoseStamped()
                pose.pose = box_transformed.center
                pose.header.frame_id = self.planning_frame
                size = [box_transformed.size.x, box_transformed.size.y, box_transformed.size.z]
                self.planning_scene.add_box(name, pose, size)
                names.append(name)
            elif len(objects_in_roi) == 1:
                # if one object found -> just return name of this
                names.append(objects_in_roi[0])
            else:
                # no idea how to handle case where more than one matching objects are found
                names.append('None')
                rospy.logdebug(f'Moveit addBoxes found more than one match: {objects_in_roi}')
        print(names)
        return addBoxesResponse(names)

if __name__=='__main__':
    rospy.init_node("moveit_node", anonymous=True)
    if len(sys.argv) < 2:
        rospy.logerr('No yaml config file was specified!')
        sys.exit(-1)
    with open(sys.argv[1]) as f:
        cfg = yaml.load(f, Loader=SafeLoader)
    MoveitServer(cfg)
    rospy.spin()
    moveit_commander.roscpp_shutdown()
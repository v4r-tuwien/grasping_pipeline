#! /usr/bin/env python3
from math import pi
from copy import deepcopy
import numpy as np
import rospy
import open3d as o3d
import actionlib
import tf
import tf.transformations
from moveit_wrapper import MoveitWrapper
from hsr_wrapper import HSR_wrapper
from v4r_util.tf2 import TF2Wrapper
from v4r_util.conversions import bounding_box_to_bounding_box_stamped, list_to_vector3, vector3_to_list, rot_mat_to_quat, quat_to_rot_mat, np_transform_to_ros_transform, np_transform_to_ros_pose
from v4r_util.util import ros_bb_to_o3d_bb, align_bounding_box_rotation
from v4r_util.util import transform_bounding_box_w_transform, transform_pose, align_bounding_box_rotation
from v4r_util.util import ros_bb_to_o3d_bb, o3d_bb_to_ros_bb
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped, Vector3Stamped, Transform, Point, Quaternion, PointStamped, Vector3, Pose
from std_msgs.msg import Header
from grasping_pipeline_msgs.msg import PlaceAction, PlaceActionResult 
from vision_msgs.msg import BoundingBox3D
from tmc_geometric_shapes_msgs.msg import Shape
from tmc_placement_area_detector.srv import DetectPlacementArea

class PlaceObjectServer():
    def __init__(self):
        self.tf2_wrapper = TF2Wrapper()
        self.moveit = MoveitWrapper(self.tf2_wrapper)
        self.hsr_wrapper = HSR_wrapper()

        self.server = actionlib.SimpleActionServer(
            'place_object', PlaceAction, self.execute, False)
        self.server.start()
        rospy.loginfo("Init Placement")
    
    def transform_plane_normal(self, table_equation, target_frame, stamp):
        header = Header()
        header.frame_id = table_equation.header.frame_id
        header.stamp = stamp
        plane_normal = np.asarray([table_equation.x, table_equation.y, table_equation.z])
        plane_normal_vec3 = list_to_vector3(plane_normal)
        plane_normal_vec3_st = Vector3Stamped(header=header, vector=plane_normal_vec3)
        plane_normal = self.tf2_wrapper.transform_vector3(target_frame, plane_normal_vec3_st)
        plane_normal = [plane_normal.vector.x, plane_normal.vector.y, plane_normal.vector.z]
        plane_normal = plane_normal / np.linalg.norm(plane_normal)

        return plane_normal
    
    def visualize_ros_bb_rviz(self, ros_bb, frame, r, g, b, id=0):
        marker_pub = rospy.Publisher(
            '/test_vis', Marker, queue_size=100, latch=True)
        marker = Marker()
        marker.header.frame_id = frame
        marker.header.stamp = rospy.Time()
        marker.ns = 'grasp_marker'
        marker.id = id
        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        marker.pose.orientation.w = 1
        marker.pose.position.x = ros_bb.center.position.x
        marker.pose.position.y = ros_bb.center.position.y
        marker.pose.position.z = ros_bb.center.position.z + ros_bb.size.z/2

        marker.scale.x = ros_bb.size.x
        marker.scale.y = ros_bb.size.y
        marker.scale.z = ros_bb.size.z

        marker.color.a = 0.5
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker_pub.publish(marker)

    def inverse_transform(self, transform):
        inverse_translation = Point(-transform.translation.x, -transform.translation.y, -transform.translation.z)
        qx = transform.rotation.x
        qy = transform.rotation.y
        qz = transform.rotation.z
        qw = transform.rotation.w
        denominator = qx*qx + qy*qy + qz*qz + qw*qw
        inverse_rotation = Quaternion(-qx/denominator, -qy/denominator, -qz/denominator, qw/denominator)
        inverse_transform = Transform(translation = inverse_translation, rotation = inverse_rotation)
        return inverse_transform
    
    def clicked_point_cb(self, data):
        self.target_point = data

    def detect_placement_areas(self, attached_object_bb, wrist_to_table, base_pose, aligned_table_bb_base_frame, placement_area_det_frame):
        obj_bb_table_frame = transform_bounding_box_w_transform(attached_object_bb, wrist_to_table)
        obj_bb_table_frame = align_bounding_box_rotation(ros_bb_to_o3d_bb(obj_bb_table_frame))
        obj_bb_table_frame = o3d_bb_to_ros_bb(obj_bb_table_frame)

        world_to_base_transform = Transform(
            translation=base_pose.position, rotation=base_pose.orientation)
        base_to_table_transform = Transform(translation=aligned_table_bb_base_frame.center.position, rotation=aligned_table_bb_base_frame.center.orientation)

        obj_bb_base_frame = transform_bounding_box_w_transform(
            obj_bb_table_frame, base_to_table_transform)
        self.visualize_ros_bb_rviz(obj_bb_base_frame, 'base_link', 0, 0, 1, id=0)
        obj_bb_map_frame = transform_bounding_box_w_transform(obj_bb_base_frame, world_to_base_transform)
        self.visualize_ros_bb_rviz(obj_bb_map_frame, 'map', 0, 1, 0, id=1)
        obj_bb_map_frame_aligned = align_bounding_box_rotation(ros_bb_to_o3d_bb(obj_bb_map_frame))
        obj_bb_map_frame_aligned = o3d_bb_to_ros_bb(obj_bb_map_frame_aligned)

        
        self.visualize_ros_bb_rviz(obj_bb_map_frame_aligned, 'map', 1, 0, 0, id=2)

        self.target_point = None
        while True:
            rospy.Subscriber('clicked_point', PointStamped, self.clicked_point_cb)
            while not rospy.is_shutdown():
                rospy.loginfo("Waiting for clicked point")
                if self.target_point is not None:
                    target_point = self.target_point
                    self.target_point = None
                    break
                rospy.sleep(1)
            if target_point.header.frame_id != placement_area_det_frame:
                rospy.logerr("Wrong frame for clicked point. Expected: %s, got: %s" % (placement_area_det_frame, target_point.header.frame_id))
                continue

            target_point = target_point.point

            #TODO take object dim into account properly

            box_filter_range = Vector3()
            box_filter_range.x = 0.2
            box_filter_range.y = 0.2
            box_filter_range.z = 0.1

            vertical_axis = Vector3()
            vertical_axis.x = 0.0
            vertical_axis.y = 0.0
            vertical_axis.z = 1.0

            tilt_threshold = np.pi * 60/180
            distance_threshold = 0.03

            obj_dim_x = obj_bb_map_frame_aligned.size.x
            obj_dim_y = obj_bb_map_frame_aligned.size.y
            obj_dim_z = obj_bb_map_frame_aligned.size.z
            object_shape = Shape()
            object_shape.type = Shape.MESH
            mesh = o3d.geometry.TriangleMesh.create_box(
                obj_dim_x, obj_dim_y, obj_dim_z)
            mesh.rotate(quat_to_rot_mat(obj_bb_map_frame_aligned.center.orientation))
            vertices_list = []
            for p in np.asarray(mesh.vertices):
                vertices_list.append(Point(p[0], p[1], p[2]))
            object_shape.triangles = np.asarray(mesh.triangles).flatten()
            object_shape.vertices = vertices_list

            object_to_surface = Pose()
            surface_range = 1.0

            rospy.wait_for_service('detect_placement_area')

            try:
                detect_placement_area = rospy.ServiceProxy(
                    'detect_placement_area', DetectPlacementArea)
                response = detect_placement_area(placement_area_det_frame, target_point, box_filter_range, vertical_axis,
                                                tilt_threshold, distance_threshold, object_shape, object_to_surface, surface_range)
            except rospy.ServiceException as e:
                print("DetectPlacmentAreaService call failed: %s" % e)

            if response.error_code.val == 1:
                return response.placement_area
            elif response.error_code.val == -1:
                print("ErrorCode: FAIL")
            elif response.error_code.val == -2:
                print("ErrorCode: INVALID_FRAME")
            elif response.error_code.val == -3:
                print("ErrorCode: NO_POINTCLOUD")
            elif response.error_code.val == -4:
                print("ErrorCode: NO_ACCEPTABLE_PLANE")
            elif response.error_code.val == -5:
                print("ErrorCode: INVALID_OBJECT_SURFACE")
            elif response.error_code.val == -1:
                print("ErrorCode: NON_POSITIVE")
            elif response.error_code.val == -1:
                print("ErrorCode: ZERO_VECTOR")
        
        
        

    def execute(self, goal):
        rospy.logerr("execute")
        res = PlaceActionResult()
        base_frame = 'base_link'
        eef_frame = 'hand_palm_link'
        placement_area_det_frame = 'map'

        base_pose_map = self.moveit.get_current_pose(placement_area_det_frame)
        
        # list is sorted by number of inliers, so the first BB is most likely the table plane
        table_bb = goal.table_bbs.boxes[0]
        table_bb_stamped = bounding_box_to_bounding_box_stamped(table_bb, goal.table_bbs.header.frame_id, rospy.Time.now())
        table_bb_stamped = self.tf2_wrapper.transform_bounding_box(table_bb_stamped, base_frame)
        self.moveit.add_box('placement_table', goal.table_bbs.header.frame_id, table_bb.center, vector3_to_list(table_bb.size))

        table_equation = goal.table_plane_equations[0]
        plane_normal = np.array([table_equation.x, table_equation.y, table_equation.z])
        plane_normal = plane_normal / np.linalg.norm(plane_normal)
        aligned_table_bb = align_bounding_box_rotation(ros_bb_to_o3d_bb(table_bb_stamped))
        aligned_table_bb_ros = o3d_bb_to_ros_bb(aligned_table_bb)
        
        quat = rot_mat_to_quat(deepcopy(aligned_table_bb.R))
        
        att_objects = self.moveit.get_attached_objects()
        att_object_pose = att_objects['object'].object.pose
        att_object_dimensions = att_objects['object'].object.primitives[0].dimensions
        att_object_bb = BoundingBox3D(center = att_object_pose, size = list_to_vector3(att_object_dimensions))
        object_placement_surface_pose = goal.placement_surface_to_wrist
        
        placement_areas = self.detect_placement_areas(att_object_bb, object_placement_surface_pose, base_pose_map.pose, aligned_table_bb_ros, placement_area_det_frame)
        
        distances = []
        for placement_area in placement_areas:
            base_pose_np = np.array([base_pose_map.pose.position.x, base_pose_map.pose.position.y])
            placement_area_np = np.array([
                placement_area.center.position.x, 
                placement_area.center.position.y])
            dist_to_placement_area = np.linalg.norm(base_pose_np - placement_area_np)
            distances.append(dist_to_placement_area)
        sorted_placement_areas = [
            placement_area 
            for _, placement_area 
            in sorted(zip(distances, placement_areas), key=lambda pair: pair[0], reverse=True)
            ]
        
        
        transform = np.eye(4)
        transform[:3, :3] = quat_to_rot_mat(object_placement_surface_pose.rotation)
        transform[:3, 3] = [
            object_placement_surface_pose.translation.x, 
            object_placement_surface_pose.translation.y, 
            object_placement_surface_pose.translation.z]
        surface_to_wrist = np.linalg.inv(transform)
        

        for i, placement_area in enumerate(sorted_placement_areas):
            # TODO frame might be wrong if we change method of obtaining placement area from clicking to loading from file
            # unless it is also saved relative to map, which would make sense
            header = Header(stamp=rospy.Time.now(), frame_id= placement_area_det_frame)
            placement_point = PoseStamped(header=header, pose=placement_area.center)
            placement_point = self.tf2_wrapper.transform_pose(base_frame, placement_point)
            placement_point.pose.orientation = quat
            self.add_marker(placement_point, 5000000, 0, 0, 1)
            
            rospy.sleep(0.2)

            safety_distance = min(0.01 + i/100, 0.06)
            waypoints = []

            placement_point_rot_mat = quat_to_rot_mat(placement_point.pose.orientation)
            placement_point_transl = np.array([placement_point.pose.position.x, placement_point.pose.position.y, placement_point.pose.position.z])
            placement_point_transform = np.eye(4)
            placement_point_transform[:3, :3] = placement_point_rot_mat
            placement_point_transform[:3, 3] = placement_point_transl
            #TODO check if hand_palm_point rotation is in x dir or y dir -> use the one that show in -x dir
            hand_palm_point = placement_point_transform @ surface_to_wrist
            hand_palm_point_ros = PoseStamped(header=placement_point.header, pose=np_transform_to_ros_pose(hand_palm_point))
            
            placement_point = hand_palm_point_ros
            self.add_marker(placement_point, 5000002, 0, 1, 0)
            
            plane_normal_eef_frame = self.transform_plane_normal(table_equation, 'hand_palm_link', rospy.Time.now())
            placement_point.pose.position.x = placement_point.pose.position.x + \
                safety_distance * plane_normal_eef_frame[0]
            placement_point.pose.position.y = placement_point.pose.position.y + \
                safety_distance * plane_normal_eef_frame[1]
            placement_point.pose.position.z = placement_point.pose.position.z + \
                safety_distance * plane_normal_eef_frame[2]
            waypoints.append(deepcopy(placement_point))
            self.add_marker(placement_point, 5000001, 1, 0, 0)

            # TODO maybe move transformation to moveit lib, have access to query planning frame there
            # reverse so that poses traversed in correct order
            waypoints_tr = [self.tf2_wrapper.transform_pose('odom', waypoint) for waypoint in reversed(waypoints)]
            for i, waypoint in enumerate(waypoints_tr):
                r = 1
                g = 1
                b = 1
                self.add_marker(waypoint, i+7000, r, g, b)           
                rospy.sleep(0.2)

            plan_found = self.moveit.whole_body_plan_and_go(waypoints_tr[0])
            if not plan_found:
                rospy.loginfo("Placement: No plan found. Trying next pose")
                continue

            execution_succesful = self.moveit.current_pose_close_to_target(waypoints_tr[0])
            if not execution_succesful:
                rospy.loginfo("Placement: Execution failed, Trying next pose")
                continue

            plane_normal_eef_frame = self.transform_plane_normal(table_equation, 'hand_palm_link', rospy.Time.now())
            self.hsr_wrapper.move_eef_by_line((-plane_normal_eef_frame[0], -plane_normal_eef_frame[1], -plane_normal_eef_frame[2]), safety_distance)

            self.hsr_wrapper.gripper_open_hsr()
            break
            
        #TODO detach object when resetting or when placed: check whether that makes sense
        self.moveit.detach_all_objects()
        #TODO add arm movement in plane normal so that robot doesnt kill item when retreating
        if res:
            rospy.loginfo("Placement: Placement successful")
            self.server.set_succeeded(res)
        else:
            rospy.loginfo("Placement: Placement failed")
            self.server.set_aborted()
    
    def add_marker(self, pose_goal, id=0, r=0, g=1, b=0):
        """ publishes a grasp marker to /grasping_pipeline/grasp_marker

        Arguments:
            pose_goal {geometry_msgs.msg.PoseStamped} -- pose for the grasp marker
        """
        br = tf.TransformBroadcaster()
        br.sendTransform((pose_goal.pose.position.x, pose_goal.pose.position.y, pose_goal.pose.position.z),
                         [pose_goal.pose.orientation.x, pose_goal.pose.orientation.y,
                             pose_goal.pose.orientation.z, pose_goal.pose.orientation.w],
                         rospy.Time.now(),
                         'grasp_pose_execute',
                         pose_goal.header.frame_id)

        marker_pub = rospy.Publisher(
            '/grasp_marker_2', Marker, queue_size=100, latch=True)
        marker = Marker()
        marker.header.frame_id = pose_goal.header.frame_id
        marker.header.stamp = rospy.Time()
        marker.header.stamp = pose_goal.header.stamp
        marker.ns = 'grasp_marker'
        marker.id = id
        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        q2 = [pose_goal.pose.orientation.w, pose_goal.pose.orientation.x,
              pose_goal.pose.orientation.y, pose_goal.pose.orientation.z]
        q = tf.transformations.quaternion_about_axis(pi / 2, (0, 1, 0))
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
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker_pub.publish(marker)


if __name__ == '__main__':
    rospy.init_node('place_object_server')
    server = PlaceObjectServer()
    rospy.spin()

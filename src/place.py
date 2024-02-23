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
from v4r_util.rviz_visualization.rviz_visualizer import RvizVisualizer
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
        self.moveit = MoveitWrapper(self.tf2_wrapper, planning_time=10.0)
        self.hsr_wrapper = HSR_wrapper()

        self.server = actionlib.SimpleActionServer(
            'place_object', PlaceAction, self.execute, False)
        self.server.start()
        self.bb_vis = RvizVisualizer('grasping_pipeline/placement_debug_bb')

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

    def detect_placement_areas(self, wrist_to_table, base_pose, aligned_table_bb_base_frame, placement_area_det_frame, placement_area_bb):
        if placement_area_bb.header.frame_id != placement_area_det_frame:
            rospy.logerr("Wrong frame for placement area bb. Should probably do a transform here at some point. Expected: %s, got: %s" % (placement_area_det_frame, placement_area_bb.header.frame_id))
            raise NotImplementedError
        
        att_objects = self.moveit.get_attached_objects()
        att_object_pose = att_objects['object'].object.pose
        att_object_dimensions = att_objects['object'].object.primitives[0].dimensions
        attached_object_bb = BoundingBox3D(center = att_object_pose, size = list_to_vector3(att_object_dimensions))
        
        obj_bb_aligned = self.get_object_bb_orientation_after_placement(base_pose, attached_object_bb, wrist_to_table, aligned_table_bb_base_frame)

        target_bb = deepcopy(aligned_table_bb_base_frame)
        target_bb.center.position.z = target_bb.center.position.z + target_bb.size.z / 2
        target_point = target_bb.center.position
        box_filter_range = target_bb.size
        rospy.logwarn(f"{box_filter_range = }, target_point = {target_point}")
        target_bb = self.tf2_wrapper.transform_bounding_box(
            bounding_box_to_bounding_box_stamped(target_bb, 'base_link', rospy.Time.now()), 
            placement_area_det_frame)
        target_bb.size = placement_area_bb.size
        target_bb.center.orientation = Quaternion(x=0, y=0, z=0, w=1)
        header = Header(stamp=rospy.Time.now(), frame_id=placement_area_det_frame)
        self.bb_vis.publish_ros_bb(target_bb, header, "target_bb")
        rospy.sleep(0.05)

        target_point = target_bb.center.position
        box_filter_range = target_bb.size
        rospy.logwarn(f"{box_filter_range = }, target_point = {target_point}")

        placement_areas = self.call_placement_area_detector(obj_bb_aligned, placement_area_det_frame, target_point, box_filter_range)
        return placement_areas
        
    
    def get_object_bb_orientation_after_placement(self, base_pose, attached_object_bb, wrist_to_table, aligned_table_bb_base_frame):
        '''Get the orientation of the object bounding box after placement
        This is needed to get the correct object dimensions and orientation for the placement area detection
        '''
        # Translate the table bb center to the top of the table. 
        # This is only needed for improving the visualization 
        # (translation does not affect the orientation or object dimensions of the object bb)
        table_plane = deepcopy(aligned_table_bb_base_frame)
        table_plane.center.position.z = table_plane.center.position.z + table_plane.size.z / 2

        obj_bb_table_frame = transform_bounding_box_w_transform(attached_object_bb, wrist_to_table)
        obj_bb_table_frame = align_bounding_box_rotation(ros_bb_to_o3d_bb(obj_bb_table_frame))
        obj_bb_table_frame = o3d_bb_to_ros_bb(obj_bb_table_frame)

        world_to_base_transform = Transform(
            translation=base_pose.position, rotation=base_pose.orientation)
        base_to_table_transform = Transform(translation=table_plane.center.position, rotation=table_plane.center.orientation)

        obj_bb_base_frame = transform_bounding_box_w_transform(
            obj_bb_table_frame, base_to_table_transform)
        header = Header(stamp=rospy.Time.now(), frame_id='base_link')
        self.bb_vis.publish_ros_bb(obj_bb_base_frame, header, "obj_bb_base_frame")

        obj_bb_map_frame = transform_bounding_box_w_transform(obj_bb_base_frame, world_to_base_transform)
        header = Header(stamp=rospy.Time.now(), frame_id='map')
        self.bb_vis.publish_ros_bb(obj_bb_map_frame, header, "obj_bb_map_frame")
        
        obj_bb_map_frame_aligned = align_bounding_box_rotation(ros_bb_to_o3d_bb(obj_bb_map_frame))
        obj_bb_map_frame_aligned = o3d_bb_to_ros_bb(obj_bb_map_frame_aligned)
        header = Header(stamp=rospy.Time.now(), frame_id='map')
        self.bb_vis.publish_ros_bb(obj_bb_map_frame_aligned, header, "obj_bb_map_frame_aligned")
        
        return obj_bb_map_frame_aligned
    
    def call_placement_area_detector(self, obj_bb_aligned, placement_area_det_frame, target_point, box_filter_range):
        vertical_axis = Vector3()
        vertical_axis.x = 0.0
        vertical_axis.y = 0.0
        vertical_axis.z = 1.0

        tilt_threshold = np.pi * 60/180
        distance_threshold = 0.03

        obj_dim_x = obj_bb_aligned.size.x
        obj_dim_y = obj_bb_aligned.size.y
        obj_dim_z = obj_bb_aligned.size.z
        object_shape = Shape()
        object_shape.type = Shape.MESH
        mesh = o3d.geometry.TriangleMesh.create_box(
            obj_dim_x, obj_dim_y, obj_dim_z)
        mesh.rotate(quat_to_rot_mat(obj_bb_aligned.center.orientation))
        vertices_list = []
        for p in np.asarray(mesh.vertices):
            vertices_list.append(Point(p[0], p[1], p[2]))
        object_shape.triangles = np.asarray(mesh.triangles).flatten()
        object_shape.vertices = vertices_list

        object_to_surface = Pose()
        surface_range = 1.0

        rospy.wait_for_service('detect_placement_area')

        while True:
            try:
                # Toyota Detect Placement Area Service:
                # frame: everything (points and x,y,z axis) are relative to this frame
                # target_point x,y,z is a rough estimation of a point on a table (best if its middle point)
                # box filter range x,y,z filters pointcloud in x,y,z direction around target point
                # (if x_range = 0.2: filters out everything that is not inside target_point +- 0.1)
                # vertical axis: looks for planes that have a surface normal in the direction of the vertical axis
                # tilt_threshold: threshold for how tilted the place is allowed to be around the vertical axis in radians
                # distance_threshold: probably RANSAC plane detection inlier distance
                # object_shape: mesh of object
                # object_to_surface, translation and rotation of object compared to surface of plane,
                # basically post-processing transformation of the point that their algorithm normally returns
                # for translation just does an addition at the end to translate pose, doesn't check wether this pose is free
                # surface_range: I ran valuse from 1E22 to 1E-22, always same behaviour unless range was zero, then it did nothing
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
    
    def test_bb_plane_intersection(self, bb_pts, plane):
        '''Check if the bounding box intersects with the plane
        bb_pts: 8 xyz points of the bounding box corners in the form of an array of shape (8, 3)
        plane: object_detector_msgs/Plane
        '''
        num_pts_above_plane = 0
        for pt in bb_pts:
            object_center_to_table_distance = abs(plane.x * pt[0] + 
                                                  plane.y * pt[1] + 
                                                  plane.z * pt[2] + 
                                                  plane.d) / np.sqrt(plane.x ** 2 + plane.y ** 2 + plane.z ** 2)
            if object_center_to_table_distance > 0:
                num_pts_above_plane += 1
        # if all points are above or below the plane, the object is not intersecting with the plane
        return num_pts_above_plane == 0 or num_pts_above_plane == 8
                
    def find_intersecting_table_plane(self, table_planes, bb):
        '''Returns the index of one of the table planes that the object is intersecting with
        table_planes: list of object_detector_msgs/Plane
        bb: o3d.geometry.OrientedBoundingBox
        '''
        bb_pts = np.asarray(bb.get_box_points())
        for i, plane in enumerate(table_planes):
            if self.test_bb_plane_intersection(bb_pts, plane):
                return i
        return None
    
    def sort_placement_areas_by_distance(self, placement_areas, base_pose_map):
        distances = []
        base_pose_np = np.array([base_pose_map.pose.position.x, base_pose_map.pose.position.y])
        for placement_area in placement_areas:
            placement_area_np = np.array([
                placement_area.center.position.x, 
                placement_area.center.position.y])
            dist_to_placement_area = np.linalg.norm(base_pose_np - placement_area_np)
            distances.append(dist_to_placement_area)
        rospy.logerr(f"Distances: {distances}, {placement_areas = }")
        sorted_placement_areas = [
            placement_area 
            for _, placement_area 
            in sorted(zip(distances, placement_areas), key=lambda pair: pair[0], reverse=True)
            ]
        rospy.logerr(f"Sorted placement areas: {sorted_placement_areas}")
        return sorted_placement_areas
    
    def get_surface_to_wrist_transform(self, object_placement_surface_pose):
        transform = np.eye(4)
        transform[:3, :3] = quat_to_rot_mat(object_placement_surface_pose.rotation)
        transform[:3, 3] = [
            object_placement_surface_pose.translation.x, 
            object_placement_surface_pose.translation.y, 
            object_placement_surface_pose.translation.z]
        surface_to_wrist = np.linalg.inv(transform)
        return surface_to_wrist

    def execute(self, goal):
        rospy.loginfo("Placement: Executing")
        base_frame = 'base_link'
        eef_frame = 'hand_palm_link'
        planning_frame = 'odom'
        placement_area_det_frame = 'map'

        base_pose_map = self.moveit.get_current_pose(placement_area_det_frame)
        
        placement_area_bb = goal.placement_area_bb
        table_idx = self.find_intersecting_table_plane(goal.table_plane_equations, ros_bb_to_o3d_bb(placement_area_bb))
        if table_idx is None:
            rospy.logwarn("Placement: No table plane intersecting with placement area. Aborting")
            self.server.set_aborted()
            return

        table_bb = goal.table_bbs.boxes[table_idx]
        self.moveit.add_box('placement_table', goal.table_bbs.header.frame_id, table_bb.center, vector3_to_list(table_bb.size))

        table_equation = goal.table_plane_equations[table_idx]
        table_bb_stamped = bounding_box_to_bounding_box_stamped(table_bb, goal.table_bbs.header.frame_id, rospy.Time.now())
        table_bb_stamped = self.tf2_wrapper.transform_bounding_box(table_bb_stamped, base_frame)
        aligned_table_bb = align_bounding_box_rotation(ros_bb_to_o3d_bb(table_bb_stamped))
        aligned_table_bb_ros = o3d_bb_to_ros_bb(aligned_table_bb)
        
        quat = rot_mat_to_quat(deepcopy(aligned_table_bb.R))
        
        placement_areas = self.detect_placement_areas(goal.placement_surface_to_wrist, base_pose_map.pose, aligned_table_bb_ros, placement_area_det_frame, placement_area_bb)
        sorted_placement_areas = self.sort_placement_areas_by_distance(placement_areas, base_pose_map)
        surface_to_wrist = self.get_surface_to_wrist_transform(goal.placement_surface_to_wrist)

        execution_succesful = False
        for i, placement_area in enumerate(sorted_placement_areas):
            header = Header(stamp=rospy.Time.now(), frame_id= placement_area_det_frame)
            placement_point = PoseStamped(header=header, pose=placement_area.center)
            placement_point = self.tf2_wrapper.transform_pose(base_frame, placement_point)
            placement_point.pose.orientation = quat
            self.add_marker(placement_point, 5000000, 0, 0, 1)
            
            rospy.sleep(0.02)

            safety_distance = min(0.01 + i/100, 0.04)
            safety_distance = np.random.uniform(0.01, 0.05)
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
            
            plane_normal_eef_frame = self.transform_plane_normal(table_equation, eef_frame, rospy.Time.now())
            placement_point.pose.position.x = placement_point.pose.position.x + \
                safety_distance * plane_normal_eef_frame[0]
            placement_point.pose.position.y = placement_point.pose.position.y + \
                safety_distance * plane_normal_eef_frame[1]
            placement_point.pose.position.z = placement_point.pose.position.z + \
                safety_distance * plane_normal_eef_frame[2]
            waypoints.append(deepcopy(placement_point))
            self.add_marker(placement_point, 5000001, 1, 0, 0)

            waypoints_tr = [self.tf2_wrapper.transform_pose(planning_frame, waypoint) for waypoint in reversed(waypoints)]
            for i, waypoint in enumerate(waypoints_tr):
                r = 1
                g = 1
                b = 1
                self.add_marker(waypoint, i+7000, r, g, b)           
                rospy.sleep(0.02)

            plan_found = self.moveit.whole_body_plan_and_go(waypoints_tr[0])
            if not plan_found:
                rospy.loginfo("Placement: No plan found. Trying next pose")
                continue

            execution_succesful = self.moveit.current_pose_close_to_target(waypoints_tr[0])
            if not execution_succesful:
                rospy.loginfo("Placement: Execution failed, Trying next pose")
                continue

            plane_normal_eef_frame = self.transform_plane_normal(table_equation, eef_frame, rospy.Time.now())
            self.hsr_wrapper.move_eef_by_line((-plane_normal_eef_frame[0], -plane_normal_eef_frame[1], -plane_normal_eef_frame[2]), safety_distance)

            self.hsr_wrapper.gripper_open_hsr()
            break
            
        if execution_succesful:
            rospy.loginfo("Placement: Placement successful")
            self.moveit.detach_all_objects()
            res = PlaceActionResult()
            self.server.set_succeeded(res)
        else:
            rospy.loginfo("Placement: Placement failed")
            self.server.set_aborted()
    
    def add_marker(self, pose_goal, id=0, r=0, g=1, b=0):
        br = tf.TransformBroadcaster()
        br.sendTransform((pose_goal.pose.position.x, pose_goal.pose.position.y, pose_goal.pose.position.z),
                         [pose_goal.pose.orientation.x, pose_goal.pose.orientation.y,
                             pose_goal.pose.orientation.z, pose_goal.pose.orientation.w],
                         rospy.Time.now(),
                         'grasp_pose_execute',
                         pose_goal.header.frame_id)

        marker_pub = rospy.Publisher(
            '/grasping_pipeline/placement_marker', Marker, queue_size=100, latch=True)
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

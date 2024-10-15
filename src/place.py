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
    '''
    Server for placing an object on a table
    
    The server assumes that the object is able to be placed on the same surface that it was standing
    on when it was grasped (i.e. bottom surface). 
    The server uses the Toyota placement area detector service to detect valid placement areas for the
    object. The service takes the object dimensions and object rotation after placement into account
    to find a suitable placement area.
    The server also executes the placement action by planning a path which leads to the object being
    placed 'downwards' onto the table.

    Attributes
    ----------
    tf2_wrapper: TF2Wrapper
        Convenience wrapper for tf2_ros
    moveit: MoveitWrapper
        Convenience wrapper for MoveIt
    hsr_wrapper: HSR_wrapper
        Convenience wrapper for the Toyota HSR
    server: actionlib.SimpleActionServer
        Action server for the place_object action
    bb_vis: RvizVisualizer
        Visualizer for bounding boxes. Used to visualize an (enlarged) bounding box around the 
        target placement area and an estimation of the object rotation after placement

    Parameters
    ----------
    placement_area_bb: grasping_pipeline_msgs/BoundingBox3DStamped
        The bounding box of the target placement area/plane
    table_plane_equations: list of object_detector_msgs/Plane
        The plane equations of the detected tables. The equations are used to calculate the plane
        normals, which are then used to define 'upwards/downwards' for the object placement.
    table_bbs: vision_msgs/BoundingBox3DArray
        The bounding boxes of the detected tables. Used to find the actual bounding box of the 
        target placement area/plane
    placement_surface_to_wrist: geometry_msgs/Transform
        Transformation from the object placement surface to the wrist of the robot. This is generally
        the center of the object plane which touched the table plane, but can also be the 
        transformation to another surface plane of the object (which would lead to the object being 
        placed on that plane). This is needed to calculate the correct object dimensions and 
        orientation for the placement area detection and to calculate the correct placement point
        for the robot. 
    
    Returns
    -------
    grasping_pipeline_msgs/PlaceActionResult
        Empty. The server can be set to succeeded or aborted. It is set to succeeded if the object
        was successfully placed and to aborted if the object could not be placed on any of the
        detected placement areas. 
    '''
    
    
    def __init__(self):
        '''
        Initialize the PlaceObjectServer class.

        Creates a TF2Wrapper, MoveitWrapper, HSR_wrapper, and an action server for the place_object
        action.
        '''
        self.tf2_wrapper = TF2Wrapper()
        self.moveit = MoveitWrapper(self.tf2_wrapper, planning_time=10.0)
        self.hsr_wrapper = HSR_wrapper()

        self.server = actionlib.SimpleActionServer(
            'place_object', PlaceAction, self.execute, False)
        self.server.start()
        self.bb_vis = RvizVisualizer('grasping_pipeline/placement_debug_bb')

        rospy.loginfo("Init Placement")
    
    def transform_plane_normal(self, table_equation, target_frame, stamp):
        '''
        Transform the plane normal to the target frame and normalize it
        
        Parameters
        ----------
        table_equation: object_detector_msgs/Plane
            The plane equation
        target_frame: str
            The target frame to transform the plane normal to
        stamp: rospy.Time
            The timestamp of the transformation
            
        Returns
        -------
        np.array of shape (3,)
            The normalized plane normal in the target frame
        '''
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
        '''
        Detect valid placement areas for the object.
        
        Uses the Toyota placement area detector service to detect the placement areas. It takes 
        the object dimension and object rotation after placement into account to find a suitable
        placement area.
        
        Parameters
        ----------
        wrist_to_table: geometry_msgs/Transform
            Transformation from the wrist to the table 
        base_pose: geometry_msgs/Pose
            The current pose of the robot base
        aligned_table_bb_base_frame: vision_msgs/BoundingBox3D
            The bounding box of the table aligned to the base frame (This means that the BB x-axis 
            will be the one that aligns the best with the x-axis of the base frame, and so on for
            the other axes)
        placemend_area_det_frame: str
            The frame in which the placement area detection should be done
        placement_area_bb: vision_msgs/BoundingBox3D
            The bounding box of the target placement area/plane
            
        Returns
        -------
        list of vision_msgs/BoundingBox3D
            The detected placement areas
        '''
        if placement_area_bb.header.frame_id != placement_area_det_frame:
            placement_area_bb = self.tf2_wrapper.transform_bounding_box(placement_area_bb, placement_area_det_frame)
            #rospy.logerr("Wrong frame for placement area bb. Should probably do a transform here at some point. Expected: %s, got: %s" % (placement_area_det_frame, placement_area_bb.header.frame_id))
            #raise NotImplementedError
        
        att_objects = self.moveit.get_attached_objects()
        att_object_pose = att_objects['object'].object.pose
        att_object_dimensions = att_objects['object'].object.primitives[0].dimensions
        attached_object_bb = BoundingBox3D(center = att_object_pose, size = list_to_vector3(att_object_dimensions))
        
        obj_bb_aligned = self.get_object_bb_orientation_after_placement(base_pose, attached_object_bb, wrist_to_table, aligned_table_bb_base_frame)

        target_bb = deepcopy(aligned_table_bb_base_frame)
        target_bb.center.position.z = target_bb.center.position.z + target_bb.size.z / 2
        target_point = target_bb.center.position
        box_filter_range = target_bb.size
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

        placement_areas = self.call_placement_area_detector(obj_bb_aligned, placement_area_det_frame, target_point, box_filter_range)
        return placement_areas
        
    
    def get_object_bb_orientation_after_placement(self, base_pose, attached_object_bb, wrist_to_table, aligned_table_bb_base_frame):
        '''
        Calculates the orientation of the object bounding box that it will have after placement.
        This is needed so that we can pass the correct object dimensions and orientation for the 
        placement area detector service, so that it can find collision-free placement areas. 
        
        Parameters
        ----------
        base_pose: geometry_msgs/Pose
            The current pose of the robot base
        attached_object_bb: vision_msgs/BoundingBox3D
            The bounding box of the object that is attached to the robot (aka the object to be placed)
        wrist_to_table: geometry_msgs/Transform
            Transformation from the wrist to the table
        aligned_table_bb_base_frame: vision_msgs/BoundingBox3D
            The bounding box of the table aligned to the base frame
        
        Returns
        -------
        vision_msgs/BoundingBox3D
            The bounding box of the object aligned to the map frame (which is the frame that the
            placement area detector service uses for the detection). Only the orientation of the
            bounding box should be used, the translation can not be estimated because it is not
            known on which part of the table the object will be placed. It's only known how the
            object will be rotated after placement.
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
        '''
        Call the Toyota placement area detector service to detect valid placement areas for the object.

        The placement area detector looks for suitable placement areas on the table
        where the object can be placed. The service takes the object dimensions and object rotation
        after placement into account for finding collision-free placement areas. The service looks
        for areas inside the box_filter_range around the target_point on the table.
        
        Parameters
        ----------
        obj_bb_aligned: vision_msgs/BoundingBox3D
            The bounding box of the object aligned to the base frame
        placement_area_det_frame: str
            The frame in which the placement area detection should be done
        target_point: geometry_msgs/Point
            A rough estimation of a point on the table (best if its middle point)
        box_filter_range: geometry_msgs/Vector3
            Filters pointcloud in x,y,z direction around target point (if x_range = 0.2: filters out
            everything that is not inside target_point +- 0.1)
        
        Returns
        -------
        list of vision_msgs/BoundingBox3D
            The detected placement areas. As long as no placement areas are found, the service will
            be called again and again (aka endless loop might happen)
        '''
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
        '''
        Check if the bounding box intersects with the plane
        
        Parameters
        ----------
        bb_pts: np.array of shape (8, 3)
            xyz-points of the bounding box corners
        plane: object_detector_msgs/Plane
            The plane equation
        
        Returns
        -------
        bool
            True if the bounding box intersects with the plane, False otherwise
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
        '''
        Returns the index of one of the table planes that the bounding box is intersecting with
        
        Parameters
        ----------
        table_planes: list of object_detector_msgs/Plane
            The plane equations of the table planes which are checked for intersection with the bb
        bb: o3d.geometry.OrientedBoundingBox
            The bounding box which is checked for intersection with the table planes
        
        Returns
        -------
        int or None
            The index of the table plane that the bounding box is intersecting with. None if no 
            intersection is found
        '''
        bb_pts = np.asarray(bb.get_box_points())
        for i, plane in enumerate(table_planes):
            if self.test_bb_plane_intersection(bb_pts, plane):
                return i
        return None
    
    def sort_placement_areas_by_distance(self, placement_areas, base_pose_map):
        '''
        Sort the placement areas by distance to the robot base.
        
        Parameters
        ----------
        placement_areas: list of vision_msgs/BoundingBox3D
            The placement areas which should be sorted
        base_pose_map: geometry_msgs/PoseStamped
            The current pose of the robot base
        
        Returns
        -------
        list of vision_msgs/BoundingBox3D
            The sorted placement areas
        '''
        distances = []
        base_pose_np = np.array([base_pose_map.pose.position.x, base_pose_map.pose.position.y])
        for placement_area in placement_areas:
            placement_area_np = np.array([
                placement_area.center.position.x, 
                placement_area.center.position.y])
            dist_to_placement_area = np.linalg.norm(base_pose_np - placement_area_np)
            distances.append(dist_to_placement_area)
        sorted_placement_areas = [
            placement_area 
            for _, placement_area 
            in sorted(zip(distances, placement_areas), key=lambda pair: pair[0], reverse=False)
            ]
        return sorted_placement_areas
    
    def get_surface_to_wrist_transform(self, object_placement_surface_pose):
        '''
        Get the transformation from the object placement surface to the wrist

        Parameters
        ----------
        object_placement_surface_pose: geometry_msgs/Pose
            The pose of the object placement surface relative to the wrist coordinate frame 
            (i.e. the 'bottom' surface of the object)
        
        Returns
        -------
        np.array of shape (4, 4)
            The transformation from the object placement surface to the wrist
        '''
        transform = np.eye(4)
        transform[:3, :3] = quat_to_rot_mat(object_placement_surface_pose.rotation)
        transform[:3, 3] = [
            object_placement_surface_pose.translation.x, 
            object_placement_surface_pose.translation.y, 
            object_placement_surface_pose.translation.z]
        surface_to_wrist = np.linalg.inv(transform)
        return surface_to_wrist

    def execute(self, goal):
        '''
        Execute the place_object action.
        
        In the first step the table plane that intersects with the target placement area is found 
        (the target placement area is the bounding box of the table which is read from the config 
        file. This is sometimes a couple of centimeters off from the currently detected tables, 
        because the map is never identical for each run. Therefore we look for the table that matches
        the closest with the one we expect from the config file)

        Then the placement areas are detected. The placement areas are the areas on the table where
        the object can be placed. The placement areas are detected by the Toyota placement area 
        service. The service takes the object dimensions and object rotation after placement into
        account to find a suitable placement area.
        
        The placement areas are sorted by distance to the robot base (farthest are tried first so
        that the object is placed as far away into the shelf as possible).

        The bounding boxes are all aligned to the base frame (i.e. the coordinate frames are rotated
        in such a way that all axes are aligned with all axes from the base frame). This makes it 
        easier to calculate what part of the objects and table are in front/left/right of the robot.

        Afterwards the robot tries all placement areas one by one until it succeeds or all placement
        areas have been tried. The robot plans a path based on two waypoints: the first waypoint is
        the actual placement point and the second waypoint is directly above the placement point. 
        This leads to a motion where the object is placed "downwards" onto the table.
        
        
        Parameters
        ----------
        goal: grasping_pipeline_msgs/PlaceActionGoal
            The goal of the place_object action. Contains the bounding box of the target placement 
            area, bounding boxes and plane equations of detected tables, and the transformation from
            the object placement surface to the wrist (transformation from the center of the object 
            plane which touched the table plane, i.e. the 'bottom' surface of the object).
        '''
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
        max_placement_attempts = rospy.get_param("/grasping_pipeline/placement/max_attempts")
        for i, placement_area in enumerate(sorted_placement_areas):
            if i >= max_placement_attempts:
                rospy.logwarn(f"Placement: {i+1} poses failed. Aborting")
                break
            header = Header(stamp=rospy.Time.now(), frame_id= placement_area_det_frame)
            placement_point = PoseStamped(header=header, pose=placement_area.center)
            placement_point = self.tf2_wrapper.transform_pose(base_frame, placement_point)
            # set the orientation of the placement point to the orientation of the table (which is aligned with the base frame)
            # This is needed because the surface to wrist transformation assumes this to be the case.
            # Otherwise the object would be placed rotated by 90 degrees
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
            self.hsr_wrapper.tts_say("I could not place the object. Switching to handover.")
            rospy.logerr("Placement: Placement failed. Make sure that the robot is physically able to place the object")
            rospy.logerr("(e.g. tall objects generally can't be placed into the shelf without collisions when grasped from the top)")
            rospy.sleep(2.0)
            self.server.set_aborted()
    
    def add_marker(self, pose_goal, id=0, r=0, g=1, b=0):
        '''
        Add a marker to the RViz visualization. The topic is grasping_pipeline/placement_marker.
        
        The marker is an arrow that points in the direction of the z-axis of the pose_goal.
        
        Parameters
        ----------
        pose_goal: geometry_msgs/PoseStamped
            The pose to visualize
        id: int
            The id of the marker. Creating a new marker with the same id will overwrite the old marker.
        r: float
            The red color value of the marker
        g: float
            The green color value of the marker
        b: float
            The blue color value of the marker
            '''
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

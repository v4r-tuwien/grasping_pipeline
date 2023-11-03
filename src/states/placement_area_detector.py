import numpy as np
import rospy
import open3d as o3d
import smach
import tf
from hsrb_interface import Robot
from v4r_util.util import transform_bounding_box, transform_pose
from geometry_msgs.msg import Pose, Vector3, Point, PointStamped
from tmc_geometric_shapes_msgs.msg import Shape
from tmc_placement_area_detector.srv import DetectPlacementArea
from visualization_msgs.msg import Marker, MarkerArray
from vision_msgs.msg import BoundingBox3D


class PlacementAreaDetector(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'], output_keys=[
                             'placement_areas'], input_keys=['grasp_object_bb'])
        # read parameter from startup.yaml
        self.global_frame = rospy.get_param("/global_frame")
        self.placement_sort = rospy.get_param("/placement_sort")
        self.target_point = None
        self.counter = 0

    def clicked_point_cb(self, data):
        self.target_point = data
    
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

    def execute(self, userdata):
        bb_frame = userdata.grasp_object_bb.header.frame_id
        grasp_obj_bb = BoundingBox3D(center = userdata.grasp_object_bb.center, size = userdata.grasp_object_bb.size)
        grasp_obj_bb = transform_bounding_box(
            grasp_obj_bb, bb_frame, self.global_frame)

        while True:
            rospy.Subscriber('clicked_point', PointStamped, self.clicked_point_cb)
            while not rospy.is_shutdown():
                rospy.loginfo("Waiting for clicked point")
                if self.target_point is not None:
                    target_point = self.target_point
                    self.target_point = None
                    break
                rospy.sleep(1)

            if target_point.header.frame_id != self.global_frame:
                rospy.logerr('Havent implemented transformation yet!')
            target_point = target_point.point

            #TODO take object dim into account properly

            box_filter_range = Vector3()
            box_filter_range.x = 0.2
            box_filter_range.y = 0.2
            box_filter_range.z = 0.2

            vertical_axis = Vector3()
            vertical_axis.x = 0.0
            vertical_axis.y = 0.0
            vertical_axis.z = 1.0

            tilt_threshold = np.pi * 60/180
            distance_threshold = 0.03

            obj_dim_x = grasp_obj_bb.size.x
            obj_dim_y = grasp_obj_bb.size.y
            obj_dim_z = grasp_obj_bb.size.z
            
            rospy.logerr(f"{obj_dim_x = }, {obj_dim_y = }, {obj_dim_z = }")
            obj_dim_x = 0.1
            obj_dim_y = 0.1
            obj_dim_z = 0.1
            object_shape = Shape()
            object_shape.type = Shape.MESH
            mesh = o3d.geometry.TriangleMesh.create_box(
                obj_dim_x, obj_dim_y, obj_dim_z)
            vertices_list = []
            for p in np.asarray(mesh.vertices):
                vertices_list.append(Point(p[0], p[1], p[2]))
            object_shape.triangles = np.asarray(mesh.triangles).flatten()
            object_shape.vertices = vertices_list

            object_to_surface = Pose()
            surface_range = 1.0

            rospy.wait_for_service('detect_placement_area')
            

            print(f"{self.global_frame = }, {target_point = }, {box_filter_range = }, {vertical_axis = }, {tilt_threshold = }, {distance_threshold = }, {object_shape = }, {object_to_surface = }, {surface_range = }")
            try:
                detect_placement_area = rospy.ServiceProxy(
                    'detect_placement_area', DetectPlacementArea)
                response = detect_placement_area(self.global_frame, target_point, box_filter_range, vertical_axis,
                                                tilt_threshold, distance_threshold, object_shape, object_to_surface, surface_range)
            except rospy.ServiceException as e:
                print("DetectPlacmentAreaService call failed: %s" % e)

            userdata.placement_areas = response.placement_area

            self.counter = pub(response.placement_area, self.counter)
            if response.error_code.val == 1:
                print(response)
                return 'succeeded'
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


def pub(areas, counter):
    rospy.logwarn("publishing")
    pub = rospy.Publisher(
        '/placement/placement_marker', MarkerArray, queue_size=10, latch=True)
    pose_list = []
    marker_arr = MarkerArray()

    for count in range(counter):
        marker = Marker()
        marker.action = Marker.DELETEALL
        marker_arr.markers.append(marker)
    if areas is None:
        pub.publish(marker_arr)
        rospy.logerr('Nothing to visualize')
        return 0
    counter = 0
    for area in areas:
        counter += 1
        goal_pose = transform_pose('map', 'map', area.center)
        pose_list.append(goal_pose)
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = rospy.Time()
        marker.id = counter
        marker.pose = goal_pose.pose
        marker.type = Marker.ARROW
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.scale.x = 0.1
        marker.scale.y = 0.05
        marker.scale.z = 0.01
        marker_arr.markers.append(marker)

    pub.publish(marker_arr)
    return counter

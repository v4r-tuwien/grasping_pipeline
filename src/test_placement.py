import numpy as np
import rospy
import open3d as o3d
from hsrb_interface import Robot
from dynamic_reconfigure.server import Server
from grasping_pipeline.cfg import placementConfig
from v4r_util.util import transform_pose
from geometry_msgs.msg import Pose, Vector3, Point
from tmc_geometric_shapes_msgs.msg import Shape
from tmc_placement_area_detector.srv import DetectPlacementArea
from visualization_msgs.msg import Marker, MarkerArray


class PlacementAreaDetector():
    def __init__(self):
        # init robot
        self.robot = Robot()
        self.gripper = self.robot.try_get('gripper')
        self.whole_body = self.robot.try_get('whole_body')
        self.omni = self.robot.try_get('omni_base')

        # read parameter from startup.yaml
        # self.global_frame = rospy.get_param("/global_frame")
        # self.placement_object_dimensions = rospy.get_param(
        #     "/placement_object_dimensions")
        # self.placement_sort = rospy.get_param("/placement_sort")
        srv = Server(placementConfig, self.dyn_reconfig_callback)

    def dyn_reconfig_callback(self, config, level):
        # rospy.loginfo("""Reconfiugre Request: {int_param}, {double_param},\
        #   {str_param}, {bool_param}, {size}""".format(**config))
        self.global_frame = config['frame']
        self.target_point_x = config['target_point_x']
        self.target_point_y = config['target_point_y']
        self.target_point_z = config['target_point_z']
        self.box_filter_range_x = config['box_filter_range_x']
        self.box_filter_range_y = config['box_filter_range_y']
        self.box_filter_range_z = config['box_filter_range_z']
        self.vertical_axis_x = config['vertical_axis_x']
        self.vertical_axis_y = config['vertical_axis_y']
        self.vertical_axis_z = config['vertical_axis_z']
        self.tilt_threshold = config['tilt_threshold']
        self.distance_threshold = config['distance_threshold']
        self.obj_dim_x = config['obj_dim_x']
        self.obj_dim_y = config['obj_dim_y']
        self.obj_dim_z = config['obj_dim_z']
        self.obj_to_surface_x = config['obj_to_surface_x']
        self.obj_to_surface_y = config['obj_to_surface_y']
        self.obj_to_surface_z = config['obj_to_surface_z']
        self.obj_to_surface_quat_x = config['obj_to_surface_quat_x']
        self.obj_to_surface_quat_y = config['obj_to_surface_quat_y']
        self.obj_to_surface_quat_z = config['obj_to_surface_quat_z']
        self.obj_to_surface_quat_w = config['obj_to_surface_quat_w']
        self.surface_range = config['surface_range']
        rospy.logwarn("Updated params")
        return config

    def execute(self):

        # Detect Placement Area Service
        # string frame_id
        # geometry_msgs/Point target_point
        # geometry_msgs/Vector3 box_filter_range
        # geometry_msgs/Vector3 vertical_axis
        # float64 tilt_threshold
        # float64 distance_threshold
        # tmc_geometric_shapes_msgs/Shape object_shape
        # geometry_msgs/Pose object_to_surface
        # float64 surface_range
        rospy.logwarn("Executing")
        frame = self.global_frame
        # print(userdata.grasped_pose)
        # grasped_pose = Pose()
        # grasped_pose.position.x = 0.189611
        # grasped_pose.position.y = 0.0962238
        # grasped_pose.position.z = 0.618759
        # grasped_pose.orientation.x = -0.457528
        # grasped_pose.orientation.y = 0.122534712
        # grasped_pose.orientation.z = 0.730142
        # grasped_pose.orientation.w = -0.492488698

        frame_id = self.global_frame
        target_point = Point()
        target_point.x = self.target_point_x
        target_point.y = self.target_point_y
        target_point.z = self.target_point_z

        box_filter_range = Vector3()
        box_filter_range.x = self.box_filter_range_x
        box_filter_range.y = self.box_filter_range_y
        box_filter_range.z = self.box_filter_range_z

        vertical_axis = Vector3()
        vertical_axis.x = self.vertical_axis_x
        vertical_axis.y = self.vertical_axis_y
        vertical_axis.z = self.vertical_axis_z

        tilt_threshold = np.pi/180*self.tilt_threshold
        distance_threshold = self.distance_threshold

        object_shape = Shape()
        object_shape.type = Shape.MESH
        print(self.obj_dim_x)
        mesh = o3d.geometry.TriangleMesh.create_box(
            self.obj_dim_x, self.obj_dim_y, self.obj_dim_z)
        vertices_list = []
        for p in np.asarray(mesh.vertices):
            vertices_list.append(Point(p[0], p[1], p[2]))
        object_shape.triangles = np.asarray(mesh.triangles).flatten()
        object_shape.vertices = vertices_list

        object_to_surface = Pose()
        object_to_surface.position.x = self.obj_to_surface_x
        object_to_surface.position.y = self.obj_to_surface_y
        object_to_surface.position.z = self.obj_to_surface_z
        object_to_surface.orientation.x = self.obj_to_surface_quat_x
        object_to_surface.orientation.y = self.obj_to_surface_quat_y
        object_to_surface.orientation.z = self.obj_to_surface_quat_z
        object_to_surface.orientation.w = self.obj_to_surface_quat_w

        surface_range = self.surface_range

        rospy.wait_for_service('detect_placement_area')

        try:
            detect_placement_area = rospy.ServiceProxy(
                'detect_placement_area', DetectPlacementArea)
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
            response = detect_placement_area(frame_id, target_point, box_filter_range, vertical_axis,
                                             tilt_threshold, distance_threshold, object_shape, object_to_surface, surface_range)
        except rospy.ServiceException as e:
            print("DetectPlacmentAreaService call failed: %s" % e)
            return

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
        return


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


if __name__ == '__main__':
    rospy.init_node("test_placement", anonymous=False)
    pl = PlacementAreaDetector()
    counter = 0
    while not rospy.is_shutdown():
        rospy.sleep(2)
        area = pl.execute()
        counter = pub(area, counter)

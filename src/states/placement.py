import numpy as np
import rospy
import open3d as o3d
import smach
from hsrb_interface import Robot
from v4r_util.util import transform_pose
from geometry_msgs.msg import Pose, Vector3, Point
from tmc_geometric_shapes_msgs.msg import Shape
from tmc_placement_area_detector.srv import DetectPlacementArea


class PlacementAreaDetector(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'], output_keys=[
                             'placement_area'], input_keys=['grasped_pose'])
        # init robot
        self.robot = Robot()
        self.gripper = self.robot.try_get('gripper')
        self.whole_body = self.robot.try_get('whole_body')
        self.omni = self.robot.try_get('omni_base')

        # read parameter from startup.yaml
        self.global_frame = rospy.get_param("/global_frame")
        self.placement_object_dimensions = rospy.get_param(
            "/placement_object_dimensions")
        self.placement_sort = rospy.get_param("/placement_sort")
        self.global_frame = rospy.get_param("/global_frame")

    def execute(self, userdata):

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

        grasped_pose = userdata.grasped_pose.pose
        frame = userdata.grasped_pose.header.frame_id
        # print(userdata.grasped_pose)
        # grasped_pose = Pose()
        # grasped_pose.position.x = 0.189611
        # grasped_pose.position.y = 0.0962238
        # grasped_pose.position.z = 0.618759
        # grasped_pose.orientation.x = -0.457528
        # grasped_pose.orientation.y = 0.122534712
        # grasped_pose.orientation.z = 0.730142
        # grasped_pose.orientation.w = -0.492488698

        grasped_pose = transform_pose(
            self.global_frame, frame, grasped_pose).pose

        robot_pose = self.omni.get_pose()

        frame_id = self.global_frame
        target_point = Point()
        target_point.x = robot_pose.pos.x
        target_point.y = robot_pose.pos.y
        target_point.z = 0.55

        box_filter_range = Vector3()
        box_filter_range.x = 4.0
        box_filter_range.y = 4.0
        box_filter_range.z = 1.0

        vertical_axis = Vector3()
        vertical_axis.x = 0.0
        vertical_axis.y = 0.0
        vertical_axis.z = 1.0

        tilt_threshold = np.pi * 60/180
        distance_threshold = 1.0

        object_shape = Shape()
        object_shape.type = Shape.MESH
        mesh = o3d.geometry.TriangleMesh.create_box(
            self.placement_object_dimensions[0], self.placement_object_dimensions[1], self.placement_object_dimensions[2])
        vertices_list = []
        for p in np.asarray(mesh.vertices):
            vertices_list.append(Point(p[0], p[1], p[2]))
        object_shape.triangles = np.asarray(mesh.triangles).flatten()
        object_shape.vertices = vertices_list

        object_to_surface = Pose()
        object_to_surface.position.x = 0
        object_to_surface.position.y = 0
        object_to_surface.position.z = 0
        object_to_surface.orientation.x = 0
        object_to_surface.orientation.y = 0
        object_to_surface.orientation.z = 0
        object_to_surface.orientation.w = 1

        surface_range = 1.0

        rospy.wait_for_service('detect_placement_area')

        try:
            detect_placement_area = rospy.ServiceProxy(
                'detect_placement_area', DetectPlacementArea)
            response = detect_placement_area(frame_id, target_point, box_filter_range, vertical_axis,
                                             tilt_threshold, distance_threshold, object_shape, object_to_surface, surface_range)
        except rospy.ServiceException as e:
            print("DetectPlacmentAreaService call failed: %s" % e)

        userdata.placement_area = response.placement_area
        

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
            print("ErrorCode: NI_ACCEPTABLE_PLANE")
        elif response.error_code.val == -5:
            print("ErrorCode: INVALID_OBJECT_SURFACE")
        elif response.error_code.val == -1:
            print("ErrorCode: NON_POSITIVE")
        elif response.error_code.val == -1:
            print("ErrorCode: ZERO_VECTOR")

        return 'aborted'


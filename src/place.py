#! /usr/bin/env python3
import rospy
import actionlib

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
import tf.transformations
import tf
from v4r_util.tf2 import TF2Wrapper
from v4r_util.conversions import bounding_box_to_bounding_box_stamped, list_to_vector3
from v4r_util.util import ros_bb_to_o3d_bb, ros_bb_to_rviz_marker
from visualization_msgs.msg import Marker
from vision_msgs.msg import BoundingBox3D
from math import pi
import numpy as np
from libhsrmoveit import LibHSRMoveit
from copy import deepcopy
from transforms3d.quaternions import rotate_vector

from placement.msg import *
from geometry_msgs.msg import Vector3Stamped, Pose, Point, Quaternion
from grasping_pipeline_msgs.msg import PlaceAction, PlaceActionResult 

class PlaceObjectServer():
    def __init__(self):
        self.tf2_wrapper = TF2Wrapper()
        self.moveit = LibHSRMoveit(self.tf2_wrapper)
               
        # TODO move elsewhere
        p = PoseStamped()
        p.header.frame_id = 'base_link'
        p.header.stamp = rospy.Time.now()
        base_pose = self.tf2_wrapper.transform_pose('map', p)
        pos = base_pose.pose.position
        ori = base_pose.pose.orientation
        floor = BoundingBox3D()
        floor.center.position.x = pos.x
        floor.center.position.y = pos.y
        floor.center.position.z = -0.07
        floor.center.orientation.w = ori.w
        size = [15, 15, 0.1]
        self.moveit.add_box('floor', 'map', floor.center, size)
        rospy.sleep(0.2)

        self.server = actionlib.SimpleActionServer(
            'place_object', PlaceAction, self.execute, False)
        self.server.start()
        rospy.logerr("init")

    def sketch(self, table_bb, plane_normal, camera_frame, is_top_grasp):
        self.add_table_marker(table_bb)
        # transform table bb to tf head rgbd frame, expect robot to look from above table plane
        # so that closest point is always part of the top plane of the BB
        table_bb_o3d = ros_bb_to_o3d_bb(table_bb)
        box_pts = np.asarray(table_bb_o3d.get_box_points())
        distances = np.linalg.norm(box_pts, axis=1)
        sort_indices = np.argsort(distances)
        sorted_box_pts = box_pts[sort_indices, :]
        # TODO add check that robot is looking from above table plane, probably with angles


        closest_points = [sorted_box_pts[0, :]]
        dot_products = []
        for i in range(1, sorted_box_pts.shape[0]):
            pt = sorted_box_pts[i, :]
            vector = pt - closest_points[0]
            vector = vector / np.linalg.norm(vector)
            dot = np.dot(vector, plane_normal)
            dot_products.append(dot)
            if abs(dot) < 0.04:
                append = True
                # Prevent taking diagonal point, normally the case because shit open3d bb sucks and plane normal of bb not the same
                # as ransac plane normal :))))))
                if len(closest_points) == 2:
                    vector2 = closest_points[1] - closest_points[0]
                    vector2 = vector2 / np.linalg.norm(vector2)
                    dot = np.dot(vector2, vector)
                    if abs(dot) > 0.02:
                        rospy.loginfo("Not appending diagonal point")
                        append = False
                if append:
                    closest_points.append(pt)
            if len(closest_points) == 3:
                break
        assert len(closest_points) == 3, f"Could not find three points of top plane?????, {dot_products = }"
        quat = Quaternion(x=0, y=0, z=0, w=1)
        header = Header(stamp=rospy.Time.now(), frame_id=camera_frame)
        self.add_marker(PoseStamped(header=header, pose=Pose(position=Point(x=closest_points[0][0], y=closest_points[0][1], z=closest_points[0][2]), orientation=quat)), 40000, 1, 0, 0)
        rospy.sleep(0.2)
        self.add_marker(PoseStamped(header=header, pose=Pose(position=Point(x=closest_points[1][0], y=closest_points[1][1], z=closest_points[1][2]), orientation=quat)), 40001, 0, 1, 0)
        rospy.sleep(0.2)
        self.add_marker(PoseStamped(header=header, pose=Pose(position=Point(x=closest_points[2][0], y=closest_points[2][1], z=closest_points[2][2]), orientation=quat)), 40002, 0, 0, 1)

        # project vectors from the two points of the top plane that are not the single closest point to the camera->closest point vector
        projection_pt1 = np.dot(closest_points[0], closest_points[1])
        projection_pt2 = np.dot(closest_points[0], closest_points[2])
        rospy.logerr(f"{projection_pt1 = }, {projection_pt2 = }")
        vec1 = closest_points[1] - closest_points[0]
        vec2 = closest_points[2] - closest_points[0]
        # pt1 is behind closest point
        if projection_pt1 > projection_pt2:
            approach_vector = vec1
            support_vector = vec2
        # pt2 is behind closest point
        else:
            approach_vector = vec2
            support_vector = vec1

        approach_vector = approach_vector / np.linalg.norm(approach_vector)
        support_vector = support_vector / np.linalg.norm(support_vector)
        if is_top_grasp:
            # Want to place from above => z axis in neg plane normal direction
            z_axis = -plane_normal
            x_axis = approach_vector
            y_axis = np.cross(z_axis, x_axis)
        else:
            # side grasp => want to place from the front => in approach direction
            z_axis = approach_vector
            x_axis = plane_normal
            y_axis = np.cross(z_axis, x_axis)
        
        rot_mat = np.stack((x_axis, y_axis, z_axis), axis=1)
        assert np.linalg.det(rot_mat) > 0.99, f"det of rot mat is {np.linalg.det(rot_mat)}, prob fucked up handedness"
        # vector with smallest projection (smallest absolute length) should build the line that is perpendicular to robot
        # vector with bigger proj -> should be line that is parallel to robot 
        # use this vector with bigger proj to calculate waypoint offset of desired, final position
        return rot_mat, approach_vector
    
    def transform_plane_normal(self, table_equation, target_frame):
        # find vectors to three closest points that are not in surface normal direction (need to find three points of top plane)
        plane_normal = np.asarray([table_equation.x, table_equation.y, table_equation.z])
        plane_normal_vec3 = list_to_vector3(plane_normal)
        plane_normal_vec3_st = Vector3Stamped(header=table_equation.header, vector=plane_normal_vec3)
        plane_normal = self.tf2_wrapper.transform_vector3(target_frame, plane_normal_vec3_st)
        plane_normal = [plane_normal.vector.x, plane_normal.vector.y, plane_normal.vector.z]
        plane_normal = plane_normal / np.linalg.norm(plane_normal)

        return plane_normal

    def execute(self, goal):
        #TODO if top grasp -> placement marker + a couple of cm in surface normal direction
        #TODO if side grasp -> placement marker + more sophisticated stuff, check closest corner and lay 
        # coordinate system on that corner, use axis to determine forward direction
        # need info of object dim, place pose, top/side grasp, object dim z for shift in z
        rospy.logerr("execute")
        res = PlaceActionResult()
        is_top_grasp = False

        camera_frame = 'head_rgbd_sensor_rgb_frame'
        # list is sorted by number of inliers, so the first BB is most likely the table plane
        table_bb = goal.table_bbs.boxes[0]
        table_bb_stamped = bounding_box_to_bounding_box_stamped(table_bb, goal.table_bbs.header.frame_id, goal.table_bbs.header.stamp)
        table_bb_stamped = self.tf2_wrapper.transform_bounding_box(table_bb_stamped, camera_frame)

        table_equation = goal.table_plane_equations[0]
        plane_normal = self.transform_plane_normal(table_equation, camera_frame)
        rot_mat, approach_vector = self.sketch(table_bb_stamped, plane_normal, camera_frame, is_top_grasp)
        
        quat = self.tf2_wrapper.rotmat2quaternion(rot_mat)
        
        for i, placement_area in enumerate(goal.placement_areas):
            # TODO frame might be wrong if we change method of obtaining placement area from clicking to loading from file
            # unless it is also saved relative to map, which would make sense
            header = Header(stamp=rospy.Time.now(), frame_id= 'map')
            placement_point = PoseStamped(header=header, pose=placement_area.center)
            # TODO take plane normal as orientation, o3d unfortunately flips orientation => not consistent
            placement_point = self.tf2_wrapper.transform_pose(camera_frame, placement_point)
            placement_point.pose.orientation = quat
            
            rospy.sleep(0.2)

            q = [0, 0, 0, 1]
            
            #TODO add real obj size, transformed so that we actually take the correct dim
            obj_size = 0.1
            safety_distance = 0.05
            approach_dist = 0.15
            waypoints = []

            placement_point.pose.position.x = placement_point.pose.position.x + \
                obj_size * plane_normal[0]
            placement_point.pose.position.y = placement_point.pose.position.y + \
                obj_size * plane_normal[1]
            placement_point.pose.position.z = placement_point.pose.position.z + \
                obj_size * plane_normal[2]
            waypoints.append(deepcopy(placement_point))

            placement_point.pose.position.x = placement_point.pose.position.x + \
                safety_distance * plane_normal[0]
            placement_point.pose.position.y = placement_point.pose.position.y + \
                safety_distance * plane_normal[1]
            placement_point.pose.position.z = placement_point.pose.position.z + \
                safety_distance * plane_normal[2]
            waypoints.append(deepcopy(placement_point))

            placement_point.pose.position.x = placement_point.pose.position.x - \
                approach_dist * approach_vector[0]
            placement_point.pose.position.y = placement_point.pose.position.y - \
                approach_dist * approach_vector[1]
            placement_point.pose.position.z = placement_point.pose.position.z - \
                approach_dist * approach_vector[2]
            waypoints.append(deepcopy(placement_point))

            # TODO maybe move transformation to moveit lib, have access to query planning frame there
            # reverse so that poses traversed in correct order
            waypoints_tr = [self.tf2_wrapper.transform_pose('odom', waypoint) for waypoint in reversed(waypoints)]
            for i, waypoint in enumerate(waypoints_tr):
                if i == 0:
                    r = 1
                    g = 0
                    b = 0
                elif i == 1:
                    r = 0
                    g = 1
                    b = 0
                elif i == 2:
                    r = 0
                    g = 0
                    b = 1
                else:
                    r = 1
                    g = 1
                    b = 1
                self.add_marker(waypoint, i+7000, r, g, b)           
                rospy.sleep(0.2)

            res = self.moveit.whole_body_IK_cartesian(
                [waypoint.pose.position.x for waypoint in waypoints_tr],
                [waypoint.pose.position.y for waypoint in waypoints_tr],
                [waypoint.pose.position.z for waypoint in waypoints_tr],
                [waypoint.pose.orientation for waypoint in waypoints_tr],
                eef_step = 0.01,
                fraction_th = -1.0,
                is_avoid_obstacle = True
            )

            #res = self.moveit.whole_body_plan_and_go(waypoints_tr)

            rospy.logerr(f"{res = } is call res")
            break
            # grasp_pose.pose.position.x = grasp_pose.pose.position.x + \
            #     safety_distance * approach_vector[0]
            # grasp_pose.pose.position.y = grasp_pose.pose.position.y + \
            #     safety_distance * approach_vector[1]
            # grasp_pose.pose.position.z = grasp_pose.pose.position.z + \
            #     safety_distance * approach_vector[2]
            # self.add_marker(grasp_pose, 1, 0, 1, 0)
            # res = self.lib['moveit'].whole_body_IK_cartesian(
            #     [grasp_pose.pose.position.x],
            #     [grasp_pose.pose.position.y],
            #     [grasp_pose.pose.position.z],
            #     [grasp_pose.pose.orientation],
            #     eef_step = 0.01,
            #     fraction_th = -1.0,
            #     is_avoid_obstacle = True
            # )
            # rospy.logerr(f"{res = } is call res")
            # rospy.sleep(0.1)

            # self.add_marker(orig_pose, 2, 0, 0, 1)
            # res = self.lib['moveit'].whole_body_IK_cartesian(
            #     [orig_pose.pose.position.x],
            #     [orig_pose.pose.position.y],
            #     [orig_pose.pose.position.z],
            #     [orig_pose.pose.orientation],
            #     eef_step = 0.01,
            #     fraction_th = 0.3,
            #     is_avoid_obstacle = False
            # )
            # rospy.sleep(0.1)
            # #self.lib['moveit'].gripper_command(np.deg2rad(-40.0), is_sync=False, time=2.0)
            # self.lib['moveit'].gripper_grasp()
            # self.lib['moveit'].attach_object(goal.grasp_object_name)
            # rospy.sleep(0.1)

            # rospy.logerr(f"{res = } is call res")
            # res = self.lib['moveit'].whole_body_IK_cartesian(
            #     [orig_pose.pose.position.x],
            #     [orig_pose.pose.position.y],
            #     [orig_pose.pose.position.z + 0.05],
            #     [orig_pose.pose.orientation],
            #     eef_step = 0.01,
            #     fraction_th = 0.3,
            #     is_avoid_obstacle = False
            # )

            
            #TODO detach object when resetting or when placed
            
        self.server.set_succeeded(res)
    
    def add_table_marker(self, table_bb):
        marker_pub = rospy.Publisher(
            '/table_marker', Marker, queue_size=100, latch=True)
        marker = ros_bb_to_rviz_marker(table_bb)
        marker_pub.publish(marker)

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


def qv_mult(q, v):
    """ Rotating the vector v by quaternion q
    Arguments:
        q {list of float} -- Quaternion w,x,y,z
        v {list} -- Vector x,y,z

    Returns:
        numpy array -- rotated vector
    """
    rot_mat = tf.transformations.quaternion_matrix(q)[:3, :3]
    v = np.array(v)
    return rot_mat.dot(v)


if __name__ == '__main__':
    rospy.init_node('place_object_server')
    server = PlaceObjectServer()
    rospy.spin()

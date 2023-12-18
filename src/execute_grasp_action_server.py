#! /usr/bin/env python3


import copy
from math import pi
import numpy as np
import rospy
import actionlib
import tf.transformations
import tf
from v4r_util.tf2 import TF2Wrapper
from v4r_util.util import align_pose_rotation, get_best_aligning_axis, Axis, rotmat_around_axis
from v4r_util.conversions import ros_pose_to_np_transform, np_transform_to_ros_pose
from moveit_wrapper import MoveitWrapper
from hsr_wrapper import HSR_wrapper
from geometry_msgs.msg import Pose, PoseStamped, Transform
from visualization_msgs.msg import Marker
from grasping_pipeline_msgs.msg import ExecuteGraspAction, ExecuteGraspResult


class ExecuteGraspServer:
    def __init__(self):
        self.tf_wrapper = TF2Wrapper()
        rospy.loginfo("Execute grasp: Waiting for moveit")
        self.moveit_wrapper = MoveitWrapper(self.tf_wrapper)
        rospy.loginfo("Execute grasp: Got Moveit")
        self.hsr_wrapper = HSR_wrapper()
        
        self.server = actionlib.SimpleActionServer(
            'execute_grasp', ExecuteGraspAction, self.execute, False)
        self.server.start()
        rospy.loginfo("Execute grasp: Init")

    def execute(self, goal):
        res = ExecuteGraspResult()
        planning_frame = self.moveit_wrapper.get_planning_frame("whole_body")
        safety_distance = rospy.get_param("/safety_distance", default=0.1)

        for grasp_pose in goal.grasp_poses:
            # assumes static scene, i.e robot didn't move since grasp pose was found
            grasp_pose.header.stamp = rospy.Time.now()
            grasp_pose = self.tf_wrapper.transform_pose(planning_frame, grasp_pose)
        
            approach_pose = copy.deepcopy(grasp_pose)
            q = [grasp_pose.pose.orientation.x, grasp_pose.pose.orientation.y,
                 grasp_pose.pose.orientation.z, grasp_pose.pose.orientation.w]
            approach_vector = qv_mult(q, [0, 0, -1])
            approach_pose.pose.position.x = approach_pose.pose.position.x + \
                safety_distance * approach_vector[0] 
            approach_pose.pose.position.y = approach_pose.pose.position.y + \
                safety_distance * approach_vector[1] 
            approach_pose.pose.position.z = approach_pose.pose.position.z + \
                safety_distance * approach_vector[2]

            plan_found = self.moveit_wrapper.whole_body_plan_and_go(approach_pose)
            if not plan_found:
                rospy.logdebug("Execute grasp: No plan found, Trying next grasp pose")
                continue

            execution_succesful = self.moveit_wrapper.current_pose_close_to_target(approach_pose)
            if not execution_succesful:
                rospy.logdebug("Execute grasp: Execution failed, Trying next grasp pose")
                continue
            
            self.hsr_wrapper.move_eef_by_line((0, 0, 1), safety_distance)
            rospy.sleep(0.1)
                
            self.hsr_wrapper.gripper_grasp_hsr(0.3)
            transform = self.get_transform_from_wrist_to_object_bottom_plane(goal, planning_frame)
            res.placement_surface_to_wrist = transform
            
            touch_links = self.moveit_wrapper.get_link_names(group='gripper')
            self.moveit_wrapper.attach_object(goal.grasp_object_name, touch_links)

            self.hsr_wrapper.move_eef_by_line((0, 0, 1), -safety_distance)

            self.hsr_wrapper.gripper_grasp_hsr(0.5)
            if not self.hsr_wrapper.grasp_succesful():
                rospy.logdebug("Execute grasp: Grasp failed")
                self.moveit_wrapper.detach_all_objects()
                self.hsr_wrapper.gripper_open_hsr()
                # Abort as in this cases the robot often touched the object and changed its position
                # which invalidates the grasp pose
                self.server.set_aborted(res)
                return
            self.server.set_succeeded(res)
            return
        
        rospy.logerr("Grasping failed")
        self.server.set_aborted(res)

    def get_transform_from_wrist_to_object_bottom_plane(self, goal, planning_frame):
        object_poses = self.moveit_wrapper.get_object_poses([goal.grasp_object_name])
        object_pose = object_poses[goal.grasp_object_name]
        object_pose_st = PoseStamped(pose=object_pose)
        object_pose_st.header.frame_id = planning_frame
        object_pose_st.header.stamp = rospy.Time.now()
        
        plane_equation = goal.table_plane_equations[0]
        object_pose_table_frame = self.tf_wrapper.transform_pose(plane_equation.header.frame_id, object_pose_st)
        self.tf_wrapper.send_transform(object_pose_table_frame.header.frame_id, 'object_center', object_pose_table_frame.pose)

        object_center_to_table_distance = abs(plane_equation.x * object_pose_table_frame.pose.position.x + 
                                                  plane_equation.y * object_pose_table_frame.pose.position.y + 
                                                  plane_equation.z * object_pose_table_frame.pose.position.z + 
                                                  plane_equation.d) / np.sqrt(plane_equation.x ** 2 + plane_equation.y ** 2 + plane_equation.z ** 2)
        
        object_bottom_surface_center = copy.deepcopy(object_pose_table_frame)
        object_bottom_surface_center.pose.position.z = object_bottom_surface_center.pose.position.z - object_center_to_table_distance * plane_equation.z
        object_bottom_surface_center.pose.position.x = object_bottom_surface_center.pose.position.x - object_center_to_table_distance * plane_equation.x
        object_bottom_surface_center.pose.position.y = object_bottom_surface_center.pose.position.y - object_center_to_table_distance * plane_equation.y
        
        # Align BB axes to the robot base frame axes, so that BB z consistently points in the same direction as the robot base frame z
        object_bottom_surface_base_frame = self.tf_wrapper.transform_pose('base_link', object_bottom_surface_center)
        object_bottom_surface_base_frame_aligned = align_pose_rotation(object_bottom_surface_base_frame.pose)
        object_bottom_surface_base_frame.pose = object_bottom_surface_base_frame_aligned
        
        axes = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
        axis_names = [Axis.X, Axis.Y, Axis.Z]
        gripper_axes = [[], [], []]
        obj_axes = [[], [], []]
        obj_transform = ros_pose_to_np_transform(object_bottom_surface_base_frame.pose)
        for axis, axis_name in zip(axes, axis_names):
            gripper_axis = self.tf_wrapper.transform_3d_array('hand_palm_link', 'base_link', axis)
            gripper_axis = gripper_axis / np.linalg.norm(gripper_axis)
            gripper_axes[axis_name] = gripper_axis
            obj_axis = obj_transform[:3, :3] @ np.array(axis)
            obj_axis = obj_axis / np.linalg.norm(obj_axis)
            obj_axes[axis_name] = obj_axis
        axis_name, is_anti_parallel = get_best_aligning_axis(gripper_axes[Axis.Z], obj_axes)
        if axis_name == Axis.Y:
            rotation_axis = obj_axes[Axis.Z]
            rotation_angle = np.pi/2 * (-1 if is_anti_parallel else 1)
            print(f"Rotating around {rotation_axis} by {rotation_angle} radians")
            rot_mat = rotmat_around_axis(rotation_axis, rotation_angle)
            print(f"Rot mat: {rot_mat}")
            obj_transform[:3, :3] = rot_mat @ obj_transform[:3, :3]
            object_bottom_surface_base_frame.pose = np_transform_to_ros_pose(obj_transform)    
        
        transform = self.tf_wrapper.transform_pose('hand_palm_link', object_bottom_surface_base_frame)
        transform = Transform(rotation = transform.pose.orientation, translation = transform.pose.position)

        return transform
        

    def add_marker(self, pose_goal, id=0, r=0, g=1, b=0):
        """ publishes a grasp marker to /grasping_pipeline/grasp_marker

        Arguments:
            pose_goal {geometry_msgs.msg.PoseStamped} -- pose for the grasp marker
        """
        rospy.logerr(f"{id = }, {r = }, {g = }, {b = }")
        br = tf.TransformBroadcaster()
        br.sendTransform((pose_goal.pose.position.x, pose_goal.pose.position.y, pose_goal.pose.position.z),
                         [pose_goal.pose.orientation.x, pose_goal.pose.orientation.y,
                             pose_goal.pose.orientation.z, pose_goal.pose.orientation.w],
                         rospy.Time.now(),
                         'grasp_pose_execute',
                         pose_goal.header.frame_id)

        marker_pub = rospy.Publisher(
            '/grasp_marker_2', Marker, queue_size=10, latch=True)
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
        rospy.loginfo('grasp_marker')


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
    rospy.init_node('execute_grasp_server')
    server = ExecuteGraspServer()
    rospy.spin()

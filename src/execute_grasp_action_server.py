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
    '''
    The ExecuteGraspServer class is a ROS action server that plans and executes a grasp pose.
    
    It receives a goal with a list of grasp poses and tries to execute them one by one. If a grasp
    pose is successfully executed, the object is grasped and the server returns the transform from
    the object's bottom surface to the wrist frame. If no grasp pose is successfully executed, the
    server returns terminates with an aborted status.

    The server first moves the robot gripper to a point that is a certain safety distance away from 
    the object, before moving the gripper to the grasp pose. This is to avoid collisions with the
    object. After grasping the object, the server moves the object up by a certain distance to avoid
    collisions with the table. Finally, 
    '''
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
        # hrisi experiments, change back to 0.1 again afterwards
        safety_distance = rospy.get_param("/safety_distance", default=0.2)

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
            self.moveit_wrapper.attach_object(goal.grasp_object_name_moveit, touch_links)

            # Move the object up to avoid collision with the table
            self.hsr_wrapper.move_eef_by_delta((0, 0, 0.05))

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
        object_poses = self.moveit_wrapper.get_object_poses([goal.grasp_object_name_moveit])
        object_pose = object_poses[goal.grasp_object_name_moveit]
        object_pose_st = PoseStamped(pose=object_pose)
        object_pose_st.header.frame_id = planning_frame
        object_pose_st.header.stamp = rospy.Time.now()
        
        plane_equation = goal.table_plane_equations[0]
        object_pose_table_frame = self.tf_wrapper.transform_pose(plane_equation.header.frame_id, object_pose_st)
        self.tf_wrapper.send_transform(object_pose_table_frame.header.frame_id, 'object_center', object_pose_table_frame.pose)

        #TODO need to add table_plane_height/2 to object_center_to_table_distance, else
        # we get the distance to the middle of the tableBB, not the distance to the table surface
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

def qv_mult(q, v):
    """
    Rotates the vector v by the quaternion q

    Parameters
    ----------
    q : list of float
        Quaternion w,x,y,z
    v : list
        Vector x,y,z
        
    Returns
    -------
    numpy array
        Rotated vector (x,y,z)
    """
    rot_mat = tf.transformations.quaternion_matrix(q)[:3, :3]
    v = np.array(v)
    return rot_mat.dot(v)


if __name__ == '__main__':
    rospy.init_node('execute_grasp_server')
    server = ExecuteGraspServer()
    rospy.spin()

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
    
    The grasp poses are sorted by their orientation and distance. Top grasps are executed last.
    This means that the closest grasp pose that is not a top grasp is executed first. After 
    trying all non-top grasps, the top grasps are executed in the same order (closest first).
    This is done because placing objects into shelves is easier when the object is grasped from
    the side.

    The server first moves the robot gripper to a point that is a certain safety distance away from 
    the object, before moving the gripper to the grasp pose. This is to avoid collisions with the
    object. After grasping the object, the server moves the object up by a certain distance to avoid
    collisions with the table. Finally, the server retreats in the same direction as the approach to
    prevent collisions with other objects on the table.
    The server checks if the grasp was successful by checking the distance between the gripper tips.
    If the grasp failed the tips are very close to each other and the server aborts the action.
    
    The transformation between the object's bottom surface and the wrist frame is saved right before
    the gripper closes around the object. This transformation is needed for placement because the 
    only assumption we can make is that the object is placeable on the surface that it was standing
    on when it was grasped.
    
    The server uses the MoveitWrapper to plan and execute the robot's movements inside the collision
    environment and the HSR_wrapper to plan and execute the robot's movements without collision
    checking (and because Moveit has no move_end_effector_by_line function which produces a smooth
    motion).
    
    Attributes
    ----------
    tf_wrapper : TF2Wrapper
        A wrapper around tf2_ros to transform poses between different frames.
    moveit_wrapper : MoveitWrapper
        A wrapper around Moveit to plan and execute the robot's movements.
    hsr_wrapper : HSR_wrapper
        A wrapper around the HSR robot to plan and execute the robot's movements.
    server : actionlib.SimpleActionServer
        The ROS action server that receives the grasp pose and executes it.
    
    Parameters
    ----------
    grasp_poses: list of PoseStamped
        The list of grasp poses to execute.
    grasp_object_name_moveit: string (optional)
        The name of the object to grasp in the Moveit environment. If provided, the object is attached
        to the gripper after grasping it. The transformation 'placement_surface_to_wrist' is only computed
        when this parameter is provided. This transformation is currently only used for placement.
    table_plane_equations: list of Plane (optional)
        The plane equations of the table surfaces. The table plane equations are used to compute the
        transformation between the object's bottom surface and the wrist frame 'placement_surface_to_wrist'.
        This transformation is currently only used for placement.
    
    Other Parameters
    ----------------
    /safety_distance : float
        The distance to move the gripper away from the object before grasping it.
    
    Returns
    -------
    ExecuteGraspResult
        The result of the action server containing the transformation from the object's bottom 
        surface to the wrist frame ('placement_surface_to_wrist') and a boolean indicating if the
        grasp was a top grasp ('top_grasp'). The transformation is only computed and returned if 
        'grasp_object_name_moveit' is provided. It is currently only used/needed for placement.
        The server aborts if no grasp pose was successfully executed or if the grasp failed. 
        If the grasp was successful, the server returns succeeded and the transformation.
    '''
    def __init__(self):
        '''
        Initialize and starts the action server.
        '''
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
        '''
        Tries to execute the grasp poses in the goal one by one until a successful grasp is done.
        
        The grasp poses are sorted by their orientation and distance. Top grasps are executed last.
        This means that the closest grasp pose that is not a top grasp is executed first. After 
        trying all non-top grasps, the top grasps are executed in the same order (closest first).
        This is done because placing objects into shelves is easier when the object is grasped from
        the side.

        For each grasp pose, the server first moves the robot gripper to a point that is a certain
        safety distance away from the object, before moving the gripper to the grasp pose. This is to
        avoid collisions with the object. After grasping the object, the server moves the object up by
        a certain distance to avoid collisions with the table. Finally, the server retreats in the same
        direction as the approach to prevent collisions with other objects on the table.
        The server checks if the grasp was successful by checking the distance between the gripper tips.
        If the grasp failed the tips are very close to each other and the server aborts the action.
        
        The transformation between the object's bottom surface and the wrist frame is saved right before
        the gripper closes around the object. This transformation is needed for placement because the
        only assumption we can make is that the object is placeable on the surface that it was standing
        on when it was grasped. The information about whether the grasp was a top grasp is also saved.
        
        Parameters
        ----------
        goal : ExecuteGraspAction
            The goal containing the grasp poses to execute. Includes the grasp poses, the name of the
            object to grasp in the Moveit environment and the plane equations of the table surfaces.
        '''
        res = ExecuteGraspResult()
        planning_frame = self.moveit_wrapper.get_planning_frame("whole_body")
        # hrisi experiments, change back to 0.1 again afterwards
        safety_distance = rospy.get_param("/safety_distance", default=0.2)
        self.moveit_wrapper.clear_path_constraints()

        
        sorted_gasp_poses, is_top_grasp_array = self.sort_grasps_by_orientation(goal.grasp_poses)
        rospy.logdebug(is_top_grasp_array)

        for grasp_pose, is_top_grasp in zip(sorted_gasp_poses, is_top_grasp_array):
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
            if goal.grasp_object_name_moveit is not None and goal.grasp_object_name_moveit != "":
                transform = self.get_transform_from_wrist_to_object_bottom_plane(
                    goal.grasp_object_name_moveit, 
                    goal.table_plane_equations[0], 
                    planning_frame
                )
                res.placement_surface_to_wrist = transform
                touch_links = self.moveit_wrapper.get_link_names(group='gripper')
                self.moveit_wrapper.attach_object(goal.grasp_object_name_moveit, touch_links)

            res.top_grasp = is_top_grasp
            # Move the object up to avoid collision with the table
            self.hsr_wrapper.move_eef_by_delta((0, 0, 0.05))

            self.hsr_wrapper.move_eef_by_line((0, 0, 1), -safety_distance)

            self.hsr_wrapper.gripper_grasp_hsr(0.5)

            res.verify_grasp = not self.hsr_wrapper.grasp_succesful()

            """
            if not self.hsr_wrapper.grasp_succesful():
                rospy.logdebug("Execute grasp: Grasp failed")
                self.moveit_wrapper.detach_all_objects()
                self.hsr_wrapper.gripper_open_hsr()
                # Abort as in this cases the robot often touched the object and changed its position
                # which potentially invalidates the grasp pose
                self.server.set_aborted(res)
                return
            """
            self.server.set_succeeded(res)
            return
        
        rospy.logerr("Grasping failed")
        self.server.set_aborted(res)

    def get_transform_from_wrist_to_object_bottom_plane(self, grasp_object_name_moveit, table_plane_equation, planning_frame):
        '''
        Compute the transformation from the wrist frame to the object's bottom surface frame.

        The transformation is computed by first transforming the object's pose to the table frame
        and then computing the object's bottom surface center in the table frame. The object's bottom
        surface center is then aligned with the robot base frame axes. This alignment is done to ensure
        that the object's bottom surface z-axis consistently points in the same direction as the robot
        base frame z-axis (etc. for x and y axes). This way we can consistently compute the transformation
        between the object's bottom surface and the wrist frame when doing placement.
        The transformation between the object's bottom surface and the wrist frame is then computed.
        
        Parameters
        ----------
        goal : ExecuteGraspAction
            The goal containing the grasp poses to execute. Includes the grasp poses, the name of the
            object to grasp in the Moveit environment and the plane equations of the table surfaces.
        planning_frame : str
            The name of the planning frame in the Moveit environment.
        
        Returns
        -------
        geometry_msgs/Transform
            The transformation from the object's bottom surface to the wrist frame.
        '''
        # Get the object's pose in the planning frame
        object_poses = self.moveit_wrapper.get_object_poses([grasp_object_name_moveit])
        object_pose = object_poses[grasp_object_name_moveit]
        object_pose_st = PoseStamped(pose=object_pose)
        object_pose_st.header.frame_id = planning_frame
        object_pose_st.header.stamp = rospy.Time.now()
        
        # Transform the object's pose to the table frame
        plane_equation = table_plane_equation
        object_pose_table_frame = self.tf_wrapper.transform_pose(plane_equation.header.frame_id, object_pose_st)
        self.tf_wrapper.send_transform(object_pose_table_frame.header.frame_id, 'object_center', object_pose_table_frame.pose)

        #TODO need to add table_plane_height/2 to object_center_to_table_distance, else
        # we get the distance to the middle of the tableBB, not the distance to the table surface
        object_center_to_table_distance = abs(plane_equation.x * object_pose_table_frame.pose.position.x + 
                                                  plane_equation.y * object_pose_table_frame.pose.position.y + 
                                                  plane_equation.z * object_pose_table_frame.pose.position.z + 
                                                  plane_equation.d) / np.sqrt(plane_equation.x ** 2 + plane_equation.y ** 2 + plane_equation.z ** 2)
        
        # Compute the object's bottom surface center in the table frame
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
        # Get vectors for the x, y, z axes direction of the gripper and object (in the base frame)
        for axis, axis_name in zip(axes, axis_names):
            gripper_axis = self.tf_wrapper.transform_3d_array('hand_palm_link', 'base_link', axis)
            gripper_axis = gripper_axis / np.linalg.norm(gripper_axis)
            gripper_axes[axis_name] = gripper_axis
            obj_axis = obj_transform[:3, :3] @ np.array(axis)
            obj_axis = obj_axis / np.linalg.norm(obj_axis)
            obj_axes[axis_name] = obj_axis
        # Get object axis that aligns best with gripper z-axis
        axis_name, is_anti_parallel = get_best_aligning_axis(gripper_axes[Axis.Z], obj_axes)
        # if object-y aligns best with gripper-z, rotate around x-axis because this means
        # that the object was grasped from the front and not from the side. This rotation ensures
        # that the object is placed from the front and not from the side, which makes it much easier
        # to place the object inside the shelf.
        if axis_name == Axis.Y:
            rotation_axis = obj_axes[Axis.Z]
            rotation_angle = np.pi/2 * (-1 if is_anti_parallel else 1)
            rot_mat = rotmat_around_axis(rotation_axis, rotation_angle)
            obj_transform[:3, :3] = rot_mat @ obj_transform[:3, :3]
            object_bottom_surface_base_frame.pose = np_transform_to_ros_pose(obj_transform)    
        
        transform = self.tf_wrapper.transform_pose('hand_palm_link', object_bottom_surface_base_frame)
        transform = Transform(rotation = transform.pose.orientation, translation = transform.pose.position)

        return transform
    
    def sort_grasps_by_orientation(self, grasp_poses):
        """Sorts grasp poses by their orientation to distinguish between top and not top grasps

        Args:
            grasp_poses (geometry_msgs.PoseStamped[]): Array with grasp poses in the head_rgbd_sensor_rgb_frame

        Returns:
            geometry_msgs.PoseStamped[]: Array with the sorted grasp poses. The grasp are sorted by their orientation 
            (not top grasps first, then top grasps) and their distances to the robot (=their original order)
            list: List with False for not top grasps and True for top grasps

        """
        top_grasps = []
        not_top_grasps = []

        for grasp in grasp_poses:
            grasp.header.stamp = rospy.Time.now()

            # Define the z-axis of the grasp in the grasp frame and the negative z-axis in the map frame
            # If the angle between the two is close to 0, the grasp is a top grasp
            z_axis_grasp = np.array([0, 0, 1])
            negative_z_axis_grasp_map = np.array([0, 0, -1])

            # Transform the z-axis of the grasp to the map frame
            grasp_in_map = self.tf_wrapper.transform_pose("map", grasp)
            grasp_in_map_rot = tf.transformations.quaternion_matrix([grasp_in_map.pose.orientation.x, grasp_in_map.pose.orientation.y, grasp_in_map.pose.orientation.z, grasp_in_map.pose.orientation.w])[:3, :3]
            z_axis_grasp_map = grasp_in_map_rot.dot(z_axis_grasp)
            
            # Compute the cosine of the angle between the two z-axes
            cosine_angle = np.dot(z_axis_grasp_map, negative_z_axis_grasp_map)

            # cosine_angle ~= 1 -> angle ~= 0 -> top grasp
            if cosine_angle > 0.9:
                top_grasps.append(grasp)
            else:
                not_top_grasps.append(grasp)

        return not_top_grasps + top_grasps, [False]*len(not_top_grasps) + [True]*len(top_grasps)    
           
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

#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Copyright (c) 2022 Hibikino-Musashi@Home
# All rights reserved.

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:

#  * Redistributions of source code must retain the above copyright notice,
#  this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above copyright
#  notice, this list of conditions and the following disclaimer in the
#  documentation and/or other materials provided with the distribution.
#  * Neither the name of Hibikino-Musashi@Home nor the names of its
#  contributors may be used to endorse or promote products derived from
#  this software without specific prior written permission.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import copy
import moveit_commander
import xml.etree.ElementTree as ET

from moveit_commander.conversions import pose_to_list

import rospy
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
import moveit_msgs
import trajectory_msgs
from moveit_msgs.msg import Constraints, OrientationConstraint
from v4r_util.conversions import euler2quaternion


class MoveitWrapper:
    """Convenience Moveit Wrapper for Sasha."""

    def __init__(self, libtf, planning_time=5.0):
        """
        Initializes the MoveitWrapper.

        Parameters
        ----------
        libtf : tf_wrapper
            Instance of the tf_wrapper class
        planning_time : float, optional
            The time to plan a trajectory, by default 5.0
        """
        self.lib = {"tf": libtf}

        self.whole_body, self.gripper, self.scene, self.robot = self.init_moveit(planning_time) 

    
    def init_moveit(self, planning_time):
        '''
        Initializes the moveit commander and returns the whole_body, gripper, scene and robot objects
        
        Parameters
        ----------
        planning_time : float
            The time to plan a trajectory
        
        Returns
        -------
        whole_body : moveit_commander.MoveGroupCommander
            The whole body move group commander
        gripper : moveit_commander.MoveGroupCommander
            The gripper move group commander
        scene : moveit_commander.PlanningSceneInterface
            The planning scene interface
        robot : moveit_commander.RobotCommander
            The robot commander
        '''
        # handle non default HSR topic name :)))))))
        moveit_commander.roscpp_initialize(['joint_states:=/hsrb/joint_states'])

        # Moveit commander
        timeout_sec = 30.0
        whole_body = moveit_commander.MoveGroupCommander("whole_body", wait_for_servers=timeout_sec)
        #gripper = moveit_commander.MoveGroupCommander("gripper", wait_for_servers=timeout_sec)
        #arm = moveit_commander.MoveGroupCommander("arm", wait_for_servers=timeout_sec)
        gripper = None
        scene = moveit_commander.PlanningSceneInterface()
        robot = moveit_commander.RobotCommander()

        # Moveit settings
        whole_body.allow_replanning(True)
        whole_body.set_workspace([-6.0, -6.0, 6.0, 6.0])
        whole_body.set_num_planning_attempts(10)
        whole_body.set_planning_time(planning_time)
        whole_body.set_max_acceleration_scaling_factor(1.0)
        whole_body.set_max_velocity_scaling_factor(1.0)
        # gripper.set_max_acceleration_scaling_factor(1.0)
        # gripper.set_max_velocity_scaling_factor(1.0)
        
        return whole_body, gripper, scene, robot

    def set_whole_body_vel(self, vel=1.0):
        """
        Set the maximum speed of the whole_body.

        Parameters
        ----------
        vel : float, optional
            Maximum speed value. The value must be between 0.0 and 1.0, by default 1.0
        """
        self.whole_body.set_max_velocity_scaling_factor(float(vel))
        return

    def set_whole_body_acc(self, acc=1.0):
        """
        Set the maximum acceleration of the whole_body.

        Parameters
        ----------
        acc : float, optional
            Maximum acceleration value. The value must be between 0.0 and 1.0, by default 1.0
        """
        self.whole_body.set_max_acceleration_scaling_factor(float(acc))
        return

    def all_close(self, goal, actual, pos_tolerance, ori_tolerance):
        """
        Judges whether the actual value is within the tolerance for the goal value.

        Parameters
        ----------
        goal : list[float], geometry_msgs/Pose, geometry_msgs/PoseStamped
            The goal value
        actual : list[float], geometry_msgs/Pose, geometry_msgs/PoseStamped
            The actual value
        pos_tolerance : float
            The tolerance for the positions
        ori_tolerance : float
            The tolerance for the orientations

        Returns
        -------
        bool
            True if the value is within the tolerance
        """
        if type(goal) is list:
            for index in range(len(goal)):
                if index > 2:
                    if abs(abs(actual[index]) - abs(goal[index])) > ori_tolerance:
                        rospy.logerr(f"ori: actual: {actual[index]}, goal: {goal[index]}")
                        return False
                else:
                    if abs(actual[index] - goal[index]) > pos_tolerance:
                        rospy.logerr(f"pos: actual: {actual[index]}, goal: {goal[index]}")
                        return False

        elif type(goal) is PoseStamped:
            return self.all_close(goal.pose, actual, pos_tolerance, ori_tolerance)

        elif type(goal) is Pose:
            return self.all_close(pose_to_list(goal), pose_to_list(actual), pos_tolerance, ori_tolerance)

        return True
    
    def current_pose_close_to_target(self, target_pose, pos_tolerance=0.04, ori_tolerance=0.17):
        '''
        Checks if the current pose is close to the target pose
        
        Parameters
        ----------
        target_pose : geometry_msgs/Pose
            The target pose
        pos_tolerance : float, optional
            The tolerance for the positions, by default 0.04
        ori_tolerance : float, optional
            The tolerance for the orientations, by default 0.17
        
        Returns
        -------
        bool
            True if the current pose is close to the target pose
        '''
        current_pose = self.whole_body.get_current_pose().pose
        return self.all_close(target_pose, current_pose, pos_tolerance, ori_tolerance)
    
    def attach_object(self, object_name, touch_links=[]):
        '''
        Attaches an object to the robot eef
        
        Parameters
        ----------
        object_name : str
            The name of the object to attach. Must be a known object in the planning scene.
        touch_links : list[str], optional
            The links that are allowed to touch the object (e.g. parts of the gripper), by default []
        '''
        self.whole_body.attach_object(object_name, touch_links=touch_links)

    def detach_all_objects(self):
        '''
        Detaches all objects from the robot eef
        
        Parameters
        ----------
        object_name : str
            The name of the object to detach. Must be a known object in the planning scene.
        '''
        self.whole_body.detach_object()

    def add_box(self, name, frame, pose, size):
        """
        Add a box to the planning scene.

        Parameters
        ----------
        name : str
            The name of the box
        frame : str
            The frame of the passed pose
        pose : geometry_msgs/Point, geometry_msgs/Pose
            Pose of the box relative to the frame. If geometry_msgs/Point is passed, the orientation
            is set to (0, 0, 0, 1), otherwise the orientation is set to the passed value.
        size : list[float]
            The size of the box as (x, y, z)
        """
        box_pose = PoseStamped()
        box_pose.header.frame_id = frame
        if isinstance(pose, Point):
            box_pose.pose = Pose(position=pose, orientation=Quaternion(0, 0, 0, 1))
        elif isinstance(pose, Pose):
            box_pose.pose = pose
        self.scene.add_box(name, box_pose, size=size)

        return

    def add_cylinder(self, name, frame, pose, height, radius):
        """
        Add a cylinder to the planning scene.

        Parameters
        ----------
        name : str
            The name of the cylinder
        frame : str
            The frame of the passed pose
        pose : geometry_msgs/Point, geometry_msgs/Pose
            Pose of the cylinder relative to the frame. If geometry_msgs/Point is passed, the 
            orientation is set to (0, 0, 0, 1), otherwise the orientation is set to the passed value
        height : float
            The height of the cylinder in meters
        radius : float
            The radius of the cylinder in meters
        """
        cylinder_pose = PoseStamped()
        cylinder_pose.header.frame_id = frame
        if isinstance(pose, Point):
            cylinder_pose.pose = Pose(position=pose, orientation=Quaternion(0, 0, 0, 1))
        elif isinstance(pose, Pose):
            cylinder_pose.pose = pose
        self.scene.add_cylinder(name, cylinder_pose, height, radius)

        return 

    def add_model_from_sdf(self, frame, point, euler, sdf):
        """
        Add a model from a sdf-file to the planning scene.

        Parameters
        ----------
        frame : str
            The name of the frame
        point : geometry_msgs/Point
            The coordinates to add the model
        euler : geometry_msgs/Point
            The euler angles of the model to be added
        sdf : str
            The path to the sdf file
        """
        tree = ET.parse(sdf)
        root = tree.getroot()
        frame_rot_cnt = 0
        for model in root:
            model_name = model.attrib["name"]
            for child in model:
                if child.tag != "link":
                    continue

                for collision in child:
                    if collision.tag != "collision":
                        continue

                    parts_name = collision.attrib["name"]
                    parts_pose = collision.find("pose").text.split(" ")

                    for geometry in collision:
                        if geometry.tag != "geometry":
                            continue
                        
                        for form in geometry:
                            q = euler2quaternion((
                                float(parts_pose[3]),
                                float(parts_pose[4]),
                                float(parts_pose[5])))

                            if euler != Point(0, 0, 0):
                                self.lib["tf"].send_transform(
                                    frame + "_rot_" + str(frame_rot_cnt),
                                    frame,
                                    Pose(Point(0, 0, 0),
                                         euler2quaternion(euler)))
                                frame = frame + "_rot_" + str(frame_rot_cnt)
                                frame_rot_cnt += 1
                                rospy.sleep(0.1)
                            
                            if form.tag == "box":
                                parts_size = form.find("size").text.split(" ")

                                self.addBox(
                                    model_name + "_" + parts_name,
                                    frame,
                                    Pose(
                                        Point(
                                            point.x + float(parts_pose[0]),
                                            point.y + float(parts_pose[1]),
                                            point.z + float(parts_pose[2])),
                                        q),
                                    (float(parts_size[0]), float(parts_size[1]), float(parts_size[2])))

                            elif form.tag == "cylinder":
                                parts_height = float(form.find("length").text)
                                parts_radius = float(form.find("radius").text)

                                self.addCylinder(
                                    model_name + "_" + parts_name,
                                    frame,
                                    Pose(
                                        Point(
                                            point.x + float(parts_pose[0]),
                                            point.y + float(parts_pose[1]),
                                            point.z + float(parts_pose[2])),
                                        q),
                                    parts_height,
                                    parts_radius)

        return

    def whole_body_IK(self, x, y, z, q, frame="odom", pos_tolerance=0.02, ori_tolerance=0.17):
        """
        Plans and executes a motion to the target pose.

        Parameters
        ----------
        x : float
            Target x-coordinate of the end-effector
        y : float
            Target y-coordinate of the end-effector
        z : float
            Target z-coordinate of the end-effector
        q : geometry_msgs/Quaternion
            Target orientation of the end-effector
        frame : str, optional
            The name of the reference frame, by default "odom"
        pos_tolerance : float, optional
            The tolerance for the positions, by default 0.02. Used to check if the IK was successful
        ori_tolerance : float, optional
            The tolerance for the orientations, by default 0.17. Used to check if the IK was successful

        Returns
        -------
        bool
            True if the IK was successful, False otherwise
        """
        p = PoseStamped()
        p.header.frame_id = frame
        p.pose.position.x = x
        p.pose.position.y = y
        p.pose.position.z = z
        p.pose.orientation = q

        self.whole_body.set_pose_target(p)
        self.whole_body.go(wait=True)
        self.whole_body.stop()
        self.whole_body.clear_pose_targets()
        self.whole_body.clear_path_constraints()

        current_pose = self.whole_body.get_current_pose().pose
        return self.all_close(p, current_pose, pos_tolerance, ori_tolerance)

    def whole_body_IK_cartesian_plan(self, waypoints, is_avoid_obstacle, eef_step, fraction_th):
        '''
        Compute a sequence of waypoints that make the end-effector move in straight line segments 
        that follow the poses specified as waypoints.

        Configurations are computed for every eef_step meters.
        
        Parameters
        ----------
        waypoints : list[geometry_msgs/Pose]
            The waypoints to plan a path for. The frame of the poses should be the same as the 
            planning frame
        is_avoid_obstacle : bool
            Whether obstacles should be avoided
        eef_step : float
            The step size for the calculation
        fraction_th : float
           Threshold for the fraction of the path. The fraction itself indicates the percentage of
           how much of the path was followed. If the fraction of the plan is smaller than the value
           of the fraction threshold, the function does not return a plan.
        
        Returns
        -------
        moveit_commander.RobotTrajectory
            The computed plan 
        '''
        plan, fraction = self.whole_body.compute_cartesian_path(
            waypoints, eef_step, 0.0, is_avoid_obstacle)
        if fraction < fraction_th:
            self.whole_body.stop()
            self.whole_body.clear_pose_targets()
            self.whole_body.clear_path_constraints()
            return None
        return plan

    def whole_body_IK_cartesian(self, x, y, z, q,
                                is_avoid_obstacle=True, 
                                eef_step=0.01,
                                fraction_th=0.5,
                                pos_tolerance=0.04, 
                                ori_tolerance=0.17):
        """
        Plan and execute a motion to the target pose using inverse kinematics control of the whole body.
        
        Assumes the coordinates to be in the planning scene frame and uses Time.now() as a stamp.
    
        Parameters
        ----------
        x : float
            Target x-coordinate of the end-effector
        y : float
            Target y-coordinate of the end-effector
        z : float
            Target z-coordinate of the end-effector
        q : geometry_msgs/Quaternion
            Target orientation of the end-effector
        is_avoid_obstacle : bool, optional
            Whether obstacles should be avoided, by default True
        eef_step : float, optional
            The step size for the calculation, by default 0.01
        fraction_th : float, optional
            Threshold for the fraction of the path. The fraction itself indicates the percentage of
            how much of the path was followed. If the fraction of the plan is smaller than the value
            of the fraction threshold, the function does not return a plan, by default 0.5
        pos_tolerance : float, optional
            The tolerance for the positions, by default 0.04
        ori_tolerance : float, optional
            The tolerance for the orientations, by default 0.17
        
        Returns
        -------
        bool
            True if the IK was successfully planned and executed, False otherwise
        """
        p = Pose()
        waypoints = []
        for i in range(len(x)):
            p.position.x = x[i]
            p.position.y = y[i]
            p.position.z = z[i]
            p.orientation = q[i]
            waypoints.append(copy.deepcopy(p))
            
        plan = self.whole_body_IK_cartesian_plan(waypoints, is_avoid_obstacle, eef_step, fraction_th)
        if plan is None:
            return False

        rospy.sleep(0.1)
        self.whole_body.execute(plan)
        rospy.sleep(0.1)
        self.whole_body.stop()
        self.whole_body.clear_pose_targets()
        self.whole_body.clear_path_constraints()

        current_pose = self.whole_body.get_current_pose().pose
        if self.all_close(p, current_pose, pos_tolerance, ori_tolerance):
            return True
        else:
            return False
        
        
    def whole_body_IK_cartesian_with_timeout(self, x, y, z, q, 
                                             is_avoid_obstacle=True, 
                                             eef_step=0.01,
                                             fraction_th=0.5,
                                             pos_tolerance=0.02, 
                                             ori_tolerance=0.17,
                                             timeout=5.0):
        """
        Plans and executes a motion to the target pose for multiple coordinates with a timeout.
        
        Parameters
        ----------
        x : float
            Target x-coordinate of the end-effector
        y : float
            Target y-coordinate of the end-effector
        z : float
            Target z-coordinate of the end-effector
        q : geometry_msgs/Quaternion
            Target orientation of the end-effector
        is_avoid_obstacle : bool, optional
            Whether obstacles should be avoided, by default True
        eef_step : float, optional
            The step size for the calculation, by default 0.01
        fraction_th : float, optional
            Threshold for the fraction of the path. The fraction itself indicates the percentage of
            how much of the path was followed. If the fraction of the plan is smaller than the value
            of the fraction threshold, the function does not return a plan, by default 0.5
        pos_tolerance : float, optional
            The tolerance for the positions, by default 0.02
        ori_tolerance : float, optional
            The tolerance for the orientations, by default 0.17
        timeout : float, optional
            The timeout in seconds, by default 5.0. If the timeout is reached, the function aborts.

        Returns
        -------
        bool
            True if the IK was successfully planned and executed, False otherwise
        """
        stats = False
        start = rospy.Time.now()
        while stats is False:
            if rospy.Time.now() - start > rospy.Duration(timeout):
                rospy.logwarn("[" + rospy.get_name() + "]: whole_body_IK_cartesian is terminated by timeout.")
                return False

            stats = self.whole_body_IK_cartesian(x, y, z, q,
                                                 is_avoid_obstacle, eef_step, fraction_th,
                                                 pos_tolerance, ori_tolerance)
        return True

    def whole_body_plan_and_go(self, pose_target):
        '''
        Plans and executes a motion to the target pose
        
        Parameters
        ----------
        pose_target : geometry_msgs/PoseStamped
            The target pose
        
        Returns
        -------
        bool
            True if the plan was found and executed, False otherwise'''
        self.whole_body.set_pose_target(pose_target)
        plan_found, plan, planning_time, error_code = self.whole_body.plan()
        if plan_found:
            self.whole_body.execute(plan)
            return True
        return False
    
    def get_objects(self, object_names = []):
        '''
        Returns the objects in the planning scene
        
        Parameters
        ----------
        object_names : list[str], optional
            The names of the objects to return. If empty, all objects are returned, by default []
        
        Returns
        -------
        dict
            The objects in the planning scene. The keys are the object names and the values are the 
            objects (moveit_msgs/CollisionObject)'''
        return self.scene.get_objects(object_names)
    
    def get_object_poses(self, object_names):
        '''
        Returns the poses of the objects in the planning scene
        
        Parameters
        ----------
        object_names : list[str]
            The names of the objects to return
        
        Returns
        -------
        dict
            The poses of the objects in the planning scene. The keys are the object names and the 
            values are the poses (geometry_msgs/Pose). The frame of the poses is the planning frame.
        '''
        return self.scene.get_object_poses(object_names)

    def get_attached_objects(self, object_names = []):
        '''
        Returns the attached objects
        
        Parameters
        ----------
        object_names : list[str], optional
            The names of the objects to return. If empty, all attached objects are returned, by 
            default []
        
        Returns
        -------
        dict
            The attached objects. The keys are the object names and the values are the objects
            (moveit_msgs/AttachedCollisionObject)    
        '''
        return self.scene.get_attached_objects(object_names)

    def get_planning_frame(self, group="whole_body"):
        '''
        Returns the planning frame of the specified group
        
        Parameters
        ----------
        group : str, optional
            The name of the group, by default "whole_body"
        
        Returns
        -------
        str
            The planning frame of the specified group
        '''
        if group == "whole_body":
            return self.whole_body.get_planning_frame()
        else:
            raise NotImplementedError
    
    def gripper_open(self):
        '''
        Opens the gripper. The gripper is opened by moving the hand_motor_joint to 1.0'''
        t = moveit_msgs.msg.RobotTrajectory()
        t.joint_trajectory.joint_names = ["hand_motor_joint"]
        p = trajectory_msgs.msg.JointTrajectoryPoint()
        p.positions = [1.0]
        p.effort = [0.01]
        p.time_from_start = rospy.Duration(1.0)
        t.joint_trajectory.points.append(p)
        self.gripper.execute(t)

    def gripper_grasp(self):
        '''
        Closes the gripper to grasp an object.
        '''
        t = moveit_msgs.msg.RobotTrajectory()
        t.joint_trajectory.joint_names = ["hand_motor_joint"]
        p = trajectory_msgs.msg.JointTrajectoryPoint()
        p.positions = [0.0]
        p.effort = [-0.01]
        p.time_from_start = rospy.Duration(1.0)
        t.joint_trajectory.points.append(p)
        self.gripper.execute(t)
    
    def get_link_names(self, group = None):
        '''
        Returns the link names of the specified group
        
        Parameters
        ----------
        group : str, optional
            The name of the group, by default None
        
        Returns
        -------
        list[str]
            All link names of the specified group.
        '''
        return self.robot.get_link_names(group)
    
    def set_endeffector(self, eef):
        '''
        Sets which link should be used as the end effector
        
        Parameters
        ----------
        eef : str
            The name of the end effector link
        '''
        self.whole_body.set_end_effector_link(eef)
    
    def get_current_pose(self, frame):
        """
        Returns the current pose of the robot base in the specified frame
        
        Parameters
        ----------
        frame : str
            The frame to transform the base pose to
        
        Returns
        -------
        geometry_msgs/PoseStamped
            The current pose of the robot base in the specified frame
        """
        
        p = PoseStamped()
        p.header.frame_id = 'base_link'
        p.header.stamp = rospy.Time.now()
        p.pose.orientation.w = 1.0
        base_pose = self.lib['tf'].transform_pose(frame, p)
        return base_pose
    
    def get_current_eef_pose(self, eef_link="hand_palm_link", reference_frame="map"):
        '''
        Returns the current pose of the end effector link in the specified frame
        
        Parameters
        ----------
        eef_link : str, optional
            The name of the end effector link, by default "hand_palm_link"
        reference_frame : str, optional
            The name of the reference frame to transform the eef pose to, by default "map"
        '''
        eef_pose = self.whole_body.get_current_pose(end_effector_link=eef_link)
        eef_pose = self.lib['tf'].transform_pose(reference_frame, eef_pose)
        return eef_pose
    
    def clear_path_constraints(self):
        '''
        Clears the path constraints of the whole body
        '''
        self.whole_body.clear_path_constraints()

    def add_orientation_constraint(self, reference_frame, end_effector_link, orientation, tolerances_xyz=[0.3, 0.3, 0.3]):
        '''
        Adds an orientation constraint to the whole body until it is cleared
        
        Parameters
        ----------
        reference_frame : str
            The name of the reference frame (i.e. the frame in which the orientation constrain is defined in)
        end_effector_link : str
            The name of the end effector link which should be constrained
        orientation : geometry_msgs/Quaternion
            The target orientation of the end effector link (i.e. the eef should have this orientation during the motion)
        tolerances_xyz : list[float], optional
            The tolerances for the orientation constraint, by default [0.3, 0.3, 0.3] (radians)
    '''
    
        orientation_constraint = OrientationConstraint()
        orientation_constraint.link_name = end_effector_link
        orientation_constraint.header.frame_id = reference_frame
        orientation_constraint.orientation = orientation

        # Set tolerances for the orientation constraint
        orientation_constraint.absolute_x_axis_tolerance = tolerances_xyz[0]
        orientation_constraint.absolute_y_axis_tolerance = tolerances_xyz[1]
        orientation_constraint.absolute_z_axis_tolerance = tolerances_xyz[2]

        # Weight the constraint such that it is strictly enforced
        orientation_constraint.weight = 1.0

        # Create a Constraints object and add the orientation constraint to it
        constraints = Constraints()
        constraints.orientation_constraints.append(orientation_constraint)

        # Apply the path constraints to the move group
        self.whole_body.set_path_constraints(constraints)
    
    def set_support_surface(self, surface_name):
        '''
        Sets the support surface for the whole body
        '''
        self.whole_body.set_support_surface_name(surface_name)
    
    def place(self, object, pose):
        '''
        Places an object at the specified pose
        
        Parameters
        ----------
        object : str
            The name of the object to place
        pose : geometry_msgs/Pose
            The pose to place the object at
        
        Returns
        -------
        not None if the object was placed successfully, None otherwise
        '''
        return self.whole_body.place(object, pose)
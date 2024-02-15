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




class MoveitWrapper:
    """Moveit Wrapper used for Sasha."""

    def __init__(self, libtf, planning_time=5.0):
        """Constructor.

        Args:
            libtf (instance): TF2 library instance
        """
        self.lib = {"tf": libtf}

        self.whole_body, self.gripper, self.scene, self.robot = self.init_moveit(planning_time) 

    
    def init_moveit(self, planning_time):
        # handle non default HSR topic name :)))))))
        moveit_commander.roscpp_initialize(['joint_states:=/hsrb/joint_states'])

        # Moveit commander
        timeout_sec = 30.0
        whole_body = moveit_commander.MoveGroupCommander("whole_body", wait_for_servers=timeout_sec)
        #gripper = moveit_commander.MoveGroupCommander("gripper", wait_for_servers=timeout_sec)
        gripper = None
        scene = moveit_commander.PlanningSceneInterface()
        robot = moveit_commander.RobotCommander()

        # Moveit settings
        whole_body.allow_replanning(True)
        whole_body.set_workspace([-3.0, -3.0, 3.0, 3.0])
        whole_body.set_num_planning_attempts(10)
        whole_body.set_planning_time(planning_time)
        whole_body.set_max_acceleration_scaling_factor(1.0)
        whole_body.set_max_velocity_scaling_factor(1.0)
        # gripper.set_max_acceleration_scaling_factor(1.0)
        # gripper.set_max_velocity_scaling_factor(1.0)
        
        return whole_body, gripper, scene, robot

    def delete(self):
        self.scene.remove_world_object()
        for key in self.lib.keys():
            self.lib[key].delete()
        return

    def set_whole_body_vel(self, vel=1.0):
        """Set the maximum speed of the whole_body.

        Args:
            vel (float, optional): Maximum speed value.
                The value must be between 0.0 and 1.0. Defaults to 1.0.
        """ 
        self.whole_body.set_max_velocity_scaling_factor(float(vel))
        return

    def set_whole_body_acc(self, acc=1.0):
        """Set the maximum acceleration of the whole_body.

        Args:
            vel (float, optional): Maximum acceleration value.
                The value must be between 0.0 and 1.0. Defaults to 1.0.
        """
        self.whole_body.set_max_acceleration_scaling_factor(float(acc))
        return

    def all_close(self, goal, actual, pos_tolerance, ori_tolerance):
        """Judges whether the actual value is within the tolerance for the goal value.

        Args:
            goal (list[float], geometry_msgs/Pose, geometry_msgs/PoseStamped): The goal value.
            actual (list[float], geometry_msgs/Pose, geometry_msgs/PoseStamped): The actual value.
            pos_tolerance (float): The tolerance of positions.
            ori_tolerance (float): The tolerance of orientations.

        Returns:
            bool: Return True if the value is within the tolerance.
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
        current_pose = self.whole_body.get_current_pose().pose
        return self.all_close(target_pose, current_pose, pos_tolerance, ori_tolerance)
    
    def attach_object(self, object_name, touch_links=[]):
        self.whole_body.attach_object(object_name, touch_links=touch_links)

    def detach_all_objects(self):
        self.whole_body.detach_object()

    def add_box(self, name, frame, pose, size):
        """Add a box to the planning scene.

        Args:
            name (str): The name of box.
            frame (str): The name of frame.
            pose (geometry_msgs/Point, geometry_msgs/Pose): Coordinates to add a box.
            size (list[float]): The size of box. The size is given as a (x, y, z).
        
        Examples:
            add_box("box_0", "world", Point(0.0, 0.1, 0.2), (0.1, 0.1, 0.1))
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
        """Add a cylinder to the planning scene.

        Args:
            name (str): The name of box.
            frame (str): The name of frame.
            pose (geometry_msgs/Point, geometry_msgs/Pose): Coordinates to add a cylinder.
            height (float): The height of cylinder.
            radius (float): The radius of cylinder.
        
        Examples:
            add_cylinder("cylinder_0", "world", Point(0.0, 0.1, 0.2), (0.1, 0.1, 0.1))
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
        """Add a model from sdf to the planning scene.

        Args:
            frame (str): The name of frame.
            point (geometry_msgs/Point): Coordinates to add a model.
            euler (geometry_msgs/Point): Euler angles of the model to be added.
            sdf (str): SDF file path.
        
        Examples:
            add_model_from_sdf("world", Point(0.0, 0.1, 0.2), Point(0.0, 0.0, 1.57))
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
                            q = self.lib["tf"].euler2quaternion((
                                float(parts_pose[3]),
                                float(parts_pose[4]),
                                float(parts_pose[5])))

                            if euler != Point(0, 0, 0):
                                self.lib["tf"].send_transform(
                                    frame + "_rot_" + str(frame_rot_cnt),
                                    frame,
                                    Pose(Point(0, 0, 0),
                                         self.lib["tf"].euler2quaternion(euler)))
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
        """Inverse kinematics control of the whole body.

        Args:
            x (float): Target x of end-effector.
            y (float): Target y of end-effector.
            z (float): Target z of end-effector.
            roll (float): Target roll of end-effector.
            pitch (float): Target pitch of end-effector.
            yaw (float): Target yae of end-effector.
            frame (str, optional): The name of reference frame. Defaults to "odom".
            pos_tolerance (float): The tolerance of positions. Defaults to 0.02[m].
            ori_tolerance (float): The tolerance of orientations. Defaults to 0.17[rad].

        Returns:
            bool: Execution result.
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
        """Inverse kinematics control of the whole body for multiple coordinates.
        Expects odom frame and uses Time.now() as a stamp

        Args:
            x (float): Target x of end-effector.
            y (float): Target y of end-effector.
            z (float): Target z of end-effector.
            roll (float): Target roll of end-effector.
            pitch (float): Target pitch of end-effector.
            yaw (float): Target yae of end-effector.
            is_avoid_obstacle (str, optional): Whether or not to avoid obstacles using planning scene.
                Defaults to True.
            eef_step (float, optional): Operating calculation steps. Defaults to 0.01[m].
            fraction_th (float, optional): This function is not executed if the calculated fraction is below the threshold.
                If the calculation was incomplete, fraction indicates the percentage of paths calculated (number of waypoints passed).
                Defaults to 0.5[m].
            pos_tolerance (float): The tolerance of positions. Defaults to 0.02[m].
            ori_tolerance (float): The tolerance of orientations. Defaults to 0.17[rad].

        Returns:
            bool: Execution result.
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
        
        
    def whole_body_IK_cartesian_with_timeout(self, x, y, z, roll, pitch, yaw, 
                                             is_avoid_obstacle=True, 
                                             eef_step=0.01,
                                             fraction_th=0.5,
                                             pos_tolerance=0.02, 
                                             ori_tolerance=0.17,
                                             timeout=5.0):
        """Inverse kinematics control of the whole body for multiple coordinates with timeout.

        Args:
            x (float): Target x of end-effector.
            y (float): Target y of end-effector.
            z (float): Target z of end-effector.
            roll (float): Target roll of end-effector.
            pitch (float): Target pitch of end-effector.
            yaw (float): Target yae of end-effector.
            is_avoid_obstacle (str, optional): Whether or not to avoid obstacles using planning scene.
                Defaults to True.
            eef_step (float, optional): Operating calculation steps. Defaults to 0.01[m].
            fraction_th (float, optional): This function is not executed if the calculated fraction is below the threshold.
                If the calculation was incomplete, fraction indicates the percentage of paths calculated (number of waypoints passed).
                Defaults to 0.5[m].
            pos_tolerance (float, optional): The tolerance of positions. Defaults to 0.02[m].
            ori_tolerance (float, optional): The tolerance of orientations. Defaults to 0.17[rad].
            timeout (float, optional): Timeout time. Defaults to 5.0[sec].

        Returns:
            bool: Execution result.
        """
        stats = False
        start = rospy.Time.now()
        while stats is False:
            if rospy.Time.now() - start > rospy.Duration(timeout):
                rospy.logwarn("[" + rospy.get_name() + "]: whole_body_IK_cartesian is terminated by timeout.")
                return False

            stats = self.whole_body_IK_cartesian(x, y, z, roll, pitch, yaw,
                                                 is_avoid_obstacle, eef_step, fraction_th,
                                                 pos_tolerance, ori_tolerance)
        return True

    def whole_body_plan_and_go(self, pose_target):
        self.whole_body.set_pose_target(pose_target)
        plan_found, plan, planning_time, error_code = self.whole_body.plan()
        if plan_found:
            self.whole_body.execute(plan)
            return True
        return False
    
    def get_objects(self, object_names = []):
        return self.scene.get_objects(object_names)
    
    def get_object_poses(self, object_names):
        return self.scene.get_object_poses(object_names)

    def get_attached_objects(self, object_names = []):
        return self.scene.get_attached_objects(object_names)

    def get_planning_frame(self, group="whole_body"):
        if group == "whole_body":
            return self.whole_body.get_planning_frame()
        else:
            raise NotImplementedError
    
    def gripper_open(self):
        t = moveit_msgs.msg.RobotTrajectory()
        t.joint_trajectory.joint_names = ["hand_motor_joint"]
        p = trajectory_msgs.msg.JointTrajectoryPoint()
        p.positions = [1.0]
        p.effort = [0.01]
        p.time_from_start = rospy.Duration(1.0)
        t.joint_trajectory.points.append(p)
        self.gripper.execute(t)

    def gripper_grasp(self):
        t = moveit_msgs.msg.RobotTrajectory()
        t.joint_trajectory.joint_names = ["hand_motor_joint"]
        p = trajectory_msgs.msg.JointTrajectoryPoint()
        p.positions = [0.0]
        p.effort = [-0.01]
        p.time_from_start = rospy.Duration(1.0)
        t.joint_trajectory.points.append(p)
        self.gripper.execute(t)
    
    def get_link_names(self, group = None):
        return self.robot.get_link_names(group)
    
    def set_endeffector(self, eef):
        self.whole_body.set_end_effector_link(eef)
    
    def get_current_pose(self, frame):
        """
        Returns the current pose of the robot base in the specified frame

        frame: str, frame to transform to
        Returns: geometry_msgs/PoseStamped 
        """
        p = PoseStamped()
        p.header.frame_id = 'base_link'
        p.header.stamp = rospy.Time.now()
        p.pose.orientation.w = 1.0
        base_pose = self.lib['tf'].transform_pose(frame, p)
        return base_pose

        
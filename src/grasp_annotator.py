# Filesystem
import os
import sys

# Visualization
import open3d as o3d
import open3d_ros_helper.open3d_ros_helper as orh

# ROS
import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header

# Transformation
import tf
import transforms3d as tf3d

# Other
import numpy as np
import copy
import yaml
import time
from scipy.spatial.distance import cdist
from enum import Enum
from collections import Counter

# V4R
from v4r_util.tf2 import TF2Wrapper
from v4r_util.conversions import trans2transmat
from v4r_util.depth_pcd import filter_pcd

REACHABLE_TOLERANCE = 0.1  # 0.05
TABLE_DISTANCE_THRESHOLD = 0.01
OBJECT_DISTANCE_THRESHOLD = 0.01
VOXEL_SIZE = 0.02

# Enum for the different fail cases as well as descriptions. Can be helpful in the debugging process
class FailList(Enum):
    IN_COLLISION = 0
    NOT_REACHABLE = 1

fail_list_description = {FailList.IN_COLLISION: "Gripper is in collision with the environment.", 
                         FailList.NOT_REACHABLE: "Grasp is not Reachable"}
    
class GraspAnnotator:
    def __init__(self, pcd_filter_distance = 0.16):
        """Init function for the GraspAnnotator class. The class is used to check if a grasp is valid by checking if it is reachable and collision free.

        Args:
            pcd_filter_distance (float, optional): Distance for the filtering of the point cloud in meters. Defaults to 0.16. If 0 or None, the point cloud is not filtered.
        """
        # Check if the point cloud should be filtered
        if pcd_filter_distance is None or pcd_filter_distance == 0:
            self.filter_pcd_flag = False
            self.filter_distance = 0
        else:
            self.filter_pcd_flag = True
            self.filter_distance = pcd_filter_distance

        # Initializing other stuff
        self.fail_list = []
        self.tf2_wrapper = TF2Wrapper()

        self.dir_path = os.path.dirname(os.path.realpath(__file__))
        gripper_cloud_file = os.path.join(
            self.dir_path, os.pardir, 'config', 'hsrb_hand.pcd')
        try:
            self.gripper_cloud = o3d.io.read_point_cloud(gripper_cloud_file)
        except FileNotFoundError:
            rospy.logerr("Gripper cloud file not found. Cannot check for collisions.")
            self.gripper_cloud = None

    def is_grasp_reachable(self, grasp_pose, cam_to_base):
        """
        Checks if the grasp is reachable based on the angle of approach. It rejects grasps that approach objects in the
        negative x direction (towards the robot body) or in the positive z direction (from below).

        Args:
            grasp_pose (np.array): 4x4 transformation matrix of the grasp in the camera frame
            cam_to_base (np.array): 4x4 transformation matrix from the camera to the base

        Returns
            bool: True if the grasp pose is reachable, false otherwise
        """

        end_point_pcd = o3d.geometry.PointCloud()
        end_point_pcd.points = o3d.utility.Vector3dVector(
            np.asarray([0, 0, 0, 0, 0, 1]).reshape(2, 3))

        # Transform points of the gripper into the camera frame
        end_point_pcd.transform(grasp_pose)

        # Transform the points of the gripper from the camera frame to the base frame
        end_point_pcd.transform(cam_to_base)


        # Check the angles
        end_points = np.asarray(end_point_pcd.points)
        vx = end_points[1][0] - end_points[0][0]
        vy = end_points[1][1] - end_points[0][1]
        vz = end_points[1][2] - end_points[0][2]
        norm = np.sqrt(vx * vx + vy * vy + vz * vz)
        nx = vx / norm
        ny = vy / norm
        nz = vz / norm


        if nx < -REACHABLE_TOLERANCE:
            self.fail_list.append(FailList.NOT_REACHABLE)
            return False
        if nz > REACHABLE_TOLERANCE:
            self.fail_list.append(FailList.NOT_REACHABLE)
            return False

        return True

    def is_gripper_collision_free(self, grasp_pose, o3d_pcd, table_plane):
        """
        Checks if the grasp pose is collision free 

        Args:
            grasp_pose (np.array): 4x4 transformation matrix of the grasp in the camera frame
            o3d_pcd (open3d.geometry.PointCloud): Pointcloud of the current scene
            table_plane (np.array, optional): Parameters of the table plane (4 values). Defaults to None.
    
        Returns
            bool: True if the grasp pose is not in collision with the scene and table, false otherwise
        """
        if self.gripper_cloud is None:
            rospy.logerr("Gripper cloud not loaded. Cannot check for collisions.")
            return False

        filtered_o3d_pcd = filter_pcd(grasp_pose[:3, 3], o3d_pcd, self.filter_distance)

        if self.filter_pcd_flag:
            scene_tree = o3d.geometry.KDTreeFlann(filtered_o3d_pcd)
        else:
            scene_tree = o3d.geometry.KDTreeFlann(o3d_pcd)

        # Transform the gripper
        gripper_cloud = copy.deepcopy(self.gripper_cloud)
        gripper_cloud.transform(grasp_pose)
        gripper_points = np.asarray(gripper_cloud.points)

        # Find if any gripper point is within a threshold to the table plane
        if table_plane is not None:
            for pt in gripper_points:
                d = self.point_to_plane_distance(pt, table_plane)
                if d < TABLE_DISTANCE_THRESHOLD:
                    self.fail_list.append(FailList.IN_COLLISION)
                    return False

        if self.filter_pcd_flag:
            if len(filtered_o3d_pcd.points) == 0:
                rospy.logwarn("Filtered pcd is empty. No detections expected.")
                return True
        
        # Find nearest point in scene for each gripper point and return collision if any point is closer than threshold
        for pt in gripper_points:
            [_, idx, _] = scene_tree.search_knn_vector_3d(pt, 1)

            if self.filter_pcd_flag:
                scene_pt = np.asarray(filtered_o3d_pcd.points)[idx[0], :]
            else:
                scene_pt = np.asarray(o3d_pcd.points)[idx[0], :]
            
            d = np.linalg.norm(pt - scene_pt)
            if d < OBJECT_DISTANCE_THRESHOLD:
                self.fail_list.append(FailList.IN_COLLISION)
                return False

        # Otherwise passed all checks and no collision detected
        return True

    def is_grasp_valid(self, grasp_pose, o3d_pcd, table_plane, cam_to_base=None):
        """
        Check if the grasp is valid by checking if it is reachable and collision free.

        Args:
            grasp_pose (np.array): 4x4 transformation matrix of the grasp in the camera frame
            o3d_pcd (open3d.geometry.PointCloud): Pointcloud of the current scene
            table_plane (np.array, optional): Parameters of the table plane (4 values). Defaults to None.
            cam_to_base (np.array, optional):  4x4 transformation matrix from the camera to the base

        Returns:
            bool: True if the grasp pose is valid (reachable and collision free), false otherwise
        """

        # If the grasp is not reachable, then cannot generate a valid pose
        if cam_to_base is not None and not self.is_grasp_reachable(grasp_pose, cam_to_base):
            return False

        return self.is_gripper_collision_free(grasp_pose, o3d_pcd, table_plane)
    

    @staticmethod
    def point_to_plane_distance(point, plane):
        dist = abs((plane[0] * point[0] + plane[1] *
                   point[1] + plane[2] * point[2] + plane[3]))
        e = np.sqrt(plane[0] * plane[0] + plane[1]
                    * plane[1] + plane[2] * plane[2])
        return dist/e
    


    def annotate(self, object_pose_stamped, scene_pcd, object_name, table_plane = None):
        """
        Goes through all annotated grasps from the .npy file and checks if they are valid. Returns the valid grasps.

        Args:
            object_pose_stamped (geometry_msgs.PoseStamped): Pose of the object to grasp
            scene_pcd (sensor_msgs.PointCloud2): Pointcloud of the current scene created by open3d_ros_helper from the depth image
            object_name (string): Name of the object
            table_plane (np.array, optional): Parameters of the table plane (4 values). Defaults to None.

        Returns:
            geometry_msgs.PoseStamped[]: List of valid grasp poses
        """
        o3d_pcd = orh.rospc_to_o3dpc(scene_pcd, True).voxel_down_sample(voxel_size=VOXEL_SIZE)
        if table_plane is None:
            rospy.logwarn("Table plane not provided, not explicitly checking for clearance from the table.")

        # Get the transformation from the camera to the base
        trans = self.tf2_wrapper.get_transform_between_frames(source_frame="head_rgbd_sensor_rgb_frame", target_frame="map")
        cam_to_base = trans2transmat(trans)

        # Get the transformation from the head to the wrist
        head_to_wrist_trans = self.tf2_wrapper.get_transform_between_frames("wrist_flex_link", "head_rgbd_sensor_rgb_frame", timeout=5)
        head_to_wrist_mat = trans2transmat(head_to_wrist_trans)

        # Get the object pose in a 4x4 transformation matrix
        rot = tf3d.quaternions.quat2mat([object_pose_stamped.pose.orientation.w, object_pose_stamped.pose.orientation.x,
                                        object_pose_stamped.pose.orientation.y, object_pose_stamped.pose.orientation.z])
        object_pose = np.eye(4)
        object_pose[:3, :3] = rot
        object_pose[:3, 3] = [object_pose_stamped.pose.position.x,
                            object_pose_stamped.pose.position.y, object_pose_stamped.pose.position.z]

        dataset = rospy.get_param("/grasping_pipeline/dataset")
        # Load the grasps from the .npy file
        grasps_path = os.path.join(
            self.dir_path, os.pardir, 'grasps', dataset, object_name +'.npy')

        try:
            grasp_poses = np.load(grasps_path)
        except FileNotFoundError:
            rospy.logerr(f"Numpy file with grasps for object {object_name} not found (Either the object name is wrong or the grasps have not been annotated yet).")
            return []
        
        grasp_poses = grasp_poses.reshape((grasp_poses.shape[0], 4, 4))
        
        # Go through all grasps and check if they are valid
        valid_grasp_poses = []
        distances = []

        for pose in grasp_poses:

            grasp_try = np.matmul(object_pose, pose)

            # Check if the grasp is valid, if not continue
            res = self.is_grasp_valid(grasp_try, o3d_pcd, table_plane, cam_to_base=cam_to_base)
            if res is False:
                continue

            # Calculate the distance from the wrist to the grasp
            grasp_try_wrist = np.matmul(head_to_wrist_mat, grasp_try)   
            dist = np.linalg.norm(grasp_try_wrist[:3, 3])
            
            # Create a PoseStamped message for the valid grasp
            pose = PoseStamped()
            pose.header.frame_id = 'head_rgbd_sensor_rgb_frame'
            pose.pose.position.x = grasp_try[0, 3]
            pose.pose.position.y = grasp_try[1, 3]
            pose.pose.position.z = grasp_try[2, 3]
            quat = tf.transformations.quaternion_from_matrix(grasp_try)
            pose.pose.orientation.x = quat[0]
            pose.pose.orientation.y = quat[1]
            pose.pose.orientation.z = quat[2]
            pose.pose.orientation.w = quat[3]
            
            # Append the valid grasp and distances to the list
            distances.append(dist)
            valid_grasp_poses.append(pose)

        distances = np.array(distances)
        valid_grasp_poses = np.array(valid_grasp_poses)
        
        # Sort the grasps by distance to wrist
        sorted_indices = np.argsort(distances)
        sorted_valid_grasp_poses = valid_grasp_poses[sorted_indices]

        if len(sorted_valid_grasp_poses) == 0:
            fail_counts = Counter(self.fail_list)
        
            # Print the descriptions with their counts
            for reason, count in fail_counts.items():
                description = fail_list_description[reason]
                rospy.logwarn(f"{description} ({count}x)")
        else:
            rospy.loginfo(f"Found {len(sorted_valid_grasp_poses)} valid grasps")
            
        return sorted_valid_grasp_poses

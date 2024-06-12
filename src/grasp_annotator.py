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
    
# NEW VERSION OF THE GRASP CHECKER
class GraspAnnotator:
    def __init__(self, object_pose, scene_pcd, object_name, filter_distance = 0.16, table_plane = None, filter_pcd_flag=True):
        """
        Init of the GraspAnnotator

        Args:
            object_pose (geometry_msgs.msg._PoseStamped.PoseStamped): Pose of the object to grasp
            scene_pcd (sensor_msgs.msg._PointCloud2.PointCloud2): Pointcloud of the current scene created by open3d_ros_helper from the depth image
            object_name (string): Name of the object
            filter_distance (float, optional): Distance for the filtering of the point cloud in meters. Defaults to 0.16.
            table_plane (np.array, optional): Parameters of the table plane (4 values). Defaults to None.
            filter_pcd_flag (bool, optional): If True, the point cloud is filtered around the grasping point. Defaults to True.
        """

        # Saving passed parameters
        self.object_pose_msg = object_pose
        self.filter_pcd_flag = filter_pcd_flag
        self.ros_pcd = scene_pcd
        self.o3d_pcd = orh.rospc_to_o3dpc(self.ros_pcd, True).voxel_down_sample(voxel_size=VOXEL_SIZE)
        self.object_name = object_name
        self.filter_distance = filter_distance
        self.table_plane = table_plane

        if self.table_plane is None:
            rospy.logwarn("Table plane not provided, not explicitly checking for clearance from the table.")

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
            self.valid_poses = []
            return None
        
        # Annotate the grasps
        self.annotate_grasp_hsr()


    def filter_pcd(self, center_point):
        """
        Filters the point cloud around a center point with a given threshold. The idea is to reduce the computational time of the collision check,
        by filtering the pointcloud so that only points close to the grasping point are considered, thus reducing the computational time of the kd-tree search.

        Args:
            center_point (np.array): x, y, z coordinates of the current grasping points
        """
        distances = np.linalg.norm(np.asarray(self.o3d_pcd.points) - center_point, axis=1)
        within_threshold_mask = distances < self.filter_distance
        filtered_point_cloud = self.o3d_pcd.select_by_index(np.where(within_threshold_mask)[0])
        self.filtered_o3d_pcd = filtered_point_cloud

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

    def is_gripper_collision_free(self, grasp_pose):
        """
        Checks if the grasp pose is collision free 

        Args:
            grasp_pose (np.array): 4x4 transformation matrix of the grasp in the camera frame
    
        Returns
            bool: True if the grasp pose is not in collision with the scene and table, false otherwise
        """
        if self.filter_pcd_flag:
            scene_tree = o3d.geometry.KDTreeFlann(self.filtered_o3d_pcd)
        else:
            scene_tree = o3d.geometry.KDTreeFlann(self.o3d_pcd)

        # Transform the gripper
        gripper_cloud = copy.deepcopy(self.gripper_cloud)
        gripper_cloud.transform(grasp_pose)
        gripper_points = np.asarray(gripper_cloud.points)

        # Find if any gripper point is within a threshold to the table plane
        if self.table_plane is not None:
            for pt in gripper_points:
                d = self.point_to_plane_distance(pt, self.table_plane)
                if d < TABLE_DISTANCE_THRESHOLD:
                    self.fail_list.append(FailList.IN_COLLISION)
                    return False

        if self.filter_pcd_flag:
            if len(self.filtered_o3d_pcd.points) == 0:
                rospy.logwarn("Filtered pcd is empty. No detections expected.")
                return True
        
        # Find nearest point in scene for each gripper point and return collision if any point is closer than threshold
        for pt in gripper_points:
            [_, idx, _] = scene_tree.search_knn_vector_3d(pt, 1)

            if self.filter_pcd_flag:
                scene_pt = np.asarray(self.filtered_o3d_pcd.points)[idx[0], :]
            else:
                scene_pt = np.asarray(self.o3d_pcd.points)[idx[0], :]
            
            d = np.linalg.norm(pt - scene_pt)
            if d < OBJECT_DISTANCE_THRESHOLD:
                self.fail_list.append(FailList.IN_COLLISION)
                return False

        # Otherwise passed all checks and no collision detected
        return True

    def is_grasp_valid(self, grasp_pose, cam_to_base=None):
        """
        Check if the grasp is valid by checking if it is reachable and collision free.

        Args:
            grasp_pose (np.array): 4x4 transformation matrix of the grasp in the camera frame
            cam_to_base (np.array, optional):  4x4 transformation matrix from the camera to the base

        Returns:
            bool: True if the grasp pose is valid (reachable and collision free), false otherwise
        """

        # If the grasp is not reachable, then cannot generate a valid pose
        if cam_to_base is not None and not self.is_grasp_reachable(grasp_pose, cam_to_base):
            return False

        self.filter_pcd(grasp_pose[:3, 3])

        return self.is_gripper_collision_free(grasp_pose)


    @staticmethod
    def point_to_plane_distance(point, plane):
        dist = abs((plane[0] * point[0] + plane[1] *
                   point[1] + plane[2] * point[2] + plane[3]))
        e = np.sqrt(plane[0] * plane[0] + plane[1]
                    * plane[1] + plane[2] * plane[2])
        return dist/e

    def annotate_grasp_hsr(self):
        """
        Goes through all annotated grasps from the .npy file and checks if they are valid. Saves the valid grasps in self.valid_poses
        """

        # In tf2.py -> rospy.Time(0) statt rospy.Time().now()

        # Get the transformation from the camera to the base
        trans = self.tf2_wrapper.get_transform_between_frames(source_frame="head_rgbd_sensor_rgb_frame", target_frame="map")
        cam_to_base = self.tf2_wrapper.trans2transmat(trans)

        # Get the transformation from the head to the wrist
        head_to_wrist_trans = self.tf2_wrapper.get_transform_between_frames("wrist_flex_link", "head_rgbd_sensor_rgb_frame", timeout=5)
        head_to_wrist_mat = self.tf2_wrapper.trans2transmat(head_to_wrist_trans)

        # Get the object pose in a 4x4 transformation matrix
        rot = tf3d.quaternions.quat2mat([self.object_pose_msg.pose.orientation.w, self.object_pose_msg.pose.orientation.x,
                                        self.object_pose_msg.pose.orientation.y, self.object_pose_msg.pose.orientation.z])
        object_pose = np.eye(4)
        object_pose[:3, :3] = rot
        object_pose[:3, 3] = [self.object_pose_msg.pose.position.x,
                            self.object_pose_msg.pose.position.y, self.object_pose_msg.pose.position.z]

        # Load the grasps from the .npy file
        grasps_path = os.path.join(
            self.dir_path, os.pardir, 'grasps', self.object_name +'.npy')

        try:
            grasp_poses = np.load(grasps_path)
        except FileNotFoundError:
            rospy.logerr(f"Numpy file with grasps for object {self.object_name} not found (Either the object name is wrong or the grasps have not been annotated yet).")
            self.valid_poses = []
        
        grasp_poses = grasp_poses.reshape((grasp_poses.shape[0], 4, 4))
        
        # Go through all grasps and check if they are valid
        valid_grasp_poses = []
        distances = []

        for i, pose in enumerate(grasp_poses):

            grasp_try = np.matmul(object_pose, pose)

            # Check if the grasp is valid, if not continue
            res = self.is_grasp_valid(grasp_try, cam_to_base=cam_to_base)
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
            
        self.valid_poses = sorted_valid_grasp_poses



if __name__ == "__main__":
    # Call f.e. using python3 grasp_cecker.py Test1 0

    multipleRuns = True

    rospy.init_node("grasp_checker")

    if len(sys.argv) < 3:
        rospy.logwarn("No run_id passed, choosing '2'.")
        run_id = 2
    if len(sys.argv) < 2:
        rospy.logwarn("No test folder passed, choosing 'Test1'.")
        test_folder = "Test1"
    else:
        test_folder = sys.argv[1]
        try:
            run_id = int(sys.argv[2])
        except:
            rospy.logerr("Passed run_id not an integer. Using default value (2).")
            run_id = 2


    current_file_path = os.path.abspath(__file__)
    grasping_pipeline_folder = os.path.dirname(os.path.dirname(current_file_path))
    test_dir = os.path.join(grasping_pipeline_folder, "Tests", test_folder)

    if not os.path.exists(test_dir):
        print("Path does not exist.")
        sys.exit()
    
    with open(os.path.join(test_dir, "data.txt"), "r") as file:
        yaml_file = yaml.safe_load(file)

    object_name = yaml_file["object_name"]
    frame_id = yaml_file["object_to_grasp_stamped"]["header"]["frame_id"]
    stamp = yaml_file["object_to_grasp_stamped"]["header"]["stamp"]
    stamp_secs = stamp["secs"]
    stamp_nsecs = stamp["nsecs"]

    pose_stamped_msg = PoseStamped()
    pose_stamped_msg.header = Header(seq=yaml_file["object_to_grasp_stamped"]["header"]["seq"], stamp=rospy.Time(stamp_secs, stamp_nsecs), frame_id=frame_id)
    pose_stamped_msg.pose.position.x = yaml_file["object_to_grasp_stamped"]["pose"]["position"]["x"]
    pose_stamped_msg.pose.position.y = yaml_file["object_to_grasp_stamped"]["pose"]["position"]["y"]
    pose_stamped_msg.pose.position.z = yaml_file["object_to_grasp_stamped"]["pose"]["position"]["z"]
    pose_stamped_msg.pose.orientation.x = yaml_file["object_to_grasp_stamped"]["pose"]["orientation"]["x"]
    pose_stamped_msg.pose.orientation.y = yaml_file["object_to_grasp_stamped"]["pose"]["orientation"]["y"]
    pose_stamped_msg.pose.orientation.z = yaml_file["object_to_grasp_stamped"]["pose"]["orientation"]["z"]
    pose_stamped_msg.pose.orientation.w = yaml_file["object_to_grasp_stamped"]["pose"]["orientation"]["w"]

    o3d_pcd = o3d.io.read_point_cloud(os.path.join(test_dir, "scene.ply"))
    ros_pcd = orh.o3dpc_to_rospc(o3d_pcd, frame_id = frame_id, stamp=stamp)
    
    if multipleRuns:
        zero_time=0
        one_time=0
        two_time=0
        three_time=0
        loop_len = 20
        for i in range(loop_len):


            

            start = time.time()
            grasp_poses_zero = GraspAnnotator(pose_stamped_msg, ros_pcd, object_name, None, visualize=True, run_id=0).valid_poses
            zero_time += (time.time() - start)
            
            start = time.time()
            grasp_poses_one = GraspAnnotator(pose_stamped_msg, ros_pcd, object_name, None, visualize=True, run_id=1).valid_poses
            one_time += (time.time() - start)
            
            start = time.time()
            grasp_poses_two = GraspAnnotator(pose_stamped_msg, ros_pcd, object_name, None, visualize=True, run_id=2).valid_poses
            two_time += (time.time() - start)

            
        print(" --- RUN TIMES ---")
        print(f"run_id 0: {zero_time/loop_len}s") # Average of 2s
        print(f"run_id 1: {one_time/loop_len}s") # average of 0.25s
        print(f"run_id 2: {two_time/loop_len}s") # average of 0.28s

        print(" --- BEST GRASP POSE ---")
        print(f"run_id 0: {grasp_poses_one[0].pose}")
        print(f"run_id 1: {grasp_poses_two[0].pose}") 
        print(f"run_id 2: {grasp_poses_two[0].pose}") 
    else:
        start = time.time()
        grasp_poses = GraspAnnotator(pose_stamped_msg, ros_pcd, object_name, None, visualize=True, run_id=run_id).valid_poses
        print(f"--- DONE AFTER {time.time() - start}s ---")
        

             


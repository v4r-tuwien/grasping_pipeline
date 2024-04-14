import numpy as np
import open3d as o3d
import copy
import os

REACHABLE_TOLERANCE = 0.1  # 0.05
TABLE_DISTANCE_THRESHOLD = 0.01
OBJECT_DISTANCE_THRESHOLD = 0.01


class GraspChecker:
    def __init__(self):
        self.dir_path = os.path.dirname(os.path.realpath(__file__))
        gripper_cloud_file = os.path.join(
            self.dir_path, os.pardir, 'config', 'hsrb_hand.pcd')
        self.gripper_cloud = o3d.io.read_point_cloud(gripper_cloud_file)
        self.scene_cloud = None

    def set_scene_data(self, scene_cloud):
        self.scene_cloud = scene_cloud

    def is_grasp_reachable(self, grasp_pose, cam_to_base):
        """
        Checks if the grasp is reachable based on the angle of approach. It rejects grasps that approach objects in the
        negative x direction (towards the robot body) or in the positive z direction (from below).

        Parameters
        ----------
        grasp_pose: np.array
            4x4 transformation matrix of the grasp in the camera frame
        cam_to_base: np.array
            4x4 transformation matrix from the camera to the base

        Returns
        -------
        bool
            True if the grasp pose is reachable, false otherwise
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

        '''
        end_point_pcd.paint_uniform_color([1.0, 0.0, 0.0])
        geometries = [end_point_pcd]
        geometries.append(o3d.geometry.create_mesh_coordinate_frame(size=0.5))

        gripper_cloud = copy.deepcopy(self.gripper_cloud)
        gripper_cloud.transform(grasp_pose)
        gripper_cloud.transform(cam_to_base)
        gripper_cloud.paint_uniform_color([0.0, 1.0, 0.0])
        geometries.append(gripper_cloud)

        scene_cloud = copy.deepcopy(self.scene_cloud)
        scene_cloud.transform(cam_to_base)
        scene_cloud.paint_uniform_color([0.5, 0.5, 0.5])
        geometries.append(scene_cloud)

        o3d.visualization.draw_geometries(geometries)
        '''

        if nx < -REACHABLE_TOLERANCE:
            return False
        if nz > REACHABLE_TOLERANCE:
            return False

        return True

    def is_gripper_collision_free(self, scene, grasp_pose, table_plane=None):
        """
        Checks if the grasp pose is collision free in the scene

        Parameters
        ----------
        scene: tuple
            Point cloud (o3d.geometry.PointCloud) and KDTree (o3d.geometry.KDTreeFlann) of the scene
        grasp_pose: np.array
            4x4 transformation matrix of the grasp in the camera frame
        table_plane: np.array, optional
            Table plane parameters (4 values)

        Returns
        -------
        bool
            True if the grasp pose is not in collision with the scene and table, false otherwise
        """

        # Transform the gripper
        gripper_cloud = copy.deepcopy(self.gripper_cloud)
        gripper_cloud.transform(grasp_pose)
        gripper_points = np.asarray(gripper_cloud.points)

        # Find if any gripper point is within a threshold to the table plane
        if table_plane is None:
            print(
                'WARNING: Table plane not provided, not explicitly checking for clearance from the table')
        else:
            for pt in gripper_points:
                d = self.point_to_plane_distance(pt, table_plane)
                if d < TABLE_DISTANCE_THRESHOLD:
                    return False

        # Find nearest point in scene for each gripper point and return collision if any point is closer than threshold
        scene_cloud, scene_tree = scene
        for pt in gripper_points:
            [_, idx, _] = scene_tree.search_knn_vector_3d(pt, 1)
            scene_pt = np.asarray(scene_cloud.points)[idx[0], :]
            d = np.linalg.norm(pt - scene_pt)
            if d < OBJECT_DISTANCE_THRESHOLD:
                return False

        # Otherwise passed all checks and no collision detected
        return True

    def is_grasp_valid(self, scene_cloud, grasp_pose, table_plane=None, cam_to_base=None):
        """
        Checks if the grasp is reachable and adjusts it by moving away from the approach direction if it is initially in
        collision until a collision-free pose is found.

        Parameters
        ----------
        scene_cloud: o3d.geometry.PointCloud
            Point cloud of the observable scene
        grasp_pose: np.array
            4x4 transformation matrix of the grasp in the camera frame
        table_plane: np.array, optional
            Table plane parameters (4 values)
        cam_to_base: np.array, optional
            4x4 transformation matrix from the camera to the base

        Returns
        -------
        bool
            True if the grasp pose is valid (reachable and collision free), false otherwise
        """

        # If the grasp is not reachable, then cannot generate a valid pose
        if cam_to_base is None:
            pass
        else:
            if not self.is_grasp_reachable(grasp_pose, cam_to_base):
                return False

        # Create a KD tree for the scene
        scene_tree = o3d.geometry.KDTreeFlann(scene_cloud)

        # If this is collision free, return the pose
        if self.is_gripper_collision_free((scene_cloud, scene_tree), grasp_pose, table_plane):
            return True

        # If no valid grasp was found, then return zero matrix
        return False

    def visualize(self, scene_cloud, grasp_poses, successes):
        np.save('scene_cloud.npy', np.asarray(scene_cloud.points))
        scene_cloud.paint_uniform_color([0.5, 0.5, 0.5])
        geometries = [scene_cloud]
        geometries.append(
            o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5))
        for i in range(len(grasp_poses)):
            g_cloud = copy.deepcopy(self.gripper_cloud)
            g_cloud.transform(grasp_poses[i])
            if successes[i]:
                g_cloud.paint_uniform_color([0, 1, 0])
            else:
                g_cloud.paint_uniform_color([1, 0, 0])
            geometries.append(g_cloud)
            np.save('g_cloud'+str(i), np.asarray(g_cloud.points))

    @staticmethod
    def point_to_plane_distance(point, plane):
        dist = abs((plane[0] * point[0] + plane[1] *
                   point[1] + plane[2] * point[2] + plane[3]))
        e = np.sqrt(plane[0] * plane[0] + plane[1]
                    * plane[1] + plane[2] * plane[2])
        return dist/e


def get_tf_transform(origin_frame, target_frame):
    import tf2_ros
    import rospy
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    tf_found = False
    while not tf_found:
        try:
            trans = tfBuffer.lookup_transform(
                origin_frame, target_frame, rospy.Time(0))
            tf_found = True
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr(e)
            rospy.sleep(0.2)
    return trans


def get_transmat_from_tf_trans(trans):
    import numpy as np
    import transforms3d as tf3d
    rot = tf3d.quaternions.quat2mat(
        [trans.transform.rotation.w, trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z])
    transmat = np.eye(4)
    transmat[:3, :3] = rot
    transmat[:3, 3] = [trans.transform.translation.x,
                       trans.transform.translation.y, trans.transform.translation.z]
    return transmat


def check_grasp_hsr(pose_odm, scene_cloud_ros, name=None, table_plane=None, visualize=False):
    """
    Takes a object pose (in '/map' frame) and detects which of the saved grasps are reachable with the GraspChecker.

    Parameters
    ----------
    pose_odm: object_detector_msgs.msg.PoseWithConfidence
        object pose, with attributes pose, name and confidence
    scene_cloud_ros: sensor_msgs.msg.PointCloud2
        Pointcloud of the scene
    table_plane: np.array, optional
        Table plane parameters (4 values)
    visualize: bool, optional
        if True, then scene cloud and gripper cloud are published for RViz. Default is False
    
    Returns
    -------
    valid_poses: geometry_msgs.msg.PoseStamped[]
        List of valid grasp poses
    """
    from open3d_ros_helper import open3d_ros_helper as orh
    import tf2_ros
    import tf
    import transforms3d as tf3d
    import rospy
    from sensor_msgs.msg import PointCloud2
    from geometry_msgs.msg import PoseStamped

    grasp_checker = GraspChecker()
    o3dcloud = orh.rospc_to_o3dpc(scene_cloud_ros, True)
    grasp_checker.set_scene_data(o3dcloud)

    trans = get_tf_transform('map', 'head_rgbd_sensor_rgb_frame')

    cam_to_base = get_transmat_from_tf_trans(trans)

    rot = tf3d.quaternions.quat2mat([pose_odm.pose.orientation.w, pose_odm.pose.orientation.x,
                                    pose_odm.pose.orientation.y, pose_odm.pose.orientation.z])
    object_pose = np.eye(4)
    object_pose[:3, :3] = rot
    object_pose[:3, 3] = [pose_odm.pose.position.x,
                          pose_odm.pose.position.y, pose_odm.pose.position.z]
    if name is None:
        name = pose_odm.name
    grasps_path = os.path.join(
        grasp_checker.dir_path, os.pardir, 'grasps', name +'.npy')

    rospy.loginfo(grasps_path)
    grasp_poses = np.load(grasps_path)
    grasp_poses = grasp_poses.reshape((grasp_poses.shape[0], 4, 4))
    successes = []
    grasp_trials = []
    index = -1
    shortest_dist = 1.0
    dist_to_wrist = []
    grasp_poses_ros = []

    trans = get_tf_transform(
        'wrist_flex_link', 'head_rgbd_sensor_rgb_frame')

    cam_to_wrist = get_transmat_from_tf_trans(trans)

    # make wrist's cam look up
    trans_to_world = get_tf_transform(
        'base_link', 'wrist_flex_link')
    wrist_to_world = get_transmat_from_tf_trans(trans_to_world)

    base_to_cam = get_tf_transform(
        'head_rgbd_sensor_rgb_frame', 'base_link')
    base_to_cam = get_transmat_from_tf_trans(base_to_cam)

    counter = 1
    for pose, i in zip(grasp_poses, range(len(grasp_poses))):
        grasp_try = np.eye(4)
        grasp_try = np.matmul(object_pose, pose)
        res = grasp_checker.is_grasp_valid(
            o3dcloud, grasp_try, table_plane=table_plane, cam_to_base=cam_to_base)
        
        grasp_try_wrist = np.matmul(cam_to_wrist, grasp_try)

        # make wrist's cam look up

        grasp_in_world = np.matmul(wrist_to_world, grasp_try_wrist)
        x, y, z = tf3d.euler.mat2euler(grasp_in_world[:3, :3], 'sxyz')

        #if z > 0.0:
        #    z -= np.pi
        #    rospy.logerr("{} out of {} - Turn cam up".format(counter, len(grasp_poses)))
        #    grasp_in_world[:3, :3] = tf3d.euler.euler2mat(x, y, z, 'sxyz')
        #    grasp_try = np.matmul(base_to_cam, grasp_in_world)
        #else:
        #    rospy.logerr("{} out of {} - Cam is correct".format(counter, len(grasp_poses)))
        
        counter += 1

        dist = np.linalg.norm(grasp_try_wrist[:3, 3])
        dist_to_wrist.append(dist)
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
        # Add to array
        grasp_poses_ros.append(pose)
        if res and (dist < shortest_dist):
            shortest_dist = dist
            index = i
        grasp_trials.append(grasp_try)
        successes.append(res)
    grasps = zip(grasp_poses_ros, grasp_trials, successes, dist_to_wrist)
    grasps = list(grasps)

    grasps = np.array(sorted(grasps, key=lambda x: (-x[2], x[3])))

    dist_to_wrist = list(grasps[:, 3])
    valid_poses = list(grasps[:, 0][grasps[:, 2].astype(bool)])

    if visualize == True:
        scenepub = rospy.Publisher(
            '/grasping_pipeline/grasp_checker/scenecloud', PointCloud2, queue_size=10, latch=True)
        gripperpub = rospy.Publisher(
            '/grasping_pipeline/grasp_checker/grippercloud', PointCloud2, queue_size=10, latch=True)
        chosenpub = rospy.Publisher(
            '/grasping_pipeline/grasp_checker/chosengrippercloud', PointCloud2, queue_size=10, latch=True)
        scenepub.publish(scene_cloud_ros)
        o3dcloud.paint_uniform_color([0.5, 0.5, 0.5])
        geometries = [o3dcloud]
        geometries.append(
            o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5))
        combined_grasps = o3d.geometry.PointCloud()
        for i in range(len(grasp_poses)):
            g_cloud = copy.deepcopy(grasp_checker.gripper_cloud)
            g_cloud.transform(grasp_poses[i])
            g_cloud.transform(object_pose)
            if successes[i]:
                g_cloud.paint_uniform_color([0, 1, 1])
            else:
                g_cloud.paint_uniform_color([1, 0, 0])
            if i == index:
                g_cloud.paint_uniform_color([0, 1, 0])
                chosenpub.publish(orh.o3dpc_to_rospc(
                    g_cloud, 'head_rgbd_sensor_rgb_frame', rospy.Time(0)))

            combined_grasps += g_cloud

        gripperpub.publish(orh.o3dpc_to_rospc(
            combined_grasps, 'head_rgbd_sensor_rgb_frame', rospy.Time(0)))
    return valid_poses

import numpy as np
import open3d as o3d
import copy
import random

REACHABLE_TOLERANCE = 0.3#0.05
TABLE_DISTANCE_THRESHOLD = 0.01
OBJECT_DISTANCE_THRESHOLD = 0.01


class GraspChecker:
    def __init__(self):
        # gripper_cloud_file = rospy.get_param('gripper_cloud_filename')
        gripper_cloud_file = '/home/v4r/Markus_L/src/grasping_pipeline/scripts/hand_open_new.pcd'
        self.gripper_cloud = o3d.io.read_point_cloud(gripper_cloud_file)
        self.scene_cloud = None

        # Advertise service
        # self.service = rospy.Service('grasp_checker', get_grasp, self.service_callback)
        # rospy.loginfo('Service ready...')

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
        end_point_pcd.points = o3d.utility.Vector3dVector(np.asarray([0, 0, 0, 0, 0, 1]).reshape(2, 3))

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
            print('Unreachable because Y direction is negative {}'.format(nx))
            return False
        if nz > REACHABLE_TOLERANCE:
            print('Unreachable because Z direction is positive {}'.format(nz))
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
            print('WARNING: Table plane not provided, not explicitly checking for clearance from the table')
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
            print('WARNING: No transform from camera to base provided, cannot check reachability')
        else:
            if not self.is_grasp_reachable(grasp_pose, cam_to_base):
                print('Not reachable')
                return False

        # Create a KD tree for the scene
        scene_tree = o3d.geometry.KDTreeFlann(scene_cloud)

        # If this is collision free, return the pose
        if self.is_gripper_collision_free((scene_cloud, scene_tree), grasp_pose, table_plane):
            print('Valid')
            return True

        # If no valid grasp was found, then return zero matrix
        print('Collision detected')
        return False

#    def service_callback(self, req):
#        rospy.loginfo('GPD service called')

    def visualize(self, scene_cloud, grasp_poses, successes):
        np.save('scene_cloud.npy', np.asarray(scene_cloud.points))
        scene_cloud.paint_uniform_color([0.5, 0.5, 0.5])
        geometries = [scene_cloud]
        geometries.append(o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5))
        for i in range(len(grasp_poses)):
            g_cloud = copy.deepcopy(self.gripper_cloud)
            g_cloud.transform(grasp_poses[i])
            if successes[i]:
                g_cloud.paint_uniform_color([0, 1, 0])
            else:
                g_cloud.paint_uniform_color([1, 0, 0])
            geometries.append(g_cloud)
            np.save('g_cloud'+str(i), np.asarray(g_cloud.points))

        #o3d.visualization.draw_geometries(geometries)

    @staticmethod
    def point_to_plane_distance(point, plane):
        dist = abs((plane[0] * point[0] + plane[1] * point[1] + plane[2] * point[2] + plane[3]))
        e = np.sqrt(plane[0] * plane[0] + plane[1] * plane[1] + plane[2] * plane[2])
        return dist/e


def points_to_plane(points):
    import pcl

    cloud = pcl.PointCloud()
    cloud.from_array(points.astype(np.float32))
    seg = cloud.make_segmenter()  # _normals(ksearch=100)
    seg.set_optimize_coefficients(True)
    seg.set_model_type(pcl.SACMODEL_PLANE)  # _NORMAL_PLANE)
    seg.set_method_type(pcl.SAC_MSAC)  # RANSAC)
    seg.set_distance_threshold(0.005)  # 0.02)
    # seg.set_normal_distance_weight(0.5)
    # seg.set_max_iterations(1000)
    indices, coefficients = seg.segment()

    '''
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D

    # Create x,y
    xx, yy = np.meshgrid(range(30), range(30))

    # Calculate corresponding z
    z1 = (-coefficients[0] * xx - coefficients[1] * yy - coefficients[3]) * 1. / coefficients[2]

    # Plot
    plt3d = plt.figure().gca(projection='3d')
    plt3d.plot_surface(xx, yy, z1, color='blue')
    m = 200
    random_indices = np.random.choice(points.shape[0], size=m, replace=False)
    points = points[random_indices, :]
    plt3d.scatter(points[:, 0], points[:, 1], points[:, 2])
    plt.show()
    '''

    return coefficients

def check_grasp_hsr(pose_odm, scene_cloud_ros, visualize=False):
    """
    Takes a object pose and detects which of the saved grasps are reachable with the GraspChecker.

    Parameters
    ----------
    pose_odm: object_detector_msgs.msg.PoseWithConfidence
        object pose, with attributes pose, name and confidence
    scene_cloud_ros: sensor_msgs.msg.PointCloud2
        Pointcloud of the scene
    
    Returns
    -------
    grasp_pose
        valid grasp pose
    """
    from open3d_ros_helper import open3d_ros_helper as orh
    import tf2_ros, tf
    import transforms3d as tf3d
    import rospy
    from sensor_msgs.msg import PointCloud2
    from geometry_msgs.msg import Pose
    grasp_checker = GraspChecker()
    o3dcloud = orh.rospc_to_o3dpc(scene_cloud_ros, True)
    grasp_checker.set_scene_data(o3dcloud)

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    tf_found = False
    while not tf_found:
        try:
            trans = tfBuffer.lookup_transform('map', 'head_rgbd_sensor_rgb_frame', rospy.Time(0))
            print(trans)
            tf_found = True
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print('No tf found')
            rospy.sleep(1)
    rot = tf3d.quaternions.quat2mat([trans.transform.rotation.w, trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z])
    cam_to_base = np.eye(4)
    cam_to_base[:3, :3] = rot
    cam_to_base[:3, 3] = [trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z]

    rot = tf3d.quaternions.quat2mat([pose_odm.pose.orientation.w, pose_odm.pose.orientation.x, 
                                    pose_odm.pose.orientation.y, pose_odm.pose.orientation.z])
    object_pose = np.eye(4)
    object_pose[:3, :3] = rot
    object_pose[:3, 3] = [pose_odm.pose.position.x, pose_odm.pose.position.y, pose_odm.pose.position.z]
    print(o3dcloud)
    grasp_poses = np.load('/home/v4r/Markus_L/src/grasping_pipeline/grasps/'+pose_odm.name+'.npy')
    grasp_poses = grasp_poses.reshape((grasp_poses.shape[0],4,4))
    successes = []
    grasp_trials = []
    index = -1
    shortest_dist = 1.0
    dist_to_cam = []
    grasp_poses_ros = []
    for pose, i in zip(grasp_poses, range(len(grasp_poses))):
        grasp_try = np.eye(4)
        grasp_try = np.matmul(object_pose, pose)
        res = grasp_checker.is_grasp_valid(o3dcloud, grasp_try, cam_to_base=cam_to_base)
        dist = np.linalg.norm(grasp_try[:3,3])
        dist_to_cam.append(dist)
        pose = Pose()
        pose.position.x = grasp_try[0, 3]
        pose.position.y = grasp_try[1, 3]
        pose.position.z = grasp_try[2, 3]
        quat = tf.transformations.quaternion_from_matrix(grasp_try)
        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]
        # Add to array
        grasp_poses_ros.append(pose)
        if res and (dist < shortest_dist):
            shortest_dist = dist
            index = i
        grasp_trials.append(grasp_try)
        successes.append(res)
    grasps = zip(grasp_poses_ros, grasp_trials, successes, dist_to_cam)
    print(dist_to_cam)
    print('2222222222222222')
    grasps = list(grasps)
    
    print('333333333333')
    grasps = np.array(sorted(grasps, key = lambda x:(-x[2],x[3])))
    print('44444444444444444')
    #chosen_pose = np.asarray(grasp_poses_ros)[index]
    chosen_pose = grasps[0,0]
    print(grasps)
    print('#####')
    dist_to_cam = list(grasps[:,3])
    print(dist_to_cam)
    print('#####')
    print('valid poses:')
    print(grasps[:,2])
    valid_poses = list(grasps[:,0][grasps[:,2].astype(bool)])
    #valid_poses = np.asarray(grasp_poses_ros)[successes]

    print(valid_poses)
    if visualize == True:
        scenepub = rospy.Publisher('/grasping_pipeline/grasp_checker/scenecloud', PointCloud2, queue_size=10, latch=True)
        gripperpub = rospy.Publisher('/grasping_pipeline/grasp_checker/grippercloud', PointCloud2, queue_size=10, latch=True)
        scenepub.publish(scene_cloud_ros)
        o3dcloud.paint_uniform_color([0.5, 0.5, 0.5])
        geometries = [o3dcloud]
        geometries.append(o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5))
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
                g_cloud.paint_uniform_color([0 ,1 ,0])
            #geometries.append(g_cloud)
            combined_grasps += g_cloud
            print(combined_grasps)            
        #np.save('g_cloud'+str(i), np.asarray(g_cloud.points))
        gripperpub.publish(orh.o3dpc_to_rospc(combined_grasps, 'head_rgbd_sensor_rgb_frame', rospy.Time(0)))
    return chosen_pose, valid_poses

if __name__ == '__main__':
    # rospy.init_node('Grasp_checker_service')
    # rospy.loginfo('Starting grasp checker service')

    grasp_checker = GraspChecker()
    # rospy.spin()

    # TESTS --
    
    scene_cloud = o3d.io.read_point_cloud('test_data/2019-11-26-10-02-34_cloud.pcd')
    scene_cloud.points = o3d.utility.Vector3dVector(np.asarray(scene_cloud.points) / 1000)
    grasp_checker.set_scene_data(scene_cloud)

    table_indices = np.loadtxt('test_data/2019-11-26-10-02-34_table.txt', dtype='int')
    scene_points = np.asarray(scene_cloud.points)
    scene_points = np.take(scene_points, table_indices, axis=0)
    #table_plane = points_to_plane(scene_points)

    cam_to_base_tf = np.loadtxt('test_data/2019-11-26-10-02-34_tf.txt')
    # t_x, t_y, t_z, o_x, o_y, oz, o_w
    import transforms3d as tf3d
    # Quaternions here consist of 4 values w, x, y, z
    rot = tf3d.quaternions.quat2mat([cam_to_base_tf[6], cam_to_base_tf[3], cam_to_base_tf[4], cam_to_base_tf[5]])
    print(rot)
    cam_to_base = np.eye(4)
    cam_to_base[:3, :3] = rot
    cam_to_base[:3, 3] = cam_to_base_tf[0:3]

    # rosrun tf tf_echo /head_rgbd_sensor_link /base_link
    # - Translation: [0.022, 0.758, 0.601]
    # - Rotation: in Quaternion[0.654, -0.654, 0.268, 0.268]
    #             in RPY(radian)[3.141, -0.779, -1.571]
    #             in RPY(degree)[179.991, -44.618, -89.987]

    # rosrun tf tf_echo /base_link /head_rgbd_sensor_link
    # - Translation: [0.117, 0.022, 0.960]
    # - Rotation: in Quaternion[0.654, -0.654, 0.268, -0.268]
    #             in RPY(radian)[-2.363, 0.000, -1.571]
    #             in RPY(degree)[-135.382, 0.000, -89.991]

    grasp_success = np.load('test_data/2019-11-26-10-02-34_grasp_1.npy').reshape((4, 4))
    
    print('##############')

    print(grasp_success)
    print('##############')

    #Try different grasp poses by randomly perturbing the input grasp that is known to be successful for the scene
    test_results = []
    grasp_trials = []
    for i in range(2):
        rot = tf3d.euler.euler2mat(random.uniform(-1, 1), random.uniform(-1, 1), random.uniform(-1, 1))
        mat_update = np.eye(4)
        mat_update[:3, :3] = rot
        mat_update[0, 3] += random.uniform(-0.1, 0.1)
        mat_update[1, 3] += random.uniform(-0.1, 0.1)
        mat_update[2, 3] += random.uniform(-0.1, 0.1)
        grasp_new = np.matmul(grasp_success, mat_update)
        grasp_trials.append(grasp_new)

        print('')
        res = grasp_checker.is_grasp_valid(scene_cloud, grasp_new, cam_to_base=cam_to_base)
        test_results.append(res)
        #grasp_checker.visualize(scene_cloud, [grasp_new], [test_results[-1]])

    res = grasp_checker.is_grasp_valid(scene_cloud, grasp_success, cam_to_base=cam_to_base)
    grasp_trials.append(grasp_success)
    test_results.append(res)
    #grasp_checker.visualize(scene_cloud, grasp_trials, test_results)
    print(grasp_trials)
    print(test_results)



    # object_poses = np.load('/home/markus/V4R/markus_ws/src/hsrb_grasping/grasps/006_mustard_bottle.npy')
    # object_poses = object_poses.reshape((object_poses.shape[0],4,4))
    # grasp_trials = []
    # test_results = []
    
    # print(grasp_success[0,:3])
    # grasp_trials.append(copy.deepcopy(grasp_success))
    # test_results.append(False)
    # op = copy.deepcopy(object_poses[1,:,:])
    # grasp_success[:3,:3] = np.matmul(grasp_success[:3,:3],np.linalg.inv(op[:3,:3]))
    # grasp_success[:3,3] = grasp_success[:3,3]-op[:3,3]
    # index = 0
    # shortest_dist = 1.0
    # for pose, i in zip(object_poses, range(len(object_poses))):
    #     grasp_try = np.eye(4)
    #     print('')
    #     grasp_try[:3,:3] = np.matmul(grasp_success[:3,:3], pose[:3,:3])
    #     grasp_try[:3,3] = grasp_success[:3,3] + pose[:3,3]
    #     #res = grasp_checker.is_grasp_valid(scene_cloud, grasp_try, table_plane=table_plane, cam_to_base=cam_to_base)
    #     res = grasp_checker.is_grasp_valid(scene_cloud, grasp_try, cam_to_base=cam_to_base)
    #     dist = np.linalg.norm(grasp_try[:3,3])
    #     print('norm = ', dist)
    #     print(res)
    #     if res and (dist < shortest_dist):
    #         shortest_dist = dist
    #         index = i
    #         print('blablares', res)
    #     grasp_trials.append(grasp_try)
    #     test_results.append(False)
    # print(object_poses.shape)
    # print(index)
    # print(shortest_dist)
    # test_results[index+1]=True

    # #res = grasp_checker.is_grasp_valid(scene_cloud, grasp_success, table_plane=table_plane, cam_to_base=cam_to_base)

    # print(test_results)
    # grasp_checker.visualize(scene_cloud, grasp_trials, test_results)
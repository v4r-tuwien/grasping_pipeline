import open3d as o3d
import rospy
from open3d_ros_helper import open3d_ros_helper as orh 
from primitect_msgs.srv import Primitect
import tf2_ros
import transforms3d as tf3d
from grasp_checker import get_tf_transform, get_transmat_from_tf_trans
from sensor_msgs.msg import PointCloud2
import numpy as np

class GetTablePlane():
    def __init__(self):
        #self.table_service = rospy.Service('/get_table_plane', GetTable, self.get_table_plane)
        self.get_plane = rospy.ServiceProxy('/get_plane', Primitect)
        self.pointcloud_topic =  '/hsrb/head_rgbd_sensor/depth_registered/rectified_points'
        self.pointcloud_sub = rospy.Subscriber(self.pointcloud_topic, PointCloud2, self.pointcloud_cb)
    
    def pointcloud_cb(self, data):
        self.cloud = data

    def get_table_plane(self, request=None):
        """
        Gets the closest table plane by calling primitect_ros service '/get_plane'. 
        Checks for plane closest to the reference plane, that is given from mongodb.

        Parameters
        ----------
        Returns
        -------
        table_plane: np.array
            Table plane parameters (4 values) in head_rgbd_sensor_rgb_frame
        """
        #planes = get_plane(scene_cloud)
        rospy.wait_for_message(self.pointcloud_topic, PointCloud2, timeout=15)
        scene_cloud = self.cloud
        cloud = orh.rospc_to_o3dpc(scene_cloud, remove_nans=True)
        cloud = cloud.voxel_down_sample(0.04)
        try:
            planes = self.get_plane(orh.o3dpc_to_rospc(cloud, 'head_rgbd_sensor_rgb_frame'))
        except:
            print('Could not call service. Set table_plane to None.')
            table_plane = None
            return None


        trans = get_tf_transform('map', 'head_rgbd_sensor_rgb_frame')
        cam_to_base = get_transmat_from_tf_trans(trans)

        min_dist = None
        for plane in planes.plane:
            plane = np.array([plane.x, plane.y, plane.z, plane.d])
            plane_trans = np.matmul(np.transpose(np.linalg.inv(cam_to_base)),plane)
            ref_plane = np.array([0.01, 0.002, 0.9999, -0.45]) #get it from Table Store
            dist = np.linalg.norm(plane_trans-ref_plane)
            if min_dist==None or dist<min_dist:
                min_dist=dist
                table_plane = plane
        
        return table_plane

    
def get_tf_transform(origin_frame, target_frame):
    import tf2_ros, rospy
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    tf_found = False
    while not tf_found:
        try:
            trans = tfBuffer.lookup_transform(origin_frame, target_frame, rospy.Time(0))
            tf_found = True
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.sleep(0.2)
    return trans

def get_transmat_from_tf_trans(trans):
    import numpy as np
    import transforms3d as tf3d
    rot = tf3d.quaternions.quat2mat([trans.transform.rotation.w, trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z])
    transmat = np.eye(4)
    transmat[:3, :3] = rot
    transmat[:3, 3] = [trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z]
    return transmat

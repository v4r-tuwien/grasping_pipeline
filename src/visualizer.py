#! /usr/bin/env python3
import os
import sys
import copy
import rospy
import ros_numpy
import open3d as o3d
from v4r_util.rviz_visualization.image_visualization import PoseEstimationVisualizer
from v4r_util.conversions import ros_poses_to_np_transforms
from sensor_msgs.msg import Image, CameraInfo
from object_detector_msgs.srv import VisualizePoseEstimation, VisualizePoseEstimationResponse

class PoseEstimationVisualizerRos(PoseEstimationVisualizer):
    
    def __init__(
        self, 
        topic, 
        image_width, 
        image_height, 
        intrinsics_matrix, 
        model_dir,
        expose_service=False, 
        service_name=None
        ):
        '''
        Renders contours of models and modelnames into an image
        topic: str, topic to publish visualization to
        image_width: int, width of image
        image_height: int, height of image
        intrinsics_matrix: flattened camera matrix
        expose_service: bool, whether to expose a service to trigger visualization
        service_name: str, name of service to expose, needed if expose_service is True
        '''
        self.image_width = image_width
        self.image_height = image_height
        self.intrinsics_matrix = intrinsics_matrix
        self.meshes = self.load_meshes(model_dir)
        rospy.loginfo(f'PoseEstimVis: Loaded {len(self.meshes)} meshes')

        self.image_pub = rospy.Publisher(topic, Image, queue_size=10, latch=True)
        if expose_service:
            rospy.loginfo(f'PoseEstimVis: Exposing service {service_name}')
            assert service_name is not None
            self.service = rospy.Service(
                service_name, 
                VisualizePoseEstimation, 
                self.service_callback
                )

        self.renderer_initialized = False
        rospy.loginfo(f'PoseEstimationVisualizerRos initialized, publishing to {topic}')
            
    def service_callback(self, req):
        '''
        Service callback for pose estimation visualization
        '''
        rospy.loginfo("PoseEstimVis: Received service call")
        # renderer needs to be initialized in the same thread that executes the callback
        # else we get some threading related cpp exceptions => most robust way to do it atm
        rospy.loginfo("PoseEstimVis: Initializing renderer")
        super().__init__(image_width, image_height, intrinsics_matrix)

        meshes = []
        for name in req.model_names:
            mesh = self.meshes.get(name, None)
            if mesh is None and name != 'Unknown':
                rospy.logwarn(f'No mesh for model {name} found!')
                continue
            meshes.append(copy.deepcopy(mesh))

        try:
            self.publish_pose_estimation_result(
                req.rgb_image, 
                req.model_poses, 
                meshes, 
                req.model_names
                )
        except Exception as e:
            print(f"AAAH: {e}")
        rospy.loginfo("PoseEstimVis: service call finished")

        # Delete renderer so that it properly cleans itself and is ready to get initialized next time
        del self.renderer 
        return VisualizePoseEstimationResponse()
    
    def publish_pose_estimation_result(self, ros_image, ros_model_poses, model_meshes, model_names):
        '''
        Renders contours of models and modelnames into an image and publishes the result
        ros_image: sensor_msgs.msg.Image
        ros_model_poses: list of geometry_msgs.msg.Pose
        model_meshes: list of open3d.geometry.TriangleMesh, scaled to meters
        model_name: list of str, names of models
        '''
        model_poses = ros_poses_to_np_transforms(ros_model_poses)
        np_img = ros_numpy.numpify(ros_image)
        vis_img = self.create_visualization(np_img, model_poses, model_meshes, model_names)
        vis_img_ros = ros_numpy.msgify(Image, vis_img, encoding='rgb8')
        self.image_pub.publish(vis_img_ros)
      
    def load_meshes(self, model_dir):
        meshes = {}
        for mesh_file in os.listdir(model_dir):
            if not mesh_file.endswith('.stl'):
                continue
            model_name = mesh_file.split('.')[0]
            path = os.path.join(model_dir, mesh_file)
            mesh = o3d.io.read_triangle_mesh(path)
            depth_mm_to_m = 0.001
            mesh.scale(depth_mm_to_m, center = [0, 0, 0])
            meshes[model_name] = mesh
        return meshes
        
if __name__ == '__main__':
    rospy.init_node('PoseEstimationVisualizer')
    if len(sys.argv) < 2:
        rospy.logerr('No model_dir was specified!')
        sys.exit(-1)

    cam_info = rospy.wait_for_message(rospy.get_param('/cam_info_topic'), CameraInfo)

    image_width = cam_info.width
    image_height = cam_info.height
    intrinsics_matrix = cam_info.K
    model_dir = sys.argv[1]
    topic = rospy.get_param('/pose_estimator/result_visualization_topic')
    service_name = rospy.get_param('/pose_estimator/result_visualization_service_name')
    
    server = PoseEstimationVisualizerRos(
        topic,
        image_width,
        image_height, 
        intrinsics_matrix, 
        model_dir, 
        expose_service=True, 
        service_name=service_name
        )

    rospy.spin()
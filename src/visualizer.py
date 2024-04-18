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
        Renders the pose estimation results of the object detector into an image and publishes it
        
        Renders model-contours and the corresponding modelnames of the detected objects 
        into the given image and publishes the result

        Parameters
        ----------
        topic: str
            ROS Topic to publish the rendered image to
        image_width: int
            width of input image and rendered image
        image_height: int
            height of input image and rendered image
        intrinsics_matrix: list or numpy array 
            flattened [9x1] camera matrix, e.g. [fx, 0, cx, 0, fy, cy, 0, 0, 1]
        expose_service: bool
            whether to expose a service which can be used to trigger the visualization
        service_name: str
            name of the service that should be exposed, only needed if expose_service is True
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
        Service callback for the pose estimation visualization service.
        
        Renders the contours of the objects based on the given poses into the passed image. 
        Additionally, the model names are rendered next to the contours.
        
        Parameters
        ----------
        req: object_detector_msgs.srv.VisualizePoseEstimationRequest
            The request containing: the rgb_image, a list of model_poses and a list of model_names
            
        Returns
        -------
        object_detector_msgs.srv.VisualizePoseEstimationResponse
            Empty response
        
        '''
        rospy.loginfo("PoseEstimVis: Received service call")
        # renderer needs to be initialized in the same thread that executes the callback
        # else we get some threading related cpp exceptions => most robust way to do it is to just
        # initialize the renderer in every callback and delete it afterwards 
        # This is not that bad since the renderer is quite lightweight (takes like ~20ms to initialize)
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
            print(f"AAAH exception in PoseEstimVis: {e}")
        rospy.loginfo("PoseEstimVis: service call finished")

        # Delete renderer so that it properly cleans itself and is ready to get initialized next time
        del self.renderer 
        return VisualizePoseEstimationResponse()
    
    def publish_pose_estimation_result(self, ros_image, ros_model_poses, model_meshes, model_names):
        '''
        Renders contours of models and modelnames into an image and publishes the result

        Parameters
        ----------
        ros_image: sensor_msgs.msg.Image
            The input image to render the model contours into
        ros_model_poses: list of geometry_msgs.msg.Pose
            The poses of the models in the camera frame
        model_meshes: list of open3d.geometry.TriangleMesh
            The meshes of the models to render, scaled to meters
        model_name: list of str
            names of the models
        '''
        model_poses = ros_poses_to_np_transforms(ros_model_poses)
        np_img = ros_numpy.numpify(ros_image)
        vis_img = self.create_visualization(np_img, model_poses, model_meshes, model_names)
        vis_img_ros = ros_numpy.msgify(Image, vis_img, encoding='rgb8')
        self.image_pub.publish(vis_img_ros)
      
    def load_meshes(self, model_dir):
        '''
        Load .stl meshes from the given directory and return them as a dictionary
        
        Parameters
        ----------
        model_dir: str
            The directory containing the .stl meshes that should be loaded
        
        Returns
        -------
        dict
            A dictionary containing the loaded meshes with the model name as the key. 
            The models are scaled to meters. The names are the filenames without the .stl extension.
        '''
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
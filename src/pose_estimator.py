#! /usr/bin/env python3
from cv_bridge import CvBridge
import rospy
from actionlib import SimpleActionClient
from actionlib_msgs.msg import GoalStatus
from robokudo_msgs.msg import GenericImgProcAnnotatorAction, GenericImgProcAnnotatorGoal
from grasping_pipeline_msgs.srv import CallPoseEstimator, CallPoseEstimatorResponse, VisualizePoseEstimation, VisualizePoseEstimationRequest

class CallPoseEstimatorService:

    def __init__(self):
        self.bridge = CvBridge()
        self.srv = rospy.Service('call_pose_estimator', CallPoseEstimator , self.execute)
        res_vis_service_name = rospy.get_param('/grasping_pipeline/result_visualization_service_name')
        self.res_vis_service = rospy.ServiceProxy(res_vis_service_name, VisualizePoseEstimation)
    
    def execute(self, req):
        topic = rospy.get_param('/grasping_pipeline/pose_estimator_topic')
        timeout = rospy.get_param('/grasping_pipeline/timeout_duration')

        pose_est = SimpleActionClient(topic, GenericImgProcAnnotatorAction)

        rospy.loginfo('Waiting for pose estimator server with topic: %s' % topic)
        if not pose_est.wait_for_server(timeout=rospy.Duration(timeout)):
            rospy.logerr(f'Connection to pose_estimator \'{topic}\' timed out!')
            raise rospy.ServiceException
        rospy.loginfo('Connected to pose estimator server')
        
        description = ''
        for name, confidence, index in zip(req.class_names, req.class_confidences, range(0, len(req.class_names))):
            if index == 0:
                description = description + f'"{name}": "{confidence}"'
            else:
                description = description + f', "{name}": "{confidence}"'
        description = '{' + description + '}'

        goal = GenericImgProcAnnotatorGoal()
        goal.rgb = req.rgb
        goal.depth = req.depth
        goal.bb_detections = req.bb_detections
        goal.mask_detections = req.mask_detections
        goal.class_names = req.class_names
        goal.description = description

        rospy.logdebug('Sending goal to pose estimator')
        pose_est.send_goal(goal)
        rospy.logdebug('Waiting for pose estimation results')
        goal_finished = pose_est.wait_for_result(rospy.Duration(timeout))
        if not goal_finished:
            rospy.logerr('Pose Estimator didn\'t return results before timing out!')
            raise rospy.ServiceException
        pose_result = pose_est.get_result()
        server_status = pose_est.get_state()

        if server_status != GoalStatus.SUCCEEDED or len(pose_result.pose_results) <= 0:
            rospy.logerr('Pose Estimator failed to estimate poses!')
            raise rospy.ServiceException
        rospy.loginfo(f'Estimated the pose of {len(pose_result.class_names)} objects.')

        self.visualize_pose_estimation_result(
            req.rgb, 
            pose_result.pose_results, 
            pose_result.class_names
        )

        res = CallPoseEstimatorResponse()
        res.class_confidences = pose_result.class_confidences
        res.class_names = pose_result.class_names
        res.pose_results = pose_result.pose_results
        
        return res
    
    def visualize_pose_estimation_result(self, rgb, model_poses, model_names):
        '''
        Visualizes the pose estimation result using the pose estimator result visualization service.
        
        The service creates and publishes an image with a contour of the detected objects and their 
        object names. This is only possible for known objects because the object's model is used to
        determine the object's contour.
        
        Parameters
        ----------
        rgb: sensor_msgs.msg.Image
            The rgb image.
        model_poses: list of geometry_msgs.msg.Pose
            The poses of the detected objects.
        model_names: list of str
            The names of the detected objects. Used to lookup the object's model which is used to 
            determine the object's contour.
        '''
        request = VisualizePoseEstimationRequest()
        request.rgb_image = rgb
        request.model_poses = model_poses
        request.model_names = model_names
        self.res_vis_service(request)
    
if __name__ == '__main__':
    node = rospy.init_node('pose_estimator')
    pose_est = CallPoseEstimatorService()
    rospy.spin()

    
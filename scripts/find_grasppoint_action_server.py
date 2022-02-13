#! /usr/bin/env python3

from math import asin, atan2, isnan, pi

import actionlib
import geometry_msgs.msg
import rospy
import numpy as np
import sensor_msgs.point_cloud2 as pc2
import tf
from grasping_pipeline.msg import (FindGrasppointAction,
                                   FindGrasppointActionResult)
from haf_grasping.msg import (CalcGraspPointsServerAction,
                              CalcGraspPointsServerActionGoal)
from hsrb_interface import Robot
from object_detector_msgs.msg import Detection as DetectronDetection
from object_detector_msgs.msg import Detections as DetectronDetections
from object_detector_msgs.srv import get_poses, start, stop
from sensor_msgs.msg import PointCloud2
from tf.transformations import (quaternion_about_axis, quaternion_from_matrix,
                                quaternion_multiply, unit_vector)
from tmc_vision_msgs.msg import Detection, DetectionArray
from visualization_msgs.msg import Marker

from grasp_checker import check_grasp_hsr, get_tf_transform, get_transmat_from_tf_trans


class FindGrasppointServer:
    """ ActionServer that finds and selects grasp poses.
    
    
    goal:
        method {uint32} -- Specify which method to use
                            1: Detectron2 & HAF
                            2: Verefine Pipeline
                            3: Verefine & HAF
                            4: PyraPose
        object_names {list of str} -- names of the detections that are 
                            considered for grasping.
    
    result:
        grasp_poses {list of geometry_msgs.msg.PoseStamped} -- 
                            grasp poses of an selected object

    """    
    def __init__(self):
        self.server = actionlib.SimpleActionServer(
            'find_grasppoint', FindGrasppointAction, self.execute, False)
        self.server.start()
        self.robot = Robot()
        self.base = self.robot.try_get('omni_base')
        self.whole_body = self.robot.try_get('whole_body')
        self.gripper = self.robot.get('gripper')
        self.Transformer = tf.TransformListener(True, rospy.Duration(10))
        self.haf_client = actionlib.SimpleActionClient(
            '/calc_grasppoints_svm_action_server', CalcGraspPointsServerAction)

        #self.yolo_detection_sub = rospy.Subscriber('/yolo2_node/detections', DetectionArray, self.yolo_detection_cb)
        self.detectron_sub = rospy.Subscriber(
            '/detectron2_service/detections', DetectronDetections, self.detectron_cb)

        self.pointcloud_topic = '/hsrb/head_rgbd_sensor/depth_registered/rectified_points'
        self.pointcloud_sub = rospy.Subscriber(
            self.pointcloud_topic, PointCloud2, self.pointcloud_cb)

        self.verefine_get_poses = rospy.ServiceProxy(
            '/hsr_grasping/get_poses', get_poses)
        self.pyrapose_get_poses = rospy.ServiceProxy(
            '/PyraPose/return_poses', get_poses)
        self.start_detectron = rospy.ServiceProxy(
            '/detectron2_service/start', start)
        self.stop_detectron = rospy.ServiceProxy(
            '/detectron2_service/stop', stop)
        rospy.loginfo('Initializing FindGrasppointServer done')

    def execute(self, goal):      
        if not goal.object_names:
            rospy.logerr("No object names given. Abort.")
            self.server.set_aborted()
            return

        object_names = goal.object_names

        rospy.loginfo('Method Number: {}'.format(goal.method))

# method 1, unknown objects, uses detectron and haf grasping
        if goal.method == 1:
            self.method1_detectron_and_haf(object_names)

# method 2, known objects, uses verefine
        elif goal.method == 2:
            self.method2_verefine_pipeline(object_names)

# method 3, known objects, but with HAF
        elif goal.method == 3:
            self.method3_verefine_haf(object_names)

# method 4, known objects, uses pyrapose
        elif goal.method == 4:
            self.method4_pyrapose(object_names)

        else:
            rospy.loginfo('Method not implemented')
            self.server.set_aborted()

 
    def method1_detectron_and_haf(self, object_names):
        """ Method 1: Detectron and HAF Grasping
        
        Calls Detectron2 to find a rough object center of an object.
        If no object that is listed in object_names is found, 
        sets ActionServer to aborted and exits function. 
        If an object is found, calls HAF Grasping to find a grasp pose.
        No succesful result -> aborts server.
        Otherwise, converts HAF result for MoveIt, writes the pose to the
        result and sets the server to succeeded.  

        Arguments:
            object_names {list of str} -- names of the detections that are 
                considered for grasping. Strings have to match detection results
        """
        result = FindGrasppointActionResult().result
        rospy.loginfo('Chosen Method is Detectron + HAF')
        
        # use detectron and find a rough grasp center
        rough_grasp_object_center = self.get_grasp_object_center_detectron(
            object_names)

        if rough_grasp_object_center == -1:
            self.server.set_aborted()
            return

        grasp_result_haf = self.call_haf_grasping(
            rough_grasp_object_center)

        if grasp_result_haf.graspOutput.eval <= 0:
            rospy.logerr(
                'HAF grasping did not deliver successful result. Eval below 0')
            self.server.set_aborted()
            return
        grasp_pose = self.convert_haf_result_for_moveit(grasp_result_haf)

        result.grasp_poses = grasp_pose
        self.server.set_succeeded(result)

    def method2_verefine_pipeline(self, object_names):
        """Method 2: Verefine Pipeline
        
        Calls the "get_poses" service from the verefine pipeline. 
        If this fails, the ActionServer will be set to aborted and the 
        function ends.
        If the service call succeeds, it will return a list of named poses 
        with confidence values. The object whose name is listed in object_names 
        and has the highest confidence value is selected.
        For this object, grasp poses are selected. 
        If there is at least one valid grasp pose, a marker for that pose 
        will be published and the poses are written to the result and the
        server succeeds.

        Arguments:
            object_names {list of str} -- names of the detections that are 
                considered for grasping. Strings have to match detection results
        """        
        result = FindGrasppointActionResult().result
        self.verefine_object_found = False
        rospy.loginfo('Chosen Method is VEREFINE')
        try:
            object_poses_result = self.verefine_get_poses()
        except BaseException:
            rospy.loginfo('Aborted: error when calling get_poses service.')
            self.server.set_aborted()
            return
        confidence = 0
        object_nr = 0

        # choose object with highest confidence from verefine
        for i in range(0, len(object_poses_result.poses)):
            print(object_poses_result.poses[i].name)
            # TODO add all objects option
            if object_poses_result.poses[i].name in object_names:
                if confidence < object_poses_result.poses[i].confidence:
                    confidence = object_poses_result.poses[i].confidence
                    object_nr = i
                    self.verefine_object_found = True
        if not self.verefine_object_found:
            rospy.logerr('No object pose found')
            self.server.set_aborted()
            return

        grasp_object = object_poses_result.poses[object_nr]

        rospy.wait_for_message(self.pointcloud_topic,
                               PointCloud2, timeout=15)
        scene_cloud = self.cloud
        # if rospy.get_param('/use_table_grasp_checker'):
        #  from get_table_plane import GetTablePlane
        #  TableGetter = GetTablePlane()
        #  table_plane = TableGetter.get_table_plane()
        #  print(table_plane)
        # else:
        #  table_plane = None
        table_plane = None

        valid_poses = check_grasp_hsr(
            grasp_object, scene_cloud, table_plane=table_plane, visualize=True)

        if len(valid_poses) == 0:
            rospy.loginfo('no grasp found')
            self.server.set_aborted()
            return

        result.grasp_poses = valid_poses

        self.add_marker(valid_poses[0])
        self.server.set_succeeded(result)

    def method3_verefine_haf(self, object_names):
        """Method 3: Verefine Pipeline and HAF Grasping
        
        Calls the "get_poses" service from the verefine pipeline. 
        If this fails, the ActionServer will be set to aborted and the 
        function ends.
        If the service call succeeds, it will return a list of named poses 
        with confidence values. The object whose name is listed in object_names 
        and has the highest confidence value is selected.
        The object pose will be transformed to the base_link frame, and acts 
        then as the search center for HAF Grasping.
        The result from HAF Grasping is converted for MoveIt and then set as
        the servers result. 

        Arguments:
            object_names {list of str} -- names of the detections that are 
                considered for grasping. Strings have to match detection results
        """  
        result = FindGrasppointActionResult().result
        rospy.loginfo('Chosen Method is VeREFINE + HAF')
        # use detectron and find a rough grasp center, that is needed for
        # HAF grasping
        self.verefine_object_found = False
        rospy.loginfo('Chosen Method is VEREFINE')
        try:
            object_poses_result = self.verefine_get_poses()
        except BaseException:
            rospy.loginfo('Aborted: error when calling get_poses service.')
            self.server.set_aborted()
            return
        confidence = 0
        object_nr = 0
        # choose object with highest confidence from verefine
        for i in range(0, len(object_poses_result.poses)):
            print(object_poses_result.poses[i].name)
            # TODO add all objects option
            if object_poses_result.poses[i].name in object_names:
                if confidence < object_poses_result.poses[i].confidence:
                    confidence = object_poses_result.poses[i].confidence
                    object_nr = i
                    self.verefine_object_found = True
        if not self.verefine_object_found:
            rospy.logerr('No object pose found')
            self.server.set_aborted()
            return

        rospy.wait_for_message(self.pointcloud_topic,
                               PointCloud2, timeout=15)
        self.my_cloud = self.cloud

        grasp_pose = geometry_msgs.msg.PointStamped()
        grasp_pose.point = object_poses_result.poses[object_nr].pose.position
        grasp_pose.header.frame_id = 'head_rgbd_sensor_rgb_frame'  # head_rgbd_sensor_link
        self.Transformer.waitForTransform(
            'base_link', 'head_rgbd_sensor_rgb_frame', rospy.Time(), rospy.Duration(4.0))
        grasp_pose = self.Transformer.transformPoint(
            'base_link', grasp_pose)

        rough_grasp_object_center = grasp_pose
        if rough_grasp_object_center == -1:
            self.server.set_aborted()
            return

        grasp_result_haf = self.call_haf_grasping(
            rough_grasp_object_center)
        if grasp_result_haf.graspOutput.eval <= -20:
            rospy.logerr(
                'HAF grasping did not deliver successful result. Eval below 0')
            self.server.set_aborted()
            return
        grasp_pose = self.convert_haf_result_for_moveit(grasp_result_haf)
        if grasp_pose == 0:
            self.server.set_aborted()
            return
        result.grasp_poses = grasp_pose
        self.server.set_succeeded(result)

    def method4_pyrapose(self, object_names):
        """Method 4: PyraPose Pipeline
        
        Calls the "return_poses" service from the PyraPose pipeline. 
        If this fails, the ActionServer will be set to aborted and the 
        function ends.
        If the service call succeeds, it will return a list of named poses 
        with confidence values. The object whose name is listed in object_names 
        and has the highest confidence value is selected.
        For this object, grasp poses are selected. 
        If there is at least one valid grasp pose, a marker for that pose 
        will be published and the poses are written to the result and the
        server succeeds.

        Arguments:
            object_names {list of str} -- names of the detections that are 
                considered for grasping. Strings have to match detection results
        """   
        result = FindGrasppointActionResult().result
        self.pyrapose_object_found = False
        rospy.loginfo('Chosen Method is PYRAPOSE')
        try:
            object_poses_result = self.pyrapose_get_poses()
            print(object_poses_result)
        except BaseException:
            rospy.loginfo('Aborted: error when calling get_poses service.')
            self.server.set_aborted()
            return
        confidence = 0
        object_nr = 0
            # choose object with highest confidence from pyrapose
        for i in range(0, len(object_poses_result.poses)):
            print(object_poses_result.poses[i].name)
                # TODO add all objects option
            if object_poses_result.poses[i].name in object_names:
                if confidence < object_poses_result.poses[i].confidence:
                    confidence = object_poses_result.poses[i].confidence
                    object_nr = i
                    self.pyrapose_object_found = True
        if not self.pyrapose_object_found:
            rospy.logerr('No object pose found')
            self.server.set_aborted()
            return

        grasp_object = object_poses_result.poses[object_nr]

        rospy.wait_for_message(self.pointcloud_topic,
                                   PointCloud2, timeout=15)
        scene_cloud = self.cloud
        valid_poses = check_grasp_hsr(grasp_object, scene_cloud, True)

        if len(valid_poses) == 0:
            rospy.loginfo('no grasp found')
            self.server.set_aborted()
            return

        result.grasp_poses = valid_poses

        self.add_marker(valid_poses[0])
        self.server.set_succeeded(result)

    def pointcloud_cb(self, data):
        self.cloud = data

    def detectron_cb(self, data):
        self.detectron_detection = data

    def get_grasp_object_center_detectron(self, object_names):
        """ gets Detectron2 detections, chooses the closest object
        to the robot from the detections and returns the pose of this object. 
        Only consideres detections that are listed in object_names, 
        since many detections like "dining table" or "person" are 
        not relevant for grasping, 
        

        Arguments:
            object_names {list of str} -- names of the objects that are
                considered for grasping. Detections like e.g. "bottle"
                or "sports ball" are graspable objects that could be listed.


        Returns:
            geometry_msgs.msg.PointStamped -- rough object position in
                "base_link" frame
        """
        # get detections from detectron
        self.start_detectron()
        detections = rospy.wait_for_message(
            '/detectron2_service/detections', DetectronDetections, timeout=10)
        print('detection received')
        self.stop_detectron()
        print('stop detectron')
        chosen_object = DetectronDetection()
        chosen_object.bbox.ymax = 0

        # choose the detected object that is closest to the robot
        for i in range(len(detections.detections)):
            name = detections.detections[i].name
            if name in object_names:
                if detections.detections[i].bbox.ymax > chosen_object.bbox.ymax:
                    chosen_object = detections.detections[i]
        if chosen_object.score == 0:
            return -1

        image_x = int(chosen_object.bbox.xmin +
                      (chosen_object.bbox.xmax - chosen_object.bbox.xmin) / 2)
        image_y = int(chosen_object.bbox.ymin +
                      (chosen_object.bbox.ymax - chosen_object.bbox.ymin) / 2)
        self.object_name = chosen_object.name

        # get bounding box center from pointcloud
        rospy.wait_for_message(self.pointcloud_topic, PointCloud2, timeout=15)
        # rospy.sleep(0.5)
        self.my_cloud = self.cloud
        points = pc2.read_points_list(
            self.my_cloud, field_names=None, skip_nans=False)
        index = image_y * self.my_cloud.width + image_x
        center = points[index]
        # check if there is a valid point, otherwise go down a row
        while isnan(center[0]):
            index = index + self.my_cloud.width
            rospy.loginfo('index = {}'.format(index))
            if index > len(points) - 1:
                return -1
            center = points[index]

        # transform the rough grasp point to base_link
        self.Transformer.waitForTransform(
            'base_link', 'head_rgbd_sensor_rgb_frame', rospy.Time(), rospy.Duration(4.0))
        pose_goal = geometry_msgs.msg.Pose()
        point = geometry_msgs.msg.PointStamped()
        point.point.x = center[0]
        point.point.y = center[1]
        point.point.z = center[2]
        point.header.frame_id = 'head_rgbd_sensor_rgb_frame'
        point_transformed = self.Transformer.transformPoint(
            'base_link', point)
        return point_transformed

    def call_haf_grasping(self, search_center):
        """ Writes the goal for HAF grasping action, calls the action and
        returns the result
        The approach vector is set to [0,0,1]

        Arguments:
            search_center {geometry_msgs.msg.PointStamped} --
                rough x-,y-position, that is the center of the area
                where grasps are searched

        Returns:
            haf_grasping.msg.CalcGraspPointsServerActionResult --
                Result from HAF grasping
                Result contains a GraspOutput message:
                https://github.com/davidfischinger/haf_grasping/blob/master/msg/GraspOutput.msg
        """
        # approach vector for top grasps
        self.approach_vector_x = 0.0
        self.approach_vector_y = 0.0
        self.approach_vector_z = 1.0

        grasp_goal = CalcGraspPointsServerActionGoal()
        grasp_goal.goal.graspinput.goal_frame_id = search_center.header.frame_id
        grasp_goal.goal.graspinput.grasp_area_center.x = search_center.point.x
        grasp_goal.goal.graspinput.grasp_area_center.y = search_center.point.y
        grasp_goal.goal.graspinput.grasp_area_center.z = search_center.point.z + 0.1
        grasp_goal.goal.graspinput.grasp_area_length_x = 20
        grasp_goal.goal.graspinput.grasp_area_length_y = 20

        grasp_goal.goal.graspinput.approach_vector.x = self.approach_vector_x
        grasp_goal.goal.graspinput.approach_vector.y = self.approach_vector_y
        grasp_goal.goal.graspinput.approach_vector.z = self.approach_vector_z

        grasp_goal.goal.graspinput.input_pc = self.my_cloud
        grasp_goal.goal.graspinput.max_calculation_time = rospy.Duration(15)
        grasp_goal.goal.graspinput.gripper_opening_width = 1
        self.haf_client.wait_for_server()
        self.haf_client.send_goal(grasp_goal.goal)
        self.haf_client.wait_for_result()
        grasp_result = self.haf_client.get_result()
        return grasp_result

    def convert_haf_result_for_moveit(self, grasp_result_haf):
        """ Takes the result from HAF grasping and converts it to a
        PoseStamped in "odom" frame. HAF does not return a quaternion
        for the orientation, therefore this conversion is necessary.

        Arguments:
            grasp_result_haf {haf_grasping.msg.CalcGraspPointsServerActionResult} --
                Result from HAF grasping
                Result contains a GraspOutput message:
                https://github.com/davidfischinger/haf_grasping/blob/master/msg/GraspOutput.msg

        Returns:
            [list of geometry_msgs.msg.PoseStamped] -- The transformed PoseStamped
                in "odom" frame
        """
        av = unit_vector([-grasp_result_haf.graspOutput.approachVector.x,
                          -grasp_result_haf.graspOutput.approachVector.y,
                          -grasp_result_haf.graspOutput.approachVector.z])

        gp1 = np.array([grasp_result_haf.graspOutput.graspPoint1.x,
                        grasp_result_haf.graspOutput.graspPoint1.y,
                        grasp_result_haf.graspOutput.graspPoint1.z])

        gp2 = np.array([grasp_result_haf.graspOutput.graspPoint2.x,
                        grasp_result_haf.graspOutput.graspPoint2.y,
                        grasp_result_haf.graspOutput.graspPoint2.z])

        gc = unit_vector(gp2 - gp1)
        c = np.cross(gc, av)
        #rot_mat = np.array([[av[0], gc[0], c[0], 0], [av[1], gc[1], c[1], 0], [av[2], gc[2], c[2], 0], [0,0,0,1]])
        rot_mat = np.array([[c[0], gc[0], av[0], 0], [c[1], gc[1], av[1], 0], [
                           c[2], gc[2], av[2], 0], [0, 0, 0, 1]])
        q = quaternion_from_matrix(rot_mat)

        self.Transformer.waitForTransform(
            'odom', 'base_link', rospy.Time(), rospy.Duration(4.0))

        grasp_pose_bl = geometry_msgs.msg.PoseStamped()

        grasp_pose_bl.pose.orientation.x = q[0]
        grasp_pose_bl.pose.orientation.y = q[1]
        grasp_pose_bl.pose.orientation.z = q[2]
        grasp_pose_bl.pose.orientation.w = q[3]
        grasp_pose_bl.pose.position.x = grasp_result_haf.graspOutput.averagedGraspPoint.x - \
            rospy.get_param("/grasppoint_offset_haf", default=0.04) * av[0]
        grasp_pose_bl.pose.position.y = grasp_result_haf.graspOutput.averagedGraspPoint.y - \
            rospy.get_param("/grasppoint_offset_haf", default=0.04) * av[1]
        grasp_pose_bl.pose.position.z = grasp_result_haf.graspOutput.averagedGraspPoint.z - \
            rospy.get_param("/grasppoint_offset_haf", default=0.04) * av[2]
        grasp_pose_bl.header.frame_id = 'base_link'

        self.add_marker(grasp_pose_bl)
        grasp_pose = []
        grasp_pose.append(
            self.Transformer.transformPose('odom', grasp_pose_bl))
        return grasp_pose

    def add_marker(self, pose_goal):
        """ publishes a grasp marker to /grasping_pipeline/grasp_marker

        Arguments:
            pose_goal {geometry_msgs.msg.PoseStamped} -- pose for the grasp marker
        """
        marker_pub = rospy.Publisher(
            '/grasping_pipeline/grasp_marker', Marker, queue_size=10, latch=True)
        marker = Marker()
        marker.header.frame_id = pose_goal.header.frame_id
        marker.header.stamp = rospy.Time()
        marker.ns = 'grasp_marker'
        marker.id = 0
        marker.type = 0
        marker.action = 0

        q2 = [pose_goal.pose.orientation.w, pose_goal.pose.orientation.x,
              pose_goal.pose.orientation.y, pose_goal.pose.orientation.z]
        q = quaternion_about_axis(pi / 2, (0, 1, 0))
        q = quaternion_multiply(q, q2)

        marker.pose.orientation.w = q[0]
        marker.pose.orientation.x = q[1]
        marker.pose.orientation.y = q[2]
        marker.pose.orientation.z = q[3]
        marker.pose.position.x = pose_goal.pose.position.x
        marker.pose.position.y = pose_goal.pose.position.y
        marker.pose.position.z = pose_goal.pose.position.z

        marker.scale.x = 0.1
        marker.scale.y = 0.05
        marker.scale.z = 0.01

        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0
        marker.color.b = 0
        marker_pub.publish(marker)
        rospy.loginfo('grasp_marker')


if __name__ == '__main__':
    rospy.init_node('find_grasppoint_server')
    server = FindGrasppointServer()
    rospy.spin()

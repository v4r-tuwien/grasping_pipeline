import rospy
import smach
from sensor_msgs.msg import PointCloud2
from table_plane_extractor.srv import TablePlaneExtractor
from v4r_util.tf2 import TF2Wrapper
from v4r_util.conversions import bounding_box_to_bounding_box_stamped
from v4r_util.util import align_bounding_box_rotation, ros_bb_to_o3d_bb, o3d_bb_to_ros_bb

class FindTablePlanes(smach.State):

    def __init__(self, enlarge_table_bb_to_floor=True):
        smach.State.__init__(
            self, outcomes=['succeeded'], output_keys=['table_bbs', 'table_plane_equations'])
        self.topic = rospy.get_param('/point_cloud_topic')
        self.table_extractor_srv_name = '/table_plane_extractor/get_planes'
        self.table_extractor = rospy.ServiceProxy(
            self.table_extractor_srv_name, TablePlaneExtractor)
        self.tf_wrapper = TF2Wrapper()
        self.enlarge_table_bb_to_floor = enlarge_table_bb_to_floor

    def execute(self, userdata):
        rospy.loginfo('Executing state FIND_TABLE_PLANES. Waiting for point cloud.')
        cloud = rospy.wait_for_message(self.topic, PointCloud2, timeout=15)
        rospy.loginfo('Received point cloud. Waiting for table plane extractor service.')
        rospy.wait_for_service(self.table_extractor_srv_name)
        rospy.loginfo('Service available. Calling table plane extractor.')
        
        response = self.table_extractor(cloud)
        boxes = response.plane_bounding_boxes
        
        if self.enlarge_table_bb_to_floor:
            transform_to_base = False
            if boxes.header.frame_id != 'base_link':
                transform_to_base = True

            new_boxes = []
            for ros_bb in boxes.boxes:
                if transform_to_base:
                    ros_bb = bounding_box_to_bounding_box_stamped(ros_bb, boxes.header.frame_id , rospy.Time.now())
                    ros_bb = self.tf_wrapper.transform_bounding_box(ros_bb, 'base_link')
                
                aligned_bb_o3d = align_bounding_box_rotation(ros_bb_to_o3d_bb(ros_bb))
                ros_bb = o3d_bb_to_ros_bb(aligned_bb_o3d)

                center = ros_bb.center.position
                old_center_z = center.z
                size = ros_bb.size
                center.z = (center.z + size.z/2)/2
                size.x = size.x + 0.04
                size.y = size.y + 0.04
                size.z = old_center_z + size.z/2 - 0.02
                new_boxes.append(ros_bb)

            response.plane_bounding_boxes.boxes = new_boxes
            response.plane_bounding_boxes.header.frame_id = 'base_link'

        userdata.table_bbs = response.plane_bounding_boxes
        userdata.table_plane_equations = response.planes
        rospy.loginfo('Table planes extracted. Returning succeeded.')
        return 'succeeded'
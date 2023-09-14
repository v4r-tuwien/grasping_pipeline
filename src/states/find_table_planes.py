import rospy
import smach
from sensor_msgs.msg import PointCloud2
from table_plane_extractor.srv import TablePlaneExtractor

class FindTablePlanes(smach.State):

    def __init__(self):
        smach.State.__init__(
            self, outcomes=['succeeded'], output_keys=['table_bbs', 'table_plane_equations'])
        self.topic = rospy.get_param('/point_cloud_topic')
        self.table_extractor_srv_name = '/test/table_plane_extractor'
        self.table_extractor = rospy.ServiceProxy(
            self.table_extractor_srv_name, TablePlaneExtractor)

    def execute(self, userdata):
        cloud = rospy.wait_for_message(self.topic, PointCloud2, timeout=15)
        rospy.wait_for_service(self.table_extractor_srv_name)

        response = self.table_extractor(cloud)
        #TODO coordinates not consistent => z is not always dimension of height
        # think of a better solution
        
        # for ros_bb in response.plane_bounding_boxes.boxes:
        #     center = ros_bb.center.position
        #     old_center_z = center.z
        #     size = ros_bb.size
        #     center.z = (center.z + size.z/2)/2
        #     size.x = size.x + 0.04
        #     size.y = size.y + 0.04
        #     size.z = old_center_z + size.z/2 - 0.02
        userdata.table_bbs = response.plane_bounding_boxes
        userdata.table_plane_equations = response.planes
        return 'succeeded'
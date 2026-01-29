#! /usr/bin/env python3

import rospy
import os
import glob
from grasping_pipeline_msgs.srv import GraspMethodSelector, GraspMethodSelectorResponse

class GraspMethodSelectorSrv():
    '''
    Returns 'pose_based_grasp' if there are no 'Unknown' objects in class_names, otherwise 'direct_grasp'.
    This assumes that pose_based_grasp methods require the object class to be known.
    
    Parameters
    ----------
    class_names : list of str
        List of object class names detected in the scene.
    
    Returns
    -------
    smach-result
        'pose_based_grasp' if there are no 'Unknown' objects in class_names, otherwise 'direct_grasp'.
    '''

    def __init__(self):
        rospy.init_node('grasp_method_selector_srv')
        self.srv = rospy.Service('grasp_method_selector_srv', GraspMethodSelector, self.handle_request)
        rospy.loginfo('Grasp Method Selector Service is ready.')

    def handle_request(self, req):
        '''
        Decides which grasp method to use based on whether there are 'Unknown' objects in class_names or objects without annotated grasp points.

        Returns
        -------
        str
            'pose_based_grasp' if there are no 'Unknown' objects in class_names, otherwise 'direct_grasp'.
        '''

        dataset = rospy.get_param('grasping_pipeline/dataset')
        grasps_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), os.pardir, 'grasps', dataset)

        files = glob.glob(os.path.join(grasps_path, '*.npy'))
        objects = [os.path.basename(f).split('.')[0] for f in files]

        res = GraspMethodSelectorResponse()
        direct_grasp = []
        for class_name in req.class_names:
            if class_name not in objects or class_name == 'Unknown':
                direct_grasp.append(True)
            else:
                direct_grasp.append(False)
        res.direct_grasp = direct_grasp

        return res
   
if __name__ == '__main__':
    srv = GraspMethodSelectorSrv()
    rospy.spin()
        
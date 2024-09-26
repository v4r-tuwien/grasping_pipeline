import smach
import rospy
import os
import glob
class GraspMethodSelector(smach.State):
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
        smach.State.__init__(self, outcomes=['pose_based_grasp', 'direct_grasp'], input_keys=['class_names'])

    def execute(self, userdata):
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

        rospy.logwarn(objects)

        for class_name in userdata.class_names:
            if class_name not in objects or class_name == 'Unknown':
                return 'direct_grasp'
        return 'pose_based_grasp'
   

        
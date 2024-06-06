import smach

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
        Decides which grasp method to use based on whether there are 'Unknown' objects in class_names.

        Returns
        -------
        str
            'pose_based_grasp' if there are no 'Unknown' objects in class_names, otherwise 'direct_grasp'.
        '''
        if 'Unknown' in userdata.class_names:
            if len(set(userdata.class_names)) != 1:
                raise ValueError('Unknown objects are present in class_names, but there are other classes as well! : %s' % userdata.class_names)
            return 'direct_grasp'
        else:
            return 'pose_based_grasp'

        
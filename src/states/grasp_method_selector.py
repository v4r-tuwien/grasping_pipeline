import smach

class GraspMethodSelector(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['pose_based_grasp', 'direct_grasp'], input_keys=['class_names'])

    def execute(self, userdata):
        if 'Unknown' in userdata.class_names:
            if len(set(userdata.class_names)) != 1:
                raise ValueError('Unknown objects are present in class_names, but there are other classes as well! : %s' % userdata.class_names)
            return 'direct_grasp'
        else:
            return 'pose_based_grasp'

        
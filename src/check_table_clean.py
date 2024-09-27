import smach
from hsr_wrapper import HSR_wrapper
class CheckTableClean(smach.State):
    '''
    Checks whether any objects remain on the table.
    
    
    '''

    def __init__(self):
        #TODO add table so that we can filter objects that are not on the table, since we are only interested in objects on the table
        smach.State.__init__(self, outcomes=['clean', 'not_clean'], input_keys=['rgb', 'depth', 'bb_detections', 'mask_detections'])
        self.hsr_wrapper = HSR_wrapper()

    def execute(self, userdata):
        if len(userdata.mask_detections) == 0 and len(userdata.bb_detections) == 0:
            self.hsr_wrapper.tts_say('The table is clean.')
            return 'clean'
        else:
            objects_left = max(len(userdata.mask_detections), len(userdata.bb_detections))
            if objects_left == 1:
                self.hsr_wrapper.tts_say(f'The table is not clean. There is 1 object left on the table.')
            else:
                self.hsr_wrapper.tts_say(f'The table is not clean. There are {objects_left} objects left on the table.')
            return 'not_clean'
   

        
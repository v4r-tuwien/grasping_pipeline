#!/usr/bin/env python3


import smach
import rospy

class SelectHighestConfidenceObject(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['succeeded', 'aborted'],
                             input_keys=['object_poses', 'class_confidences', 'class_names'],
                             output_keys=['object_to_grasp'])  # Added output_keys
    
    def execute(self, userdata):
        if not userdata.object_poses or not userdata.class_confidences or not userdata.class_names:
            rospy.logerr("Missing input data for selecting the object.")
            return 'aborted'
        
        # Find the object with the highest confidence
        max_confidence_index = userdata.class_confidences.index(max(userdata.class_confidences))
        
        # Select the corresponding object pose and name
        selected_object_pose = userdata.object_poses[max_confidence_index]
        selected_object_name = userdata.class_names[max_confidence_index]
        userdata.object_to_grasp = selected_object_name
        print(userdata.class_confidences)
        
        rospy.loginfo(f"Selected object: {selected_object_name} with highest confidence.")
        print(f"Selected object: {selected_object_name} with highest confidence.")
        
        return 'succeeded'

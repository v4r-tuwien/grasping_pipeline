from hsrb_interface import Robot, geometry

class HSR_wrapper:
    '''
    Convenience wrapper for commonly used HSR robot commands.'''

    def __init__(self):
        self.robot = Robot()
        self.whole_body = self.robot.try_get('whole_body')
        self.gripper = self.robot.try_get('gripper')
        
    def move_eef_by_line(self, direction, distance):
        '''
        direction: Tuple of shape 3 with direction of movement in (x,y,z)
        distance: Distance to move in meters
        '''
        self.whole_body.move_end_effector_by_line(direction, distance)
    
    def gripper_grasp_hsr(self, force):
        self.gripper.apply_force(force)
        
    def gripper_open_hsr(self):
        self.gripper.command(1.0)
    
    def grasp_succesful(self):
        return self.gripper.get_distance() > -0.004
    
    def move_eef_by_delta(self, delta, ref_frame='base_link'):
        '''Move the end effector by the given delta in the direction of the reference frame axes
        delta: Tuple of shape 3 with the distance to move in (x,y,z)'''
        pose_vec, pose_quat = self.whole_body.get_end_effector_pose(ref_frame)
        new_pose_vec = geometry.Vector3(
            pose_vec.x + delta[0], pose_vec.y + delta[1], pose_vec.z + delta[2])
        new_pose = geometry.Pose(new_pose_vec, pose_quat)
        self.whole_body.move_end_effector_pose(new_pose)
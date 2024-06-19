from hsrb_interface import Robot, geometry

class HSR_wrapper:
    '''
    Convenience wrapper for commonly used HSR robot commands.'''

    def __init__(self):
        self.robot = Robot()
        self.whole_body = self.robot.try_get('whole_body')
        self.gripper = self.robot.try_get('gripper')
        self.tts = self.robot.try_get('default_tts')
        self.tts.language = self.tts.ENGLISH
        
    def move_eef_by_line(self, direction, distance):
        '''
        Moves the end effector by the given distance in the given direction along a line.
        
        Parameters
        ----------
        direction: (x,y,z) tuple
            Direction of movement in meters.
        distance: float
            Distance to move in meters.
        '''
        self.whole_body.move_end_effector_by_line(direction, distance)
    
    def gripper_grasp_hsr(self, force):
        '''
        Close the gripper with the given force to grasp an object.
        
        Parameters
        ----------
        force: float
            The force to apply to the gripper in Newton.
        '''
        self.gripper.apply_force(force)
        
    def gripper_open_hsr(self):
        '''
        Open the gripper.
        '''
        self.gripper.command(1.0)
    
    def grasp_succesful(self):
        '''
        Check if the gripper has successfully grasped an object.
        
        The check is based on the gripper distance (distance between the two tips). If the 
        grasp failed, the tips are very close to each other. If the grasp is successful, the
        tips are further apart. The threshold is set to -0.004 meters.
        '''
        return self.gripper.get_distance() > -0.004
    
    def move_eef_by_delta(self, delta, ref_frame='base_link'):
        '''
        Move the end effector by a given distance in the direction of the reference frame axes.
        
        Parameters
        ----------
        delta: (x,y,z) tuple
            The distance to move in meters.
        ref_frame: str
            The reference frame to move in.
        '''
        pose_vec, pose_quat = self.whole_body.get_end_effector_pose(ref_frame)
        new_pose_vec = geometry.Vector3(
            pose_vec.x + delta[0], pose_vec.y + delta[1], pose_vec.z + delta[2])
        new_pose = geometry.Pose(new_pose_vec, pose_quat)
        self.whole_body.move_end_effector_pose(new_pose)

    def tts_say(self, text):
        '''
        Let the robot say the given text.
        
        Parameters
        ----------
        text: str
            The text to say.
        '''
        self.tts.say(text)
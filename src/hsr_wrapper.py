from hsrb_interface import Robot

class HSR_wrapper:

    def __init__(self):
        self.hsr_robot, self.hsr_whole_body, self.hsr_gripper = self.init_hsr()
        
    def init_hsr(self):
        robot = Robot()
        whole_body = robot.try_get('whole_body')
        gripper = robot.try_get('gripper')
        return robot, whole_body, gripper
    
    def move_eef_by_line(self, direction, distance):
        '''
        direction: Tuple of shape 3 with direction of movement in (x,y,z)
        distance: Distance to move in meters
        '''
        self.hsr_whole_body.move_end_effector_by_line(direction, distance)
    
    def gripper_grasp_hsr(self, force):
        self.hsr_gripper.apply_force(force)
        
    def gripper_open_hsr(self):
        self.hsr_gripper.command(1.0)
    
    def grasp_succesful(self):
        return self.hsr_gripper.get_distance() > -0.004
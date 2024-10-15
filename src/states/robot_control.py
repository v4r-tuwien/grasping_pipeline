from math import pi
from tf.transformations import quaternion_about_axis 
import rospy
import actionlib
import smach
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from hsrb_interface import Robot
from hsrb_interface.exceptions import MobileBaseError
from grasping_pipeline_msgs.msg import BoundingBox3DStamped


class GoToNeutral(smach.State):
    """ Smach state that will move the robots joints to a
    predefined position, gazing at a fixed point.

    Returns
    -------
    smach-result
        'succeeded': The state only returns succeeded.
    """
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])
        # Robot initialization
        self.robot = Robot()
        self.whole_body = self.robot.try_get('whole_body')

    def execute(self, userdata):
        rospy.loginfo('Executing state GoToNeutral')
        joint_positions = {
            'arm_flex_joint': 0, 
            'arm_lift_joint': 0, 
            'arm_roll_joint': pi/2, 
            'head_pan_joint': 0, 
            'head_tilt_joint': -0.675, 
            'wrist_flex_joint': -pi/2, 
            'wrist_roll_joint': 0
            }
        self.whole_body.move_to_joint_positions(joint_positions)
        return 'succeeded'

class MoveToJointPositions(smach.State):
    """ Smach state that will move the robots joints to the
    defined position.
    
    Returns
    -------
    smach-result
        'succeeded': The state only returns succeeded.
    """

    def __init__(self, joint_positions_dict):
        '''
        Initializes the MoveToJointPositions state. Sets the joint positions dictionary.

        Parameters
        ----------
        joint_positions_dict: dict
            Dictionary with joint names as keys and joint positions as values. Possible keys are:
            'arm_flex_joint', 'arm_lift_joint', 'arm_roll_joint', 'head_pan_joint', 'head_tilt_joint',
            'wrist_flex_joint', 'wrist_roll_joint'. The angles are in radians.
        '''
        smach.State.__init__(self, outcomes=['succeeded'])
        # Robot initialization
        self.robot = Robot()
        self.whole_body = self.robot.try_get('whole_body')
        self.joint_positions_dict = joint_positions_dict

    def execute(self, userdata):
        rospy.loginfo('Executing state MoveToJointPositions')
        self.whole_body.move_to_joint_positions(self.joint_positions_dict)
        return 'succeeded'


class GoBack(smach.State):
    """ Smach state that will move the robot backwards.
    
    Returns
    -------
    smach-result
        'succeeded': The state only returns succeeded.
    """

    def __init__(self, distance):
        '''
        Initializes the GoBack state. Sets the distance to move backwards.
        
        Parameters
        ----------
        distance: float
            The distance to move backwards in meters.
        '''
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'])
        # Robot initialization
        self.robot = Robot()
        self.base = self.robot.try_get('omni_base')
        self.whole_body = self.robot.try_get('whole_body')
        self.distance = distance

    def execute(self, userdata):
        rospy.loginfo('Executing state GoBack')
        try:
            self.base.go_rel(-self.distance, 0, 0, timeout=20.0)
        except MobileBaseError as e:
            rospy.logerr(e)
            return 'aborted'
        return 'succeeded'


class OpenGripper(smach.State):
    """ Opens the robots gripper
    
    Returns
    -------
    smach-result
        'succeeded': The state only returns succeeded.
    """

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])

        # Robot initialization
        self.robot = Robot()
        self.gripper = self.robot.try_get('gripper')

    def execute(self, userdata):
        rospy.loginfo('Executing state OpenGripper')
        self.gripper.command(1.0)
        return 'succeeded'

class GoToWaypoint(smach.State):
    '''
    Moves the robot to a given waypoint.
    
    Returns
    -------
    smach-result
        'succeeded': If the robot reached the waypoint.
        'aborted': If the robot could not reach the waypoint, either because action took to long and
        timed out or because the connection to the move server could not be established.
    '''
    def __init__(self, x, y, phi_degree, frame_id='map', timeout=30.0):
        '''
        Initializes the GoToWaypoint state. Sets the target pose and frame_id.
        
        Parameters
        ----------
        x: float
            x-coordinate of the target pose in meters
        y: float
            y-coordinate of the target pose in meters
        phi_degree: float
            Rotation around the z-axis in degrees of the target pose
        frame_id: str
            The frame_id of the target pose.
        timeout: float
            Timeout in seconds. The state will abort if the robot does not reach the target pose in 
            this time.
        '''
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'])
        self.move_client = actionlib.SimpleActionClient(
            '/move_base/move', MoveBaseAction)
        self.robot = Robot()
        self.whole_body = self.robot.try_get('whole_body')
        self.x = x
        self.y = y
        self.phi = phi_degree
        self.frame_id = frame_id
        self.timeout = timeout

    def execute(self, userdata):
        move_goal = MoveBaseGoal()
        
        move_goal.target_pose.header.frame_id = self.frame_id
        move_goal.target_pose.pose.position.x = self.x
        move_goal.target_pose.pose.position.y = self.y
        quat = quaternion_about_axis(self.phi * pi/180, (0, 0, 1))
        move_goal.target_pose.pose.orientation.x = quat[0]
        move_goal.target_pose.pose.orientation.y = quat[1]
        move_goal.target_pose.pose.orientation.z = quat[2]
        move_goal.target_pose.pose.orientation.w = quat[3]
        
        rospy.loginfo("Waiting for move client server!")
        finished = self.move_client.wait_for_server(rospy.Duration(self.timeout))
        
        if finished:
            self.move_client.send_goal(move_goal)
            rospy.loginfo("Waiting for result")
            finished = self.move_client.wait_for_result(rospy.Duration(self.timeout))

            if finished:
                # wait for robot to settle down
                rospy.sleep(1.0)
                return 'succeeded'
            else:
                rospy.logerr("Move server execution timed out!")
                self.move_client.cancel_all_goals()
                return 'aborted'
        else:
            rospy.logerr("Could not connect to move server!")
            return 'aborted'
        
    
class GoToAndLookAtPlacementArea(smach.State):
    '''
    Navigates the robot to the waypoint of the placement area and makes it look at the placement area.
    
    Parameters
    ----------
    grasp_object_name: str
        The name of the object that was grasped. This is used to determine the placement area as each
        object has a predefined placement area.
    
    Returns
    -------
    smach-result
        'succeeded': If the robot reached the waypoint and looked at the placement area.
        'aborted': If the robot could not reach the waypoint, either because action took to long and
        timed out or because the connection to the move server could not be established, or if no
        placement area is specified for this object.
    placement_area_bb: BoundingBox3DStamped
        The bounding box of the placement area the robot should place the object in. This is read
        from the parameter server.
    '''
    def __init__(self, outcomes=['succeeded', 'aborted'], input_keys=['grasp_object_name'], output_keys=['placement_area_bb'] ):
        smach.State.__init__(self, outcomes=outcomes, input_keys=input_keys, output_keys=output_keys)
        self.move_client = actionlib.SimpleActionClient(
            '/move_base/move', MoveBaseAction)
        self.robot = Robot()
        self.whole_body = self.robot.try_get('whole_body')
        self.timeout = 40.0
    
    def execute(self, userdata):
        '''
        Moves the robot to the waypoint of the placement area and makes it look at the placement area.
        
        Loads the placement planes and objects from the parameter server. The placement area of the
        object is determined by the object name. The robot moves to the waypoint of the placement area
        and looks at the placement area based on the data loaded from the parameter server.
        '''
        placement_planes = rospy.get_param('/placement_planes')
        placement_objects = rospy.get_param('/placement_objects')
        grasp_object_name = userdata.grasp_object_name

        if grasp_object_name not in placement_objects:
            rospy.logerr(f"Object {grasp_object_name} not found in placement_objects!")
            return 'aborted'
        
        placement_plane_name = placement_objects[grasp_object_name]
        placement_plane_config = placement_planes['planes'][placement_plane_name]
        frame_id = placement_planes['metadata']['frame_id']
        waypoint = placement_plane_config['waypoint']
        placement_area = placement_plane_config['placement_area']
        placement_area_bb = BoundingBox3DStamped()
        placement_area_bb.center.position.x = placement_area['center'][0]
        placement_area_bb.center.position.y = placement_area['center'][1]
        placement_area_bb.center.position.z = placement_area['center'][2]
        placement_area_bb.center.orientation.w = 1.0
        placement_area_bb.size.x = placement_area['size'][0]
        placement_area_bb.size.y = placement_area['size'][1]
        placement_area_bb.size.z = placement_area['size'][2]
        placement_area_bb.header.frame_id = frame_id
        userdata.placement_area_bb = placement_area_bb
        
        move_goal = MoveBaseGoal()
        
        move_goal.target_pose.header.frame_id = frame_id
        move_goal.target_pose.pose.position.x = waypoint[0]
        move_goal.target_pose.pose.position.y = waypoint[1]
        quat = quaternion_about_axis(waypoint[2] * pi/180, (0, 0, 1))
        move_goal.target_pose.pose.orientation.x = quat[0]
        move_goal.target_pose.pose.orientation.y = quat[1]
        move_goal.target_pose.pose.orientation.z = quat[2]
        move_goal.target_pose.pose.orientation.w = quat[3]

        # arm_lift moves by ~ 10 cm for each 0.2 increase and has to be between 0 and 0.6
        arm_lift_joint = max(0.0, (placement_area['center'][2] - 0.45)*2.0)
        arm_lift_joint = min(arm_lift_joint, 0.6)

        # move that pesky arm out of the way
        arm_flex_joint = -2.6 if arm_lift_joint > 0.1 else -0.1
        self.whole_body.move_to_joint_positions({'arm_flex_joint': arm_flex_joint, 'arm_lift_joint': arm_lift_joint})
        rospy.sleep(5.0)
        
        rospy.loginfo("Waiting for move client server!")
        finished = self.move_client.wait_for_server(rospy.Duration(self.timeout))
        
        if finished:
            self.move_client.send_goal(move_goal)
            finished = self.move_client.wait_for_result(rospy.Duration(self.timeout))

            if finished:
                # wait for robot to settle down
                rospy.sleep(1.0)
                return 'succeeded'
            else:
                rospy.logerr("Move server execution timed out!")
                self.move_client.cancel_all_goals()
                return 'aborted'
        else:
            rospy.logerr("Could not connect to move server!")
            return 'aborted'
        

class CheckTopGrasp(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['top_grasp', 'not_top_grasp'],
                             input_keys=['top_grasp'])
    
    def execute(self, userdata):
        if userdata.top_grasp:
            return 'top_grasp'
        else:
            return 'not_top_grasp'
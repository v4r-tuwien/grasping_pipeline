from math import pi
from tf.transformations import quaternion_about_axis 
import rospy
import actionlib
import smach
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from hsrb_interface import Robot


class GoToNeutral(smach.State):
    """ Smach state that will move the robots joints to a
    predefined position, gazing at a fixed point.

    Outcomes:
        succeeded
    """

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])
        # Robot initialization
        self.robot = Robot()
        self.whole_body = self.robot.try_get('whole_body')

    def execute(self, userdata):
        rospy.loginfo('Executing state GoToNeutral')
        self.whole_body.move_to_neutral()
        self.whole_body.move_to_joint_positions({'arm_roll_joint': pi / 2})
        self.whole_body.gaze_point((0.8, 0.05, 0.4))
        return 'succeeded'

class MoveToJointPositions(smach.State):
    """ Smach state that will move the robots joints to a
    predefined position, gazing at a fixed point.

    Outcomes:
        succeeded
    """

    def __init__(self, joint_positions_dict):
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

    Outcomes:
        succeeded
    """

    def __init__(self, go_back_in_meters):
        smach.State.__init__(self, outcomes=['succeeded'])
        # Robot initialization
        self.robot = Robot()
        self.base = self.robot.try_get('omni_base')
        self.whole_body = self.robot.try_get('whole_body')
        self.go_back_in_meters = go_back_in_meters

    def execute(self, userdata):
        rospy.loginfo('Executing state GoBack')
        self.base.go_rel(-self.go_back_in_meters, 0, 0)
        return 'succeeded'


class OpenGripper(smach.State):
    """ Opens the robots gripper

    Outcomes:
        succeeded
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
    def __init__(self, x, y, phi_degree, frame_id='map'):
        '''
        phi is angle in degree
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
        self.move_client.wait_for_server()
        self.move_client.send_goal(move_goal)
        result = self.move_client.wait_for_result()

        if result:
            # wait for robot to settle down
            rospy.sleep(1.0)
            return 'succeeded'
        else:
            return 'aborted'
        
    
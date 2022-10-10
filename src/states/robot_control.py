from math import pi
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


class GoBack(smach.State):
    """ Smach state that will move the robot backwards.

    Outcomes:
        succeeded
    """

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])
        # Robot initialization
        self.robot = Robot()
        self.base = self.robot.try_get('omni_base')
        self.whole_body = self.robot.try_get('whole_body')

    def execute(self, userdata):
        rospy.loginfo('Executing state GoBack')
        self.base.go_rel(-0.1, 0, 0)
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


class GoToTable(smach.State):
    """Moves the Robot to a fixed position near the table
    using a move_base action goal.

    Outcomes:
        succeeded
        aborted - when the move-actionserver fails
    """

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'])
        self.move_client = actionlib.SimpleActionClient(
            '/move_base/move', MoveBaseAction)
        self.robot = Robot()
        self.whole_body = self.robot.try_get('whole_body')

    def execute(self, userdata):
        move_goal = MoveBaseGoal()
        move_goal.target_pose.header.frame_id = 'map'
        move_goal.target_pose.pose.position.x = 0.5
        move_goal.target_pose.pose.position.y = 0.1
        move_goal.target_pose.pose.orientation.z = 0.0
        move_goal.target_pose.pose.orientation.w = 1.0
        self.move_client.wait_for_server()
        self.move_client.send_goal(move_goal)
        result = self.move_client.wait_for_result()

        if result:
            # go to neutral
            self.whole_body.move_to_neutral()
            self.whole_body.move_to_joint_positions({'arm_roll_joint': pi / 2})
            self.whole_body.gaze_point((0.8, 0.05, 0.4))
            rospy.sleep(1.0)

            vel = self.whole_body.joint_velocities
            while all(abs(i) > 0.05 for i in vel.values()):
                vel = self.whole_body.joint_velocities
            rospy.sleep(2)

            return 'succeeded'
        else:
            return 'aborted'

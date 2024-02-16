import smach
import rospy


class UserInput(smach.State):

    def __init__(self, key_outcome_description):
        self.map = key_outcome_description
        outcomes = []
        descriptions = []
        keys = []

        for key in key_outcome_description:

            if len(key) != 1:
                rospy.logerr(
                    'Error while seting up UserInput: Key should only be a single character!')
                rospy.logerr('Entry that led to error: ' +
                             str(key) + ' : ' + str(key_outcome_description[key]))

            if len(key_outcome_description[key]) != 2:
                rospy.logerr(
                    'Error while setting up UserInput: Value should have exactly two entries!')
                rospy.logerr('Entry that led to error: ' +
                             str(key) + ' : ' + str(key_outcome_description[key]))

            outcomes.append(key_outcome_description[key][0])

        smach.State.__init__(self, outcomes=outcomes)

    def execute(self, userdata):
        rospy.loginfo('Executing state UserInput')
        self.print_menu()
        return self.handle_userinput()

    def print_menu(self):
        print('Enter command:')
        for key in self.map:
            print('\t' + str(key) + ' - ' + str(self.map[key][1]))

    def handle_userinput(self):
        while True:
            user_input = input('CMD> ')
            if len(user_input) != 1:
                print('Please enter only one character')
                continue
            char_input = user_input.lower()
            if char_input not in self.map:
                print('Invalid key!')
                continue
            return self.map[char_input][0]


if __name__ == '__main__':
    map = {'c': ['success', 'continue'], 'r': ['abort', 'reset state']}
    u = UserInput(map)
    u.execute()

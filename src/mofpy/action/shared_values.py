import rospy

from .action import Action
from ..shared import Shared


class SharedValues(Action):
    NAME = 'shared_values'

    def __init__(self, definition):
        super(SharedValues, self).__init__(definition)
        Action.actions[self.__class__.NAME] = self.__class__

        self.__key = self.get_required('key')
        # Index of the values that's currently selected
        self.__shared_index_key = self.__key + '_index'
        # All possible values
        self.__values_key = self.__key + '_values'

        # TODO: In bidirectional, allow specifying values in only one action
        self.__all_values = self.get_required('values')

        # Selection: round-robin or bidirectional
        selection = self.get('selection', 'round_robin')
        self.__is_bidirectional = selection == 'bidirectional'

        # Push of a button increments/decrements the index
        direction = self.get('direction', 'inc').lower()
        self.__is_increment = direction.startswith('inc') or direction == '+'

        # Automatically select the first value
        self.__select__(0)

    def execute(self, named_joy=None):
        # Note: Handles increment/decrement
        index = self.__next_index__()
        value = self.__select__(index)

        rospy.loginfo('{0} : {1}'.format(self.__key, value))

    def __select__(self, index):
        Shared.add(self.__shared_index_key, index)

        value = self.__all_values[index]
        Shared.add(self.__key, value)
        return value

    def __next_index__(self):
        curr_index = Shared.get(self.__shared_index_key)
        if self.__is_increment:
            next_index = curr_index + 1
        else:
            next_index = curr_index - 1

        next_index = next_index % len(self.__all_values)
        self.__select__(next_index)
        return next_index


Action.register_preset(SharedValues)

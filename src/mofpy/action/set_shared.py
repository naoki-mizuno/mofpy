from .action import Action
from ..shared import Shared

import six


class SetShared(Action):
    NAME = 'set_shared'

    """
    :type __duration: float
    """
    def __init__(self, definition):
        super(SetShared, self).__init__(definition)
        Action.actions[self.__class__.NAME] = self.__class__

        self.__key = self.get_required_key('key')
        value = self.get_required_key('value')
        if isinstance(value, six.string_types):
            self.__value = [value]
        else:
            self.__value = value
        self.__value_index = 0

    def execute(self, named_joy=None):
        val = self.__value[self.__value_index]
        Shared.add(self.__key, val)
        self.__value_index = (self.__value_index + 1) % len(self.__value)


Action.register_preset(SetShared)

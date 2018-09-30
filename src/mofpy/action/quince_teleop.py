import rospy

from .action import Action
from ..shared import Shared


class QuinceTeleop(Action):
    """
    :type __duration: float
    """

    NAME = 'quince_teleop'

    def __init__(self, definition):
        super(QuinceTeleop, self).__init__(definition)
        Action.actions[self.__class__.NAME] = self.__class__

        self.__publish_mode = self.get('publish_mode')
        self.__publish_rate = self.get('publish_rate')
        self.__cmd_vel_topic = self.get_required('cmd_vel_topic')
        self.__flipper_topic = self.get_required('flipper_topic')

    def execute(self, named_joy=None):
        if Shared.get('twist_mode') != 'quince':
            return

        pass


Action.register_preset(QuinceTeleop)

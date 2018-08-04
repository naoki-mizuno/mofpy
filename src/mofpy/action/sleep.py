import rospy

from .action import Action


class Sleep(Action):
    """
    :type __duration: float
    """

    NAME = 'sleep'

    def __init__(self, definition):
        super(Sleep, self).__init__(definition)
        Action.actions[self.__class__.NAME] = self.__class__

        self.__duration = self.get_required_key('duration')

    def execute(self, named_joy=None):
        rospy.loginfo('Sleeping for {0} seconds'.format(self.__duration))
        rospy.sleep(self.__duration)


Action.register_preset(Sleep)

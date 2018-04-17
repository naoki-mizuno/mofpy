import rospy


from .preset_task import PresetTask


class Sleep(PresetTask):
    """
    :type __duration: float
    """
    def __init__(self, definition):
        super(Sleep, self).__init__(definition)

        self.__duration = self.get_required_key('duration')

    def execute(self):
        rospy.loginfo('Sleeping for {0} seconds'.format(self.__duration))
        rospy.sleep(self.__duration)

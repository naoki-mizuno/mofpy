import rospy
from std_msgs.msg import String

from .action import Action

import datetime


class PublishString(Action):
    """
    Publishes a String message to the specified topic

    :type __topic_name: str
    """

    NAME = 'publish_string'

    def __init__(self, definition):
        super(PublishString, self).__init__(definition)
        Action.actions[self.__class__.NAME] = self.__class__

        self.__topic_name = self.get_required('topic')
        self.__value = self.get_required('value')
        self.__include_time = self.get('include_time', False)

        self.__pub = rospy.Publisher(self.__topic_name,
                                     String,
                                     queue_size=1)

    def execute(self, named_joy=None):
        s = String()
        s.data = ''
        if self.__include_time:
            unix_time = rospy.Time.now().to_sec()
            dt = datetime.datetime.fromtimestamp(unix_time)
            s.data = '[{0}] '.format(dt.strftime('%Y-%m-%dT%H:%M:%S'))
        s.data += str(self.__value)
        self.__pub.publish(s)


Action.register_preset(PublishString)

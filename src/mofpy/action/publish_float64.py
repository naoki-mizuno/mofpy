import rospy
from std_msgs.msg import Float64

from .action import Action


class PublishFloat64(Action):
    """
    Publishes a Float64 message to the specified topic

    :type __topic_name: str
    """

    NAME = 'publish_float64'

    def __init__(self, definition):
        super(PublishFloat64, self).__init__(definition)
        Action.actions[self.__class__.NAME] = self.__class__

        self.__topic_name = self.get_required('topic')
        self.__value = self.get_required('value')

        self.__pub = rospy.Publisher(self.__topic_name,
                                     Float64,
                                     queue_size=1)

    def execute(self, named_joy=None):
        f = Float64()
        f.data = self.__value
        self.__pub.publish(f)


Action.register_preset(PublishFloat64)

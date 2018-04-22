import rospy
from std_msgs.msg import Float64

from .preset_task import PresetTask


class PublishFloat64(PresetTask):
    """
    Publishes a Float64 message to the specified topic

    :type __topic_name: str
    """
    def __init__(self, definition):
        super(PublishFloat64, self).__init__(definition)

        self.__topic_name = self.get_required_key('topic')
        self.__value = self.get_required_key('value')

        self.__pub = rospy.Publisher(self.__topic_name,
                                     Float64,
                                     queue_size=1)

    def execute(self):
        f = Float64()
        f.data = self.__value
        self.__pub.publish(f)

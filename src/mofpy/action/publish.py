import rospy
import rosgraph
import roslib.message
import rostopic
import yaml

from .action import Action


class Publish(Action):
    """
    :type __topic_name: str
    :type __topic_type: str
    :type __values: dict
    :type __latch: bool
    """

    NAME = 'publish'

    def __init__(self, definition):
        super(Publish, self).__init__(definition)
        Action.actions[self.__class__.NAME] = self.__class__

        self.__topic_name = self.get_required('topic/name')
        self.__topic_type = self.get_required('topic/type')
        self.__values = self.get_required('values')
        self.__latch = self.get('topic/latch', False)

        self.__pub, self.__msg_class = Publish.create_publisher(
            self.__topic_name, self.__topic_type, self.__latch
        )

    @staticmethod
    def create_publisher(topic_name, topic_type, latch):
        """
        See rostopic.create_publisher
        """
        resolved_topic_name = rosgraph.names.script_resolve_name('mofpy',
                                                                 topic_name)
        try:
            msg_class = roslib.message.get_message_class(topic_type)
        except Exception:
            err_msg = 'Invalid topic type: {0}'.format(topic_type)
            raise rostopic.ROSTopicException(err_msg)

        if msg_class is None:
            err_msg = 'Invalid message type: {0}'.format(topic_type)
            raise rostopic.ROSTopicException(err_msg)

        pub = rospy.Publisher(resolved_topic_name,
                              msg_class,
                              latch=latch,
                              queue_size=100)

        return pub, msg_class

    def execute(self, named_joy=None):
        yaml_vals = [yaml.load(str(self.__values))]
        msg = self.__msg_class()
        try:
            rostopic._fillMessageArgs(msg, yaml_vals)
        except rostopic.ROSTopicException as e:
            rospy.logerr(e)
            return

        self.__pub.publish(msg)


Action.register_preset(Publish)

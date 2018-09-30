import rospy
from flipper_msgs.msg import flipper_control

from .action import Action


class PubFlipperControl(Action):
    """
    :type __duration: float
    """

    NAME = 'flipper_control'

    def __init__(self, definition):
        super(PubFlipperControl, self).__init__(definition)
        Action.actions[self.__class__.NAME] = self.__class__

        self.__topic = self.get_required('topic')
        self.__angle = self.get_required('angle')

        self.__pub = rospy.Publisher(self.__topic,
                                     flipper_control,
                                     queue_size=1)

    def execute(self, named_joy=None):
        msg = flipper_control()
        # TODO: Populate all values in the message
        msg.fl = self.__angle['fl']
        msg.fr = self.__angle['fr']
        msg.rl = self.__angle['rl']
        msg.rr = self.__angle['rr']

        self.__pub.publish(msg)


Action.register_preset(PubFlipperControl)

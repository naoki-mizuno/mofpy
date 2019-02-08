import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState

from .action import Action


class Backhoe(Action):
    """
    """

    NAME = 'backhoe'

    def __init__(self, definition):
        super(Backhoe, self).__init__(definition)
        Action.actions[self.__class__.NAME] = self.__class__

        self.__prefix = self.get('topics/prefix', '')
        self.__suffix = self.get('topics/suffix', '')
        self.__topics = self.get_required('topics/joints')

        self.__use = self.get_required('use')
        self.__js_topic = self.get('joint_states', 'joint_states')
        self.__scale = self.get('scale', 1)
        self.__methods = self.get_required('methods')
        self.__controls = self.__methods[self.__use]
        self.__current_values = {}

        self.__pubs = {}
        for joint_name in self.__controls.keys():
            topic_name = '{0}{1}{2}'.format(self.__prefix,
                                            self.__topics[joint_name],
                                            self.__suffix)
            self.__pubs[joint_name] = rospy.Publisher(topic_name,
                                                      Float64,
                                                      queue_size=1)

        self.__joint_states_sub = rospy.Subscriber(self.__js_topic,
                                                   JointState,
                                                   self.cb_js,
                                                   queue_size=1)

    def cb_js(self, msg):
        for i in range(len(msg.name)):
            joint_name = msg.name[i]
            joint_pos = msg.position[i]
            self.__current_values[joint_name] = joint_pos

    def execute(self, named_joy=None):
        if len(self.__current_values) == 0:
            return

        for joint_name in self.__controls.keys():
            current_value = self.__current_values[joint_name]
            axis_name = self.__controls[joint_name].lstrip('-')
            is_flip = self.__controls[joint_name].startswith('-')
            pub = self.__pubs[joint_name]

            delta = named_joy['axes'][axis_name].value * self.__scale
            if delta == 0:
                continue
            if is_flip:
                delta *= -1
            msg = Float64()
            msg.data = current_value + delta
            pub.publish(msg)

            # TODO: Race condition
            self.__current_values[joint_name] = msg.data


Action.register_preset(Backhoe)

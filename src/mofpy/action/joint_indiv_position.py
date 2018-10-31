import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState

from .action import Action


class JointIndivPosition(Action):
    """
    :type __joints: dict[str, dict[str, str]]
    :type __latest_joint_state: JointState
    """
    NAME = 'joint_indiv_position'

    def __init__(self, definition):
        super(JointIndivPosition, self).__init__(definition)
        Action.actions[self.__class__.NAME] = self.__class__

        self.__step = self.get_required('step_angle')
        self.__control = self.get_required('control_axis')
        self.__stale_timeout = rospy.Duration(self.get('stale_timeout', 1))
        self.__joints = self.get_required('joints')
        joint_states_topic = self.get('joint_states', 'joint_states')

        self.__pubs = {}
        self.__last_pub_time = rospy.Time(0)
        for name in self.__joints.keys():
            topic_name = self.__joints[name]['topic']
            self.__pubs[name] = rospy.Publisher(topic_name,
                                                Float64,
                                                queue_size=1)

        # Current joint values
        self.__joint_values = {}
        for name in self.__joints.keys():
            self.__joint_values[name] = None

        # Actual joint values obtained from the joint_states topic
        self.__latest_joint_state = JointState()
        self.__sub_joint_states = rospy.Subscriber(joint_states_topic,
                                                   JointState,
                                                   self.cb_joint_states,
                                                   queue_size=1)

    def cb_joint_states(self, msg):
        self.__latest_joint_state = msg

    def __reset_to_actual_joint_values__(self):
        not_updated = len(self.__latest_joint_state.name) == 0
        if not_updated:
            msg = 'Current joint values empty. joint_states not published?'
            rospy.logwarn_throttle(10, msg)
            return False

        for name in self.__joint_values.keys():
            if name not in self.__latest_joint_state.name:
                continue
            idx = self.__latest_joint_state.name.index(name)
            actual_val = self.__latest_joint_state.position[idx]
            self.__joint_values[name] = actual_val
        return True

    def execute(self, named_joy=None):
        if rospy.Time.now() - self.__last_pub_time > self.__stale_timeout:
            success = self.__reset_to_actual_joint_values__()
            if not success:
                return

        # Don't publish anything on zero input
        zero_input = True

        # Apply the changes
        delta = named_joy['axes'][self.__control].value * self.__step
        pressed = filter(lambda name: named_joy['buttons'][name],
                         named_joy['buttons'].keys())
        for name in self.__joint_values.keys():
            enable_button = self.__joints[name]['enable_button']
            if enable_button in pressed:
                self.__joint_values[name] += delta
                zero_input = False

        if zero_input or delta == 0:
            return

        # Publish the new values
        for name in self.__joint_values.keys():
            msg = Float64()
            msg.data = self.__joint_values[name]
            self.__pubs[name].publish(msg)
        self.__last_pub_time = rospy.Time.now()


Action.register_preset(JointIndivPosition)

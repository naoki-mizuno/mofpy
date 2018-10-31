import rospy

from .action import Action
from ..joint_trajectory_publisher import JointTrajectoryPublisher
from ..shared import Shared


class JointIndivTrajectory(Action):
    """
    :type __enable_buttons: dict[str, str]
    """
    NAME = 'joint_indiv_trajectory'

    def __init__(self, definition):
        super(JointIndivTrajectory, self).__init__(definition)
        Action.actions[self.__class__.NAME] = self.__class__

        self.__out_topic = self.get_required('topic')
        self.__frame_id = self.get('frame_id', 'world')
        self.__trajectory_duration = self.get('trajectory_duration', 0.1)
        self.__enable_buttons = self.get_required('enable_button')
        self.__control_axis = self.get_required('control_axis')
        self.__ang_speed = self.get_required('angular_speed')
        self.__stale_timeout = rospy.Duration(self.get('stale_timeout', 1))

        self.__jtpub = JointTrajectoryPublisher(self.__out_topic)
        self.__joint_values = {}

        self.__last_pub_time = rospy.Time(0)

    def __reset_to_actual_joint_values__(self):
        self.__joint_values = self.__jtpub.get_current_joint_values()

    def execute(self, named_joy=None):
        if Shared.get('move_group_disabled'):
            msg = 'move_group is disabled. Doing nothing!'
            rospy.logerr_throttle(10, msg)
            return

        if rospy.Time.now() - self.__last_pub_time > self.__stale_timeout:
            self.__reset_to_actual_joint_values__()

        # Don't publish anything on zero input
        zero_input = True

        # Apply the changes
        delta = self.__ang_speed * self.__trajectory_duration
        delta *= named_joy['axes'][self.__control_axis].value
        pressed = filter(lambda name: named_joy['buttons'][name].value,
                         named_joy['buttons'].keys())
        for name in self.__enable_buttons.keys():
            if name not in self.__joint_values:
                msg = 'Joint {0} is specified in YAML but is unknown'
                rospy.logerr(msg.format(name))
                continue
            enable_button = self.__enable_buttons[name]
            if enable_button in pressed:
                self.__joint_values[name] += delta
                zero_input = False

        if zero_input or delta == 0:
            return

        # Publish a JointTrajectory message
        self.__jtpub.publish(self.__joint_values,
                             self.__frame_id,
                             self.__trajectory_duration)
        self.__last_pub_time = rospy.Time.now()


Action.register_preset(JointIndivTrajectory)

import rospy
from geometry_msgs.msg import Twist, TwistStamped

from .action import Action
from ..shared import Shared


class Twist2DOF(Action):
    """
    :type __out_topic: str
    """

    NAME = 'twist_2dof'

    def __init__(self, definition):
        super(Twist2DOF, self).__init__(definition)
        Action.actions[self.__class__.NAME] = self.__class__

        self.__out_topic = self.get('out_topic', 'cmd_vel')
        self.__frame_id = self.get('frame_id', 'base_link')
        self.__not_stamped = self.get('not_stamped', False)
        self.__scale_translation = self.get('scale/translation', 1)
        self.__scale_rotation = self.get('scale/rotation', 1)

        cls = Twist if self.__not_stamped else TwistStamped
        self.__pub_cmd_vel = rospy.Publisher(self.__out_topic,
                                             cls,
                                             queue_size=1)

    def execute(self, named_joy=None):
        self.pub_cmd_vel(named_joy['axes'])

    def pub_cmd_vel(self, named_axes):
        mode = Shared.get('cmd_vel_mode', 'normal')
        if mode == 'beginner':
            # Left vertical: velocity in X, left horizontal: ang vel in yaw
            # Note: Left is 1.0 and right is -1.0 in horizontal
            v = named_axes[self.get_required('beginner/v')].value
            w = named_axes[self.get_required('beginner/w')].value
        elif mode == 'tank':
            left = named_axes[self.get_required('tank/left')].value
            right = named_axes[self.get_required('tank/right')].value
            v = float(right + left) / 2
            w = float(right - left) / 2
        else:  # mode == 'normal'
            # Left vertical: velocity in X, right horizontal: ang vel in yaw
            v = named_axes[self.get_required('normal/v')].value
            w = named_axes[self.get_required('normal/w')].value

        # Apply scaling
        v *= float(Shared.get('scale_translation', self.__scale_translation))
        w *= float(Shared.get('scale_rotation', self.__scale_rotation))

        twist = Twist()
        twist.linear.x = v
        twist.angular.z = w

        if self.__not_stamped:
            msg = twist
        else:
            msg = TwistStamped()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = self.__frame_id
            msg.twist = twist
        self.__pub_cmd_vel.publish(msg)


Action.register_preset(Twist2DOF)

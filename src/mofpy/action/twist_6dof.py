import rospy
from geometry_msgs.msg import Twist, TwistStamped

from .action import Action


class Twist6DOF(Action):
    NAME = 'twist_6dof'

    def __init__(self, definition):
        super(Twist6DOF, self).__init__(definition)

        self.__frame_id = self.get('frame_id', 'base_link')
        self.__scale_trn = self.get('scale/translation', 0.1)
        self.__scale_rot = self.get('scale/rotation', 0.01)
        self.__quiet_on_zero = self.get('quiet_on_zero', True)
        self.__out_topic = self.get('out_topic', 'cmd_vel')
        self.__not_stamped = self.get('not_stamped', False)
        self.__mapping = self.__mapping__()
        self.__published_zero = False

        cls = Twist if self.__not_stamped else TwistStamped
        self.__pub = rospy.Publisher(self.__out_topic,
                                     cls,
                                     queue_size=1)

    def execute(self, named_joy=None):
        twist, is_quiet = self.__get_twist__(named_joy['axes'])

        if self.__quiet_on_zero:
            if is_quiet:
                # Publish the all-zero message just once
                if not self.__published_zero:
                    self.__pub.publish(twist)
                    self.__published_zero = True
                return

        self.__pub.publish(twist)
        self.__published_zero = False

    def __mapping__(self):
        params = self.get('mapping', dict())
        mapping = {}
        for key in params.keys():
            val = params[key]
            if type(val) is tuple or type(val) is list:
                mapping[key] = [val[0], val[1]]
            else:
                mapping[key] = [val]

        return mapping

    def __get_twist__(self, named_axes):
        dx = self.__scale_trn * self.__get_value__('x', named_axes)
        dy = self.__scale_trn * self.__get_value__('y', named_axes)
        dz = self.__scale_trn * self.__get_value__('z', named_axes)
        d_roll = self.__scale_rot * self.__get_value__('R', named_axes)
        d_pitch = self.__scale_rot * self.__get_value__('P', named_axes)
        d_yaw = self.__scale_rot * self.__get_value__('Y', named_axes)

        twist = Twist()
        twist.linear.x = dx
        twist.linear.y = dy
        twist.linear.z = dz
        twist.angular.x = d_roll
        twist.angular.y = d_pitch
        twist.angular.z = d_yaw

        if self.__not_stamped:
            msg = twist
        else:
            msg = TwistStamped()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = self.__frame_id
            msg.twist = twist

        is_quiet = all(map(lambda val: val == 0, [
            dx, dy, dz, d_roll, d_pitch, d_yaw
        ]))
        return msg, is_quiet

    def __get_value__(self, axis, named_axes):
        """
        Extracts the axis value from joy
        :param axis: one of x, y, z, R, P, Y to get the value of
        :param named_axes: the processed joy values to get the value from
        :return: the value
        """
        if axis not in self.__mapping:
            return 0

        # List of button names to be added in order to get the value.
        # A name could start with '-', indicating to invert the value
        names = self.__mapping[axis]

        val = 0
        for name in names:
            v = named_axes[name.lstrip('-')].value
            if name.startswith('-'):
                v = -v
            val += v
        return val


Action.register_preset(Twist6DOF)

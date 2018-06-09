import rospy
from geometry_msgs.msg import TwistStamped

from .action import Action
from ..shared import Shared


class ArmTwist(Action):
    NAME = 'arm_twist'

    def __init__(self, definition):
        super(ArmTwist, self).__init__(definition)

        self.__frame_id, _ = self.get_key('frame_id', 'world')
        self.__scale_trn, _ = self.get_key('scale/translation', 0.1)
        self.__scale_rot, _ = self.get_key('scale/rotation', 0.01)
        self.__quiet_on_zero, _ = self.get_key('quiet_on_zero', True)
        self.__mapping = self.__mapping__()
        self.__published_zero = False

        is_default, _ = self.get_key('is_default_twist_mode', False)
        if is_default:
            Shared.add('twist_mode', 'arm')

        self.cmd_delta_pub = rospy.Publisher('cmd_delta',
                                             TwistStamped,
                                             queue_size=1)

    def execute(self, named_joy=None):
        if Shared.get('twist_mode') != 'arm':
            return

        self.publish_cmd_delta(named_joy)

    def publish_cmd_delta(self, named_joy):
        twist, is_quiet = self.__get_twist__(named_joy['axes'])

        if self.__quiet_on_zero:
            if is_quiet:
                # Publish the all-zero message just once
                if not self.__published_zero:
                    self.cmd_delta_pub.publish(twist)
                    self.__published_zero = True
                return

        self.cmd_delta_pub.publish(twist)
        self.__published_zero = False

    def __mapping__(self):
        params, found = self.get_key('mapping', dict())
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

        twist = TwistStamped()
        twist.header.stamp = rospy.Time.now()
        twist.header.frame_id = self.__frame_id
        twist.twist.linear.x = dx
        twist.twist.linear.y = dy
        twist.twist.linear.z = dz
        twist.twist.angular.x = d_roll
        twist.twist.angular.y = d_pitch
        twist.twist.angular.z = d_yaw

        is_quiet = all(map(lambda val: val == 0, [
            dx, dy, dz, d_roll, d_pitch, d_yaw
        ]))
        return twist, is_quiet

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


Action.register_preset(ArmTwist)

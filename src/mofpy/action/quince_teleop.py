import rospy
from geometry_msgs.msg import Twist
from flipper_msgs.msg import flipper_control

from .action import Action
from ..shared import Shared


class QuinceTeleop(Action):
    """
    :type __cmd_vel_topic: str
    :type __flipper_topic: str
    """

    NAME = 'quince_teleop'

    def __init__(self, definition):
        super(QuinceTeleop, self).__init__(definition)
        Action.actions[self.__class__.NAME] = self.__class__

        self.__cmd_vel_topic = self.get_required('cmd_vel_topic')
        self.__flipper_topic = self.get_required('flipper_topic')
        self.__mapping = self.__read_mappings__()
        self.__frame_id = self.get('frame_id', 'base_link')
        self.__flipper_low = self.get('flipper/limits/low', -0.5)
        self.__flipper_high = self.get('flipper/limits/high', 0.5)
        self.__flipper_step = self.get('flipper/step', 0.005)

        if not Shared.has('__flipper_command__'):
            Shared.add('__flipper_command__', flipper_control())

        self.__pub_cmd_vel = rospy.Publisher(self.__cmd_vel_topic,
                                             Twist,
                                             queue_size=1)
        self.__pub_flipper = rospy.Publisher(self.__flipper_topic,
                                             flipper_control,
                                             queue_size=1)

    def execute(self, named_joy=None):
        self.pub_cmd_vel(named_joy['axes'])
        self.pub_flipper(named_joy['buttons'])

    def pub_cmd_vel(self, named_axes):
        mode = Shared.get('cmd_vel_mode')
        lsv = named_axes[self.__mapping['LSV']].value
        lsh = named_axes[self.__mapping['LSH']].value
        rsv = named_axes[self.__mapping['RSV']].value
        rsh = named_axes[self.__mapping['RSH']].value

        if mode == 'beginner':
            # Left vertical: velocity in X, left horizontal: ang vel in yaw
            # Note: Left is 1.0 and right is -1.0 in horizontal
            v, w = lsv, -lsh
        elif mode == 'tank':
            v = float(lsv + rsv) / 2
            w = float(lsv - rsv) / 2
        else:  # mode == 'normal'
            # Left vertical: velocity in X, right horizontal: ang vel in yaw
            v, w = lsv, -rsh

        # Apply scaling
        v *= float(Shared.get('scale_translation'))
        w *= float(Shared.get('scale_rotation'))

        # Front-side-back
        if Shared.get('front_direction') == 'inverted':
            v *= -1

        cmd_vel = Twist()
        cmd_vel.linear.x = v
        cmd_vel.angular.z = w
        self.__pub_cmd_vel.publish(cmd_vel)

    def pub_flipper(self, buttons):
        mode = Shared.get('flipper_control_mode')
        # Variables front, rear, fl, fr, rl, rr are numbers in range [-1, 1]
        if mode == 'synchronized':
            s = self.__mapping['sync']
            f_raise = buttons[s['front']['raise']].value
            f_lower = buttons[s['front']['lower']].value
            r_raise = buttons[s['rear']['raise']].value
            r_lower = buttons[s['rear']['lower']].value

            front = f_raise - f_lower
            rear = r_raise - r_lower

            dfl = front
            dfr = front
            drl = rear
            drr = rear

        else:  # mode == 'independent'
            is_raise = buttons[self.__mapping['indep']['raise']].value
            is_lower = buttons[self.__mapping['indep']['lower']].value
            if is_raise:
                direction = 1
            elif is_lower:
                direction = -1
            else:
                direction = 0
            dfl = 1 if buttons[self.__mapping['indep']['fl']].value else 0
            dfl = dfl * direction
            dfr = 1 if buttons[self.__mapping['indep']['fr']].value else 0
            dfr = dfr * direction
            drl = 1 if buttons[self.__mapping['indep']['rl']].value else 0
            drl = drl * direction
            drr = 1 if buttons[self.__mapping['indep']['rr']].value else 0
            drr = drr * direction

        # Quiet input
        if dfl == dfr == drl == drr == 0:
            return

        # When inverted, fl -> rr, fr -> rl
        if Shared.get('front_direction') == 'inverted':
            dfl, drr = drr, dfl
            dfr, drl = drl, dfr

        msg = Shared.get('__flipper_command__')
        msg.fl += dfl * self.__flipper_step
        msg.fr += dfr * self.__flipper_step
        msg.rl += drl * self.__flipper_step
        msg.rr += drr * self.__flipper_step

        msg = QuinceTeleop.__clamp_flipper__(msg,
                                             self.__flipper_low,
                                             self.__flipper_high)
        self.__pub_flipper.publish(msg)
        Shared.add('__flipper_command__', msg)

    def __read_mappings__(self):
        return {
            'LSV': self.get_required('mapping/sticks/left/vertical'),
            'LSH': self.get_required('mapping/sticks/left/horizontal'),
            'RSV': self.get_required('mapping/sticks/right/vertical'),
            'RSH': self.get_required('mapping/sticks/right/horizontal'),
            'indep': self.get_required('mapping/independent'),
            'sync': self.get_required('mapping/synchronized')
        }

    @staticmethod
    def __clamp_flipper__(flipper, lo, hi):
        flipper.fl = max(lo, min(hi, flipper.fl))
        flipper.fr = max(lo, min(hi, flipper.fr))
        flipper.rl = max(lo, min(hi, flipper.rl))
        flipper.rr = max(lo, min(hi, flipper.rr))

        return flipper


Action.register_preset(QuinceTeleop)

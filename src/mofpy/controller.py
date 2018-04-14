import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64
from geometry_msgs.msg import TwistStamped

from .joy_mapping import JoyMapping
from .preset_handler import PresetHandler


class Controller:
    def __init__(self):
        self.__frame_id = rospy.get_param('~frame_id', 'world')
        self.__scale_trn = rospy.get_param('~scale/translation', 0.1)
        self.__scale_rot = rospy.get_param('~scale/rotation', 0.01)
        self.__scale_hand = rospy.get_param('~scale/hand', 0.1)
        self.__hand_topic = rospy.get_param('~hand_topic', 'hand')
        self.__quiet_on_zero = rospy.get_param('~quiet_on_zero', True)
        self.__rate = rospy.get_param('~rate', 20)
        self.__mapping = Controller.__mapping__()
        self.__received_first_msg = False
        self.__last_msg = Joy()
        self.__published_zero = False
        # Only handle preset poses if registered
        self.__preset_handler = None

        # Presets
        if rospy.has_param('~presets'):
            move_group_name = rospy.get_param('~presets/move_group_name')
            mappings = rospy.get_param('~presets/mappings')
            self.__preset_handler = PresetHandler(move_group_name)
            self.__preset_handler.register_dict_callbacks(mappings)

        self.cmd_delta_pub = rospy.Publisher('cmd_delta',
                                             TwistStamped,
                                             queue_size=1)
        self.cmd_delta_hand_pub = rospy.Publisher(self.__hand_topic,
                                                  Float64,
                                                  queue_size=1)
        self.joy_sub = rospy.Subscriber('joy',
                                        Joy,
                                        self.cb_joy,
                                        queue_size=1)

    def cb_joy(self, msg):
        self.__last_msg = msg
        self.__received_first_msg = True
        if self.__preset_handler is not None:
            self.__preset_handler.handle(msg)

    def publish_cmd_delta(self, msg):
        if not self.__received_first_msg:
            return

        twist, is_quiet = self.__get_twist__(msg)

        # Don't stop on release
        d_hand = self.__scale_hand * self.__get_value__('hand', msg)
        if d_hand != 0:
            d_hand_msg = Float64()
            d_hand_msg.data = d_hand
            self.cmd_delta_hand_pub.publish(d_hand_msg)

        if self.__quiet_on_zero:
            if is_quiet:
                # Publish the all-zero message just once
                if not self.__published_zero:
                    self.cmd_delta_pub.publish(twist)
                    self.__published_zero = True
                return

        self.cmd_delta_pub.publish(twist)
        self.__published_zero = False

    def spin(self):
        rate = rospy.Rate(self.__rate)
        while not rospy.is_shutdown():
            msg = self.__last_msg
            msg.header.stamp = rospy.Time.now()
            self.publish_cmd_delta(msg)
            rate.sleep()

    @staticmethod
    def __mapping__():
        params = rospy.get_param('~mapping', dict())
        mapping = {}
        for key in params.keys():
            param_name = '~mapping/' + key
            params[key] = rospy.get_param(param_name)
            # Convert to JoyMapping objects
            if type(params[key]) is tuple or type(params[key]) is list:
                mapping[key] = [JoyMapping(params[key][0]),
                                JoyMapping(params[key][1])]
            else:
                mapping[key] = JoyMapping(params[key])

        return mapping

    def __get_twist__(self, joy_msg):
        dx = self.__scale_trn * self.__get_value__('x', joy_msg)
        dy = self.__scale_trn * self.__get_value__('y', joy_msg)
        dz = self.__scale_trn * self.__get_value__('z', joy_msg)
        d_roll = self.__scale_rot * self.__get_value__('roll', joy_msg)
        d_pitch = self.__scale_rot * self.__get_value__('pitch', joy_msg)
        d_yaw = self.__scale_rot * self.__get_value__('yaw', joy_msg)

        twist = TwistStamped()
        twist.header = joy_msg.header
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

    def __get_value__(self, key, msg):
        if key not in self.__mapping:
            return 0

        mapping = self.__mapping[key]

        if type(mapping) is tuple or type(mapping) is list:
            val = mapping[0].get_value(msg) - mapping[1].get_value(msg)
        else:
            val = mapping.get_value(msg)
        return val

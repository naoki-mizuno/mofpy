#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64
from geometry_msgs.msg import TwistStamped

from .joy_mapping import JoyMapping


class Controller:
    def __init__(self):
        self.__frame_id = rospy.get_param('~frame_id', 'world')
        self.__scale_trn = rospy.get_param('~scale/translation', 0.1)
        self.__scale_rot = rospy.get_param('~scale/rotation', 0.01)
        self.__scale_hand = rospy.get_param('~scale/hand', 0.1)
        self.__throttle_ms = rospy.get_param('~throttle_ms', 50)
        self.__hand_topic = rospy.get_param('~hand_topic', 'hand')
        self.__quiet_on_zero = rospy.get_param('~quiet_on_zero', True)
        self.__mapping = Controller.__mapping__()
        self.__last_msg = Joy()
        self.__published_zero = False

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
        curr_msg = msg.header.stamp.to_sec()
        last_msg = self.__last_msg.header.stamp.to_sec()
        since_last_msg = curr_msg - last_msg
        if since_last_msg < self.__throttle_ms / 1000.0:
            return
        self.__last_msg = msg

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

    @staticmethod
    def __mapping__():
        # Default mapping
        mapping_str = {
            # Left Stick Up/Down
            'x': 'a1',
            # Left Stick Left/Right
            'y': 'a0',
            # R2 for plus and L2 for minus (active-low w/ button counterparts)
            'z': ('a4al7', 'a3al6'),
            # Right Stick Left/Right
            'roll': '-a2',
            # Right Stick Up/Down
            'pitch': 'a5',
            # R1 for plus (CCW) and L1 for minus (CW)
            'yaw': ('b4', 'b5'),
            # Hand open/close
            'hand': ('b1', 'b2')
        }

        mapping = {}
        for key in mapping_str.keys():
            param_name = '~mapping/' + key
            mapping_str[key] = rospy.get_param(param_name, mapping_str[key])
            # Convert to JoyMapping objects
            if type(mapping_str[key]) is tuple or \
                    type(mapping_str[key]) is list:
                mapping[key] = [JoyMapping(mapping_str[key][0]),
                                JoyMapping(mapping_str[key][1])]
            else:
                mapping[key] = JoyMapping(mapping_str[key])

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
        mapping = self.__mapping[key]

        if type(mapping) is tuple or type(mapping) is list:
            val = mapping[0].get_value(msg) - mapping[1].get_value(msg)
        else:
            val = mapping.get_value(msg)
        return val

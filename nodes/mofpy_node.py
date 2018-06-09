#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy

from mofpy.named_mapping import NamedMappings
from mofpy.preset_handler import PresetHandler

rospy.init_node('mofpy_node')

if rospy.has_param('~presets'):
    definitions = rospy.get_param('~presets')
    joy_names = NamedMappings()
    handler = PresetHandler(joy_names)
    handler.bind_presets(definitions)

    joy_sub = rospy.Subscriber('joy',
                               Joy,
                               handler.handle,
                               queue_size=1)
    rospy.spin()
else:
    rospy.logwarn('No presets found. Why bother spinning?')

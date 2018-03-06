#!/usr/bin/env python

import rospy

from mofpy import Controller

rospy.init_node('mofpy_node')

controller = Controller()

rospy.spin()

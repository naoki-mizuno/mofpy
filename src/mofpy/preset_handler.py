import rospy
from moveit_commander import roscpp_initializer
from moveit_commander.robot import RobotCommander
from moveit_commander.move_group import MoveGroupCommander
from moveit_commander.exception import MoveItCommanderException

from .joy_event_callback import JoyEventCallback
from .recovery_action import *

import sys


class PresetHandler:
    """
    :type __callbacks: list[JoyEventCallback]
    :type __recovery_actions: list[RecoveryAction]
    """
    def __init__(self, move_group_name):
        self.__callbacks = []

        roscpp_initializer.roscpp_initialize(sys.argv)
        self.__robot = RobotCommander()
        # Sometimes, MoveGroupCommander fails to initialize when Rviz
        # startup is too slow. Try several times and then give up.
        attempt = 0
        initialized_commander = False
        while not initialized_commander:
            if attempt > 3:
                rospy.logwarn('Could not connect to MoveGroupCommander. '
                              'Disabling presets')
                return
            try:
                self.__group = MoveGroupCommander(move_group_name)
                initialized_commander = True
            except RuntimeError as e:
                rospy.logwarn(e)
                rospy.logwarn('Trying again to initialize MoveGroupCommander')
                attempt += 1

        self.__recovery_actions = PresetHandler.read_recovery_params(
            self.__robot, self.__group
        )

    def register_callback(self, callback):
        """
        Registers a callback when a button event is triggered

        :param callback:
        :type callback: JoyEventCallback
        """
        self.__callbacks.append(callback)

    def remove_callback(self, callback):
        self.__callbacks.remove(callback)

    def handle(self, msg):
        """
        Handles a Joy message
        :param msg: Joy message
        :type msg: Joy
        """
        for callback in self.__callbacks:
            if callback.check(msg):
                callback.call()

    def move_to(self, preset_name):
        rospy.loginfo('Moving to {0}'.format(preset_name))
        self.__group.set_start_state_to_current_state()
        try:
            self.__group.set_named_target(preset_name)
        except MoveItCommanderException as e:
            rospy.logerr(e)
            return

        self.__group.go(wait=True)

    def remember_current_pose(self, preset_name):
        rospy.loginfo('Remembered current pose as {0}'.format(preset_name))
        self.__group.remember_joint_values(preset_name)

    def execute_recovery(self):
        rospy.loginfo('Executing recovery behavior')
        for action in self.__recovery_actions:
            action.execute()

    @staticmethod
    def read_recovery_params(robot, group):
        if not rospy.has_param('~recovery'):
            return []

        actions = []
        definitions = rospy.get_param('~recovery')
        for definition in definitions:
            # TODO: trajectory, command
            if 'type' not in definition:
                rospy.logerr('Required key "type" not set')
                continue
            action_type = definition['type']
            if action_type == 'single_point_trajectory':
                actions.append(SinglePointTrajectory(definition, group))
            elif action_type == 'sleep':
                actions.append(Sleep(definition))
            elif action_type == 'move_group_state':
                actions.append(MoveGroupState(definition, robot, group))
            else:
                msg = 'Action type {0} not implemented'.format(action_type)
                rospy.logerr(msg)
        return actions

    def register_dict_callbacks(self, mapping):
        """
        Converts and registers a dictionary describing callbacks

        The dictionary is typically obtained from the ROS param server,
        and has the name as the state name and the buttons as values.

        Registered keywords:
          recover: execute recovery behavior
          custom*: remember current joint positions by the given name when
                   long-pressing for 3 seconds and recall by triple-pressing
        :param mapping:
        :type mapping: dict[str, int]
        """
        for preset_name in mapping.keys():
            button = mapping[preset_name]
            if preset_name == 'recovery':
                cb = JoyEventCallback(button,
                                      JoyEventCallback.LONG_PRESS,
                                      self.execute_recovery,
                                      press_duration=3)
                self.register_callback(cb)
            elif preset_name.startswith('custom'):
                # Remember
                cb = JoyEventCallback(button,
                                      JoyEventCallback.LONG_PRESS,
                                      self.remember_current_pose,
                                      (preset_name,),
                                      press_duration=3)
                self.register_callback(cb)
                # Recall
                cb = JoyEventCallback(button,
                                      JoyEventCallback.TRIPLE_PRESS,
                                      self.move_to,
                                      (preset_name,))
                self.register_callback(cb)
            else:
                cb = JoyEventCallback(button,
                                      JoyEventCallback.TRIPLE_PRESS,
                                      self.move_to,
                                      (preset_name,))
                self.register_callback(cb)

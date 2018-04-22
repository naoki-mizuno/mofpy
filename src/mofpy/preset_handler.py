import rospy
from moveit_commander import roscpp_initializer
from moveit_commander.robot import RobotCommander
from moveit_commander.move_group import MoveGroupCommander

from .joy_event_callback import JoyEventCallback
from .preset_action import *

import sys


class PresetHandler:
    """
    :type __callbacks: list[JoyEventCallback]
    :type __presets: dict[str, list[PresetTask]]
    """
    def __init__(self):
        self.__callbacks = []

        roscpp_initializer.roscpp_initialize(sys.argv)
        move_group_name = rospy.get_param('~presets/move_group_name')
        self.__robot, self.__group = PresetHandler.__connect_to_move_group__(
            move_group_name
        )

        # MoveGroupCommander has failed to initialize
        if self.__group is None:
            return

        self.__presets = PresetHandler.read_presets(
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

    def execute(self, preset_name):
        if preset_name not in self.__presets:
            rospy.logerr('{0} not found in presets'.format(preset_name))
            return

        for preset in self.__presets[preset_name]:
            preset.execute()

    def register_mappings(self, mappings):
        """
        :type mappings: dict[str, dict[str, int | str]]
        """
        for preset_name in mappings.keys():
            button = mappings[preset_name]['index']
            event = mappings[preset_name]['event']
            if event == JoyEventCallback.LONG_PRESS:
                duration = mappings[preset_name]['duration']
            else:
                duration = None

            cb = JoyEventCallback(button=button,
                                  event=event,
                                  callback=self.execute,
                                  callback_args=(preset_name,),
                                  press_duration=duration)
            self.register_callback(cb)

    @staticmethod
    def __connect_to_move_group__(move_group_name):
        robot = RobotCommander()

        # Sometimes, MoveGroupCommander fails to initialize when Rviz
        # startup is too slow. Try several times and then give up.
        attempt = 0
        while attempt <= 3:
            try:
                group = MoveGroupCommander(move_group_name)
                return robot, group
            except RuntimeError as e:
                rospy.logwarn(e)
                rospy.logwarn('Trying again to initialize MoveGroupCommander')
                attempt += 1

        msg = 'Could not connect to MoveGroupCommander. Disabling presets'
        rospy.logwarn(msg)
        return robot, None

    @staticmethod
    def read_presets(robot, group):
        preset_definitions = rospy.get_param('~presets', dict())
        preset_actions = dict()
        for preset_name in preset_definitions.keys():
            if preset_name == 'move_group_name':
                continue

            preset = PresetHandler.__read_preset__(preset_name,
                                                   robot,
                                                   group)
            preset_actions[preset_name] = preset
        return preset_actions

    @staticmethod
    def __read_preset__(preset_name, robot, group):
        tasks = []
        preset = rospy.get_param('~presets/{0}'.format(preset_name))
        for task in preset:
            # TODO: trajectory, command
            if 'type' not in task:
                rospy.logerr('Required key "type" not set')
                continue
            task_type = task['type']
            if task_type == 'single_point_trajectory':
                tasks.append(SinglePointTrajectory(task, group))
            elif task_type == 'sleep':
                tasks.append(Sleep(task))
            elif task_type == 'move_group_state':
                tasks.append(MoveGroupState(task, robot, group))
            elif task_type == 'remember_move_group_state':
                tasks.append(RememberMoveGroupState(task, robot, group))
            elif task_type == 'publish_float64':
                tasks.append(PublishFloat64(task))
            else:
                msg = 'Task type {0} not implemented'.format(task_type)
                rospy.logerr(msg)
        return tasks

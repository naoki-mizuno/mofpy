import rospy

from .action import *
from .event_manager import EventManager
from .named_mapping import NamedMappings
from .move_group_utils import MoveGroupUtils
from .shared import Shared

from threading import Thread, Lock


class PresetHandler:
    """
    :type __joy_namer: NamedMappings
    :type __presets: dict[str, list[Action]]
    """
    def __init__(self, joy_names):
        self.__joy_namer = joy_names
        self.__callbacks = []
        self.__named_joy = None
        self.__named_joy_lock = Lock()

        self.__presets = dict()
        self.__check_rate = rospy.Rate(rospy.get_param('~check_rate', 100))
        timeout_press = rospy.get_param('~timeout/press', 0.05)
        timeout_sequence = rospy.get_param('~timeout/sequence', 0.20)
        self.__event_manager = EventManager(timeout_press, timeout_sequence)
        move_group_name = rospy.get_param('~move_group_name', None)
        connected = MoveGroupUtils.connect(move_group_name)
        Shared.add('move_group_disabled', not connected)

        self.__trigger_monitor = Thread(target=self.__trigger_spin__)
        self.__trigger_monitor.start()

    def bind_presets(self, definition):
        """
        Binds the triggers (input commands) to the preset to be executed
        :param definition:
        :return:
        """
        for preset_name in definition.keys():
            trigger, preset = PresetHandler.__read_preset__(preset_name)
            self.__presets[preset_name] = preset

            trigger = definition[preset_name]['trigger']
            self.__event_manager.register(preset_name, trigger)

    def handle(self, msg):
        """
        Handles a Joy message
        :param msg: Joy message
        :type msg: Joy
        """
        named_joy = self.__joy_namer.convert(msg)
        self.__event_manager.append(named_joy['buttons'])

        self.__named_joy_lock.acquire(True)
        self.__named_joy = named_joy
        self.__named_joy_lock.release()

    def __trigger_spin__(self):
        while not rospy.is_shutdown():
            triggered_presets = self.__event_manager.get_triggered()

            # TODO: Remove this part! Only for debugging
            if len(triggered_presets) != 0:
                print(triggered_presets)

            # Presets that are triggered by commands
            for preset_name in triggered_presets:
                if preset_name not in self.__presets:
                    rospy.logerr(preset_name + ' not found in presets')
                    return

                # Execute each action
                for preset_action in self.__presets[preset_name]:
                    preset_action.execute()

            # Presets that are always triggered
            self.__named_joy_lock.acquire(True)
            for preset_name in self.__event_manager.get_any_triggered():
                if preset_name not in self.__presets:
                    rospy.logerr(preset_name + ' not found in presets')
                    return
                for preset_action in self.__presets[preset_name]:
                    if self.__named_joy is not None:
                        preset_action.execute(self.__named_joy)
            # Don't trigger the 'any' presets until a new message is received
            self.__named_joy = None
            self.__named_joy_lock.release()

            self.__check_rate.sleep()

    @staticmethod
    def __read_preset__(preset_name):
        actions = []
        preset = rospy.get_param('~presets/{0}'.format(preset_name))
        trigger = preset['trigger']
        for action_definition in preset['action']:
            if 'type' not in action_definition:
                rospy.logerr('Required key "type" not set')
                continue
            action_type = action_definition['type']
            cls = Action.actions[action_type]
            if action_type not in Action.actions:
                msg = 'Action type {0} not implemented'.format(action_type)
                rospy.logerr(msg)
            else:
                actions.append(cls(action_definition))

        return trigger, actions

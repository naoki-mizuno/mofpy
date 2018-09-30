import rospy

from .joy_mapping import JoyMapping

import copy
import six
from threading import Lock, Thread


class EventManager:
    """
    :type __def_seq: dict[frozenset[str], list[str]
    :type __def_long: dict[frozenset[str], dict[str, str | rospy.Time]]
    :type __pressed: set[str]
    :type __long_pressed: set[str]
    :type __pressed_mutex: Lock
    :type __sequence: list[frozenset[str]]
    :type __press_stamp: rospy.Time
    :type __sequence_stamp: rospy.Time
    :type __triggered_presets: list[str]
    :type __prev_named_buttons: dict[str, JoyMapping]
    """
    RESET = rospy.Time(0)

    def __init__(self, timeout_press, timeout_sequence):
        self.__def_seq = dict()
        self.__def_long = dict()

        self.__pressed = set()
        self.__long_pressed = set()
        self.__pressed_mutex = Lock()
        # Ongoing sequence
        self.__sequence = []

        # Timeout stamp for single button press
        self.__press_stamp = EventManager.RESET
        # Timeout stamp for sequence completion
        self.__sequence_stamp = EventManager.RESET

        # After how long do we consider it a timeout
        self.TIMEOUT_PRESS = rospy.Duration.from_sec(timeout_press)
        self.TIMEOUT_SEQUENCE = rospy.Duration.from_sec(timeout_sequence)

        # Actual presets that were triggered by the sequence
        self.__triggered_presets = []
        # Presets that are triggered every time
        self.__triggered_always = []

        # Used to determine key down events
        self.__prev_named_buttons = dict()

        self.__timer_thread = Thread(target=self.__event_spinner__)
        self.__timer_thread.start()

    def register(self, preset_name, sequence):
        """
        Registers a present name with the trigger (command sequence)

        For example, the trigger ('X', ('O', 'X'), 'O') means a sequence of
        'X', followed by a simultaneous press of 'O' and 'X', followed by 'O'.
        Another example (('X',),) means a single press of 'X'. There can only
        be one long press for a trigger.
        :param preset_name:
        :param sequence:
        :return:
        """
        if sequence == 'always':
            self.__triggered_always.append(preset_name)
        elif isinstance(sequence, six.string_types):
            # Single press
            #   foo: 'X'
            sequence = (frozenset((sequence,)),)
        elif EventManager.__is_long_press__(sequence):
            # Long press
            #   foo: ['X', 3]
            trigger, duration = sequence
            if isinstance(trigger, six.string_types):
                trigger = frozenset((trigger,))
            else:
                trigger = frozenset(trigger)
            self.__def_long[trigger] = {
                'preset_name': preset_name,
                'duration': rospy.Duration.from_sec(duration),
                'timeout': EventManager.RESET,
            }
            return
        else:
            tmp_sequence = []
            for t in sequence:
                if isinstance(t, six.string_types):
                    # Simple sequence
                    #   foo: ['X', 'O']
                    tmp_sequence.append(frozenset((t,)))
                else:
                    # Sequence of simultaneous buttons
                    #   foo: ['X', ['X', 'O']]
                    tmp_sequence.append(frozenset(t))
            sequence = tuple(tmp_sequence)

        if sequence not in self.__def_seq:
            self.__def_seq[sequence] = []

        self.__def_seq[sequence].append(preset_name)

    def append(self, named_buttons):
        """
        :param named_buttons:
        :type named_buttons: dict[str, JoyMapping]
        :return:
        """
        key_up, key_down = self.__get_changed_buttons__(named_buttons)

        now = rospy.Time.now()

        self.__pressed_mutex.acquire(True)
        for name in key_down:
            if self.__press_stamp == EventManager.RESET:
                self.__press_stamp = now + self.TIMEOUT_PRESS
            self.__pressed.add(name)
            self.__sequence_stamp = EventManager.RESET
            self.__long_pressed.add(name)

        for name in key_up:
            # Immediately append to sequence
            self.__press_stamp = now
            self.__sequence_stamp = now + self.TIMEOUT_SEQUENCE
            if name in self.__long_pressed:
                self.__long_pressed.remove(name)
        self.__pressed_mutex.release()

    def __get_changed_buttons__(self, named_buttons):
        """
        Detects the key up and key down
        :param named_buttons: Current input from the controller
        :type named_buttons: dict[str, JoyMapping]
        :return: Tuple of button names that changed
        """
        key_down = []
        key_up = []
        for name in named_buttons.keys():
            curr = named_buttons[name]
            if name not in self.__prev_named_buttons:
                self.__prev_named_buttons[name] = copy.deepcopy(curr)
                continue
            prev = self.__prev_named_buttons[name]

            if prev.value != curr.value:
                # Falling edge, key down (high -> low)
                if prev.value and not curr.value:
                    key_up.append(name)
                # Rising edge, key up (low -> high)
                if not prev.value and curr.value:
                    key_down.append(name)

                self.__prev_named_buttons[name] = copy.deepcopy(curr)
        return key_up, key_down

    def get_sequence_triggered(self):
        """
        Gets the presets that are triggered by the currently-input sequence

        :return: names of the presets that are triggered
        """
        if len(self.__triggered_presets) == 0:
            return []
        triggered = copy.deepcopy(self.__triggered_presets)

        self.__triggered_presets = []
        return triggered

    def get_always_triggered(self):
        return self.__triggered_always

    def __event_spinner__(self):
        while not rospy.is_shutdown():
            rospy.sleep(0.001)

            now = rospy.Time.now()

            # Short Press timed out
            if self.__press_stamp != EventManager.RESET and \
                    now >= self.__press_stamp:
                pressed_keys = frozenset(self.__pressed)
                if len(pressed_keys) != 0:
                    self.__sequence.append(pressed_keys)

                self.__pressed_mutex.acquire(True)
                self.__pressed.clear()
                self.__press_stamp = EventManager.RESET
                self.__pressed_mutex.release()

            # Sequence timed out
            if self.__sequence_stamp != EventManager.RESET and \
                    now >= self.__sequence_stamp:
                cmd = tuple(self.__sequence)
                if cmd in self.__def_seq:
                    self.__triggered_presets = self.__def_seq[cmd]

                self.__pressed_mutex.acquire(True)
                self.__sequence = []
                self.__sequence_stamp = EventManager.RESET
                self.__pressed_mutex.release()

            # Long press
            long_pressed = frozenset(self.__long_pressed)
            if long_pressed in self.__def_long:
                timeout = self.__def_long[long_pressed]['timeout']
                # Started to long press
                if timeout == EventManager.RESET:
                    duration = self.__def_long[long_pressed]['duration']
                    timeout = now + duration
                    self.__def_long[long_pressed]['timeout'] = timeout
                # Finished long press
                elif now >= timeout:
                    preset_name = self.__def_long[long_pressed]['preset_name']
                    self.__triggered_presets.append(preset_name)

                    self.__pressed_mutex.acquire(True)
                    self.__long_pressed.clear()
                    self.__pressed_mutex.release()
                    # We also want to clear any sequence that were created
                    self.__sequence = []
            else:
                for key in self.__def_long:
                    self.__def_long[key]['timeout'] = EventManager.RESET

    @staticmethod
    def __is_long_press__(sequence):
        if len(sequence) != 2:
            return False
        try:
            float(sequence[1])
        except ValueError:
            return False
        except TypeError:
            return False
        return True

from sensor_msgs.msg import Joy


class JoyEventCallback:
    BUTTON_DOWN = 'button_down'
    BUTTON_UP = 'button_up'
    DOUBLE_PRESS = 'double_press'
    TRIPLE_PRESS = 'triple_press'
    LONG_PRESS = 'long_press'

    def __init__(self,
                 button,
                 event,
                 callback,
                 callback_args=None,
                 press_duration=None):
        """
        :param button: index of the button
        :param event: name of the event
        :param callback: callback function to be called
        :param callback_args: arguments passed to the callback function
        :param press_duration: duration of the press for a long press
        """
        if type(button) is int:
            self.__button = button
        elif button.startswith('b'):
            self.__button = int(button[1:])
        else:
            raise ValueError('Invalid button {0}'.format(button))
        self.__event = event
        self.__func = callback
        self.__args = callback_args
        self.__press_duration = press_duration
        if event == JoyEventCallback.LONG_PRESS and press_duration is None:
            raise ValueError('Missing required argument press_duration')

        # Simple finite state machine
        self.__state = JoyEventCallback.BUTTON_UP
        self.__last_press_time = None
        self.__press_count = 0
        self.__press_start_time = None

    def call(self):
        if self.__args is None:
            self.__func()
        else:
            self.__func(*self.__args)

    def check(self, msg):
        """
        Checks if this event should be triggered given a Joy message
        :param msg: Joy message
        :type msg: Joy
        :return: True if the callback function should be called
        """
        res = False
        next_state = self.__state

        # Pressed
        if self.__state == JoyEventCallback.BUTTON_UP:
            if self.__is_down__(msg):
                self.__press_start_time = msg.header.stamp
                next_state = JoyEventCallback.BUTTON_DOWN
                res = self.__event == JoyEventCallback.BUTTON_DOWN
        # Released
        elif self.__state == JoyEventCallback.BUTTON_DOWN:
            if self.__is_up__(msg):
                if self.__timed_out__(msg):
                    self.__press_count = 0

                self.__press_count += 1

                self.__last_press_time = msg.header.stamp

                if self.__press_count == 3:
                    res = self.__event == JoyEventCallback.TRIPLE_PRESS
                    next_state = JoyEventCallback.BUTTON_UP
                    # Note: Remove when doing quadruple press and more
                    self.__press_count = 0
                elif self.__press_count == 2:
                    res = self.__event == JoyEventCallback.DOUBLE_PRESS
                    next_state = JoyEventCallback.BUTTON_UP
                else:
                    res = self.__event == JoyEventCallback.BUTTON_UP
                    next_state = JoyEventCallback.BUTTON_UP
            elif self.__is_long_pressed__(msg):
                res = self.__event == JoyEventCallback.LONG_PRESS
                next_state = JoyEventCallback.LONG_PRESS
        # Not released after a long press
        elif self.__state == JoyEventCallback.LONG_PRESS:
            if self.__is_up__(msg):
                next_state = JoyEventCallback.BUTTON_UP
                res = self.__event == JoyEventCallback.BUTTON_UP

        self.__state = next_state
        return res

    def __is_down__(self, msg):
        """
        :param msg:
        :type msg: Joy
        :return: True if this button is down
        """
        if self.__button >= len(msg.buttons):
            msg = 'Invalid index {0} specified'.format(self.__button)
            raise IndexError(msg)
        return msg.buttons[self.__button] == 1

    def __is_up__(self, msg):
        return not self.__is_down__(msg)

    def __timed_out__(self, msg):
        """
        Check if a button press has timed out

        TODO: Parameterize timeout?
        :param msg:
        :type msg: Joy
        :return: True if too much time has passed since last press
        """
        if self.__last_press_time is None:
            return True
        # Note: Ideally we want to use msg.header.stamp but time stamp is not
        # updated when using autorepeat_rate
        return (rospy.Time.now() - self.__last_press_time).to_sec() > 0.25

    def __is_long_pressed__(self, msg):
        if self.__press_duration is None or self.__press_start_time is None:
            return False

        # Note: Ideally we want to use msg.header.stamp but time stamp is not
        # updated when using autorepeat_rate
        pressed_duration = rospy.Time.now() - self.__press_start_time
        return pressed_duration.to_sec() > self.__press_duration

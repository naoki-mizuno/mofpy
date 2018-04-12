import re


class JoyMapping:
    AXIS = 0
    BUTTON = 1

    def __init__(self, identifier):
        self.identifier = identifier

        pat = re.compile(
            '(?P<inverse>-)?'
            '(?P<input_type>[ab])'
            '(?P<index>\d+)'
            '(?P<al>al(?P<button>\d+)?)?'
        )
        m = re.search(pat, self.identifier)

        groups = m.groupdict()
        self.inverse = groups['inverse'] is not None
        if groups['input_type'] == 'a':
            self.input_type = JoyMapping.AXIS
        elif groups['input_type'] == 'b':
            self.input_type = JoyMapping.BUTTON
        else:
            raise ValueError('Input type must be "a" or "b"')
        self.index = int(groups['index'])
        self.is_active_low = groups['al'] is not None
        if groups['button'] is not None:
            self.__button_counterpart = int(groups['button'])
        else:
            self.__button_counterpart = None

    def get_value(self, msg):
        if self.input_type == JoyMapping.AXIS:
            arr = msg.axes
        elif self.input_type == JoyMapping.BUTTON:
            arr = msg.buttons
        else:
            raise ValueError('Invalid input type')

        if self.is_active_low:
            if self.__not_pressed_yet__(msg):
                return 0

            # 1.0 when not pressed, -1.0 when fully pressed
            val = (-arr[self.index] + 1) * 0.5
        else:
            val = arr[self.index]
        mult = -1 if self.inverse else 1
        return mult * val

    def __not_pressed_yet__(self, msg):
        """
        There seems to be a bug in the joy node where unpressed active-low
        axes (in most cases, L2 and R2 shoulder pads) have values of 0
        (active-low axes should have values of 1.0 when unpressed and -1.0
        when pressed). As a workaround to determine whether an active-low axis
        has been pressed, we look at the button counterpart and if both the
        axis value and the button value are 0, we consider the axis unpressed.
        :param msg:
        :return: True if the axis hasn't been pressed yet
        """
        if not self.is_active_low:
            return False
        if self.__button_counterpart is None:
            return False

        return msg.axes[self.index] == 0 and \
            msg.buttons[self.__button_counterpart] == 0

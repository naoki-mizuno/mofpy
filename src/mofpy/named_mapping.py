import rospy

from .joy_mapping import JoyMapping, VirtAxis, VirtButton


class NamedMappings:
    """
    :type __v_axes: dict[str, JoyMapping]
    :type __v_buttons: dict[str, JoyMapping]
    """
    def __init__(self):
        self.__v_axes = {}
        v_axes = rospy.get_param('~virtual/axes', dict())
        for name in v_axes.keys():
            k = '~virtual/axes/{0}'.format(name)
            definition = rospy.get_param(k)
            self.__v_axes[name] = VirtAxis(name, definition)

        self.__v_buttons = {}
        v_buttons = rospy.get_param('~virtual/buttons', dict())
        for name in v_buttons.keys():
            k = '~virtual/buttons/{0}'.format(name)
            definition = rospy.get_param(k)
            self.__v_buttons[name] = VirtButton(name, definition)

    def convert(self, msg):
        """
        Processes the raw Joy message and converts them into a named,
        normalized dictionary
        :param msg:
        :type msg: Joy
        :return:
        """
        new_axes = {}
        new_buttons = {}

        # Axes
        for k in self.__v_axes.keys():
            va = self.__v_axes[k]
            va.update_value(msg)
            new_axes[va.name] = va

        # Buttons
        for k in self.__v_buttons.keys():
            vb = self.__v_buttons[k]
            vb.update_value(msg)
            new_buttons[vb.name] = vb

        named_joy = {
            'axes': new_axes,
            'buttons': new_buttons,
        }
        return named_joy

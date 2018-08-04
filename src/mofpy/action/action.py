from abc import *


class Action(object):
    # Make sure to define the NAME in the derived class!
    NAME = ''
    actions = {}

    def __init__(self, definition):
        """
        Define a action

        :param definition: a dictionary containing the description of action
        """
        self.definition = definition

    @abstractmethod
    def execute(self, named_joy=None):
        """
        Method that is called when a preset is triggered.
        The behavior of the method should be defined in the derived class, and
        the required parameters should be retrieved in the __init__ method of
        the derived class.
        :param named_joy:
        """
        raise NotImplementedError()

    def get_required(self, key_name):
        if not self.has(key_name):
            msg = 'Missing required key "{0}"'.format(key_name)
            raise KeyError(msg)

        val = self.get(key_name)

        return val

    def get(self, key_name, default_val=None):
        """
        Get the value in self.definition from a slash-separated key
        :param key_name: slash-separated key name
        :param default_val: value to be used if key is not found
        :return: The value
        """
        definition = self.definition

        for key in key_name.split('/'):
            if key not in definition:
                return default_val
            definition = definition[key]

        return definition

    def has(self, key_name):
        definition = self.definition
        for key in key_name.split('/'):
            if key not in definition:
                return False
            definition = definition[key]
        return True

    @staticmethod
    def register_preset(cls):
        if cls.NAME == '':
            msg = 'NAME not defined for class {0}'.format(cls.__name__)
            raise ValueError(msg)
        Action.actions[cls.NAME] = cls

from abc import *


class RecoveryAction(object):
    def __init__(self, definition):
        """
        Define the recovery action

        :param definition: a dictionary containing the description of action
        """
        self.definition = definition

    @abstractmethod
    def execute(self):
        raise NotImplementedError()

    def get_required_key(self, key_name):
        val, found = self.get_key(key_name)

        if not found:
            msg = 'Missing required key "{0}"'.format(key_name)
            raise KeyError(msg)

        return val

    def get_key(self, key_name, default_val=None):
        """
        Get the value in self.definiton from a slash-separated key
        :param key_name: slash-separated key name
        :param default_val: value to be used if key is not found
        :return: The value and whether the key was found or not
        """
        definition = self.definition

        for key in key_name.split('/'):
            if key not in definition:
                return default_val, False
            definition = definition[key]

        return definition, True

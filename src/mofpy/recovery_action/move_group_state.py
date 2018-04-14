from moveit_commander.robot import RobotCommander
from moveit_commander.move_group import MoveGroupCommander

from .recovery_action import RecoveryAction


class MoveGroupState(RecoveryAction):
    """
    :type __robot: RobotCommander
    :type __group: MoveGroupCommander
    """
    def __init__(self, definition, robot, move_group):
        super().__init__(definition)

        self.__robot = robot
        self.__group = move_group
        self.__state_name = self.get_required_key('preset_name')

    def execute(self):
        self.__group.set_start_state_to_current_state()
        self.__group.set_named_target(self.__state_name)

        self.__group.go(wait=True)

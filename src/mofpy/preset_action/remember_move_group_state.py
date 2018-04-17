import rospy
from moveit_commander.robot import RobotCommander
from moveit_commander.move_group import MoveGroupCommander

from .preset_task import PresetTask


class RememberMoveGroupState(PresetTask):
    """
    :type __robot: RobotCommander
    :type __group: MoveGroupCommander
    """
    def __init__(self, definition, robot, move_group):
        super(RememberMoveGroupState, self).__init__(definition)

        self.__robot = robot
        self.__group = move_group
        self.__state_name = self.get_required_key('state_name')

    def execute(self):
        rospy.loginfo('Remembered current pose as ' + self.__state_name)
        self.__group.remember_joint_values(self.__state_name)

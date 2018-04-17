import rospy
from moveit_commander.robot import RobotCommander
from moveit_commander.move_group import MoveGroupCommander
from moveit_commander.exception import MoveItCommanderException

from .preset_task import PresetTask


class MoveGroupState(PresetTask):
    """
    :type __robot: RobotCommander
    :type __group: MoveGroupCommander
    """
    def __init__(self, definition, robot, move_group):
        super(MoveGroupState, self).__init__(definition)

        self.__robot = robot
        self.__group = move_group
        self.__state_name = self.get_required_key('state_name')

    def execute(self):
        rospy.loginfo('Moving to {0}'.format(self.__state_name))
        self.__group.set_start_state_to_current_state()
        try:
            self.__group.set_named_target(self.__state_name)
        except MoveItCommanderException as e:
            rospy.logerr(e)
            return

        self.__group.go(wait=True)

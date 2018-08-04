import rospy
from moveit_commander.exception import MoveItCommanderException

from .action import Action
from ..move_group_utils import MoveGroupUtils
from ..shared import Shared


class MoveGroupState(Action):
    """
    :type __robot: RobotCommander
    :type __group: MoveGroupCommander
    """

    NAME = 'move_group_state'

    def __init__(self, definition):
        super(MoveGroupState, self).__init__(definition)
        Action.actions[self.__class__.NAME] = self.__class__

        self.__robot = MoveGroupUtils.robot
        self.__group = MoveGroupUtils.group
        self.__state_name = self.get_required_key('state_name')
        self.__action, _ = self.get_key('action', 'move')

    def execute(self, named_joy=None):
        if Shared.get('move_group_disabled'):
            msg = 'move_group disabled; not executing: {0} {1}'.format(
                self.__action,
                self.__state_name
            )
            rospy.logerr(msg)
            return

        if self.__action == 'move':
            self.__move__()
        elif self.__action == 'remember':
            self.__remember__()

    def __move__(self):
        rospy.loginfo('Moving to {0}'.format(self.__state_name))
        self.__group.set_start_state_to_current_state()
        try:
            self.__group.set_named_target(self.__state_name)
        except MoveItCommanderException as e:
            rospy.logerr(e)
            return

        self.__group.go(wait=True)

    def __remember__(self):
        rospy.loginfo('Remembered current pose as ' + self.__state_name)
        self.__group.remember_joint_values(self.__state_name)


Action.register_preset(MoveGroupState)

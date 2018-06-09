import rospy
from moveit_commander.robot import RobotCommander
from moveit_commander.move_group import MoveGroupCommander


class MoveGroupUtils:
    robot = None
    group = None

    def __init__(self):
        pass

    @staticmethod
    def connect(move_group_name):
        if move_group_name is None:
            return False

        MoveGroupUtils.robot = RobotCommander()

        # Sometimes, MoveGroupCommander fails to initialize.
        # Try several times and then give up.
        attempt = 0
        while attempt <= 3:
            try:
                MoveGroupUtils.group = MoveGroupCommander(move_group_name)
                return True
            except RuntimeError as e:
                rospy.logwarn(e)
                rospy.logwarn('Trying again to initialize MoveGroupCommander')
                attempt += 1

        msg = 'Could not connect to MoveGroupCommander. Disabling presets'
        rospy.logwarn(msg)
        # MoveGroupCommander has failed to initialize
        return False


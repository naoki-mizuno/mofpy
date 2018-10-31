import rospy

from .action import Action
from ..joint_trajectory_publisher import JointTrajectoryPublisher
from ..shared import Shared


class SinglePointTrajectory(Action):
    """
    Publishes a JointTrajectory message to the specified topic

    The published JointTrajectory message contains a point, which is also
    specified in the parameters.

    :type __topic_name: str
    :type __time_from_start: float
    :type __frame_id: str
    :type __joints: dict
    """

    NAME = 'single_point_trajectory'

    def __init__(self, definition):
        super(SinglePointTrajectory, self).__init__(definition)

        self.__topic_name = self.get_required('topic')
        self.__time_from_start = self.get_required('execution_time')
        self.__frame_id = self.get('frame_id', 'world')
        self.__joints = self.get_required('joints')

        self.__jtpub = JointTrajectoryPublisher(self.__topic_name)

    def execute(self, named_joy=None):
        if Shared.get('move_group_disabled'):
            rospy.logerr('move_group disabled; not publishing trajectory')
            return

        joints = self.__jtpub.get_current_joint_values()
        for name in self.__joints.keys():
            ang = self.__joints[name]
            if type(ang) is int or type(ang) is float:
                joints[name] = ang
            elif ang.endswith('++'):
                joints[name] += float(ang[:-2])
            elif ang.endswith('--'):
                joints[name] -= float(ang[:-2])
            else:
                raise ValueError("Couldn't understand: {0}".format(ang))

        self.__jtpub.publish(joints, self.__frame_id, self.__time_from_start)


Action.register_preset(SinglePointTrajectory)

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .action import Action
from ..move_group_utils import MoveGroupUtils
from ..shared import Shared


class SinglePointTrajectory(Action):
    NAME = 'single_point_trajectory'

    """
    Publishes a JointTrajectory message to the specified topic

    The published JointTrajectory message contains a point, which is also
    specified in the parameters.

    :type __topic_name: str
    :type __time_from_start: float
    :type __frame_id: str
    :type __joints: dict
    :type __group: MoveGroupCommander
    """
    def __init__(self, definition):
        super(SinglePointTrajectory, self).__init__(definition)

        self.__topic_name = self.get_required_key('topic')
        self.__time_from_start = self.get_required_key('execution_time')
        self.__frame_id, found = self.get_key('frame_id', 'world')
        if not found:
            rospy.logwarn('frame_id not found for single_point_trajectory.'
                          ' Using {0}'.format(self.__frame_id))
        self.__joints = self.get_required_key('joints')
        self.__group = MoveGroupUtils.group

        self.__pub = rospy.Publisher(self.__topic_name,
                                     JointTrajectory,
                                     queue_size=1)

    def execute(self, named_joy=None):
        if Shared.get('move_group_disabled'):
            rospy.logerr('move_group disabled; not publishing trajectory')
            return

        jt = JointTrajectory()
        jt.header.stamp = rospy.Time.now()
        jt.header.frame_id = self.__frame_id
        jt.joint_names = self.__group.get_active_joints()

        jtp = JointTrajectoryPoint()
        jtp.time_from_start = rospy.Duration.from_sec(self.__time_from_start)
        jtp.positions = self.__group.get_current_joint_values()
        if len(jtp.positions) == 0:
            rospy.logerr('Joint states not obtained. Are you sure'
                         ' /joint_states topic is published?')
            return

        # Only change the value of joints specified
        for joint_name in self.__joints.keys():
            idx = jt.joint_names.index(joint_name)
            ang = self.__joints[joint_name]
            if type(ang) is float or type(ang) is int:
                jtp.positions[idx] = ang
            elif ang.endswith('++'):
                jtp.positions[idx] += float(ang[:-2])
            else:
                raise ValueError("String {0} doesn't end with ++".format(ang))

        jt.points.append(jtp)

        self.__pub.publish(jt)


Action.register_preset(SinglePointTrajectory)

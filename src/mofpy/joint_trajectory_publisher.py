import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .move_group_utils import MoveGroupUtils


class JointTrajectoryPublisher:
    def __init__(self, out_topic):
        self.__group = MoveGroupUtils.group

        self.__pub = rospy.Publisher(out_topic,
                                     JointTrajectory,
                                     queue_size=1)

    def publish(self, joints, frame_id, time_from_start):
        """
        Publish the given joint values
        :param joints: the joint values. Does not need to contain all joints
        :type joints: dict[str, float | str]
        :param frame_id: the frame ID of the JointTrajectory message
        :type frame_id: str
        :param time_from_start: the time_from_start to use
        :type time_from_start: float
        :return: the published JointTrajectory message
        """
        current_vals = self.get_current_joint_values()

        jt = JointTrajectory()
        jt.header.stamp = rospy.Time.now()
        jt.header.frame_id = frame_id
        jt.joint_names = []

        jtp = JointTrajectoryPoint()
        jtp.time_from_start = rospy.Duration.from_sec(time_from_start)

        for name in current_vals.keys():
            jt.joint_names.append(name)
            jtp.positions.append(current_vals[name])

        # Only change the value of joints specified
        for joint_name in joints.keys():
            idx = jt.joint_names.index(joint_name)
            ang = joints[joint_name]
            jtp.positions[idx] = ang

        jt.points.append(jtp)

        self.__pub.publish(jt)

        return jt

    def get_current_joint_values(self):
        """
        Retrieves the current joint values from move_group
        :return: map of the joint names and their values
        """

        names = self.__group.get_active_joints()
        pos = self.__group.get_current_joint_values()
        assert(len(names) == len(pos))
        ret = {}
        for n, p in zip(names, pos):
            ret[n] = p
        return ret

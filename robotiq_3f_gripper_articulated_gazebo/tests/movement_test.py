#!/usr/bin/env python

import rospy

from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory


def main():
    rospy.init_node('joint_trajectory_publisher')
    pub1 = rospy.Publisher('/robotiq_3f_controller/command',
                           JointTrajectory,
                           queue_size=1)
    rate = rospy.Rate(50)
    msg = JointTrajectory()
    msg.header = Header()
    msg.joint_names = ['finger_1_joint_1', 'finger_1_joint_2', 'finger_1_joint_3',
                       'finger_2_joint_1', 'finger_2_joint_2', 'finger_2_joint_3',
                       'finger_middle_joint_1', 'finger_middle_joint_2', 'finger_middle_joint_3',
                       'palm_finger_1_joint', 'palm_finger_2_joint']
    msg.points = [JointTrajectoryPoint()]

    while not rospy.is_shutdown():
        try:
            msg.points[0].positions = [1.22, 1.22, 1.22, 1.22, 1.22, 1.22, 1.22, 1.22, 1.22, 0.0, 0.0]
            msg.points[0].time_from_start = rospy.Duration.from_sec(0.1)
            pub1.publish(msg)
            rate.sleep()
            rospy.sleep(1)

            msg.points[0].positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            msg.points[0].time_from_start = rospy.Duration.from_sec(0.1)
            pub1.publish(msg)
            rate.sleep()
            rospy.sleep(1)
        except rospy.ROSInterruptException:
            pass


if __name__ == "__main__":
    main()

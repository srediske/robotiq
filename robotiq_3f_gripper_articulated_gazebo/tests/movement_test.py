import rospy

from std_msgs.msg import Float64
from std_msgs.msg import Header
from trajectory_msgs.msg import *


def main():
    rospy.init_node('down_node')
    pub1 = rospy.Publisher('robotiq_3f_controller/command',
                           JointTrajectory,
                           queue_size=1)
    rate = rospy.Rate(100)  # 50hz
    traj = JointTrajectory()
    traj.header = Header()
    traj.joint_names = ['finger_1_joint_1', 'finger_1_joint_2', 'finger_1_joint_3',
                        'finger_2_joint_1', 'finger_2_joint_2', 'finger_2_joint_3',
                        'finger_middle_joint_1', 'finger_middle_joint_2', 'finger_middle_joint_3',
                        'palm_finger_1_joint', 'palm_finger_2_joint']
    traj.header.stamp = rospy.Time.now()
    pts = JointTrajectoryPoint()
    # Publisher to Unity
    pub2 = rospy.Publisher('float_unity0', Float64, queue_size=1)
    pub3 = rospy.Publisher('float_unity1', Float64, queue_size=1)
    pub4 = rospy.Publisher('float_unity2', Float64, queue_size=1)

    print("Waiting...")

    print("Activated gripper!")

    # scale_factor = 1023/255
    scale_factor = (1.1 - 0.05) / 500.0
    value_in = 20
    while not rospy.is_shutdown():
        try:
            rec_int = [value_in, value_in, value_in]  # Range: 0 - 500
            print(rec_int)
            # print(float(rec_int[1]))
            # print(scale_factor)

            finger_1 = (0.05 + rec_int[0] * scale_factor)
            finger_2 = (0.05 + rec_int[1] * scale_factor)
            finger_middle = (0.05 + rec_int[2] * scale_factor)
            pose = [finger_1, finger_1, (-1.22 + finger_1), finger_2, finger_2, (-1.22 + finger_2), finger_middle,
                    finger_middle, (-1.22 + finger_middle), 0.19, -0.19]
            pts.positions = pose
            pts.time_from_start = rospy.Duration(0.1)
            traj.points = []
            traj.points.append(pts)
            rate.sleep()
            pub1.publish(traj)  # Range of gripper : 0.05 between 1.1 (rad)
            print("Finger A: " + str(finger_1))
            print("Finger B: " + str(finger_2))
            print("Finger C: " + str(finger_middle))
            pub2.publish(rec_int[0])
            pub3.publish(rec_int[1])
            pub4.publish(rec_int[2])
            print("Writing...")

            rospy.sleep(2)
            rec_int = [400, 20, 200]  # Range: 0 - 500
            print(rec_int)
            # print(float(rec_int[1]))
            # print(scale_factor)

            finger_1 = (0.05 + rec_int[0] * scale_factor)
            finger_2 = (0.05 + rec_int[1] * scale_factor)
            finger_middle = (0.05 + rec_int[2] * scale_factor)
            pose = [finger_1, finger_1, (-1.22 + finger_1), finger_2, finger_2, (-1.22 + finger_2), finger_middle,
                    finger_middle, (-1.22 + finger_middle), 0.19, -0.19]
            pts.positions = pose
            pts.time_from_start = rospy.Duration(0.1)
            traj.points = []
            traj.points.append(pts)
            rate.sleep()
            pub1.publish(traj)  # Range of gripper : 0.05 between 1.1 (rad)
            print("Finger A: " + str(finger_1))
            print("Finger B: " + str(finger_2))
            print("Finger C: " + str(finger_middle))
            pub2.publish(rec_int[0])
            pub3.publish(rec_int[1])
            pub4.publish(rec_int[2])
            print("Writing...")

            rospy.sleep(2)
            rec_int = [0, 500, 0]  # Range: 0 - 500
            print(rec_int)
            # print(float(rec_int[1]))
            # print(scale_factor)

            finger_1 = (0.05 + rec_int[0] * scale_factor)
            finger_2 = (0.05 + rec_int[1] * scale_factor)
            finger_middle = (0.05 + rec_int[2] * scale_factor)
            pose = [finger_1, finger_1, (-1.22 + finger_1), finger_2, finger_2, (-1.22 + finger_2), finger_middle,
                    finger_middle, (-1.22 + finger_middle), 0.19, -0.19]
            pts.positions = pose
            pts.time_from_start = rospy.Duration(0.1)
            traj.points = []
            traj.points.append(pts)
            rate.sleep()
            pub1.publish(traj)  # Range of gripper : 0.05 between 1.1 (rad)
            print("Finger A: " + str(finger_1))
            print("Finger B: " + str(finger_2))
            print("Finger C: " + str(finger_middle))
            pub2.publish(rec_int[0])
            pub3.publish(rec_int[1])
            pub4.publish(rec_int[2])
            print("Writing...")

            rospy.sleep(2)
            rec_int = [250, 100, 250]  # Range: 0 - 500
            print(rec_int)
            # print(float(rec_int[1]))
            # print(scale_factor)

            finger_1 = (0.05 + rec_int[0] * scale_factor)
            finger_2 = (0.05 + rec_int[1] * scale_factor)
            finger_middle = (0.05 + rec_int[2] * scale_factor)
            pose = [finger_1, finger_1, (-1.22 + finger_1), finger_2, finger_2, (-1.22 + finger_2), finger_middle,
                    finger_middle, (-1.22 + finger_middle), 0.19, -0.19]
            pts.positions = pose
            pts.time_from_start = rospy.Duration(0.1)
            traj.points = []
            traj.points.append(pts)
            rate.sleep()
            pub1.publish(traj)  # Range of gripper : 0.05 between 1.1 (rad)
            print("Finger A: " + str(finger_1))
            print("Finger B: " + str(finger_2))
            print("Finger C: " + str(finger_middle))
            pub2.publish(rec_int[0])
            pub3.publish(rec_int[1])
            pub4.publish(rec_int[2])
            print("Writing...")

            rospy.sleep(2)

        except ValueError:  # TODO: implement better error detection
            print("ValueError")


if __name__ == "__main__":
    main()

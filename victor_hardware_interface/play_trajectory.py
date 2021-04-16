#!/usr/bin/env python

from victor_hardware_interface import victor_utils as vu
from victor_hardware_interface.msg import *
import rospy
import scipy.io as sio
import sys

# ros publishers do not immediately connect to the topic after the constructer is called.
# Without a sleep param the message is lost
# Sleeping for some small time allows the publisher to connect
# A more guaranteed but also more complicated method would be to periodly republish the message until the desired state is reached
# MAGIC_SLEEP_PARAM = 0.2



def run_trajectory(mat_location, MAGIC_SLEEP_PARAM=0.2, inds=1000000):
    loaded = sio.loadmat(mat_location)
    vu.set_control_mode(ControlMode.JOINT_IMPEDANCE, "left_arm", stiffness=vu.Stiffness.STIFF)
    traj = loaded['traj']
    if inds == 1000000:
        inds = traj.shape[0]
    for i in range(inds):
        msg = MotionCommand()
        msg.control_mode.mode = ControlMode.JOINT_IMPEDANCE;
        msg.joint_position.joint_1 = traj[i][0];
        msg.joint_position.joint_2 = traj[i][1];
        msg.joint_position.joint_3 = traj[i][2];
        msg.joint_position.joint_4 = traj[i][3];
        msg.joint_position.joint_5 = traj[i][4];
        msg.joint_position.joint_6 = traj[i][5];
        msg.joint_position.joint_7 = traj[i][6];
        
        pub = rospy.Publisher("left_arm/motion_command", MotionCommand, queue_size=10)
        rospy.sleep(MAGIC_SLEEP_PARAM)
        pub.publish(msg)


if __name__ == "__main__":

    rospy.init_node("initialize_fake_victor")
    rospy.sleep(0.5)
    mat_location = sys.argv[1]
    if len(sys.argv) == 2:
        run_trajectory(mat_location)
    if len(sys.argv) >= 3:
        sleep_time = float(sys.argv[2])
        run_trajectory(mat_location, sleep_time)
    if len(sys.argv) >= 4:
        sleep_time = float(sys.argv[2])
        inds = int(sys.argv[3])
        run_trajectory(mat_location, sleep_time, inds)

import rospy
from sensor_msgs.msg import JointState
from victor_hardware_interface.msg import MotionStatus
from victor_hardware_interface.msg import Robotiq3FingerStatus
from threading import Lock
from math import radians
from numpy import *
import scipy.io as sio

def compute_finger_angles(control):
    # the control is from 0 to 1, however, the relationship takes 0 to 255
    g = control * 255
    max_angle = [70.0, 90.0, 43.0]
    min_3 = -55.0
    m1 = max_angle[0] / 140.0
    m2 = max_angle[1] / 100.0

    # TODO: Add a link to the documentation/paper we used for this math
    # Based on the relationship from the documentation, set each joint angle based on the "phase" of the motion
    if g <= 110.0:
        theta1 = m1 * g
        theta2 = 0
        theta3 = -m1 * g
    elif 110.0 < g <= 140.0:
        theta1 = m1 * g
        theta2 = 0
        theta3 = min_3
    elif 140.0 < g <= 240.0:
        theta1 = max_angle[0]
        theta2 = m2 * (g-140)
        theta3 = min_3
    else:
        theta1 = max_angle[0]
        theta2 = max_angle[1]
        theta3 = min_3

    return [radians(theta1), radians(theta2), radians(theta3)]


def compute_scissor_angle(control):
    # 0 corresponds to fully open at -16 degrees, 1 is fully closed with at +10 degrees
    return radians(26.0 * control - 16.0)


class VictorJointStatePublisher:
    def __init__(self):
        # We don't want to do this programatically, because the callback functions are using assuming a specific order
        # for the joints. We could change the callback functions to remove this assumption, but that would increase the
        # complexity of the callback functions unnecessarily
        self.joint_names = [
                'victor_left_arm_joint_1',  'victor_left_arm_joint_2',  'victor_left_arm_joint_3',  'victor_left_arm_joint_4',  'victor_left_arm_joint_5',  'victor_left_arm_joint_6',  'victor_left_arm_joint_7',
                'victor_right_arm_joint_1', 'victor_right_arm_joint_2', 'victor_right_arm_joint_3', 'victor_right_arm_joint_4', 'victor_right_arm_joint_5', 'victor_right_arm_joint_6', 'victor_right_arm_joint_7',
                                                        'victor_left_gripper_fingerA_joint_2',  'victor_left_gripper_fingerA_joint_3',  'victor_left_gripper_fingerA_joint_4',
                'victor_left_gripper_fingerB_knuckle',  'victor_left_gripper_fingerB_joint_2',  'victor_left_gripper_fingerB_joint_3',  'victor_left_gripper_fingerB_joint_4',
                'victor_left_gripper_fingerC_knuckle',  'victor_left_gripper_fingerC_joint_2',  'victor_left_gripper_fingerC_joint_3',  'victor_left_gripper_fingerC_joint_4',
                                                        'victor_right_gripper_fingerA_joint_2', 'victor_right_gripper_fingerA_joint_3', 'victor_right_gripper_fingerA_joint_4',
                'victor_right_gripper_fingerB_knuckle', 'victor_right_gripper_fingerB_joint_2', 'victor_right_gripper_fingerB_joint_3', 'victor_right_gripper_fingerB_joint_4',
                'victor_right_gripper_fingerC_knuckle', 'victor_right_gripper_fingerC_joint_2', 'victor_right_gripper_fingerC_joint_3', 'victor_right_gripper_fingerC_joint_4']

        # Setup the output message with default values
        self.joint_state_lock = Lock()
        self.joint_state_msg = JointState()
        self.joint_state_msg.name = self.joint_names
        self.joint_state_msg.position = [0] * len(self.joint_names)
        self.joint_state_msg.velocity = []
        self.joint_state_msg.effort = [0] * len(self.joint_names)

        # Setup the publishers and subscribers that will be used
        self.joint_state_pub = rospy.Publisher("joint_states", JointState, queue_size = 1)
        self.left_arm_sub = rospy.Subscriber("left_arm/motion_status", MotionStatus, self.left_arm_motion_status_callback)
        self.right_arm_sub = rospy.Subscriber("right_arm/motion_status", MotionStatus, self.right_arm_motion_status_callback)
        self.left_gripper_sub = rospy.Subscriber("left_arm/gripper_status", Robotiq3FingerStatus, self.left_gripper_motion_status_callback)
        self.right_gripper_sub = rospy.Subscriber("right_arm/gripper_status", Robotiq3FingerStatus, self.right_gripper_motion_status_callback)
        self.joint_manual_msg = MotionStatus()
        self.leftarm_pub = rospy.Publisher("left_arm/motion_status", MotionStatus, queue_size = 1)

    def run(self, loop_rate, traj, reptimes):
        rate = rospy.Rate(loop_rate)
        # while not rospy.is_shutdown():
        for i in range(len(traj)):
            for _ in range(reptimes):
                self.publish_joint_values()
                self.publish_leftarm_values(traj[i])
                rate.sleep()

    def left_arm_motion_status_callback(self, motion_status):
        self.set_arm_position_values(motion_status, offset = 0)
        self.set_arm_effort_values(motion_status, offset = 0)

    def right_arm_motion_status_callback(self, motion_status):
        self.set_arm_position_values(motion_status, offset = 7)
        self.set_arm_effort_values(motion_status, offset = 7)

    def left_gripper_motion_status_callback(self, gripper_status):
        self.set_gripper_position_values(gripper_status, offset = 14)

    def right_gripper_motion_status_callback(self, gripper_status):
        self.set_gripper_position_values(gripper_status, offset = 25)

    def set_arm_position_values(self, motion_status, offset):
        with self.joint_state_lock:
            self.joint_state_msg.position[offset + 0] = motion_status.measured_joint_position.joint_1
            self.joint_state_msg.position[offset + 1] = motion_status.measured_joint_position.joint_2
            self.joint_state_msg.position[offset + 2] = motion_status.measured_joint_position.joint_3
            self.joint_state_msg.position[offset + 3] = motion_status.measured_joint_position.joint_4
            self.joint_state_msg.position[offset + 4] = motion_status.measured_joint_position.joint_5
            self.joint_state_msg.position[offset + 5] = motion_status.measured_joint_position.joint_6
            self.joint_state_msg.position[offset + 6] = motion_status.measured_joint_position.joint_7

    def set_gripper_position_values(self, gripper_status, offset):
        with self.joint_state_lock:
            self.joint_state_msg.position[offset + 0: offset + 3] = compute_finger_angles(gripper_status.finger_a_status.position)

            self.joint_state_msg.position[offset + 3] = compute_scissor_angle(gripper_status.scissor_status.position)
            self.joint_state_msg.position[offset + 4: offset + 7] = compute_finger_angles(gripper_status.finger_b_status.position)

            self.joint_state_msg.position[offset + 7] = compute_scissor_angle(gripper_status.scissor_status.position)
            self.joint_state_msg.position[offset + 8: offset + 11] = compute_finger_angles(gripper_status.finger_c_status.position)

    def set_arm_effort_values(self, motion_status, offset):
        # We use measured joint torque for now. As Kuka mentioned, it is the currently measured "raw" torque sensor
        # data. There is an estimated_external_torque message, which is the current external torque sensor data for
        # this robot.
        with self.joint_state_lock:
            self.joint_state_msg.effort[offset + 0] = motion_status.measured_joint_torque.joint_1
            self.joint_state_msg.effort[offset + 1] = motion_status.measured_joint_torque.joint_2
            self.joint_state_msg.effort[offset + 2] = motion_status.measured_joint_torque.joint_3
            self.joint_state_msg.effort[offset + 3] = motion_status.measured_joint_torque.joint_4
            self.joint_state_msg.effort[offset + 4] = motion_status.measured_joint_torque.joint_5
            self.joint_state_msg.effort[offset + 5] = motion_status.measured_joint_torque.joint_6
            self.joint_state_msg.effort[offset + 6] = motion_status.measured_joint_torque.joint_7

    def publish_joint_values(self):
        with self.joint_state_lock:
            self.joint_state_msg.header.stamp = rospy.Time.now()
            print(self.joint_state_msg)
            self.joint_state_pub.publish(self.joint_state_msg)

    def publish_leftarm_values(self, vals):
        with self.joint_state_lock:
            self.joint_manual_msg.commanded_joint_position.joint_1 = vals[0]
            self.joint_manual_msg.commanded_joint_position.joint_2 = vals[1]
            self.joint_manual_msg.commanded_joint_position.joint_3 = vals[2]
            self.joint_manual_msg.commanded_joint_position.joint_4 = vals[3]
            self.joint_manual_msg.commanded_joint_position.joint_5 = vals[4]
            self.joint_manual_msg.commanded_joint_position.joint_6 = vals[5]
            self.joint_manual_msg.commanded_joint_position.joint_7 = vals[6]
            self.joint_manual_msg.measured_joint_position.joint_1 = vals[0]
            self.joint_manual_msg.measured_joint_position.joint_2 = vals[1]
            self.joint_manual_msg.measured_joint_position.joint_3 = vals[2]
            self.joint_manual_msg.measured_joint_position.joint_4 = vals[3]
            self.joint_manual_msg.measured_joint_position.joint_5 = vals[4]
            self.joint_manual_msg.measured_joint_position.joint_6 = vals[5]
            self.joint_manual_msg.measured_joint_position.joint_7 = vals[6]
            print(self.joint_manual_msg)
            self.leftarm_pub.publish(self.joint_manual_msg)


if __name__ == '__main__':
    rospy.init_node('victor_joint_state_publisher')
    rospy.loginfo('Starting the victor joint state broadcaster...')

    rate = rospy.get_param("~rate", 20.0)
    pub = VictorJointStatePublisher()
    # traj = sio.loadmat('/home/armlab/Downloads/gchou/plan_new_1_v2_exp.mat')
    traj = sio.loadmat('/home/armlab/Downloads/gchou/demo_ctrarm_v2_3_exp2.mat')
    # traj = sio.loadmat('/home/armlab/Downloads/gchou/demo_arm_2_exp.mat')
    traj_full = traj.get('traj_full_exp').T
    pub.run(rate, traj_full, 1)

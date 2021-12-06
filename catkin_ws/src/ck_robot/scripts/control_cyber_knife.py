#!/usr/bin/env python

import math
import random

import sympy

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

print("Finished with imports.")

MAX_JOINT_VELOCITY = 1.5

joint_positions = [-1, -1, -1, -1, -1, -1]
joint_velocities = [-1, -1, -1, -1, -1, -1]


joint_1_pub = rospy.Publisher("ck_robot/trans_1/command", Float64, queue_size=10)
joint_2_pub = rospy.Publisher("ck_robot/trans_2/command", Float64, queue_size=10)
joint_3_pub = rospy.Publisher("ck_robot/trans_3/command", Float64, queue_size=10)
joint_4_pub = rospy.Publisher("ck_robot/trans_4/command", Float64, queue_size=10)
joint_5_pub = rospy.Publisher("ck_robot/trans_5/command", Float64, queue_size=10)
joint_6_pub = rospy.Publisher("ck_robot/trans_6/command", Float64, queue_size=10)


tumor_location = None


theta1, theta2, theta3, theta4, theta5, theta6 = sympy.symbols("theta1, theta2, theta3, theta4, theta5, theta6")
generic_jacobian = None
generic_t_0_to_6 = None


def joint_state_callback(joint_states):
    global joint_positions, joint_velocities
    joint_positions = joint_states.position 
    joint_velocities = joint_states.velocity

    # TODO - remove this temporary test
    # test_rotate_joint_6()


# Generates a list of transformation matrices from world frame to frame N
def transformation_matrix(a, alpha, d, theta):
    matrix_list = []  # Will store matrix from T0_n frames
    A = np.identity(4)
    dh_table = np.array([[a[0], alpha[0], d[0], theta[0]],
                         [a[1], alpha[1], d[1], theta[1]],
                         [a[2], alpha[2], d[2], theta[2]],
                         [a[3], alpha[3], d[3], theta[3]],
                         [a[4], alpha[4], d[4], theta[4]],
                         [a[5], alpha[5], d[5], theta[5]]])
   
    for i in range(0, len(dh_table)):
        T = sympy.Matrix([[sympy.cos(dh_table[i, 3]), -sympy.sin(dh_table[i, 3]) * sympy.cos(dh_table[i, 1]), sympy.sin(dh_table[i, 3]) * sympy.sin(dh_table[i, 1]), dh_table[i, 0] * sympy.cos(dh_table[i, 3])],
             [sympy.sin(dh_table[i, 3]), sympy.cos(dh_table[i, 3]) * sympy.cos(dh_table[i, 1]), -sympy.cos(
                 dh_table[i, 3]) * sympy.sin(dh_table[i, 1]), dh_table[i, 0] * sympy.sin(dh_table[i, 3])],
             [0, sympy.sin(dh_table[i, 1]), sympy.cos(
                 dh_table[i, 1]), dh_table[i, 2]],
             [0, 0, 0, 1]])
                
        A = A @ T
        matrix_list.append(A)
        # with np.printoptions(precision=2, suppress=True):
        #    print(A)
    return matrix_list



def receive_tumor_location(msg):
    global tumor_location
    tumor_location = msg.pose.position   

    pursue_tumor_location()


def pursue_tumor_location():
    if not generic_jacobian:
        return

    this_j = generic_jacobian.subs([(theta1, joint_positions[0]),
        (theta2, joint_positions[1]),
        (theta3, joint_positions[2]),
        (theta4, joint_positions[3]),
        (theta5, joint_positions[4]),
        (theta6, joint_positions[5])])
    inverse_j = this_j.inv() # TODO

    transform_0_to_6 = generic_t_0_to_6.subs([(theta1, joint_positions[0]),
        (theta2, joint_positions[1]),
        (theta3, joint_positions[2]),
        (theta4, joint_positions[3]),
        (theta5, joint_positions[4]),
        (theta6, joint_positions[5])])
    ee_position = transform_0_to_6 * sympy.Matrix([[0], [0], [0], [1]])

    # Calculate the change for the end effector for each of the 6 coordinates
    dx = tumor_location.x - ee_position[0, 0]
    dy = tumor_location.y - ee_position[1, 0]
    dz = tumor_location.z - ee_position[2, 0]
    # TODO - how do we calculate the angle of the end effector? 

    desired_ee_movement = sympy.Matrix([[dx], [dy], [dz], [0], [0], [0]])
    q_prime = inverse_j * desired_ee_movement

    max_velocity = 0
    for velocity in q_prime:
        if abs(velocity) > max_velocity:
            max_velocity = velocity

    ratio = 1
    if max_velocity > MAX_JOINT_VELOCITY:
        ratio = MAX_JOINT_VELOCITY / abs(max_velocity)

    joint_1_pub.publish(q_prime[0, 0] / ratio)
    joint_2_pub.publish(q_prime[1, 0] / ratio)
    joint_3_pub.publish(q_prime[2, 0] / ratio)
    joint_4_pub.publish(q_prime[3, 0] / ratio)
    joint_5_pub.publish(q_prime[4, 0] / ratio)
    joint_6_pub.publish(q_prime[5, 0] / ratio)


# def test_rotate_joint_6():
#     max_rotation_speed = 0.2
#
#     joint_6_position = joint_positions[5]
#
#     recommended_speed = (math.pi - joint_6_position) / 10
#     rotation_speed = min(max_rotation_speed, recommended_speed)
#
#     # print("Rotating with speed " + rotation_speed)
#     joint_6_pub.publish(rotation_speed)


def generate_generic_jacobian(ts):
    global generic_jacobian, generic_t_0_to_6

    generic_t_0_to_6 = ts[-1]

    j1 = sympy.zeros(6, 1)
    j1[0:3, 0] = sympy.Matrix([[0], [0], [1]]).cross(ts[-1][0:3, 3] - sympy.Matrix([[0], [0], [0]]))
    j1[3:6, 0] = sympy.Matrix([[0], [0], [1]])

    j2 = sympy.zeros(6, 1)
    j2[0:3, 0] = ts[0][0:3, 2].cross(ts[-1][0:3, 3] - ts[0][0:3, 3])
    j2[3:6, 0] = ts[0][0:3, 2]

    j3 = sympy.zeros(6, 1)
    j3[0:3, 0] = ts[1][0:3, 2].cross(ts[-1][0:3, 3] - ts[1][0:3, 3])
    j3[3:6, 0] = ts[1][0:3, 2]

    j4 = sympy.zeros(6, 1)
    j4[0:3, 0] = ts[2][0:3, 2].cross(ts[-1][0:3, 3] - ts[2][0:3, 3])
    j4[3:6, 0] = ts[2][0:3, 2]

    j5 = sympy.zeros(6, 1)
    j5[0:3, 0] = ts[3][0:3, 2].cross(ts[-1][0:3, 3] - ts[3][0:3, 3])
    j5[3:6, 0] = ts[3][0:3, 2]

    j6 = sympy.zeros(6, 1)
    j6[0:3, 0] = ts[4][0:3, 2].cross(ts[-1][0:3, 3] - ts[4][0:3, 3])
    j6[3:6, 0] = ts[4][0:3, 2]

    generic_jacobian = sympy.zeros(6, 6)
    generic_jacobian[:, 0] = j1
    generic_jacobian[:, 1] = j2
    generic_jacobian[:, 2] = j3
    generic_jacobian[:, 3] = j4
    generic_jacobian[:, 4] = j5
    generic_jacobian[:, 5] = j6

def init_cyberknife_control():
    rospy.init_node('cyberknife_control', anonymous=True)

    rospy.Subscriber("ck_robot/joint_states", JointState, joint_state_callback)

    # theta = [np.pi/2, 0, 0, 0, 0, 0, 0]
    theta = [theta1, theta2, theta3, theta4, theta5, theta6]
    d = [630, 0, 0, 190, 0, 0] 
    alpha = [np.pi/2, 0, np.pi/2, -np.pi/2, np.pi/2, -np.pi/2]
    a = [300, 680, 0, 0, 0, 0]

    T0_n = transformation_matrix(a, alpha, d, theta)

    generate_generic_jacobian(T0_n)

    rospy.spin()


if __name__ == '__main__':
    try:
        init_cyberknife_control()
    except rospy.ROSInterruptException:
        pass

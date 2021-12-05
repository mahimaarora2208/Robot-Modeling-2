#!/usr/bin/env python

import math
import random

import sympy

import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

print("Finished with imports.")

joint_positions = [-1, -1, -1, -1, -1, -1]
joint_velocities = [-1, -1, -1, -1, -1, -1]


joint_1_pub = rospy.Publisher("ck_robot/trans_1/command", Float64, queue_size=10)
joint_2_pub = rospy.Publisher("ck_robot/trans_2/command", Float64, queue_size=10)
joint_3_pub = rospy.Publisher("ck_robot/trans_3/command", Float64, queue_size=10)
joint_4_pub = rospy.Publisher("ck_robot/trans_4/command", Float64, queue_size=10)
joint_5_pub = rospy.Publisher("ck_robot/trans_5/command", Float64, queue_size=10)
joint_6_pub = rospy.Publisher("ck_robot/trans_6/command", Float64, queue_size=10)


tumor_location = None


def joint_state_callback(joint_states):
    global joint_positions, joint_velocities
    joint_positions = joint_states.position 
    joint_velocities = joint_states.velocity
    

    # TODO - remove this temporary test
    # test_rotate_joint_6()


def receive_tumor_location(msg):
    global tumor_location
    tumor_location = msg.pose.position   

    pursue_tumor_location()


def pursue_tumor_location():
    J = None # TODO - this is a sympy matrix of the Jacobian 
    this_j = J.subs([(theta1, joint_positions[0]),
        (theta2, joint_positions[1]),
        (theta3, joint_positions[2]),
        (theta4, joint_positions[3]),
        (theta5, joint_positions[4]),
        (theta6, joint_positions[5])])
    inverse_j = None # TODO

    transform_0_to_6 = t_0_to_6.subs([(theta1, joint_positions[0]),
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

    joint_1_pub.publish(q_prime[0, 0])
    joint_2_pub.publish(q_prime[1, 0])
    joint_3_pub.publish(q_prime[2, 0])
    joint_4_pub.publish(q_prime[3, 0])
    joint_5_pub.publish(q_prime[4, 0])
    joint_6_pub.publish(q_prime[5, 0])


def test_rotate_joint_6():
    max_rotation_speed = 0.2 

    joint_6_position = joint_positions[5]
    
    recommended_speed = (math.pi - joint_6_position) / 10
    rotation_speed = min(max_rotation_speed, recommended_speed)
    
    # print("Rotating with speed " + rotation_speed)
    joint_6_pub.publish(rotation_speed)


def init_cyberknife_control():
    rospy.init_node('cyberknife_control', anonymous=True)

    rospy.Subscriber("ck_robot/joint_states", JointState, joint_state_callback)

    rospy.spin()


if __name__ == '__main__':
    try:
        init_cyberknife_control()
    except rospy.ROSInterruptException:
        pass

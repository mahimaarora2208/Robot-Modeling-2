#!/usr/bin/env python

import math
import random

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


def joint_state_callback(joint_states):
    global joint_positions, joint_velocities
    joint_positions = joint_states.position 
    joint_velocities = joint_states.velocity
    

    # TODO - remove this temporary test
    test_rotate_joint_6()


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

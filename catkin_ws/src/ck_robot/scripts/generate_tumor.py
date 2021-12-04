#!/usr/bin/env python

import math
import random

import rospy
from geometry_msgs.msg import PoseStamped

def init_tumor_generator():
    tumor_pub = rospy.Publisher('tumor_location', PoseStamped, queue_size=10)
    rospy.init_node('generate_tumor', anonymous=True)

    tumor_distance = 10 * random.random()
    tumor_angle = 2 * math.pi * random.random()
    tumor_elevation = 10 * random.random()

    tumor_location = PoseStamped()
    tumor_location.pose.position.x = tumor_distance * math.cos(tumor_angle) 
    tumor_location.pose.position.y = tumor_distance * math.sin(tumor_angle) 
    tumor_location.pose.position.z = tumor_elevation

    tumor_pub.publish(tumor_location)

if __name__ == '__main__':
    try:
        init_tumor_generator()
    except rospy.ROSInterruptException:
        pass

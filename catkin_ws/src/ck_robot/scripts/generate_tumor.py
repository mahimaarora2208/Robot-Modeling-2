#!/usr/bin/env python

import math
import random
import os

import rospy
from geometry_msgs.msg import PoseStamped

from gazebo_msgs.msg import ModelStates, ModelState
from gazebo_msgs.srv import SetModelState


def spawn_tumor(x, y, z):
    spawn_command = "rosrun gazebo_ros spawn_model -file urdf/sphere.urdf -urdf -model tumor"
    spawn_command += " -x " + str(x)
    spawn_command += " -y " + str(y)
    spawn_command += " -z " + str(z)

    # os.system("source ~/.bashrc && roscd ck_robot && " + spawn_command)
    os.system(spawn_command)

def move_tumor(x, y, z):
    new_tumor_state = ModelState()
    new_tumor_state.model_name = "tumor"
    new_tumor_state.pose.position.x = x
    new_tumor_state.pose.position.y = y
    new_tumor_state.pose.position.z = z

    print("Moving tumor to " + str(x) + ", " + str(y) + ", " + str(z))
    rospy.wait_for_service('/gazebo/set_model_state')
    set_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
    set_state(new_tumor_state)


def init_tumor_generator():
    rospy.init_node('generate_tumor', anonymous=True)

    # Generate tumor coordinates
    tumor_distance = random.random() + 0.5
    tumor_angle = 2 * math.pi * random.random()
    tumor_elevation = random.random() + 0.5

    x = tumor_distance * math.cos(tumor_angle) 
    y = tumor_distance * math.sin(tumor_angle) 
    z = tumor_elevation

    # Wait for a list of models currently in Gazebo, and spawn/move tumor accordingly
    model_states = rospy.wait_for_message("/gazebo/model_states", ModelStates, timeout=None)
    model_names = model_states.name
    print("Model names: " + str(model_names))

    if "tumor" not in model_names:
        spawn_tumor(x, y, z)
    else:
        move_tumor(x, y, z)

    # Publish our coordinates to our own topic
    tumor_pub = rospy.Publisher('/tumor_location', PoseStamped, queue_size=10)
    tumor_location = PoseStamped()
    tumor_location.pose.position.x = x
    tumor_location.pose.position.y = y
    tumor_location.pose.position.z = z
 
    while tumor_pub.get_num_connections() == 0:
        pass

    tumor_pub.publish(tumor_location)
    

if __name__ == '__main__':
    try:
        init_tumor_generator()
    except rospy.ROSInterruptException:
        pass

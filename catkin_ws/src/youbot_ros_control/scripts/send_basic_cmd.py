#!/usr/bin/env python


import rospy
import tf 
import numpy as np

from brics_actuator.msg import JointPositions, JointVelocities, JointValue
from sensor_msgs.msg import JointState

"""
VARIABLES
"""

thetas = [0, 0, 0, 0, 0]
omega = [0, 0, 0, 0, 0]
gamma = [0, 0, 0, 0, 0]

numberOfJoints = 5

"""
FUNCTIONS
"""

def getJointState(joints):
    global thetas
    try:
        for i in xrange(0, numberOfJoints):    
			thetas[i] = joints.position[i]          
			omega[i] = joints.velocity[i]
			gamma[i] = joints.effort[i]
    except:
        print("An error occured while reading the robot joint state")

"""
MAIN
"""

if __name__=="__main__":

	rospy.init_node('name')
	
	pub_pos = rospy.Publisher('arm_1/arm_controller/position_command', JointPositions)
	sub = rospy.Subscriber("joint_states", JointState, getJointState)
	
	

	while not rospy.is_shutdown():

		rate.sleep()
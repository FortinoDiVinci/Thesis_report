#!/usr/bin/env python

import sys, time, os
import numpy as np
import math
import csv
import subprocess, shlex

import rospy

from brics_actuator.msg import JointPositions, JointValue
from sensor_msgs.msg import JointState

####################
# GLOBAL VARIABLES
####################

jointsUpperLimit = [
        5.84014 - 1e-5,
        2.61799 - 1e-5,
        -0.015708 - 1e-5,
        3.4292 - 1e-5,
        5.64159 - 1e-5,
]

jointsLowerLimit = [
        0.0100692 + 1e-5,
        0.0100692 + 1e-5,
        -5.02655 + 1e-5,
        0.0221239 + 1e-5,
        0.110619 + 1e-5,
]

thetas = [0, 0, 0, 0, 0]

NB_JOINTS = 5  

####################
# CLASSES
####################

class Trajectory:
    def __init__(self):
        self.time_stamps = []
        self.positions = []
        self.velocities = []
        self.accelerations = []

####################
# FUNCTIONS
####################

# get trajectories data
def getTrajectoriesFromCSV(relative_paths):  

    nb_paths = len(relative_paths);
    base_path = os.path.dirname(os.path.abspath(__file__))
    trajectories_path = [base_path + relative_path for relative_path in relative_paths];
    trajectories_data = [Trajectory() for i in xrange(nb_paths)]

    for ii in xrange(nb_paths):
        try:
            with open(trajectories_path[ii]) as csvfile:
                readCSV = csv.reader(csvfile, delimiter=',')
                headers = next(readCSV)
                for row in readCSV:
                    trajectories_data[ii].time_stamps.append(float(row[0]))
                    trajectories_data[ii].positions.append(float(row[1]))
                    trajectories_data[ii].velocities.append(float(row[2]))
                    trajectories_data[ii].accelerations.append(float(row[3]))         
        except OSError as e:
            print(e)
            return e
    
    return trajectories_data

"""
# initial values of the robot (get thetas states)
def getJointState(joints):
    global thetas, init_flag, sub#
    try:
        for theta in xrange(0, numberOfJoints):    
			thetas[theta] = joints.position[theta]          
            # Initial speed state is alway null
			thetas_speed[theta] = 0 
    except:
        print("An error occured while reading the robot joint state")
    
    sub.unregister()
    init_flag = True
    print("Joint state initialized")

def subJointStateOnce():
    # Get initial values and set them to thetas
    global sub
    sub = rospy.Subscriber("joint_states", JointState, getJointState)
    
    counter = 0
    # Wait for the execution of getJointState (subscriber callback function)
    while(init_flag == False):
        time.sleep(0.1)
        counter = counter + 1
        if counter > 50:
            print("Failure to get initial state")
            return -1	
    return 0
"""
    
####################
# MAIN
####################    
 
if __name__=="__main__":

    rospy.init_node("joint_trajectory")
	
    pub_pos = rospy.Publisher('arm_1/arm_controller/position_command', JointPositions)  
    
    base_path = os.path.dirname(os.path.abspath(__file__))
    
    # create position variable to send
    armJointPositions = JointPositions()
    pos = JointValue()
    pos.unit = "rad"
    armJointPositions.positions.append(pos)  
        
    # Read Trajectories data
    trajectories = getTrajectoriesFromCSV(['/trajectory_data/sinusoidal_trajectory.csv', '/trajectory_data/triangle_trajectory.csv'])
    freq = 1/(trajectories[0].time_stamps[1] - trajectories[0].time_stamps[0])
    rate = rospy.Rate(freq) 
    
    time.sleep(2.0)
    
    ## initialise position
    for joint in xrange(NB_JOINTS):
        armJointPositions.positions[0].joint_uri = "arm_joint_" + str(joint + 1)
        if joint == 2:
            armJointPositions.positions[0].value = jointsUpperLimit[joint]
        else:
            armJointPositions.positions[0].value = jointsLowerLimit[joint]
            
        pub_pos.publish(armJointPositions)
        rate.sleep()
        
    ## main loop
    
    
    for joint in xrange(NB_JOINTS):
    
        armJointPositions.positions[0].joint_uri = "arm_joint_" + str(joint + 1)
        
        for idx, trajectory in enumerate(trajectories):
        
            # to account for negative values of joint 3
            if joint == 2:
                sign = -1.
            else:
                sign = 1.
        
            # rosbag shell process to record data
            # rosbag_path = base_path + "/rosbag/"
            rosbag_name = armJointPositions.positions[0].joint_uri + "_trajectory" + str(idx)
            rosbag_command_str = "rosbag record -O " + rosbag_name + " /joint_states /arm_1/joint_set_points"
            print(rosbag_command_str)
            rosbag_cmd = shlex.split(rosbag_command_str)
            rosbag_proc = subprocess.Popen(rosbag_cmd)
    
            for traj_point in trajectory.positions:
                
                armJointPositions.positions[0].value = sign*traj_point
                # Send new joint speed
                pub_pos.publish(armJointPositions)
                rate.sleep()
                
            rosbag_proc.send_signal(subprocess.signal.SIGINT)
            
            # reset joint position
            if sign == 1:
                armJointPositions.positions[0].value = jointsLowerLimit[joint]
                # Send new joint speed
                pub_pos.publish(armJointPositions)
            else:
                armJointPositions.positions[0].value = jointsUpperLimit[joint]
                # Send new joint speed
                pub_pos.publish(armJointPositions) 
            
            time.sleep(2.0)
    """
    print("Processing Data... This might take a while")        
    # Data processing from rosbag to csv
    for joint in xrange(NB_JOINTS):
    
        armJointPositions.positions[0].joint_uri = "arm_joint_" + str(joint + 1)
        
        for idx, trajectory in enumerate(trajectories):
    
            rosbag_name = armJointPositions.positions[0].joint_uri + "_trajectory" + str(idx)
    
            rosbag_command_str = "rosbag reindex " + rosbag_name + ".bag.active"
            print(rosbag_command_str)
            rosbag_cmd = shlex.split(rosbag_command_str)
            rosbag_proc = subprocess.Popen(rosbag_cmd)
            
            #rosbag_command_str = "for topic in `rostopic list -b " + rosbag_name + ".bag.active` ; do rostopic echo -p -b " + rosbag_name + ".bag.active $topic >bagfile-${topic//\//_}.csv ; done"           
            #print(rosbag_command_str)
            #rosbag_cmd = shlex.split(rosbag_command_str)
            #rosbag_proc = subprocess.Popen(rosbag_cmd)"""


#!/usr/bin/env python

import sys, time, os
import numpy as np
import math
import csv

import rospy

from brics_actuator.msg import JointPositions, JointValue
from sensor_msgs.msg import JointState

####################
# GLOBAL VARIABLES
####################

jointsUpperLimit = [
        5.84014 - 1e-5,
        2.61799 - 1e-5,
        -5.02655 + 1e-5,
        3.4292 - 1e-5,
        5.64159 - 1e-5,
]

jointsLowerLimit = [
        0.0100692 + 1e-5,
        0.0100692 + 1e-5,
        -0.015708 - 1e-5,
        0.0221239 + 1e-5,
        0.110619 + 1e-5,
]

thetas = [0, 0, 0, 0, 0]
efforts = [0, 0, 0, 0, 0]

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
        self.joint = ""

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
                trajectories_data[ii].joint = headers[4]
                for row in readCSV:
                    trajectories_data[ii].time_stamps.append(float(row[0]))
                    trajectories_data[ii].positions.append(float(row[1]))
                    trajectories_data[ii].velocities.append(float(row[2]))
                    trajectories_data[ii].accelerations.append(float(row[3]))         
        except OSError as e:
            print(e)
            return e
    
    return trajectories_data

# initial values of the robot (get thetas states)
def getJointState(joints):
    global thetas, efforts
    for theta in xrange(0, numberOfJoints):    
        thetas[theta] = joints.position[theta]
        efforts[theta] = joints.effort[theta]

# 
def sendTrajectory(jnts_trajs, pub):
    global thetas, efforts

    freq = 1/(jnts_trajs[0].time_stamps[1] - jnts_trajs[0].time_stamps[0])
    rate = rospy.Rate(freq) 
    
    nb_joints_ctrl = len(jnts_trajs)
    
    # initialize msg to be published
    arm_joints = JointPositions()
    pos = JointValue()
    pos.unit = "rad"
    
    for joint in xrange(nb_joints_ctrl):
        pos.joint_uri = jnts_trajs[joint].joint
        pos.value = 0
        arm_joints.positions.append(pos)  
    
    # initialize arrays for recording joint state during trial
    joints_theta = [ [None for _ in xrange(6)] for _ in xrange(len(jnts_trajs[0].positions) + 1)]
    joints_effort = [ [None for _ in xrange(6)] for _ in xrange(len(jnts_trajs[0].positions) + 1)]
    # format is [[jnt1, jnt2, ..., jnt5, time_stamp], ... []]
    joints_theta[0][0] = "arm_joint_1.pos"
    joints_theta[0][1] = "arm_joint_2.pos"
    joints_theta[0][2] = "arm_joint_3.pos"
    joints_theta[0][3] = "arm_joint_4.pos"
    joints_theta[0][4] = "arm_joint_5.pos"
    joints_theta[0][5] = "time_stamp"
    
    joints_effort[0][0] = "arm_joint_1.eff"
    joints_effort[0][1] = "arm_joint_2.eff"
    joints_effort[0][2] = "arm_joint_3.eff"
    joints_effort[0][3] = "arm_joint_4.eff"
    joints_effort[0][4] = "arm_joint_5.eff"
    joints_effort[0][5] = "time_stamp"

    print("Starting trajectory session.")

    # start loop
    for i in xrange(len(jnts_trajs[0].positions)):
    
        for joint in xrange(nb_joints_ctrl):
            arm_joints.positions[joint].value = jnts_trajs[joint].positions[i]
        
        pub.publish(arm_joints)
        
        # record data
        joints_theta[i+1][5] = rospy.time.now()
        joints_effort[i+1][5] = joints_theta[i+1][5]
        for joint in xrange(5):
            joints_theta[i+1][joint] = thetas[joint]
            joints_effort[i+1][joint] = efforts[joint]
        
        rate.sleep()
    
    print("Trajectory session is over.")
    
    return (joints_theta, joints_effort)

#
def save_data(file_name, data):
    
    print("Starting to write data")
        
    myFile = open(file_name, 'w')
    with myFile:
        writer = csv.writer(myFile)
        writer.writerows(data)
    
    print("Data writting is over")
          
####################
# MAIN
####################    
 
if __name__=="__main__":

    rospy.init_node("joint_trajectory")
	
    pub_pos = rospy.Publisher('arm_1/arm_controller/position_command', JointPositions)  
    sub = rospy.Subscriber("joint_states", JointState, getJointState)
    
    base_path = os.path.dirname(os.path.abspath(__file__))
    
    # create position variable to send
    armJointPositions = JointPositions()
    pos = JointValue()
    pos.unit = "rad"
    for joint in xrange(NB_JOINTS):
        pos.joint_uri = "arm_joint_" + str(joint + 1)
        pos.value = jointsLowerLimit[joint]
        armJointPositions.positions.append(pos)  
        
    # Read Trajectories data
    trajectories = getTrajectoriesFromCSV(['/trajectory_data/sinusoidal_trajectory.csv', '/trajectory_data/triangle_trajectory.csv'])

    # initialize position
    pub_pos.publish(armJointPositions)
    
    time.sleep(2.0)

    ###############   
    ## main loop ##
    ###############
    
    trajectory_joints_names = []
    trajectory_sizes = 0
    joints_trajectories = [Trajectory()]
    
    # regroup trajectories (1 complete trajectory may contains trajectories for all
    # 5 joints, or less)
    
    traj_idx = 0

    for idx, trajectory in enumerate(trajectories): 
        
        joint_name = trajectory.joint
        trajectory_size = length(trajectory.positions)
        
        # this joint has already been define in current trajectory
        if (joint_name in trajectory_joints_names):
            ## send trajectory
            data_th, data_ef = sendTrajectory(joints_trajectories, pub_pos)
            # write data
            file_name_th = base_path + "/trajectory_data/joint_state_pos_traj_" + str(traj_idx)
            file_name_ef = base_path + "/trajectory_data/joint_state_eff_traj_" + str(traj_idx)
            save_data(file_name_th, data_th)
            save_data(file_name_ef, data_ef)
            
            ## start new trajectory
            traj_idx = traj_idx + 1
            trajectory_joints_names = []
            trajectory_sizes = 0
            joints_trajectories = [Trajectory()]
            
        # for a complete trajectory, all joints trajectories must have the same size    
        elif (trajectory_size != trajectory_sizes):
            ## send trajectory
            data_th, data_ef = sendTrajectory(joints_trajectories, pub_pos)
            # write data
            file_name_th = base_path + "/trajectory_data/joint_state_pos_traj_" + str(traj_idx)
            file_name_ef = base_path + "/trajectory_data/joint_state_eff_traj_" + str(traj_idx)
            save_data(file_name_th, data_th)
            save_data(file_name_ef, data_ef)
            
            ## start new trajectory
            traj_idx = traj_idx + 1
            trajectory_joints_names = []
            trajectory_sizes = 0
            joints_trajectories = [Trajectory()]
            
        else:
            trajectory_joints_names.append(joint_name)
            trajectory_sizes = trajectory_size
            joints_trajectories.append(trajectory)
      

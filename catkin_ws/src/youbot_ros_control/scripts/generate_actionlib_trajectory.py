#!/usr/bin/env python

import numpy as np
import math
import sys
import os
import csv

import itertools
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

import rospy
import actionlib
from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
)
from trajectory_msgs.msg import (
    JointTrajectoryPoint,
)


class Trajectory:
    def __init__(self):
        self.time_stamps = []
        self.positions = []
        self.velocities = []
        self.accelerations = []

#generate a trajectory for joint 5

def trajectoryGoalJoint5(trajectory):

    goal = FollowJointTrajectoryGoal()
    goal.trajectory.header.stamp = rospy.Time.now()
    for i in xrange(5):
        goal.trajectory.joint_names.append('arm_joint_' + str(i+1))
       
    for time, position, velocity, acceleration in itertools.izip(trajectory.time_stamps, trajectory.positions, trajectory.velocities, trajectory.accelerations):
    
        traj_point = JointTrajectoryPoint()
        
        for i in xrange(4):
            traj_point.positions.append(0.)
            traj_point.velocities.append(0.)
            traj_point.accelerations.append(0.)
         
        traj_point.positions.append(position)
        traj_point.velocities.append(velocity)
        traj_point.accelerations.append(acceleration)
        
        traj_point.time_from_start.secs = math.floor(time)
        traj_point.time_from_start.nsecs = math.floor((time%1)*1e9)
        
        goal.trajectory.points.append(traj_point) 
        
    goal.trajectory.points.append(traj_point) 
    
    return goal

if __name__=="__main__":

    """ Data Aquisition """
    
    base_path = os.path.dirname(os.path.abspath(__file__))
    print(base_path)
    trajectory_path = [base_path+'/trajectory_data/sinusoidal_trajectory.csv', base_path+'/trajectory_data/triangle_trajectory.csv'];
    trajectory_data = [Trajectory() for i in xrange(2)]

    for ii in xrange(1):
        try:
            with open(trajectory_path[ii]) as csvfile:
                readCSV = csv.reader(csvfile, delimiter=',')
                headers = next(readCSV)
                for row in readCSV:
                    trajectory_data[ii].time_stamps.append(float(row[0]))
                    trajectory_data[ii].positions.append(float(row[1]))
                    trajectory_data[ii].velocities.append(float(row[2]))
                    trajectory_data[ii].accelerations.append(float(row[3]))
                    
        except OSError as e:
            print(e)
            exit()              
    
    fig = plt.figure()
    plt.plot(trajectory_data[0].time_stamps, trajectory_data[0].positions, label='Position')
    plt.plot(trajectory_data[0].time_stamps, trajectory_data[0].velocities, label='Velocity')
    plt.plot(trajectory_data[0].time_stamps, trajectory_data[0].accelerations, label='Acceleration')
    plt.legend()
    fig.savefig(base_path + '/test.png')

    """ ROS """
    
    rospy.init_node("joint_trajectory_client")
    
    action_address = "/arm_1/arm_controller/follow_joint_trajectory"
    client = actionlib.SimpleActionClient(action_address, FollowJointTrajectoryAction)
    client.wait_for_server(timeout=rospy.Duration(10.0))
    
    goal = trajectoryGoalJoint5(trajectory_data[0])
    
    client.send_goal(goal)
    
    if client.wait_for_result(rospy.Duration.from_sec(30.0)):
        result = client.get_result()
    else:
        print('the joint angle action timed-out')
        result = client.cancel_all_goals()
    
    print(result)

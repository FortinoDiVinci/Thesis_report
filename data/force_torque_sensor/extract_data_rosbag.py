#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Tue Apr  2 09:49:00 2019

@author: Vincent
"""

#import numpy as np
import csv
import sys
import numpy as np
from matplotlib import pyplot as plt
#from matplotlib import animation as anm

def quaternionToEuler(ar):
    q1 = ar[0] #x
    q2 = ar[1] #y
    q3 = ar[2] #z
    q0 = ar[3] #w
    
    phi = np.arctan2(2*(q0*q1 + q2*q3), 1 - 2*(q1**2 + q2**2)) #RX
    theta = np.arcsin(2*(q0*q2 - q3*q1)) # Ry
    psi = np.arctan2(2*(q0*q3 + q1*q2), 1 - 2*(q2**2 + q3**2)) #Rz
    
    return [phi, theta, psi]

if len(sys.argv) < 3:
    raise OSError("Please specify 2 file pathes as script argument!") 

try:
    with open(sys.argv[1]) as csvfile:
        readCSV = csv.reader(csvfile, delimiter=',')
        stamp = []
        temp_sensor_force = []
        temp_sensor_torque = []
        next(readCSV) # jump headers
        for row in readCSV:
            stamp.append(float(row[0])/1.0e9)
            temp_sensor_force.append([float(row[4]), float(row[5]), float(row[6])])
            temp_sensor_torque.append([float(row[7]), float(row[8]), float(row[9])])
        sensor_force = np.array(temp_sensor_force).transpose()
        sensor_torque = np.array(temp_sensor_torque).transpose()
        del temp_sensor_force
        del temp_sensor_torque
        
    with open(sys.argv[2], 'rU') as csvfile:
        readCSV = csv.reader(csvfile, delimiter='\t')
        stamp2 = []
        temp_arm_pose_trans = []
        temp_arm_pose_rot = []
        temp_arm_pose_rot_euler = []
        next(readCSV)
        for row in readCSV:
            stamp2.append(float(row[0])/1.0e9)
            temp_arm_pose_trans.append([float(row[5]), float(row[6]), float(row[7])])
            temp_arm_pose_rot.append([float(row[8]), float(row[9]), float(row[10]), float(row[11])])        
        arm_pose_trans = np.array(temp_arm_pose_trans).transpose()
        #arm_pose_rot = np.array(temp_arm_pose_rot).transpose()
        del temp_arm_pose_trans
        for quaternion in temp_arm_pose_rot:
            temp_arm_pose_rot_euler.append(quaternionToEuler(quaternion))
        arm_pose_rot = np.array(temp_arm_pose_rot_euler).transpose()      

except OSError as e:
    print(e)
    exit()

offset = stamp[0]

for time_i in xrange(len(stamp)):
    stamp[time_i] = (stamp[time_i] - offset)
    
for time_i in xrange(len(stamp2)):
    stamp2[time_i] = (stamp2[time_i] - offset)

    
fig = plt.figure("Force")

plt.plot(stamp, sensor_force[0], 'b', linewidth=0.7, label='Fx')
plt.plot(stamp, sensor_force[1], 'r', linewidth=0.7, label='Fy')
plt.plot(stamp, sensor_force[2], 'g', linewidth=0.7, label='Fz')
plt.legend(loc='best')
plt.grid(axis='x', linestyle='--', linewidth=0.5)

fig = plt.figure("Torque")

plt.plot(stamp, sensor_torque[0], 'b', linewidth=0.7, label='Fx')
plt.plot(stamp, sensor_torque[1], 'r', linewidth=0.7, label='Fy')
plt.plot(stamp, sensor_torque[2], 'g', linewidth=0.7, label='Fz')
plt.legend(loc='best')
plt.grid(axis='x', linestyle='--', linewidth=0.5)

fig = plt.figure("Translation")

plt.plot(stamp2, arm_pose_trans[0], 'bx', markersize=1.0, label='x')
plt.plot(stamp2, arm_pose_trans[1], 'rx', markersize=1.0, label='y')
plt.plot(stamp2, arm_pose_trans[2], 'gx', markersize=1.0, label='z')
plt.legend(loc='best')
plt.grid(axis='x', linestyle='--', linewidth=0.5)

fig = plt.figure("Rotation")

plt.plot(stamp2, arm_pose_rot[0], 'bx', markersize=1.0, label='x')
plt.plot(stamp2, arm_pose_rot[1], 'rx', markersize=1.0, label='y')
plt.plot(stamp2, arm_pose_rot[2], 'gx', markersize=1.0, label='z')
plt.legend(loc='best')
plt.grid(axis='x', linestyle='--', linewidth=0.5)
    
"""    
offset = stamp[0]

for time_i in xrange(len(stamp)):
    stamp[time_i] = (stamp[time_i] - offset) / 10**(9)

offset = stamp2[0]

for time_i in xrange(len(stamp2)):
    stamp2[time_i] = (stamp2[time_i] - offset) / 10**(9)

print ("Echantillonnage arm tf: %f"%stamp2[2])
print ("Echantillonnage ball: %f"%stamp[1])

split_idx_0 = stamp.index(next(x for x in stamp if x > 1.85))
split_idx_1st = stamp.index(next(x for x in stamp if x > 32))
split_idx_2nd = stamp.index(next(x for x in stamp if x > 53.2))

split2_idx_0 = stamp2.index(next(x for x in stamp2 if x > 1.85))
split2_idx_1st = stamp2.index(next(x for x in stamp2 if x > 32))
split2_idx_2nd = stamp2.index(next(x for x in stamp2 if x > 53.2))

stamp_trial1 = stamp[split_idx_0:split_idx_1st]
stamp_trial2 = stamp[split_idx_2nd:]

stamp2_trial1 = stamp2[split2_idx_0:split2_idx_1st]
stamp2_trial2 = stamp2[split2_idx_2nd:]

arm_pose_trial1 = arm_pose[split2_idx_0:split2_idx_1st]
arm_pose_trial2 = arm_pose[split2_idx_2nd:]

ball_pose_trial1 = ball_pose[split_idx_0:split_idx_1st]
ball_pose_trial2 = ball_pose[split_idx_2nd:]

fig = plt.figure("arm_height and ball rosbag ses1")

plt.plot(stamp2_trial1, arm_pose_trial1, 'bx', markersize=1.0, label='arm')
plt.plot(stamp_trial1, ball_pose_trial1, 'ro', markersize=1.5, label='ball')
plt.legend(loc='best')
plt.grid(axis='x', linestyle='--', linewidth=0.5)

fig = plt.figure("arm_height and ball rosbag ses2")

plt.plot(stamp2_trial2, arm_pose_trial2, 'bx', markersize=1.0, label='arm')
plt.plot(stamp_trial2, ball_pose_trial2, 'ro', markersize=1.5, label='ball')
plt.legend(loc='best')
plt.grid(axis='x', linestyle='--', linewidth=0.5)
"""
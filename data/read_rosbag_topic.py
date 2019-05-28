# -*- coding: utf-8 -*-
"""
Spyder Editor

This is a temporary script file.
"""

import numpy as np
import csv
import sys
from matplotlib import pyplot as plt
#from matplotlib import animation as anm

if len(sys.argv) < 2:
    raise OSError("Please specify file path as script argument!") 

try:
    with open(sys.argv[1]) as csvfile:
        readCSV = csv.reader(csvfile, delimiter=',')
        stamp = []
        arm_height = []
        virtual_paddle_height = []
        ball_height = []
        real_paddle_pose = []
        force_feedback = []
        
        for row in readCSV:
            stamp.append(float(row[0]))
            arm_height.append(float(row[1])*(-1))
            virtual_paddle_height.append(row[2])
            ball_height.append(row[3])
            real_paddle_pose.append(row[4])
            force_feedback.append(row[5])

except OSError as e:
    print(e)
    exit()

offset = stamp[0]

for time_i in xrange(len(stamp)):
    stamp[time_i] = (stamp[time_i] - offset)

time_interval = stamp[2]
#time_interval = stamp[56]

print("The time step is %f ms" %(time_interval*1000))

#print(stamp)
#print(arm_height)
#print(virtual_paddle_height)
#print(ball_height)
#print(real_paddle_pose)

for x in xrange(len(ball_height)):
    if ball_height[x] == '':
        ball_height[x] = np.nan
    else:
        ball_height[x] = float(ball_height[x]) * (-1)
        
for x in xrange(len(virtual_paddle_height)):
    if virtual_paddle_height[x] == '':
        virtual_paddle_height[x] = np.nan
    else:
        virtual_paddle_height[x] = float(virtual_paddle_height[x]) * (-1)        
        
for x in xrange(len(real_paddle_pose)):
    if real_paddle_pose[x] == '':
        real_paddle_pose[x] = np.nan

fig = plt.figure("arm_height and ball")

plt.plot(stamp, arm_height, 'bx', markersize=1.0, label='arm')
plt.plot(stamp, ball_height, 'ro', markersize=1.0, label='ball')
plt.legend(loc='best')
plt.grid(axis='x', linestyle='--', linewidth=0.5)
#plt.ylim([0, 4])

fig = plt.figure("real VS virtual paddle pose")

plt.plot(stamp, arm_height, 'b', markersize=1.0, linewidth=1.50, label='height')
plt.plot(stamp, virtual_paddle_height, 'g+', markersize=1.5, label='virtual')
plt.plot(stamp, real_paddle_pose, 'r+', markersize=6.0, label='real')
plt.legend(loc='best')

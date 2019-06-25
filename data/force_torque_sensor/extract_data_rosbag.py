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
import scipy.signal as sp
#from matplotlib import animation as anm

g = 9.80665
_QUAT_MATRIX_ = True  # do not use euler angle transformation if true
                      # should be set to true !! (data error occurs otherwise)
_CHECK_FREQ_ = False 
_INTERPOLATION_ = True # interpolate data between pose and force torque if they are not synchronized
_DISP_INTERP_ = False # diplay the result of interpolation
_DISP_DATA_ = False # display data: rotation, translation, force and torque
_AVGING_DATA_ = False # averaging data slows down significantly computation
_FILTER_DATA_ = False # use low pass filter (butter) 
_LINEAR_SOLVING_ = False
_LST_SQ_SOLVING_ = True # use least square algorithm to solve
_ERR_CALC_THETA_ = False # deprecated...
_TRANSLATION_DATA_ = False # if only quaternion are given as input check False
_TORQUE_ID_ = True

def find_indices(lst, condition):
    return [i for i, elem in enumerate(lst) if condition(elem)]

# Returns a matrix whom each elem is the mean of the same elements of all
# matrices given as input
def matricesAverage(matrices):
    col_size = len(matrices[0][0])
    row_size = len(matrices[0])
    nb_matrices = len(matrices)
    
    sum_matrix = np.zeros((row_size, col_size))
    
    for matrix in matrices:
        sum_matrix = sum_matrix + matrix
        
    avg_matrix = np.multiply(sum_matrix, 1./nb_matrices)
    
    return avg_matrix

def rotationMatrix(th, axis):
    if axis == 'x' or axis == 'X':
        return np.array([[1,          0,           0],
                         [0, np.cos(th), -np.sin(th)], 
                         [0, np.sin(th),  np.cos(th)]])
    elif axis == 'y' or axis == 'Y':
        return np.array([[ np.cos(th), 0, np.sin(th)],
                         [          0, 1,          0], 
                         [-np.sin(th), 0, np.cos(th)]])
    elif axis == 'z' or axis == 'Z':
        return np.array([[ np.cos(th), -np.sin(th), 0],
                         [ np.sin(th),  np.cos(th), 0], 
                         [-np.sin(th),           0, 1]])
    else:
        print('Please specify an axis: x, y or z')
        return 0

def eulerToMatrix(angle):
    phi = angle[0]
    th = angle[1]
    psi = angle[2]
    c = np.cos
    s = np.sin
    return np.array([[c(psi)*c(phi) - s(psi)*c(th)*s(phi), -c(psi)*s(phi) - s(psi)*c(th)*c(phi),  s(psi)*s(th)],
                     [s(psi)*c(phi) + c(psi)*c(th)*s(phi), -s(psi)*s(phi) + c(psi)*c(th)*c(phi), -c(psi)*s(th)],
                     [s(th)*s(phi)                       , s(th)*c(phi)                        ,         c(th)]])

def quaternionToEuler(quaternions):
    q1 = quaternions[0] 
    q2 = quaternions[1] 
    q3 = quaternions[2] 
    q0 = quaternions[3]
    
    phi = np.arctan2(2*(q0*q1 + q2*q3), 1 - 2*(q1**2 + q2**2)) #RX
    theta = np.arcsin(2*(q0*q2 - q3*q1)) # Ry
    psi = np.arctan2(2*(q0*q3 + q1*q2), 1 - 2*(q2**2 + q3**2)) #Rz
    
    return [phi, theta, psi]    
    
def quaternionToMatrix(quaternions):
    qx = quaternions[0] 
    qy = quaternions[1] 
    qz = quaternions[2] 
    qw = quaternions[3]         

    return np.array(
        [[1 - 2*(qy**2 + qz**2),     2*(qx*qy - qz*qw),     2*(qx*qz + qy*qw)],
         [    2*(qx*qy + qz*qw), 1 - 2*(qx**2 + qz**2),     2*(qy*qz - qx*qw)], 
         [    2*(qx*qz - qy*qw),     2*(qy*qz + qx*qw), 1 - 2*(qx**2 - qy**2)]])
 
def lowPassFilter(data, freq, fs, order=10):
    
    nyq = 0.5 * fs # nyquist freq is half sampling rate
    cutoff = freq / nyq
    b, a = sp.butter(order, cutoff, btype='low', analog=False)
    filtered_data = sp.lfilter(b, a, data)
    
    return filtered_data

def lstsq(phi, y):
    
    if len(phi) != len(y):
        print("Dimension of inputs must fit")
        # should raise error...
    
    phi_ = np.array(phi)
    y_ = np.array(y)
    
    a = np.matmul(phi_.T, phi_)
    b = np.matmul(phi_.T, y_)
    
    return np.matmul(np.linalg.inv(a), b)

def hamilton_product(q1, q2):
    ret = np.array([0, 0, 0, 0])
    print(q1)
    a1 = q1[0]
    b1 = q1[1]
    c1 = q1[2]
    d1 = q1[3]
    
    a2 = q2[0]
    b2 = q2[1]
    c2 = q2[2]
    d2 = q2[3]
    
    ret[0] = a1*a2 - b1*b2 - c1*c2 - d1*d2
    ret[1] = a1*b2 + b1*a2 + c1*d2 - d1*c2
    ret[2] = a1*c2 - b1*d2 + c1*a2 + d1*b2
    ret[3] = a1*d2 + b1*c2 - c1*b2 + d1*a2
    
    return ret
    
def quat_conj(q):
    return np.array([q[0], -q[1], -q[2], q[3]])

def quat_norm(q):
    return np.sqrt(q[0]**2 + q[1]**2 + q[2]**2 + q[3]**2)

def quat_inv(q):
    return np.array(quat_conj(q)/quat_norm(q))

def quat_rotation(q, v):
    v_ = np.concatenate(([0], v), axis=None);
    print(q)
    return hamilton_product(hamilton_product(q, v_), quat_inv(q))

#####################
#       MAIN
#####################
    
plt.close('all')

if len(sys.argv) < 3:
    raise OSError("Please specify 2 file pathes as script argument!") 

try:
    with open(sys.argv[1]) as csvfile:
        readCSV = csv.reader(csvfile, delimiter=',')
        stamp = []
        temp_sensor_force = []
        temp_sensor_torque = []
        sensor_force = []
        sensor_torque = []
        next(readCSV) # jump headers
        for row in readCSV:
            stamp.append(float(row[0])/1.0e9)
            temp_sensor_force.append([np.float64(row[-6]), 
                                      np.float64(row[-5]), np.float64(row[-4])])
            temp_sensor_torque.append([np.float64(row[-3]), 
                                       np.float64(row[-2]), np.float64(row[-1])])
   
        sensor_force_unfiltered = np.array(temp_sensor_force).transpose()
        sensor_torque_unfiltered = np.array(temp_sensor_torque).transpose()
        
        if _FILTER_DATA_ :
        
            sensor_force.append(lowPassFilter(sensor_force_unfiltered[0], 20, 500))
            sensor_force.append(lowPassFilter(sensor_force_unfiltered[1], 20, 500))
            sensor_force.append(lowPassFilter(sensor_force_unfiltered[2], 20, 500))
            
            sensor_torque.append(lowPassFilter(sensor_torque_unfiltered[0], 20, 500))
            sensor_torque.append(lowPassFilter(sensor_torque_unfiltered[1], 20, 500))
            sensor_torque.append(lowPassFilter(sensor_torque_unfiltered[2], 20, 500))
        
        else:
            
            sensor_force = sensor_force_unfiltered
            sensor_torque = sensor_torque_unfiltered

        del temp_sensor_force
        del temp_sensor_torque
        
    with open(sys.argv[2], 'rU') as csvfile:
        readCSV = csv.reader(csvfile, delimiter='\t') # \t for tf #, for quat
        stamp2 = []
        temp_arm_pose_trans = []
        temp_arm_pose_rot = [] 
        temp_arm_pose_rot_euler = []
        arm_pose_rot_mat = []
        arm_pos_quat = []
        next(readCSV)
        for row in readCSV:
            stamp2.append(float(row[0])/1.0e9)
            if _TRANSLATION_DATA_:
                temp_arm_pose_trans.append([np.float64(row[5]), 
                                            np.float64(row[6]), np.float64(row[7])])
            temp_arm_pose_rot.append([np.float64(row[-4]), np.float64(row[-3]), 
                                      np.float64(row[-2]), np.float64(row[-1])])        
        if _TRANSLATION_DATA_:
            arm_pose_trans = np.array(temp_arm_pose_trans).transpose()
            arm_pose_rot = np.array(temp_arm_pose_rot).transpose()
        del temp_arm_pose_trans
        for quaternion in temp_arm_pose_rot:
            temp_arm_pose_rot_euler.append(quaternionToEuler(quaternion))
            arm_pose_rot_mat.append(quaternionToMatrix(quaternion))
            arm_pos_quat.append(quaternion)
        arm_pose_rot = np.array(temp_arm_pose_rot_euler).transpose()      

except OSError as e:
    print(e)
    exit()

offset = np.minimum(stamp[0], stamp2[2])

for time_i in xrange(len(stamp)):
    stamp[time_i] = (stamp[time_i] - offset)
    
for time_i in xrange(len(stamp2)):
    stamp2[time_i] = (stamp2[time_i] - offset)

# Checking that data frequency is 500 Hz
# Checking synchonization between force data and robot transform
if _CHECK_FREQ_:
    print(stamp[1])
    print(stamp[2]-stamp[1])
    print(stamp2[1])
    print(stamp2[2]-stamp2[1])
    
    i = find_indices(stamp, lambda e: e >= 10.5)   
    print(i[0])
    print(stamp[i[0]])
    
    i = find_indices(stamp2, lambda e: e >= 10.5)   
    print(i[0])
    print(stamp2[i[0]])

# if data is too unsynchonized, an interpolation is necessary
if _INTERPOLATION_:
    """INTERPOLATION"""
    fx = np.interp(stamp2, stamp, sensor_force[0])
    fy = np.interp(stamp2, stamp, sensor_force[1])
    fz = np.interp(stamp2, stamp, sensor_force[2])
    
    tx = np.interp(stamp2, stamp, sensor_torque[0])
    ty = np.interp(stamp2, stamp, sensor_torque[1])
    tz = np.interp(stamp2, stamp, sensor_torque[2])
    
    sensor_force_original = sensor_force
    del sensor_force
    sensor_force = np.array([fx, fy, fz])
    
    sensor_torque_original = sensor_torque
    del sensor_torque
    sensor_torque = np.array([tx, ty, tz])

    if _DISP_INTERP_:

        fig = plt.figure("Interpolation checking")
        plt.subplot(2,2,1)
        plt.plot(stamp2, sensor_force[1], 'm', linewidth=0.7, label='fy')
        plt.plot(stamp, sensor_force_original[1], 'r+', markersize=1.0, label='Fy')
        plt.title("Fy")
        plt.subplot(2,2,2)
        plt.plot(stamp2, sensor_force[0], 'c', linewidth=0.7, label='fx')
        plt.plot(stamp, sensor_force_original[0], 'b+', markersize=1.0, label='Fx')
        plt.plot(stamp2, sensor_force[2], 'y', linewidth=0.7, label='fz')
        plt.plot(stamp, sensor_force_original[2], 'g+', markersize=1.0, label='Fz')
        plt.title("Fx, Fz")    
        plt.subplot(2,2,3)
        plt.plot(stamp2, sensor_torque[1], 'm', linewidth=0.7, label='ty')
        plt.plot(stamp, sensor_torque_original[1], 'r+', markersize=1.0, label='Ty')
        plt.title("Ty")
        plt.subplot(2,2,4)
        plt.plot(stamp2, sensor_torque[0], 'c', linewidth=0.7, label='tx')
        plt.plot(stamp, sensor_torque_original[0], 'b+', markersize=1.0, label='Tx')
        plt.plot(stamp2, sensor_torque[2], 'y', linewidth=0.7, label='tz')
        plt.plot(stamp, sensor_torque_original[2], 'g+', markersize=1.0, label='Tz')
        plt.title("Tx, Tz") 

    del stamp
    stamp = stamp2

if _DISP_DATA_:
    
    fig = plt.figure("ATI 6 axis force torque sensor data")
    
    plt.subplot(2,2,1)
    plt.plot(stamp, sensor_force[0], 'b', linewidth=0.7, label='Fx')
    plt.plot(stamp, sensor_force[1], 'r', linewidth=0.7, label='Fy')
    plt.plot(stamp, sensor_force[2], 'g', linewidth=0.7, label='Fz')
    #plt.legend(loc='best')
    plt.grid(axis='x', linestyle='--', linewidth=0.5)
    plt.title("Force")
    
    plt.subplot(2,2,2)
    plt.plot(stamp, sensor_torque[0], 'b', linewidth=0.7, label='Fx')
    plt.plot(stamp, sensor_torque[1], 'r', linewidth=0.7, label='Fy')
    plt.plot(stamp, sensor_torque[2], 'g', linewidth=0.7, label='Fz')
    #plt.legend(loc='best')
    plt.grid(axis='x', linestyle='--', linewidth=0.5)
    plt.title("Torque")
    
    if _TRANSLATION_DATA_:
            
        plt.subplot(2,2,3)
        plt.plot(stamp2, arm_pose_trans[0], 'bx', markersize=1.0, label='x')
        plt.plot(stamp2, arm_pose_trans[1], 'rx', markersize=1.0, label='y')
        plt.plot(stamp2, arm_pose_trans[2], 'gx', markersize=1.0, label='z')
        #plt.legend(loc='best')
        plt.grid(axis='x', linestyle='--', linewidth=0.5)
        plt.title("Translation")
    
    plt.subplot(2,2,4)
    plt.plot(stamp2, arm_pose_rot[0], 'bx', markersize=1.0, label='x')
    plt.plot(stamp2, arm_pose_rot[1], 'rx', markersize=1.0, label='y')
    plt.plot(stamp2, arm_pose_rot[2], 'gx', markersize=1.0, label='z')
    plt.legend(loc='best')
    plt.grid(axis='x', linestyle='--', linewidth=0.5)
    plt.title("Rotation")


f = 500. # frequency in Hz

# value are averaged on a time interval of 1 sec  
#seconds = [1, 10.5, 17, 20.5, 25.5, 30, 36.5, 50] # manually choosen values

if _AVGING_DATA_ :
    intv = 10.
    seconds = np.arange((intv)/(2), (len(stamp) - intv/2), intv)
    print("number of systems: %s" %(f * (seconds[-1] - seconds[0]) / intv) )
else :
    seconds = np.arange(0, len(stamp))
    print("number of systems: %s" %len(stamp))

data_force = []
data_torque = []
data_trans = []
data_rot = []  
Rpsi = []
Rth = []
Rphi = []
Rtot = []
Rtot_qu = []

for sec in seconds:
    
    data_1_force = []
    data_1_torque = []
    data_1_trans = []
    data_1_rot = []
    
    for i in xrange(3):
        if _AVGING_DATA_ :
            data_1_force.append(np.mean(
                    sensor_force[i][int((1./2)*sec):int((3./2)*sec)]))
            data_1_torque.append(np.mean(
                    sensor_torque[i][int((1./2)*sec):int((3./2)*sec)]))
            if _TRANSLATION_DATA_:
                data_1_trans.append(np.mean(
                        arm_pose_trans[i][int((1./2)*sec):int((3./2)*sec)]))
            data_1_rot.append(np.mean(
                    arm_pose_rot[i][int((1./2)*sec):int((3./2)*sec)]))
        
        else:
            data_1_force.append(sensor_force[i][sec])
            data_1_torque.append(sensor_torque[i][sec])
            if _TRANSLATION_DATA_:
                data_1_trans.append(arm_pose_trans[i][sec])
            data_1_rot.append(arm_pose_rot[i][sec])

    data_force.append(data_1_force)
    data_torque.append(data_1_torque)
    if _TRANSLATION_DATA_:
        data_trans.append(data_1_trans)
    data_rot.append(data_1_rot)
 
    #Rpsi.append(rotationMatrix(data_1_rot[0], 'z'))
    #Rth.append(rotationMatrix(data_1_rot[1], 'x'))
    #Rphi.append(rotationMatrix(data_1_rot[2], 'z'))
    
    Rtot.append(eulerToMatrix(data_1_rot)) 
    
    if _AVGING_DATA_ :
        Rtot_qu.append(matricesAverage(
            arm_pose_rot_mat[int((1./2)*sec):int((3./2)*sec)])) # directly from quat
    else:
        Rtot_qu.append(arm_pose_rot_mat[sec])

#for i in xrange(len(seconds)):
#    Rtot.append(np.matmul(Rphi[i], np.matmul(Rth[i], Rpsi[i])))   

"""
# Total rotation and translation matrix normalized
Trans = np.array(data_trans[0])[np.newaxis].T # transpose

Ttot = np.concatenate((np.concatenate((Rtot, Trans), axis=1), 
                       [[0, 0, 0, 1]]), axis=0)
"""

# An example of a system
# Rtot^T * [0, 0, -mg]^T + [xB, yB, zB]^T = [fx, fy, fz]^T 

#print(Rtot_qu[0])
#print(Rtot[0])

#########
# / ! \ #
#########

# Rtot should give the same results as Rtot_qu, but considerable
# differences occur... Rtot_qu is closer to expected data !!

if _QUAT_MATRIX_:
    ROT = Rtot_qu
else:
    ROT = Rtot  

if _LINEAR_SOLVING_: 
    """LINEAR SOLVING USING 4 EQUATIONS"""
    # x systems, each of them have 3 equations
    # to solves the 4 unknowns, 3 equ. are taken from a system, and the third one
    # of an other system is used as the fourth equation
    # This approach gives x * (x-1) possibles combinations, therefore results
    # will be averaged
    
    x = []
    count = 0
    for i in xrange(len(ROT)):
        for j in xrange(len(ROT)):
            if i == j:
                count += 1
                continue
            if np.abs(ROT[i][2][2] - ROT[j][2][2]) < 1.0e-4: # avoid this cases
                # equations that are too close will bring weird results
                count += 1
                continue
            a = np.concatenate((
                    np.concatenate((np.eye(3), ROT[i][2][np.newaxis].T*(-g)), axis=1),
                    [[0, 0, 1, ROT[j][2][2]*(-g)]]), axis=0)
            b = np.concatenate((data_force[i], data_force[j][2]), axis=None)
            try:
                x.append(np.linalg.solve(a, b))
            except:
                count += 1
                print("error at (%s, %s), the equations provided "  
                      "are not independant" %(i,j))
    
    print("Number of linear systems: %d" %(len(ROT)*(len(ROT) - 1) - count))
            
    sum_x = np.sum(x, axis = 0)
    mean_x = sum_x / len(x)
    std_dev = np.std(x, axis = 0)
    print("############################\n"
          "#LINEAR SOLVING COMPUTATION#\n"
          "############################")
    print("Result:\nbx = %f\nby = %f\nbz = %f\nm = %f" % (mean_x[0], mean_x[1], mean_x[2], mean_x[3])) 
    print("Standard deviation:\nbx = %f\nby = %f\nbz = %f\nm = %f" % (std_dev[0], std_dev[1], std_dev[2], std_dev[3]))

if _LST_SQ_SOLVING_:    
    """LINEAR SOLVING USING LEAST SQUARE"""
    # Each system has 3 equations
    # We have a total of 4 unknowns
    # The number of system is defined by the list seconds
               
    a_ls = np.concatenate((np.eye(3), ROT[0][2][np.newaxis].T*(-g)), axis=1)  
    b_ls = data_force[0]       
    for i in xrange(1, len(ROT)):
        a_ls = np.concatenate((a_ls,
                np.concatenate((np.eye(3), ROT[i][2][np.newaxis].T*(-g)), axis=1)),
                axis=0)
        b_ls = np.concatenate((b_ls, data_force[i]), axis=None) 
          
    x_ls = np.linalg.lstsq(a_ls, b_ls)
    
    
    er_x = []
    er_y = []
    er_z = []

    xb = x_ls[0][0]
    yb = x_ls[0][1]
    zb = x_ls[0][2]
    m = x_ls[0][3]
    
    for i, Ri in enumerate(ROT):

        fx = data_force[i][0]
        fy = data_force[i][1]
        fz = data_force[i][2]
        er_x.append(xb + Ri[2][0] * (-m*g) - fx) # Ri[2][0] <=> Ri.T[0][2]
        er_y.append(yb + Ri[2][1] * (-m*g) - fy)
        er_z.append(zb + Ri[2][2] * (-m*g) - fz)
     
    mean_er = []
    std_er = []    
        
    mean_er.append(np.mean(er_x))
    mean_er.append(np.mean(er_y))
    mean_er.append(np.mean(er_z))
    
    std_er.append(np.std(er_x, axis = 0))
    std_er.append(np.std(er_y, axis = 0))
    std_er.append(np.std(er_z, axis = 0))
    print("##########################\n"
          "#LEAST SQUARE COMPUTATION#\n"
          "##########################")
    print("Result:\nbx = %f\nby = %f\nbz = %f\nm = %f" % (xb, yb, zb, m)) 
    print("Standard deviation:\nfx = %f\nfy = %f\nfz = %f" % (std_er[0], std_er[1], std_er[2]))
    print("Gaussian mean:\nfx = %f\nfy = %f\nfz = %f" % (mean_er[0], mean_er[1], mean_er[2]))

    #print(x_ls)

    f_sol = np.matmul(a_ls, x_ls[0])
    fx_lst = []
    fy_lst = []
    fz_lst = []
    
    for i, obj in enumerate(f_sol):
        if i%3 == 0:
            fx_lst.append(obj)
        elif i%3 == 1:
            fy_lst.append(obj)
        else:
            fz_lst.append(obj)
    
    tdata_force = np.array(data_force).T
    
    fig = plt.figure("ATI 6 axis F/T sensor force data VS estimation")
    plt.subplot(2,2,1)
    plt.plot(seconds, tdata_force[0], 'b+',  markersize=1.0, label='Fx')
    plt.plot(seconds, fx_lst, 'c+', markersize=1.0, label='fx_ls')
    plt.title("Fx vs fx_ls")
    plt.grid(axis='x', linestyle='--', linewidth=0.5)
    plt.subplot(2,2,2)
    plt.plot(seconds, tdata_force[1], 'r+', linewidth=0.7, markersize=1.0, label='Fy')
    plt.title("Fy vs fy_ls")
    plt.plot(seconds, fy_lst, 'm+', markersize=1.0, label='fy_ls')
    plt.grid(axis='x', linestyle='--', linewidth=0.5)
    plt.subplot(2,2,3)
    plt.plot(seconds, tdata_force[2], 'g+', linewidth=0.7, markersize=1.0, label='Fz')
    plt.plot(seconds, fz_lst, 'y+', markersize=1.0, label='fz_ls')
    plt.title("Fz vs fz_ls")
    plt.grid(axis='x', linestyle='--', linewidth=0.5)


# This section is out of date...
if _ERR_CALC_THETA_:
    
    err = -15 * np.pi / 180
    m_ = 0.182# kg
    total_err = 0
    
    for rot in ROT:
        err_x = rotationMatrix(err, 'x')
        err_y = rotationMatrix(err, 'y')
        err_xy = np.matmul(err_x, err_y)
        rot_err = np.array(np.matmul(err_xy, rot))
        
        err_force = np.matmul(rot_err.T, [[0], [0], [-m_*g]])
        real_force = np.matmul(rot.T, [[0], [0], [-m_*g]])
        
        total_err += abs(real_force - err_force)
    
    mean_err = total_err / len(ROT)
    
    print("Mean error is\nx: %f\ny: %f\nz:%f" %(mean_err[0], mean_err[1], mean_err[2]))


if _TORQUE_ID_:

    if _LST_SQ_SOLVING_:    
        """LINEAR SOLVING USING LEAST SQUARE"""
        # Each system has 3 equations
        # We have a total of 4 unknowns (3 bias and l)
        # The number of system is defined by the list "seconds"
        
        # xg, yg are necessary for the cross product between rot and the weight
        
        xg = ROT[0][2][0] * (-m*g) # <=> ROT[i].T[0][2]
        yg = ROT[0][2][1] * (-m*g)
           
        at_ls = np.concatenate((np.eye(3), [[yg], [-xg], [0]]), axis=1)  
        bt_ls = data_torque[0]       
        for i in xrange(1, len(ROT)):
            xg = ROT[i][2][0] * (-m*g) # <=> ROT[i].T[0][2]
            yg = ROT[i][2][1] * (-m*g)
            #zg = ROT[i][2][2] * (-m*g)
         
            at_ls = np.concatenate((at_ls,
                    np.concatenate((np.eye(3), [[yg], [-xg], [0]]), axis=1)),
                    axis=0)
            bt_ls = np.concatenate((bt_ls, data_torque[i]), axis=None) 
              
        xt_ls = np.linalg.lstsq(at_ls, bt_ls)
        
        er_tx = []
        er_ty = []
        er_tz = []
        
        xtb = xt_ls[0][0]
        ytb = xt_ls[0][1]
        ztb = xt_ls[0][2]
        l = -xt_ls[0][3]  # there must be an error smwh because the result
                          # as a negative sign (but the right abs value)
        
        for i, Ri in enumerate(ROT):
            tx = data_torque[i][0]
            ty = data_torque[i][1]
            tz = data_torque[i][2]
            er_tx.append(xtb + Ri[2][0] * l - tx)
            er_ty.append(ytb - Ri[2][1] * l - ty)
            er_tz.append(ztb + 0. - tz)
         
        mean_er_t = []
        std_er_t = []    
            
        mean_er_t.append(np.mean(er_tx))
        mean_er_t.append(np.mean(er_ty))
        mean_er_t.append(np.mean(er_tz))
        
        std_er_t.append(np.std(er_tx, axis = 0))
        std_er_t.append(np.std(er_ty, axis = 0))
        std_er_t.append(np.std(er_tz, axis = 0))
        print("##########################\n"
              "#LEAST SQUARE COMPUTATION#\n"
              "##########################")
        print("Result:\nbtx = %f\nbty = %f\nbtz = %f\nl = %f" % (xtb, ytb, ztb, l)) 
        print("Standard deviation:\ntx = %f\nty = %f\ntz = %f" % (std_er_t[0], std_er_t[1], std_er_t[2]))
        print("Gaussian mean:\ntx = %f\nty = %f\ntz = %f" % (mean_er_t[0], mean_er_t[1], mean_er_t[2]))
    
        #print(x_ls)
    
        t = np.matmul(at_ls, xt_ls[0])
        tx_lst = []
        ty_lst = []
        tz_lst = []
        
        for i, obj in enumerate(t):
            if i%3 == 0:
                tx_lst.append(obj)
            elif i%3 == 1:
                ty_lst.append(obj)
            else:
                tz_lst.append(obj)
                
        tdata_torque = np.array(data_torque).T
        
        fig = plt.figure("ATI 6 axis F/T sensor torque data VS estimation")
        plt.subplot(2,2,1)
        plt.plot(seconds, tdata_torque[0], 'b',  linewidth=0.7, label='tx')
        plt.plot(seconds, tx_lst, 'c+', markersize=1.0, label='tx_lst')
        plt.title("tx vs tx_lst")
        plt.grid(axis='x', linestyle='--', linewidth=0.5)
        plt.subplot(2,2,2)
        plt.plot(seconds, tdata_torque[1], 'r', linewidth=0.7, label='ty')
        plt.plot(seconds, ty_lst, 'm+', markersize=1.0, label='ty_lst')
        plt.title("ty vs ty_lst")
        plt.grid(axis='x', linestyle='--', linewidth=0.5)
        plt.subplot(2,2,3)
        plt.plot(seconds, tdata_torque[2], 'g', linewidth=0.7, label='tz')
        plt.plot(seconds, tz_lst, 'y+', markersize=1.0, label='tz_lst')
        plt.title("tz vs tz_lst")
        plt.grid(axis='x', linestyle='--', linewidth=0.5)    

      
"""
row = ['fx', 'fy', 'fz'] 
row2 = ['a1', 'b1', 'c1', 'd1']
row3 = ["qx", "qy", "qz", "qw"]  


for i in xrange(len(tdata_force[0])):
    row.append([tdata_force[0][i], tdata_force[1][i], tdata_force[2][i]])

for i in (a_ls):
    row2.append([i[0], i[1], i[2], i[3]])
    
for i in (arm_pos_quat):
    row3.append([i[0], i[1], i[2], i[3]])

with open('LST_DATA_force', 'w') as csvFile:
    writer = csv.writer(csvFile)
    writer.writerows(row)
    
csvFile.close()
  
with open('LST_DATA_phi', 'w') as csvFile:
    writer = csv.writer(csvFile)
    writer.writerows(row2)

csvFile.close()

with open('quaternion_data', 'w') as csvFile:
    writer = csv.writer(csvFile)
    writer.writerows(row3)

csvFile.close()
""" 
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
#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np

# This forward kinematics model of the youbot (KUKA) was computed by 
# Maria MAKAROV (L2S - CentraleSupélec) using Matlab(R) and converted 
# to python by Vincent FORTINEAU (L2S - CentraleSupélec & Université 
# Paris-Sud) with minor modifications
# execution time estimated around 29.99us +/- 0.005us 

def forward_kinematic(th):

    th1 = th[0]
    th2 = th[1]
    th3 = th[2]
    th4 = th[3]
    th5 = th[4]
    
    sin = np.sin
    cos = np.cos
    pi = np.pi
        
    t2 = pi*(5.0/7.2e1)
    t3 = t2 + th5
    t4 = pi*(1.1e1/1.8e2)
    t5 = t4 + th1
    t6 = pi*(5.0/3.6e1)
    t7 = t6 + th2
    t8 = sin(t5)
    t9 = pi*(1.4e1/4.5e1)
    t10 = t9 + th3
    t11 = cos(t7)
    t12 = cos(t5)
    t13 = sin(t7)
    t14 = pi*(3.1e1/7.2e1)
    t15 = t14 + th4
    t16 = cos(t10)
    t17 = 0.0 #t8*t11*6.123233995736766e-17
    t18 = t12*t13
    t19 = t17 + t18
    t20 = sin(t10)
    t21 = 0.0 #t8*t13*6.123233995736766e-17
    t26 = t11*t12
    t22 = t21 - t26
    t23 = sin(t3)
    t24 = cos(t15)
    t25 = t16*t19
    t33 = t20*t22
    t27 = t25 - t33
    t28 = sin(t15)
    t29 = t16*t22
    t30 = t19*t20
    t31 = t29 + t30
    t32 = cos(t3)
    t34 = t24*t27
    t38 = t28*t31
    t35 = t34 - t38
    t36 = t24*t31
    t37 = t27*t28
    t39 = t8*t13
    t44 = 0.0 #t11*t12.*6.123233995736766e-17
    t40 = t39 - t44
    t41 = t8*t11
    t42 = 0.0 #t12*t13.*6.123233995736766e-17
    t43 = t41 + t42
    t45 = t16*t40
    t46 = t20*t43
    t47 = t45 + t46
    t48 = t16*t43
    t51 = t20*t40
    t49 = t48 - t51
    t50 = t24*t47
    t52 = t28*t49
    t53 = t24*t49
    t54 = t53 - t28*t47;
    t57 = pi*(1.0/2.0e1)
    t55 = -t57 + th2 + th3 + th4 + th5
    t56 = th2 + th3 + th4 - th5 - pi*(1.7e1/9.0e1)
    t58 = th2 + th3 + th4 - pi*(4.3e1/3.6e2)
    t59 = cos(t58)

    R11 = t32*(t36 + t37) - t8*t23 #+ t23*t35*6.123233995736766e-17
    R12 = t23*(t36 + t37) + t8*t32 #- t32*t35*6.123233995736766e-17
    R13 = t34 - t38 #+ t8*6.123233995736766e-17
    R21 = -t12*t23 + t32*t54 #+ t23*(t50 + t52)*(-6.123233995736766e-17) 
    R22 = t12*t32 + t23*t54 #+ t32*(t50 + t52)*6.123233995736766e-17
    R23 = -t50 - t52 #+ t12*6.123233995736766e-17
    R31 = sin(t55)*5.0e-1 + sin(t56)*5.0e-1 #+ t23*(-6.123233995736766e-17)
    R32 = -cos(t55)*5.0e-1 + cos(t56)*5.0e-1 #+ t32*6.123233995736766e-17 
    R33 = t59 #+ 3.749399456654644e-33
    tx = -t12*(3.3e1/1.0e3) + t11*t12*(3.1e1/2.0e2) - t16*t19*(2.7e1/2.0e2) + t24*t27*(1.09e2/5.0e2) - t28*t31*(1.09e2/5.0e2) + t20*(t21 - t26)*(2.7e1/2.0e2) #+ t8*1.334865011070615e-17 - t8*t13*9.491012693391987e-18 
    ty = t8*(3.3e1/1.0e3) - t8*t11*(3.1e1/2.0e2) + t20*t43*(2.7e1/2.0e2) - t24*t47*(1.09e2/5.0e2) - t28*t49*(1.09e2/5.0e2)+t16*(t39 - t44)*(2.7e1/2.0e2) #+ t12*1.334865011070615e-17 - t12*t13*9.491012693391987e-18
    tz = t13*(3.1e1/2.0e2) + t59*(1.09e2/5.0e2) - sin(-t57 + th2 + th3)*(2.7e1/2.0e2) + 1.47e-1

    return np.array([[R11, R12, R13, tx],
		     [R21, R22, R23, ty],
		     [R31, R32, R33, tz],
		     [0.0, 0.0, 0.0, 1.]])		     
		     
if __name__ == "__main__":

    th_KUKA = np.array([0, 0, 0, 0, 0])
    
    print(forward_kinematic(th_KUKA))


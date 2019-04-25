#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np

# This forward kinematics model of the youbot (KUKA) was computed by Maria MAKAROV (L2S - CentraleSupélec) using Matlab(R) and converted to python by Vincent FORTINEAU (L2S - CentraleSupélec & Université Paris-Sud)

def forward_kinematic(th):

    th1 = th[0]
    th2 = th[1]
    th3 = th[2]
    th4 = th[3]
    th5 = th[4]
    
    d2 = 0.033
    d3 = 0.155
    d4 = 0.135
    lend = 0.218
    r1 = 0.147

    sin = np.sin
    cos = np.cos

    t2 = th1 + th2 + th3 + th4 + th5
    t3 = th1 - th5
    t4 = th1 + th2 + th3 + th4 - th5
    t5 = -th1 + th2 + th3 + th4 + th5
    t6 = th1 + th5
    t7 = -th1 + th2 + th3 + th4 - th5
    t8 = th1 + th2 + th3 + th4
    t9 = sin(t8)
    t10 = sin(th1)
    t11 = -th1 + th2 + th3 + th4
    t12 = sin(t11)
    t13 = sin(t2)
    t14 = t13 * 2.5e-1
    t15 = sin(t3)
    t16 = t15 * (1.0/2.0)
    t17 = sin(t4)
    t18 = sin(t5)
    t19 = t18*2.5e-1
    t20 = sin(t6)
    t21 = t20*(1.0/2.0)
    t22 = sin(t7)
    t23 = cos(t2)
    t24 = t23*2.5e-1
    t25 = cos(t3)
    t26 = cos(t4)
    t27 = t26*2.5e-1
    t28 = cos(t5)
    t29 = t28*2.5e-1
    t30 = cos(t6)
    t31 = t30*(1.0/2.0)
    t32 = cos(t7)
    t33 = t32*2.5e-1
    t34 = cos(th1)
    t35 = -th1 + th2 + th3
    t36 = cos(t8)
    t37 = th1 + th2
    t38 = cos(t11)
    t39 = th1 - th2
    t40 = th1 + th2 + th3
    t41 = th2 + th3 + th4 + th5
    t42 = th2 + th3 + th4 - th5
    t43 = th2 + th3 + th4
    t44 = cos(t43)

    r00 = t24 - t25*(1.0/2.0) + t27 + t29 + t31 + t33
    r10 = -t14 + t16 - t17*2.5e-1 + t19 - t21+ t22*2.5e-1
    r20 = sin(t41)*(-5.0e-1) - sin(t42)*5.0e-1 + sin(th5)*6.123233995736766e-17
    r01 = t14 + t16 - t17*2.5e-1 + t19 + t21 - t22*2.5e-1
    r11 = t24 + t25*(1.0/2.0) - t27 - t29 + t31 + t33
    r21 = cos(t41)*5.0e-1 - cos(t42)*5.0e-1 - cos(th5)*6.123233995736766e-17
    r02 = t9*5.0e-1 - t10*6.123233995736766e-17 + t12*5.0e-1
    r12 = t34*(-6.123233995736766e-17) + t36*5.0e-1 - t38*5.0e-1
    r22 = t44 + 3.749399456654644e-33
    
    tx = d2*t34 + lend*t9*5.0e-1 - lend*t10*6.123233995736766e-17 + lend*t12*5.0e-1 + d4*sin(t35)*5.0e-1 + d3*sin(t37)*5.0e-1 - d3*sin(t39)*5.0e-1 + d4*sin(t40)*5.0e-1
    ty = -d2*t10 - lend*t34*6.123233995736766e-17 + lend*t36*5.0e-1 - lend*t38*5.0e-1 - d4*cos(t35)*5.0e-1 + d3*cos(t37)*5.0e-1 - d3*cos(t39)*5.0e-1 + d4*cos(t40)*5.0e-1
    tz = lend*3.749399456654644e-33 + r1 + lend*t44 + d4*cos(th2 + th3) + d3*cos(th2)
    
    return np.array([[r00, r10, r20, tx],
                     [r01, r11, r21, ty],
                     [r02, r12, r22, tz],
                     [  0,   0,   0,  1]])
    

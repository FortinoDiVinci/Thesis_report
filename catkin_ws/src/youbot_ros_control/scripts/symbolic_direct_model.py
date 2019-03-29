#!/usr/bin/env python

import tf 
import numpy as np
import sympy as sp

"""
def matlab_mgd(d2,d3,d4,lend,r1,th1,th2,th3,th4,th5):

    t2 = th1+th2+th3+th4+th5
    t3 = th1-th5
    t4 = th1+th2+th3+th4-th5
    t5 = -th1+th2+th3+th4+th5
    t6 = th1+th5
    t7 = -th1+th2+th3+th4-th5
    t8 = th1+th2+th3+th4
    t9 = np.sin(t8)
    t10 = np.sin(th1)
    t11 = -th1+th2+th3+th4
    t12 = np.sin(t11)
    t13 = np.sin(t2)
    t14 = t13.*2.5e-1
    t15 = np.sin(t3)
    t16 = t15.*(1.0./2.0)
    t17 = np.sin(t4)
    t18 = np.sin(t5)
    t19 = t18.*2.5e-1
    t20 = np.sin(t6)
    t21 = t20.*(1.0./2.0)
    t22 = np.sin(t7)
    t23 = np.cos(t2)
    t24 = t23.*2.5e-1
    t25 = np.cos(t3)
    t26 = np.cos(t4)
    t27 = t26.*2.5e-1
    t28 = np.cos(t5)
    t29 = t28.*2.5e-1
    t30 = np.cos(t6)
    t31 = t30.*(1.0./2.0)
    t32 = np.cos(t7)
    t33 = t32.*2.5e-1
    t34 = np.cos(th1)
    t35 = -th1+th2+th3
    t36 = np.cos(t8)
    t37 = th1+th2
    t38 = np.cos(t11)
    t39 = th1-th2
    t40 = th1+th2+th3
    t41 = th2+th3+th4+th5
    t42 = th2+th3+th4-th5
    t43 = th2+th3+th4
    t44 = np.cos(t43)
    
    return [t24-t25*(1.0/2.0)+t27+t29+t31+t33, -t14+t16-t17*2.5e-1+t19-t21+t22*2.5e-1, np.sin(t41)*(-5.0e-1)-sin(t42).*5.0e-1+sin(th5).*6.123233995736766e-17,0.0,t14+t16-t17.*2.5e-1+t19+t21-t22.*2.5e-1,t24+t25.*(1.0./2.0)-t27-t29+t31+t33,cos(t41).*5.0e-1-cos(t42).*5.0e-1-cos(th5).*6.123233995736766e-17,0.0,t9.*5.0e-1-t10.*6.123233995736766e-17+t12.*5.0e-1,t34.*(-6.123233995736766e-17)+t36.*5.0e-1-t38.*5.0e-1,t44+3.749399456654644e-33,0.0,d2.*t34+lend.*t9.*5.0e-1-lend.*t10.*6.123233995736766e-17+lend.*t12.*5.0e-1+d4.*sin(t35).*5.0e-1+d3.*sin(t37).*5.0e-1-d3.*sin(t39).*5.0e-1+d4.*sin(t40).*5.0e-1,-d2.*t10-lend.*t34.*6.123233995736766e-17+lend.*t36.*5.0e-1-lend.*t38.*5.0e-1-d4.*cos(t35).*5.0e-1+d3.*cos(t37).*5.0e-1-d3.*cos(t39).*5.0e-1+d4.*cos(t40).*5.0e-1,lend.*3.749399456654644e-33+r1+lend.*t44+d4.*cos(th2+th3)+d3.*cos(th2),1.0].reshape((4, 4));
"""

def transformMatrixSymbolic(alpha, a, theta, d):
    
    r11 = sp.cos(theta)
    r12 = -sp.sin(theta)
    r13 = 0
    tx = a
    r21 = sp.sin(theta) * sp.cos (alpha)
    r22 = sp.cos(theta) * sp.cos (alpha)
    r23 = -sp.sin(alpha)
    ty = -sp.sin(alpha) * d
    r31 = sp.sin(theta) * sp.sin(alpha)
    r32 = sp.cos(theta) * sp.sin(alpha)
    r33 = sp.cos(alpha)
    tz = sp.cos(alpha) * d
    
    tf_mat = sp.Matrix([[r11, r12, r13, tx],
                     [r21, r22, r23, ty],
                     [r31, r32, r33, tz],
                     [ 0 ,  0 ,  0 , 1 ]])
    
    return tf_mat

def transformMatrix(alpha, a, theta, d):
    
    r11 = np.cos(theta)
    r12 = -np.sin(theta)
    r13 = 0
    tx = a
    r21 = np.sin(theta) * np.cos(alpha)
    r22 = np.cos(theta) * np.cos(alpha)
    r23 = -np.sin(alpha)
    ty = -np.sin(alpha) * d
    r31 = np.sin(theta) * np.sin(alpha)
    r32 = np.cos(theta) * np.sin(alpha)
    r33 = np.cos(alpha)
    tz = np.cos(alpha) * d
    
    tf_mat = np.array([[r11, r12, r13, tx],
                     [r21, r22, r23, ty],
                     [r31, r32, r33, tz],
                     [ 0 ,  0 ,  0 , 1 ]])
    
    return tf_mat


symbolic = False


if __name__=="__main__":

    if symbolic == True:
    
        th1, th2, th3, th4, th5 = sp.symbols('th1 th2 th3 th4 th5')
        r1, d2, d3, d4, lend = sp.symbols('r1 d2 d3 d4 lend')
        
        th = [th1, th2+sp.pi/2, th3, th4-sp.pi/2, th5]
        
        T01 = transformMatrixSymbolic(0, 0, th[0], r1)
        T12 = transformMatrixSymbolic(sp.pi/2, d2, th[1], 0)
        T23 = transformMatrixSymbolic(0, d3, th[2], 0)
        T34 = transformMatrixSymbolic(0, d4, th[3], 0)
        T45 = transformMatrixSymbolic(-sp.pi/2, 0, th[4], 0)
        #T5end = sp.eye(4) + sp.Matrix([[0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, lend], [0, 0, 0, 0]])
        
        T = []
        T.extend([T01, T12, T23, T34, T45])#, T5end])

        Tend = sp.eye(4) # identity matrix
        
        for t in T:
            Tend = Tend * t
        
        T05_end = sp.simplify(Tend)
             
        T05_end = T05_end.subs([(r1, 0.147), (d2, 0.033), (d3, 0.155), (d4, 0.135), (lend, 0.218)])

        fast_mgd = sp.lambdify((th1, th2, th3, th4, th5), T05_end)
        
        print(fast_mgd(np.pi/2, 0, np.pi/2, 0, 0))
        
    else:
        
        DH = {
            'r1': 0.147,
            'd2': 0.033,
            'd3': 0.155,
            'd4': 0.135,
            'lend': 0.218,
        }
        
        th = [np.pi/2., 0., np.pi/2., 0., 0.]
        
        T01 = transformMatrix(0, 0, th[0], DH['r1'])
        T12 = transformMatrix(sp.pi/2, DH['d2'], th[1], 0)
        T23 = transformMatrix(0, DH['d3'], th[2], 0)
        T34 = transformMatrix(0, DH['d4'], th[3], 0)
        T45 = transformMatrix(-sp.pi/2, 0, th[4], 0)
        
        T = []
        T.extend([T01, T12, T23, T34, T45])
        
        Tend = np.identity(4)
        
        for t in T:
            Tend = np.matmul(Tend, t)
        
        print(Tend)
        
        
        
        
        
        
        
        
        
        
        

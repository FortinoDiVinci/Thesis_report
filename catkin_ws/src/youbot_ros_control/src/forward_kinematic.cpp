//
//  forward_kinematic.cpp
//  
//
//  Created by Vincent Fortineau on 14/05/2019.
//
//

#include "youbot_ros_control/forward_kinematic.h"

/*
# This forward kinematics model of the youbot (KUKA) was computed by
# Maria MAKAROV (L2S - CentraleSupélec) using Matlab(R) and converted
# to python by Vincent FORTINEAU (L2S - CentraleSupélec & Université
# Paris-Sud) with minor modifications
# execution time estimated around 29.99us +/- 0.005us
# C++ code created to increase execution time of the main function
 */

void forward_kinematic(const float th[5], float rot_matrix[][4]) {

    float th1 = th[0];
    float th2 = th[1];
    float th3 = th[2];
    float th4 = th[3];
    float th5 = th[4];

    float t2 = M_PI*(5.0/7.2e1);
    float t3 = t2 + th5;
    float t4 = M_PI*(1.1e1/1.8e2);
    float t5 = t4 + th1;
    float t6 = M_PI*(5.0/3.6e1);
    float t7 = t6 + th2;
    float t8 = sin(t5);
    float t9 = M_PI*(1.4e1/4.5e1);
    float t10 = t9 + th3;
    float t11 = cos(t7);
    float t12 = cos(t5);
    float t13 = sin(t7);
    float t14 = M_PI*(3.1e1/7.2e1);
    float t15 = t14 + th4;
    float t16 = cos(t10);
    //float t17 = 0.0; // t8*t11*6.123233995736766e-17;
    float t18 = t12*t13;
    float t19 = t18; //+ t17
    float t20 = sin(t10);
    //float t21 = 0.0; //t8*t13*6.123233995736766e-17;
    float t26 = t11*t12;
    float t22 = -t26; //+ t21
    float t23 = sin(t3);
    float t24 = cos(t15);
    float t25 = t16*t19;
    float t33 = t20*t22;
    float t27 = t25 - t33;
    float t28 = sin(t15);
    float t29 = t16*t22;
    float t30 = t19*t20;
    float t31 = t29 + t30;
    float t32 = cos(t3);
    float t34 = t24*t27;
    float t38 = t28*t31;
    float t35 = t34 - t38;
    float t36 = t24*t31;
    float t37 = t27*t28;
    float t39 = t8*t13;
    //float t44 = 0.0; // #t11*t12.*6.123233995736766e-17
    float t40 = t39;// - t44;
    float t41 = t8*t11;
    //float t42 = 0.0; // #t12*t13.*6.123233995736766e-17
    float t43 = t41; // + t42;
    float t45 = t16*t40;
    float t46 = t20*t43;
    float t47 = t45 + t46;
    float t48 = t16*t43;
    float t51 = t20*t40;
    float t49 = t48 - t51;
    float t50 = t24*t47;
    float t52 = t28*t49;
    float t53 = t24*t49;
    float t54 = t53 - t28*t47;
    float t57 = M_PI*(1.0/2.0e1);
    float t55 = -t57 + th2 + th3 + th4 + th5;
    float t56 = th2 + th3 + th4 - th5 - M_PI*(1.7e1/9.0e1);
    float t58 = th2 + th3 + th4 - M_PI*(4.3e1/3.6e2);
    float t59 = cos(t58);

    float R11 = t32*(t36 + t37) - t8*t23; // #+ t23*t35*6.123233995736766e-17
    float R12 = t23*(t36 + t37) + t8*t32; // #- t32*t35*6.123233995736766e-17
    float R13 = t34 - t38; // #+ t8*6.123233995736766e-17
    float R21 = -t12*t23 + t32*t54; // #+ t23*(t50 + t52)*(-6.123233995736766e-17)
    float R22 = t12*t32 + t23*t54; // #+ t32*(t50 + t52)*6.123233995736766e-17
    float R23 = -t50 - t52; // #+ t12*6.123233995736766e-17
    float R31 = sin(t55)*5.0e-1 + sin(t56)*5.0e-1; // #+ t23*(-6.123233995736766e-17)
    float R32 = -cos(t55)*5.0e-1 + cos(t56)*5.0e-1; // #+ t32*6.123233995736766e-17
    float R33 = t59; //#+ 3.749399456654644e-33;
    float tx = -t12*(3.3e1/1.0e3) + t11*t12*(3.1e1/2.0e2) - t16*t19*(2.7e1/2.0e2) + t24*t27*(1.09e2/5.0e2) - t28*t31*(1.09e2/5.0e2) + t20*(-t26)*(2.7e1/2.0e2); // #+ t8*1.334865011070615e-17 - t8*t13*9.491012693391987e-18
    float ty = t8*(3.3e1/1.0e3) - t8*t11*(3.1e1/2.0e2) + t20*t43*(2.7e1/2.0e2) - t24*t47*(1.09e2/5.0e2) - t28*t49*(1.09e2/5.0e2)+t16*(t39)*(2.7e1/2.0e2); // #+ t12*1.334865011070615e-17 - t12*t13*9.491012693391987e-18
    float tz = t13*(3.1e1/2.0e2) + t59*(1.09e2/5.0e2) - sin(-t57 + th2 + th3)*(2.7e1/2.0e2) + 1.47e-1;
	
    rot_matrix[0][0] = R11;
    rot_matrix[0][1] = R12;
    rot_matrix[0][2] = R13;
    rot_matrix[0][3] = tx;

    rot_matrix[1][0] = R21;
    rot_matrix[1][1] = R22;
    rot_matrix[1][2] = R23;
    rot_matrix[1][3] = ty;

    rot_matrix[2][0] = R31;
    rot_matrix[2][1] = R32;
    rot_matrix[2][2] = R33;
    rot_matrix[2][3] = tz;

    rot_matrix[3][0] = 0.0;
    rot_matrix[3][1] = 0.0;
    rot_matrix[3][2] = 0.0;
    rot_matrix[3][3] = 1.0;

    #if 0
    std::cout << R11 << ' ' << R12 << ' ' << R13 << ' ' << tx << '\n';
    std::cout << R21 << ' ' << R22 << ' ' << R23 << ' ' << ty << '\n';
    std::cout << R31 << ' ' << R32 << ' ' << R33 << ' ' << tz << '\n';
    #endif
}


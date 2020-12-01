function C = C_3DOF(q,dq)
%
%    C = C_3DOF(dq1,dq2,dq3,q2,q3)

% Coriolis matrix of the youBot considering its 2nd,3rd and 4th joints

dq1 = dq(1);
dq2 = dq(2);
dq3 = dq(3);
q2 = q(1);
q3 = q(2);

t2 = sin(q2);
t3 = sin(q3);
t4 = q2+q3;
t5 = dq1+dq2+dq3;
t6 = sin(t4);
t7 = t3.*(2.7e+1./2.0e+2);
t9 = t2.*3.718295e-2;
t11 = dq3.*t3.*1.226097e-2;
t8 = t6.*(3.1e+1./2.0e+2);
t12 = -t11;
t13 = t6.*1.407741e-2;
t10 = t7+t8;
t14 = t9+t13;
t15 = dq1.*t14;
t16 = dq2.*t14;
t17 = -t16;
C = reshape([t17-dq3.*t10.*9.0822e-2,t12+t15,dq2.*t3.*1.226097e-2+dq1.*t10.*9.0822e-2,-t15+t17-dq3.*(t3.*1.226097e-2+t13),t12,t3.*(dq1+dq2).*1.226097e-2,t5.*t10.*(-9.0822e-2),t3.*t5.*(-1.226097e-2),0.0],[3,3]);

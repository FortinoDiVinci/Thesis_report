function M = M_3DOF(q)
%
%    M = M_3DOF(q2,q3)

% Inertia matrix of the youBot considering its 2nd,3rd and 4th joints
q2 = q(1);
q3 = q(2);

t2 = cos(q2);
t3 = cos(q3);
t4 = q2+q3;
t5 = cos(t4);
t6 = t2.*3.718295e-2;
t7 = t3.*2.452194e-2;
t8 = t3.*1.226097e-2;
t9 = t5.*1.407741e-2;
t10 = t8+1.7544e-2;
t11 = t9+t10;
t12 = t6+t7+t9+1.29924e-1;
M = reshape([t2.*7.43659e-2+t5.*2.815482e-2+t7+4.47134e-1,t12,t11,t12,t7+1.29924e-1,t10,t11,t10,1.7544e-2],[3,3]);

function G = G_3DOF(q)
%
%    G = G_3DOF(q1,q2,q3)

% Gravity matrix of the youBot considering its 2nd,3rd and 4th joints

q1 = q(1);
q2 = q(2);
q3 = q(3);

t2 = q1+q2;
t3 = cos(t2);
t4 = q3+t2;
t5 = cos(t4);
t6 = t3.*2.3533209;
t7 = -t6;
t8 = t5.*8.9096382e-1;
t9 = -t8;
G = [t7+t9-cos(q1).*3.9515661;t7+t9;t9];
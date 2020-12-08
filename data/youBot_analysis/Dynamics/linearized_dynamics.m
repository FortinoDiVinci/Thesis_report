% Linear simulation

% parameters
K = 0.015;
Ki = 0.08;

Kv = [2500;1500;2000]/256;
Kvi = [3000;900;1000]/65536;

q0_kuka = [1.676; -4.363; 1.497];
q_dh = [-90 0 90]'*pi/180;
offset = [65 -146 102.5]'.*pi/180;
%q0 = q0_kuka - offset;
q0 = (q0_kuka - q_dh) - offset;

tau0 = G_3DOF(q0);

% For this analysis, the inertia, jacobian and coriolis matrices are
% evaluated at the configuration q0 after a linearization at this neigborhood

iM = inv(M_3DOF(q0(2:3)));
J_tmp = J0E_3DOF(q0(1:3));
J = J_tmp([1,3,5],:); clear J_tmp
iJ = inv(J);
Fv = zeros(3,3); % TODO: assign value

s = tf('s');

I = eye(3);

Cv = zeros(3); 
C  = zeros(3);
C(2,2)  = K  +  Ki/s;
for i = 1:3
    Cv(i,i) = Kv(i) + Kvi(i)/s;
end

syms q1 q2 q3 dq1 dq2 dq3;
Ksym = jacobian(inv(M_3DOF([q2;q3]))*tau0, [q1;q2;q3]) + ...
    jacobian(inv(M_3DOF([q2;q3]))*G_3DOF([q1;q2;q3]), [q1;q2;q3]);
Kd = double(vpa(subs(Ksym,[q1;q2;q3],q0)));

Vsym = -iM*(Fv - jacobian(C_3DOF(q0(2:3),dq)*dq, [dq1;dq2;dq3]));
Vd = double(vpa(subs(Vsym,[dq1;dq2;dq3],dq0)));

J_tmp = J0E_3DOF([q1;q2;q3]);
Jsym = J_tmp([1,3,5],:);
Jdotsym = diff(Jsym,q1,q2,q3);
Jdot = double(vpa(subs(Jsym,[q1;q2;q3],q0)));

% see computation
A = J*iM*(Cv + iJ*C + J');
B = s*I - Jdot*iJ + K*iM *(Cv + Vd)*iJ + J*Kd*iJ/s;

H_rob = B\A;

Hz_rob = H_rob(2,2);

figure
bode(Hz_rob,Hz_rob/s)
legend('vz/fz','z/fz')

[b,a] = tfdata(Hz_rob/s);
ssHz_rob = tf2ss(b{1},a{1});
zpk(Hz_rob/s)
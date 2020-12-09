% Init sim
clear all

KUKA_offset = [169 65 -146 102.5 167.5]'*pi/180;
theta_DH = [0 pi/2 0 -pi/2 0]';
q0 = [1.676; -4.363; 1.497];
dq0 = [0; 0; 0];
th0 = theta_DH(2:4) - (q0 - KUKA_offset(2:4));
dth0 = dq0;
% limits
qmax = [5.7401; 2.5179; -0.1157; 3.3292; 5.5415];
qmin = [1.101e-1; 1.101e-1; -4.9266; 1.221e-1; 2.106e-1];
thmin = theta_DH - (qmax - KUKA_offset);
thmax = theta_DH - (qmin - KUKA_offset);
%admittance controller
K = 0.015;
Ki = 0.08;
%velocity controller
Kv = [2500,1500,2000]/256;
Kvi = [3000,900,1000]/65536;
%position controller
Kx = [0;0;0];%20;0;0
Kxi = [0;0;0];

fz0 = 0;

K_env = [0,100,0];
B_env = [0,10,0];
M_env = [0,1,0];

x0 = 0;
le = 0.1;
[p0, r, ~, jp] = DGM_youBot(th0, x0, le);

simulateYouBotKinematics([0,th0',0]);

equil = [p0(1);p0(3);acos(r(1,1))];

return 
%% after simulink finished execution

simulateYouBotKinematics([zeros(size(th_rec.signals.values,1),1), ...
    th_rec.signals.values, zeros(size(th_rec.signals.values,1),1)],1e-3, ...
    [fe_rec.signals.values(:,1),zeros(size(fe_rec.signals.values,1),1),fe_rec.signals.values(:,2)] );

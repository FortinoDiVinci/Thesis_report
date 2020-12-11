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
Kv = [2500;1500;2000]/256;
Kvi = [3000;900;1000]/65536;
%position controller
Kx = [20;0;0];%20;0;0
Kxi = [0;0;0];
%joint position controller
Kj = [1;1;1]*5;
Kjd = [1;1;1]*0.1;

fz0 = 0;

K_env = [0;100;0];
B_env = [0;10;0];
M_env = [0;1;0];

x0 = 0;
le = 0.1;
[p0, r, ~, jp] = DGM_youBot(th0, x0, le);

simulateYouBotKinematics([0,th0',0]);

x0 = p0(1);
z0 = p0(3);
ry0 = acos(r(1,1));

return 
%% after simulink finished execution

th_sim = [zeros(size(th_rec.signals.values,1),1), ...
    th_rec.signals.values, zeros(size(th_rec.signals.values,1),1)];
f_sim = -1*[fe_rec.signals.values(:,1),zeros(size(fe_rec.signals.values,1),1),...
    fe_rec.signals.values(:,2)];
dt = 1e-3;
p0_sim = [p0_rec.signals.values(:,1),zeros(size(p0_rec.signals.values,1),1),...
    p0_rec.signals.values(:,2)];

simulateYouBotKinematics(th_sim, dt, f_sim, p0_sim);

figure
subplot(2,1,1)
plot(pos_rec.signals.values)
legend('x','z','\theta')
subplot(2,1,2)
plot(vel_rec.signals.values)
legend('dx','dz','d\theta')
% for synth_hinf_ctrl_lin_model_z & analysis_PI_ctrl_lin_model_z
clear all
addpath('../Dynamics')
addpath('../Dynamics/dynamic_sim')

% youbot decoupled linearized dynamic model along z axis
load('../Dynamics/youbot_lin_model_z.mat');
Sigma = youbot_lin_model_ss_z;
clear youbot_lin_model_ss_z

% equilibrium (around which the model was linearized)
q0 = [1.676; -4.363; 1.497];
dq0 = [0;0;0];
ev0 = [0;0;0];

% env
K = 150;
B = 10;
M = 0.45;
tau_m = 1/(10*2*pi);

% PI ctrl
Kp = 0.015;
Ki = 0.08;

argout = linmod('analysis_PI_ctrl_lin_model_z');
% input 1) is fin, 2) is b (cmd)
% output 1) is eps, 2) is u (cmd), 3) is r (fz)

S = minreal(ss(argout.a, argout.b(:,1), argout.c(1,:), argout.d(1,1))); % sensivity
T = minreal(ss(argout.a, argout.b(:,2), argout.c(1,:), argout.d(1,2))); % comp. sens.
Ks = minreal(ss(argout.a, argout.b(:,2), argout.c(2,:), argout.d(2,2))); % u/w
Ss = minreal(ss(argout.a, argout.b(:,1), argout.c(2,:), argout.d(2,1))); % u/fin

figure
subplot(2,2,1)
bodemag(S)
title('Fonction de sensibilité S')
subplot(2,2,2)
bodemag(T)
title('Fonction de sensibilité complémentaire T')
subplot(2,2,3)
bodemag(Ks)
title('Fonction de sensibilité K = u/w')
subplot(2,2,4)
bodemag(Ss)
title('Fonction de sensibilité K = u/fin')

% fc(S) = 0.4 Hz
% fc(T) = 0.5 Hz

W1 = 1/makeweight(0.1,[2*pi*0.4,1],2);
W2 = 1/makeweight(2,[2*pi*0.5,1],0.1);

figure
subplot(2,1,1)
bodemag(1/W1)
title('1/W1 (epsilon)')
subplot(2,1,2);
bodemag(1/W2)
title('1/W2 (cmd)')

[A_p,B_p,C_p,D_p] = gettf('synth_hinf_ctrl_lin_model_z',1:3,1:2);
nb_meas = 1;  % nb input for controller
nb_cmd = 1;   % nb cmd
%
H = ss(A_p,B_p,C_p,D_p);
[ctrl,bf,gamma] = hinfsyn(H,nb_meas,nb_cmd,'display','on');

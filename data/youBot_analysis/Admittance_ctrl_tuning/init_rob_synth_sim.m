clear all

addpath('../Dynamics/dynamic_sim')
%
time_start = 0;
time_end = 20;
dt = 1e-3; % time sampling

KUKA_offset = [169 65 -146 102.5 167.5]'*pi/180;
theta_DH = [0 pi/2 0 -pi/2 0]';
% equilibrium points
q0 = [1.676; -4.363; 1.497];
dq0 = [0; 0; 0];
th0 = theta_DH(2:4) - (q0 - KUKA_offset(2:4));
dth0 = dq0;
% limits
qmax = [5.7401; 2.5179; -0.1157; 3.3292; 5.5415];
qmin = [1.101e-1; 1.101e-1; -4.9266; 1.221e-1; 2.106e-1];
thmin = theta_DH - (qmax - KUKA_offset);
thmax = theta_DH - (qmin - KUKA_offset);

% dynamics
Fv = diag([0.5,0.37,0.7]); % viscous frictions
Fs = [0.9, 1.3, 0.5];%[0.97571; 0.65131; 0.25819];  % static frictions
Fc = Fs; % Coulomb friction
dv = 1e-3; % velocity bound to avoid unstable behaviour (static frictions)
% external force to robot torque transmission efficiency
nu = [0.8554, 0.5780, 0.9997]; %[0.9488; 0.9216; 0.8345];
tau_0 = [1.0000; 0.6119; 0.8268];%[1.0234;1.0516;1.0901]; %

% joint velocity controller
Kv = [2500;1500;2000]/256;
Kvi = [3000;900;1000]/65536;
Tc = [0.0335;0.0335;0.051]; % Torque constants: conversion from current to torque
R = [156;100;71]; % Gear ratios: conversion from motor torque to joint torque

%joint position controller
Kj = [1;1;1]*5;
Kjd = [1;1;1]*0.1;

% environment
K = 150;    % N/m
B = 10;     % Ns/m
M = 0.45;   % kg
fc = 10;    % 10hz cutoff
tau_m = 1/(fc*2*pi); % s


%%
%[num,den] = filter1(0,1.5,2*pi*0.5);
%W1 = ss(tf(num,den));
% joint velocity loop 10 ms
% robot mechanical force constant 30 ms ?

tau_v = 0.01;
tau_f = 0.03;

W1 = 1/makeweight(0.1,[2*pi/tau_f,1],2);
%W2 = 1/makeweight(0.1,[2*pi*3,1],2);
W2 = 1/makeweight(2,[2*pi/tau_f,1],0.1); 
%W4 = 1/makeweight(0,[2*pi*1,1],1.5)*makeweight(1.5,[2*pi*1,1],0); 

figure
subplot(2,1,1);
bodemag(1/W1)
title('1/W1 (epsilon)')
subplot(2,1,2);
bodemag(1/W2)
title('1/W2')
% subplot(2,2,3);
% bodemag(1/W3)
% title('1/W3 (u)')
% subplot(2,2,4);
% bodemag(1/W4)
% title('1/W4')

[A_p,B_p,C_p,D_p] = gettf('dynamic_simulation_synth_admittance_alone',1:3,1:2);

Co = length(A_p) - rank(ctrb(A_p,B_p));
Ob = length(A_p) - rank(obsv(A_p,C_p));

% 'state-space'
H = ss(A_p,B_p,C_p,D_p);
 
nb_meas = 1;  % nb input for controller
nb_cmd = 1;   % nb cmd

[ctrl,bf,gamma] = hinfsyn(H,nb_meas,nb_cmd,'display','on');





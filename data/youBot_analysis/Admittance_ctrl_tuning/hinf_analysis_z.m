% for synth_hinf_ctrl_lin_model_z & analysis_PI_ctrl_lin_model_z
clear all
addpath('../Dynamics')
addpath('../Dynamics/dynamic_sim')

s = tf('s');

% youbot decoupled linearized dynamic model along z axis
load('../Dynamics/youbot_lin_model_z.mat');
Sigma = youbot_lin_model_ss_z;
clear youbot_lin_model_ss_z

[Sigma, Usig] = minreal(Sigma, [], false); % delete the 3 ev states
% Usig*Sigma.A/Usig

% equilibrium (around which the model was linearized)
q0 = [1.676; -4.363; 1.497];
dq0 = [0;0;0];
ev0 = [0;0;0];

% env
K = 150;
B = 10;
M = 0.45;
tau_m = 1/(10*2*pi);

% H_env = K + B*s + M*s^2;
% H_filt = 1/(1 + tau_m*s)^2;
% H_beta = H_env*H_filt/s;
% 
% ss_beta = ss(H_beta);

% PI ctrl
Kp = 0.015;
Ki = 0.08;
H_pi = Ki/s + Kp;

argout = linmod('analysis_PI_ctrl_lin_model_z');
% input 1) is fin, 2) is b (cmd), 3) is w (meas. noise)
% output 1) is eps, 2) is u (cmd), 3) is r (fz)

S = minreal(ss(argout.a, argout.b(:,1), argout.c(1,:), argout.d(1,1))); % sensivity
T = minreal(ss(argout.a, argout.b(:,3), argout.c(1,:), argout.d(1,2))); % comp. sens.
KS = minreal(ss(argout.a, argout.b(:,1), argout.c(2,:), argout.d(2,2))); % u/fin
SG = minreal(ss(argout.a, argout.b(:,2), argout.c(1,:), argout.d(2,1))); % eps/b
H_cl = minreal(ss(argout.a, argout.b(:,1), argout.c(4,:), argout.d(1,1))); % close loop s/fin

figure(99)
subplot(2,2,1)
bodemag(S)
grid on
title('Fonction de sensibilité S')
subplot(2,2,2)
bodemag(T)
grid on
title('Fonction de sensibilité complémentaire T')
subplot(2,2,3)
bodemag(KS)
grid on
title('Fonction de sensibilité KS = u/fin')
subplot(2,2,4)
bodemag(SG)
grid on
title('Fonction de sensibilité -SG = eps/b')

fc_s = 1;% fc(S) = [1, 0.55] Hz
fc_t = 0.8;% fc(T) = [0.8, 0.45] Hz

W1 = 1/makeweight(1e-6,[2*pi*fc_s,1],2,0,2);
W2 = 1/makeweight(2,[2*pi*fc_t,1],0.1); %,0,2

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

[HS,HNS] = stabsep(H);
if (rank(ctrb(HNS.A,HNS.B)) - size(HNS.A,1)) == 0
    disp('H is controllable')
else
    disp('H is not controllable')
end
%
if (rank(obsv(HNS.A,HNS.C)) - size(HNS.A,1)) == 0
    disp('H is observable')
else
    disp('H is not observable')
end

[Hinf_ctrl,bf,gamma] = hinfsyn(H,nb_meas,nb_cmd,'display','on');
%Hinf_ctrl_red = minreal(Hinf_ctrl, 0.01);
H_inf_tf = zpk(Hinf_ctrl);
    
z_sel = H_inf_tf.Z{1}((H_inf_tf.Z{1} > -1e3) & (H_inf_tf.Z{1} < -5e-1));
p_sel = H_inf_tf.P{1}((H_inf_tf.P{1} > -1e3) & (H_inf_tf.P{1} < -5e-1));
nb_integrator = sum(~(H_inf_tf.P{1} < -5e-1));
p_sel = [p_sel; 0];

sys = zpk(z_sel,p_sel,H_inf_tf.K);
% Hinf_ctrl = ss(sys);

Hinf_ctrl_red = ss(minreal(sys, 0.5));

figure(98)
bode(Hinf_ctrl)
hold on
bode(Hinf_ctrl_red)
bode(H_pi)
legend('Hinf', 'Hinf red', 'H pi')

% figure
% bode(Hinf_ctrl)
% hold on
% bode(H_pi)
% legend("H_{inf}", "PI")

%%

argout_hinf = linmod('analysis_hinf_ctrl_lin_model_z');
% input 1) is fin, 2) is b (cmd), 3) is w (meas. noise)
% output 1) is eps, 2) is u (cmd), 3) is r (fz)

S2 = minreal(ss(argout_hinf.a, argout_hinf.b(:,1), argout_hinf.c(1,:), argout_hinf.d(1,1))); % sensivity
T2 = minreal(ss(argout_hinf.a, argout_hinf.b(:,3), argout_hinf.c(1,:), argout_hinf.d(1,2))); % comp. sens.
KS2 = minreal(ss(argout_hinf.a, argout_hinf.b(:,1), argout_hinf.c(2,:), argout_hinf.d(2,2))); % u/fin
SG2 = minreal(ss(argout_hinf.a, argout_hinf.b(:,2), argout_hinf.c(1,:), argout_hinf.d(2,1))); % eps/b
H_cl2 = minreal(ss(argout_hinf.a, argout_hinf.b(:,1), argout_hinf.c(4,:), argout_hinf.d(1,1))); % close loop s/fin

Hinf_ctrl_tmp = Hinf_ctrl;
Hinf_ctrl = Hinf_ctrl_red;
argout_hinf = linmod('analysis_hinf_ctrl_lin_model_z');

S1 = minreal(ss(argout_hinf.a, argout_hinf.b(:,1), argout_hinf.c(1,:), argout_hinf.d(1,1))); % sensivity
T1 = minreal(ss(argout_hinf.a, argout_hinf.b(:,3), argout_hinf.c(1,:), argout_hinf.d(1,2))); % comp. sens.
KS1 = minreal(ss(argout_hinf.a, argout_hinf.b(:,1), argout_hinf.c(2,:), argout_hinf.d(2,2))); % u/fin
SG1 = minreal(ss(argout_hinf.a, argout_hinf.b(:,2), argout_hinf.c(1,:), argout_hinf.d(2,1))); % eps/b
H_cl1 = minreal(ss(argout_hinf.a, argout_hinf.b(:,1), argout_hinf.c(4,:), argout_hinf.d(1,1))); % close loop s/fin

Hinf_ctrl = Hinf_ctrl_tmp;
Hinf_ctrl_red_dis = c2d(Hinf_ctrl_red, 1e-3, 'least-squares');
tmp = tf(Hinf_ctrl_red_dis);
num_dis_hinf = tmp.Numerator{:};
den_dis_hinf = tmp.Denominator{:};

argout_hinf = linmod('analysis_discrete_hinf_ctrl_lin_model_z');
S1d = minreal(ss(argout_hinf.a, argout_hinf.b(:,1), argout_hinf.c(1,:), argout_hinf.d(1,1))); % sensivity
T1d = minreal(ss(argout_hinf.a, argout_hinf.b(:,3), argout_hinf.c(1,:), argout_hinf.d(1,2))); % comp. sens.
KS1d = minreal(ss(argout_hinf.a, argout_hinf.b(:,1), argout_hinf.c(2,:), argout_hinf.d(2,2))); % u/fin
SG1d = minreal(ss(argout_hinf.a, argout_hinf.b(:,2), argout_hinf.c(1,:), argout_hinf.d(2,1))); % eps/b
H_cl1d = minreal(ss(argout_hinf.a, argout_hinf.b(:,1), argout_hinf.c(4,:), argout_hinf.d(1,1))); % close loop s/fin

figure(99)
subplot(2,2,1)
bodemag(S,S1,S2)
grid on
title('Fonction de sensibilité S')
subplot(2,2,2)
hold on
bodemag(T,T1,T2)
grid on
title('Fonction de sensibilité complémentaire T')
legend("PI", "H_{inf}^{red}", "H_{inf}")
subplot(2,2,3)
hold on
bodemag(KS,KS1,KS2)
grid on
title('Fonction de sensibilité KS = u/fin')
subplot(2,2,4)
hold on
bodemag(SG,SG1,SG2)
grid on
title('Fonction de sensibilité -SG = eps/b')


t = (0:1e-3:10);
u = sin(2*pi*0.9.*t);
y_pi = lsim(H_cl, u, t);
y_hinf = lsim(H_cl1, u, t);
%y_hinf2 = lsim(minreal(H_cl2), u, t);
y_hinfd = lsim(minreal(H_cl1d), u, t);

figure
plot(t, y_pi)
hold on
plot(t, y_hinf)
plot(t, y_hinfd)
yyaxis right
plot(t, u)
legend('PI', 'H_{inf}^{red}', 'H_{inf}^{red} disc', 'u')

figure(98)
bode(Hinf_ctrl_red_dis)
legend('Hinf', 'Hinf red', 'H pi', 'Hinf red dis')

figure
step(H_cl)
hold on
step(H_cl1)
step(H_cl1d)


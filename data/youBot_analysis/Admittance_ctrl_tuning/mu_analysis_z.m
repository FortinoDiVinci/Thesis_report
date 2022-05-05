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
K = ureal('K',160,'Percentage',50);
B = 11; % +/- 15%
M = 0.42; % +/- 10%
tau_m = 1/(10*2*pi);

% u_env = (K + B*s + M*s^2)/(1 + tau_m*s)^2;

% unc_pole = ureal('unc_pole',-5,'Range',[-10 -4]);
% plant = ss(unc_pole,5,1,0);

% H_env = K + B*s + M*s^2;
% H_filt = 1/(1 + tau_m*s)^2;
% H_beta = H_env*H_filt/s;
% 
% ss_beta = ss(H_beta);

% PI ctrl
Kp = 0.015;
Ki = 0.08;
H_pi = Ki/s + Kp;

%open_system('analysis_PI_ctrl_lin_uncertain_model_z')
load_system('analysis_PI_ctrl_lin_uncertain_model_z');
io(1) = linio('analysis_PI_ctrl_lin_uncertain_model_z/fin',1, 'input');
io(2) = linio('analysis_PI_ctrl_lin_uncertain_model_z/b',1, 'input');
io(3) = linio('analysis_PI_ctrl_lin_uncertain_model_z/w',1, 'input');
io(4) = linio('analysis_PI_ctrl_lin_uncertain_model_z/Sum',1, 'output'); % eps
io(5) = linio('analysis_PI_ctrl_lin_uncertain_model_z/Admittance PI',1, 'output'); % u
io(6) = linio('analysis_PI_ctrl_lin_uncertain_model_z/Sum1',1, 'output'); % w
io(7) = linio('analysis_PI_ctrl_lin_uncertain_model_z/endpoint model of youBot',1, 'output'); % s
%io = getlinio('analysis_PI_ctrl_lin_uncertain_model_z');

argout = ulinearize('analysis_PI_ctrl_lin_uncertain_model_z',io);
% argout = linmod('analysis_PI_ctrl_lin_model_z');
% input 1) is fin, 2) is b (cmd), 3) is w (meas. noise)
% output 1) is eps, 2) is u (cmd), 3) is r (fz)

S = minreal(ss(argout.a, argout.b(:,1), argout.c(1,:), argout.d(1,1))); % sensivity
T = minreal(ss(argout.a, argout.b(:,1), argout.c(3,:), argout.d(3,1))); % comp. sens.
KS = minreal(ss(argout.a, argout.b(:,1), argout.c(2,:), argout.d(2,1))); % u/fin
SG = minreal(ss(argout.a, argout.b(:,2), argout.c(1,:), argout.d(1,2))); % eps/b
Hcl = minreal(ss(argout.a, argout.b(:,1), argout.c(4,:), argout.d(4,1))); % close loop s/fin

figure(99)
hold on
subplot(2,2,1)
hold on
bodemag(S, S.NominalValue)
grid on
title('Fonction de sensibilité S')
subplot(2,2,2)
hold on
bodemag(T, T.NominalValue)
grid on
title('Fonction de sensibilité complémentaire T')
subplot(2,2,3)
hold on
bodemag(KS, KS.NominalValue)
grid on
title('Fonction de sensibilité KS = u/fin')
subplot(2,2,4)
hold on
bodemag(SG,SG.NominalValue)
grid on
title('Fonction de sensibilité -SG = eps/b')

fc_s = 0.55;%1;% fc(S) = [1, 0.55] Hz
fc_t = 0.45;%0.8;% fc(T) = [0.8, 0.45] Hz

W1 = 1/makeweight(1e-6,[2*pi*fc_s,1],2,0,2);
W2 = 1/makeweight(15,[2*pi*fc_t,1],0.1); %,0,2

figure(97)
subplot(2,1,1)
bodemag(1/W1)
title('1/W1 (epsilon)')
subplot(2,1,2);
bodemag(1/W2)
title('1/W2 (cmd)')

load_system('synth_hinf_ctrl_lin_uncertain_model_z')
io_hinf(1) = linio('synth_hinf_ctrl_lin_uncertain_model_z/fin',1, 'input');
io_hinf(2) = linio('synth_hinf_ctrl_lin_uncertain_model_z/cmd',1, 'input');
io_hinf(3) = linio('synth_hinf_ctrl_lin_uncertain_model_z/W1',1, 'output');
io_hinf(4) = linio('synth_hinf_ctrl_lin_uncertain_model_z/W2',1, 'output');
io_hinf(5) = linio('synth_hinf_ctrl_lin_uncertain_model_z/Sum',1, 'output');

H = ulinearize('synth_hinf_ctrl_lin_uncertain_model_z',io_hinf);
%[A_p,B_p,C_p,D_p] = gettf('synth_hinf_ctrl_lin_model_z',1:3,1:2);
nb_meas = 1;  % nb input for controller
nb_cmd = 1;   % nb cmd
%

[Hinf_ctrl,cl_perf,info] = musyn(H,nb_meas,nb_cmd);
gamma = info(end).gamma;
%Hinf_ctrl_red = minreal(Hinf_ctrl, 0.01);
H_inf_tf = zpk(Hinf_ctrl);
    
z_sel = H_inf_tf.Z{1}((H_inf_tf.Z{1} > -8e1) & (H_inf_tf.Z{1} < -5e-1));
p_sel = H_inf_tf.P{1}((H_inf_tf.P{1} > -1.5e2) & (H_inf_tf.P{1} < -5e-1));
nb_integrator = sum(~(H_inf_tf.P{1} < -5e-1));
p_sel = [p_sel; 0];

sys = zpk(z_sel,p_sel,H_inf_tf.K);
% Hinf_ctrl = ss(sys);

Hinf_ctrl_red = ss(minreal(sys, 0.15));

mag_red = bode(Hinf_ctrl_red, 2*pi);
mag_hinf = bode(H_inf_tf, 2*pi);
gain_red = mag_hinf/mag_red;

Hinf_ctrl_red_corr = gain_red*Hinf_ctrl_red;

figure(98)
bode(Hinf_ctrl)
hold on
%bode(Hinf_ctrl_red)
bode(Hinf_ctrl_red_corr)
bode(H_pi)
legend('Hinf', 'Hinf red', 'H pi')

% figure
% bode(Hinf_ctrl)
% hold on
% bode(H_pi)
% legend("H_{inf}", "PI")

%%
clear io
load_system('analysis_hinf_ctrl_lin_uncertain_model_z')
io(1) = linio('analysis_hinf_ctrl_lin_uncertain_model_z/fin',1, 'input');
io(2) = linio('analysis_hinf_ctrl_lin_uncertain_model_z/b',1, 'input');
io(3) = linio('analysis_hinf_ctrl_lin_uncertain_model_z/w',1, 'input');
io(4) = linio('analysis_hinf_ctrl_lin_uncertain_model_z/Sum',1, 'output'); % eps
io(5) = linio('analysis_hinf_ctrl_lin_uncertain_model_z/H_inf_ctrl',1, 'output'); % u
io(6) = linio('analysis_hinf_ctrl_lin_uncertain_model_z/Sum1',1, 'output'); % w
io(7) = linio('analysis_hinf_ctrl_lin_uncertain_model_z/endpoint model of youBot',1, 'output'); % s
%io = getlinio('analysis_PI_ctrl_lin_uncertain_model_z');

argout_hinf = ulinearize('analysis_hinf_ctrl_lin_uncertain_model_z',io);
% input 1) is fin, 2) is b (cmd), 3) is w (meas. noise)
% output 1) is eps, 2) is u (cmd), 3) is r (fz)

S_mu = minreal(ss(argout_hinf.a, argout_hinf.b(:,1), argout_hinf.c(1,:), argout_hinf.d(1,1))); % sensivity
T_mu = minreal(ss(argout_hinf.a, argout_hinf.b(:,1), argout_hinf.c(3,:), argout_hinf.d(3,1))); % comp. sens.
KS_mu = minreal(ss(argout_hinf.a, argout_hinf.b(:,1), argout_hinf.c(2,:), argout_hinf.d(2,1))); % u/fin
SG_mu = minreal(ss(argout_hinf.a, argout_hinf.b(:,2), argout_hinf.c(1,:), argout_hinf.d(1,2))); % eps/b
Hcl_mu = minreal(ss(argout_hinf.a, argout_hinf.b(:,1), argout_hinf.c(4,:), argout_hinf.d(4,1))); % close loop s/fin

Hinf_ctrl_tmp = Hinf_ctrl;
Hinf_ctrl = ss(Hinf_ctrl_red_corr);%Hinf_ctrl_red;
argout_hinf = ulinearize('analysis_hinf_ctrl_lin_uncertain_model_z',io);

S_mu_red = minreal(ss(argout_hinf.a, argout_hinf.b(:,1), argout_hinf.c(1,:), argout_hinf.d(1,1))); % sensivity
T_mu_red = minreal(ss(argout_hinf.a, argout_hinf.b(:,1), argout_hinf.c(3,:), argout_hinf.d(3,1))); % comp. sens.
KS_mu_red = minreal(ss(argout_hinf.a, argout_hinf.b(:,1), argout_hinf.c(2,:), argout_hinf.d(2,1))); % u/fin
SG_mu_red = minreal(ss(argout_hinf.a, argout_hinf.b(:,2), argout_hinf.c(1,:), argout_hinf.d(1,2))); % eps/b
Hcl_mu_red = minreal(ss(argout_hinf.a, argout_hinf.b(:,1), argout_hinf.c(4,:), argout_hinf.d(4,1))); % close loop s/fin

Hinf_ctrl = Hinf_ctrl_tmp;

Hinf_ctrl_red_dis = c2d(ss(Hinf_ctrl_red_corr), 1e-3,  'tustin');%c2d(Hinf_ctrl_red, 1e-3, 'tustin');
figure
bode(Hinf_ctrl_red_corr, Hinf_ctrl_red_dis)

tmp = tf(Hinf_ctrl_red_dis);
num_dis_hinf = tmp.Numerator{:};
den_dis_hinf = tmp.Denominator{:};

load_system('analysis_discrete_hinf_ctrl_lin_uncertain_model_z')
io_d(1) = linio('analysis_discrete_hinf_ctrl_lin_uncertain_model_z/fin',1, 'input');
io_d(2) = linio('analysis_discrete_hinf_ctrl_lin_uncertain_model_z/b',1, 'input');
io_d(3) = linio('analysis_discrete_hinf_ctrl_lin_uncertain_model_z/w',1, 'input');
io_d(4) = linio('analysis_discrete_hinf_ctrl_lin_uncertain_model_z/Sum',1, 'output'); % eps
io_d(5) = linio('analysis_discrete_hinf_ctrl_lin_uncertain_model_z/LTI System',1, 'output'); % u
io_d(6) = linio('analysis_discrete_hinf_ctrl_lin_uncertain_model_z/Sum1',1, 'output'); % w
io_d(7) = linio('analysis_discrete_hinf_ctrl_lin_uncertain_model_z/endpoint model of youBot',1, 'output'); % s
opts = linearizeOptions('SampleTime',1e-3);
argout_hinf = ulinearize('analysis_discrete_hinf_ctrl_lin_uncertain_model_z', io_d, opts);

S_mu_red_d = minreal(ss(argout_hinf.a, argout_hinf.b(:,1), argout_hinf.c(1,:), argout_hinf.d(1,1), 1e-3)); % sensivity
T_mu_red_d = minreal(ss(argout_hinf.a, argout_hinf.b(:,1), argout_hinf.c(3,:), argout_hinf.d(3,1), 1e-3)); % comp. sens.
KS_mu_red_d = minreal(ss(argout_hinf.a, argout_hinf.b(:,1), argout_hinf.c(2,:), argout_hinf.d(2,1), 1e-3)); % u/fin
SG_mu_red_d = minreal(ss(argout_hinf.a, argout_hinf.b(:,2), argout_hinf.c(1,:), argout_hinf.d(1,2), 1e-3)); % eps/b
Hcl_mu_red_d = minreal(ss(argout_hinf.a, argout_hinf.b(:,1), argout_hinf.c(4,:), argout_hinf.d(4,1), 1e-3)); % close loop s/fin

figure
bode(Hcl_mu_red)
hold on
bode(Hcl_mu_red_d)

figure(99)
subplot(2,2,1)
bodemag(S,S_mu,S_mu_red)
grid on
title('Fonction de sensibilité S')
subplot(2,2,2)
hold on
bodemag(T,T_mu,T_mu_red)
grid on
title('Fonction de sensibilité complémentaire T')
legend("PI", "H_{inf}^{red}", "H_{inf}")
subplot(2,2,3)
hold on
bodemag(KS,KS_mu,KS_mu_red)
grid on
title('Fonction de sensibilité KS = u/fin')
subplot(2,2,4)
hold on
bodemag(SG,SG_mu,SG_mu_red)
grid on
title('Fonction de sensibilité -SG = eps/b')

t = (0:1e-3:10);
u = sin(2*pi*0.9.*t);
y_pi = lsim(Hcl, u, t);
y_hinf = lsim(Hcl_mu_red, u, t);
%y_hinf2 = lsim(minreal(H_cl2), u, t);
y_hinfd = lsim(Hcl_mu_red_d, u, t);

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
step(Hcl)
hold on
step(Hcl_mu_red)
step(Hcl_mu_red_d)

%%%

% in = zeros(1,7);
% out = zeros(1,7);
% for i = 1:length(u)
%     n_in = u(i);
%     in = circshift(in,1);
%     out = circshift(out,1);
%     in(1) = n_in;
%     out = num_dis_hinf.*in - den_dis_hinf.*out;  
%     y_cpp(i) = out(1);
% end

% y_matlab = lsim(Hinf_ctrl_red_dis, u, t);
% y_hinf_ctrl = lsim(Hinf_ctrl_red, u, t);
% 
% figure
% plot(t, y_matlab)
% hold on
% plot(t, y_cpp)
% yyaxis right
% plot(t,u)
% 
% figure
% plot(t, y_hinf_ctrl)

%% genetic tuning of the weights
W1_old = W1;
W2_old = W2;

lb = [0.01;0.01;1.0001;1.0001];
ub = [15;15;30;30];
[x, fval, exitflag, output, pop, scores] = ga(@muCostFunc,4,[],[],[],[],lb,ub);
% A = -1*eye(4);
% b = [0;0;-1;-1]-1e-4;
% [x, fval, exitflag, output, pop, scores] = ga(@muCostFunc,4,A,b);
% x = [0.1000    1.0000    1.0010    1.0010];
% x = [0.1000    0.3074    1.0010    1.0010];
% x = [0.1037    0.0100    1.0010    1.0010];
% x = [0.9545    0.0100    1.1569    1.0001];

W1 = 1/makeweight(1e-6,[2*pi*x(1),1],x(3),0,2);
W2 = 1/makeweight(x(4),[2*pi*x(2),1],0.1); %,0,2

figure(97)
subplot(2,1,1)
bodemag(1/W1_old)
hold on
bodemag(1/W1)
title('1/W1 (epsilon)')
subplot(2,1,2);
bodemag(1/W2_old)
hold on
bodemag(1/W2)
title('1/W2 (cmd)')

load_system('synth_hinf_ctrl_lin_uncertain_model_z')
clear io_hinf
io_hinf(1) = linio('synth_hinf_ctrl_lin_uncertain_model_z/fin',1, 'input');
io_hinf(2) = linio('synth_hinf_ctrl_lin_uncertain_model_z/cmd',1, 'input');
io_hinf(3) = linio('synth_hinf_ctrl_lin_uncertain_model_z/W1',1, 'output');
io_hinf(4) = linio('synth_hinf_ctrl_lin_uncertain_model_z/W2',1, 'output');
io_hinf(5) = linio('synth_hinf_ctrl_lin_uncertain_model_z/Sum',1, 'output');

H_new = ulinearize('synth_hinf_ctrl_lin_uncertain_model_z',io_hinf);
nb_meas = 1;  % nb input for controller
nb_cmd = 1;   % nb cmd

%opts = musynOptions('Display','full');
%[Hinf_ctrl_ga,~,info] = musyn(H_new,nb_meas,nb_cmd,opts);
[Hinf_ctrl_ga,~,info] = musyn(H_new,nb_meas,nb_cmd);
gamma_ga = info(end).gamma;
H_inf_tf_ga = zpk(Hinf_ctrl_ga);

figure(101)
bode(H_pi, Hinf_ctrl, Hinf_ctrl_ga)

% order reduction
H_inf_tf_tmp_ga = minreal(H_inf_tf_ga, 0.05);
z_sel_ga = H_inf_tf_tmp_ga.Z{1}((abs(H_inf_tf_tmp_ga.Z{1}) < 50) & (abs(H_inf_tf_tmp_ga.Z{1}) > 5e-2));
p_sel_ga = H_inf_tf_tmp_ga.P{1}((abs(H_inf_tf_tmp_ga.P{1}) < 100) & (abs(H_inf_tf_tmp_ga.P{1}) > 5e-2));
nb_integrator = sum(~(abs(H_inf_tf_tmp_ga.P{1}) > 5e-2));
%p_sel_ga = [p_sel_ga; zeros(nb_integrator,1)];
if nb_integrator > 0
    p_sel_ga = [p_sel_ga; 0];
end
sys_ga = zpk(z_sel_ga,p_sel_ga,H_inf_tf_ga.K);
sign_gain = sum([H_inf_tf_tmp_ga.Z{1}; H_inf_tf_tmp_ga.P{1}] > 0) - sum([sys_ga.Z{1}; sys_ga.P{1}] > 0);
sys_ga = sys_ga*(-1)^sign_gain;

gain_corr = bode(Hinf_ctrl_ga, 2*pi)/bode(sys_ga, 2*pi);
sys_ga = sys_ga*gain_corr;
Hinf_ctrl_red_ga = ss(sys_ga);

figure
bode(H_inf_tf_ga, sys_ga, H_pi)
legend('H_{inf}', 'H_{inf}^{red}', 'H_{pi}')

Hinf_ctrl_tmp = Hinf_ctrl;
Hinf_ctrl = Hinf_ctrl_red_ga;

load_system('analysis_hinf_ctrl_lin_uncertain_model_z')
io_hinf_ga(1) = linio('analysis_hinf_ctrl_lin_uncertain_model_z/fin',1, 'input');
io_hinf_ga(2) = linio('analysis_hinf_ctrl_lin_uncertain_model_z/b',1, 'input');
io_hinf_ga(3) = linio('analysis_hinf_ctrl_lin_uncertain_model_z/w',1, 'input');
io_hinf_ga(4) = linio('analysis_hinf_ctrl_lin_uncertain_model_z/Sum',1, 'output'); % eps
io_hinf_ga(5) = linio('analysis_hinf_ctrl_lin_uncertain_model_z/H_inf_ctrl',1, 'output'); % u
io_hinf_ga(6) = linio('analysis_hinf_ctrl_lin_uncertain_model_z/Sum1',1, 'output'); % w
io_hinf_ga(7) = linio('analysis_hinf_ctrl_lin_uncertain_model_z/endpoint model of youBot',1, 'output'); % s

argout_hinf_ga = ulinearize('analysis_hinf_ctrl_lin_uncertain_model_z',io_hinf_ga);

Sga = minreal(ss(argout_hinf_ga.a, argout_hinf_ga.b(:,1), argout_hinf_ga.c(1,:), argout_hinf_ga.d(1,1))); % sensivity
Tga = minreal(ss(argout_hinf_ga.a, argout_hinf_ga.b(:,1), argout_hinf_ga.c(3,:), argout_hinf_ga.d(3,1))); % comp. sens.
KSga = minreal(ss(argout_hinf_ga.a, argout_hinf_ga.b(:,1), argout_hinf_ga.c(2,:), argout_hinf_ga.d(2,1))); % u/fin
SGga = minreal(ss(argout_hinf_ga.a, argout_hinf_ga.b(:,2), argout_hinf_ga.c(1,:), argout_hinf_ga.d(1,2))); % eps/b
Hclga = minreal(ss(argout_hinf_ga.a, argout_hinf_ga.b(:,1), argout_hinf_ga.c(4,:), argout_hinf_ga.d(4,1))); % close loop s/fin

Hinf_ctrl = Hinf_ctrl_tmp;

figure(99)
subplot(2,2,1)
bodemag(S,S_mu,S_mu_red,Sga)
grid on
title('Fonction de sensibilité S')
subplot(2,2,2)
hold on
bodemag(T,T_mu,T_mu_red,Tga)
grid on
title('Fonction de sensibilité complémentaire T')
legend("PI", "H_{inf}^{red}", "H_{inf}", "H_{inf}^{red} ga")
subplot(2,2,3)
hold on
bodemag(KS,KS_mu,KS_mu_red,KSga)
grid on
title('Fonction de sensibilité KS = u/fin')
subplot(2,2,4)
hold on
bodemag(SG,SG_mu,SG_mu_red,SGga)
grid on
title('Fonction de sensibilité -SG = eps/b')

y_hinf_ga = lsim(Hclga, u, t);

figure
plot(t, y_pi)
hold on
plot(t, y_hinf)
plot(t, y_hinfd)
plot(t, y_hinf_ga)
yyaxis right
plot(t, u)
legend('PI', 'H_{inf}^{red}', 'H_{inf}^{red} disc', 'H_{inf}^{red} ga', 'u')

% discrete form
Hinf_ctrl_red_ga_dis = c2d(Hinf_ctrl_red_ga, 1e-3, 'tustin');
tmp = tf(Hinf_ctrl_red_ga_dis);
num_dis_hinf_ga = tmp.Numerator{:};
den_dis_hinf_ga = tmp.Denominator{:};
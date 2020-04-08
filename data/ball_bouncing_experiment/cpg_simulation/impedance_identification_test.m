clear all
close all
clc

%% Variables %%

imp_data = [];
imp_data_2 = [];
nb_iter = 5;   
    
% Initial conditions -----------------------------------------------------
Vb0 = 0;                        % Ball speed (m/s)
Zb0 = 0.55;                     % Ball height (m)
Hp = 0.55;                      % Target height (m)
alpha = 0.48;                   % Restitution coeff paddle/ball
g = 9.81;                       % Gravity acc (SI)
t_max = 10;                     % Trial duration (s)
t_s = 0.003;                    % Sampling time (s)
t_pert = 5.0;                   % time of a disturbance
pert_dur = 0.1;                 % duration of the disturbance (s)
tau_e_0 = -4;                   % force disturbance (N)

% Impedance of the arm ---------------------------------------------------
I = 0.113;                      % Inertia
B = 1.8;                        % Damping
K = 25;                         % Stiffness
h1 = 0.610;
arm_pos_0 = 0.0;                % Arm position along z axis
arm_speed_0 = 0.0;              % Arm speed along z axis
arm_acc_0 = 0.0;                % Arm acceleration along z axis

% Storing data variables -------------------------------------------------

n_step = uint32(t_max/t_s);
Za_d = zeros(n_step,1);                 % arm position along z
Za_dp = zeros(n_step,1);                % arm position along z with perturbation
Va_d = zeros(n_step,1);                 % arm speed along z
Tau_d = zeros(n_step,1);                % tau
Tau_e_d = zeros(n_step,1);              % tau induced by the disturbance

%% Initialization %%

% arm = ARM(K, B, I, arm_pos_0, arm_speed_0, arm_acc_0, t_s, 0);

s = tf('s');

H_arm = 1/(K + B*s + I*s^2);

%input signal
[Tau_d, t_exp] = gensig('square', 1, t_max, t_s); % 1sec period 
%system response to input signal
[Za_d, t_exp] = lsim(H_arm, Tau_d, t_exp);
[Va_d, t_exp] = lsim(s*H_arm, Tau_d, t_exp);
%generation of perturbation signal
for ii = 1:1:n_step+1
    if (t_exp(ii) >= t_pert) && (t_exp(ii) < t_pert + pert_dur)
        Tau_e_d(ii) = -4;
    else
        Tau_e_d(ii) = 0;
    end
end
%system response to input signal
[Za_dp, t_exp] = lsim(H_arm, Tau_d + Tau_e_d, t_exp);
[Va_dp, t_exp] = lsim(s*H_arm, Tau_d + Tau_e_d, t_exp);

%% IMPEDANCE COMPUTATION

Za_tild = Za_dp - Za_d;
Va_tild = Va_dp - Va_d;
Aa_d = zeros(length(Va_d) - 1, 1);
Aa_dp = zeros(length(Va_d) - 1, 1);
nb_fil = 1;         % nb of pts (bf&af) used to compute derivation
% acceleration computed using central difference to avoid phase shift
for ii=nb_fil+1:length(Va_d)-nb_fil
    Aa_d(ii) = (-sum(Va_d(ii-nb_fil:ii-1)) + sum(Va_d(ii+1:ii+nb_fil)))/(2*t_s);
    Aa_dp(ii) = (-sum(Va_dp(ii-nb_fil:ii-1)) + sum(Va_dp(ii+1:ii+nb_fil)))/(2*t_s);
end
Aa_tild = Aa_dp - Aa_d;
Va_tild = Va_tild(1:end-1);
Za_tild = Za_tild(1:end-1);
% impedance is computed using a window of 250ms
idx_1 = intersect(find(t_exp >= t_pert), find(t_exp < t_pert+t_s));
idx_2 = idx_1 + floor(0.25/t_s);
phi = [Za_tild(idx_1:idx_2), Va_tild(idx_1:idx_2), Aa_tild(idx_1:idx_2)];
phi1 = [Za_tild(idx_1:idx_2), Va_tild(idx_1:idx_2), Aa_tild(idx_1:idx_2), ones(length(Aa_tild(idx_1:idx_2)),1)];
phi2 = [Za_tild(idx_1:idx_2), Va_tild(idx_1:idx_2), ones(length(Aa_tild(idx_1:idx_2)),1)];
y = Tau_e_d(idx_1:idx_2);    
% least square algorithm
impedance = (phi'*phi)\phi'*y;
impedance1 = (phi1'*phi1)\phi1'*y;
impedance2 = (phi2'*phi2)\phi2'*y;

tau_e_res = phi*impedance;
tau_e_res1 = phi1*impedance1;
tau_e_res2 = phi2*impedance2;

figure
plot(y)
hold on, grid on 
plot(tau_e_res)
plot(tau_e_res1)
plot(tau_e_res2)
legend('data', 'KBI', 'KBI+c', 'KB')
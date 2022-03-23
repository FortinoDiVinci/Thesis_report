addpath('../Dynamics')

load("robot_model_endpoint_input_output.mat");
% robot dynamic model
% inputs are fz (sigma_f) and vz* (sigma_v)
% ouput is vz 

Ki = 0.08;
Kp = 0.015;

K = 150;    % N/m
B = 10;     % Ns/m
M = 0.45;   % kg

fc = 10;
tau_m = 1/(fc*2*pi); % s 10hz cutoff

s = tf('s');

C_PI = Kp + Ki/s;
Z_env = K + B*s + M*s^2;
Z_env_filt = Z_env/(1 + tau_m*s)^2;

% actual tuning
H_BO = Z_env_filt/s*(sigma_v*C_PI + sigma_f);
H_CD = (sigma_v*C_PI + sigma_f);
H_BF = minreal( feedback(H_CD, Z_env_filt/s) );
%H_BF_2 = H_CD/(1 + H_BO); % same thing as H_BF

figure
bode(H_BF)

isstable(H_BF)
%bode(H_BF, H_BF_2)

%%%%%%%%%%%%
%% robust synthesis
%%%%%%%%%%%%

%% parametric variability of the environment

uK = ureal('K',150,'Percentage',100);           % N/m
uB = ureal('B',10,'Percentage',50);             % Ns/m
uM = ureal('M',0.45,'Percentage',35);           % kg
ufc = ureal('fc',10,'Percentage',50);           % Hz
utau_m = 1/(ufc*2*pi); 

uZ_env = uK + uB*s + uM*s^2;
uZ_env_filt = uZ_env/(1 + utau_m*s)^2;
    
% robust tuning (from env uncertainties)
% force input to velocity output (along z)
uH_BO = uZ_env_filt/s*(sigma_v*C_PI + sigma_f);
uH_CD = minreal( (sigma_v*C_PI + sigma_f) );
uH_BF = feedback(uH_CD, uZ_env_filt/s);

figure
bodeplot(uH_BF)

figure
nyquist(uZ_env_filt/s*minreal((sigma_v*C_PI + sigma_f)))

opts = robOptions('Display', 'on', 'Sensitivity', 'on');
[stab_margin, wcu] = robstab(uH_BF, opts); 

%% Hinf

% velocity cmd & force inputs along z axis only
youBot_z = [dyn_model_ss_b(1), dyn_model_ss_b(3)]; 
youBot_z.InputName = {'u_vz', 'fz'};
youBot_z.OutputName = {'vz'};

%tzero(youBot_z('vz','u_vz'))

% uncertain state space of the arm impedance (only K)
Z_env_filt_nom = Z_env_filt;

lp_gain = 5;   % dB
hp_gain = -20;  % dB
fcW_env = 5;    % Hz
magW_env = -3;   % dB

W_env = makeweight(10^(lp_gain/20), [2*pi*fcW_env, 10^(magW_env/20)], ...
    10^(hp_gain/20));
unc = ultidyn('unc',[1 1]);

figure
bodemag(W_env)
grid on

Z_env_filt_unc = Z_env_filt_nom*(1 + W_env*unc);
Z_env_filt_unc.InputName = 'vz';
Z_env_filt_unc.OutputName = 'fe_z';

figure
bode(Z_env_filt_unc,'b',Z_env_filt_unc.NominalValue,'r+')
grid on

% %% tmp to delete
% 
% K_old = 479;        % N/m
% B_old = 11.5;       % Ns/m
% M_old = 0.6260;     % kg
% 
% Z_env_old = K_old + B_old*s + M_old*s^2;
% Z_env_filt_old = Z_env_old/(1 + tau_m*s)^2;
% 
% Hbf_tmp = minreal( feedback(H_CD, Z_env_filt_old/s) );
% Hbf_tmp_2 = minreal( (1 + Z_env_filt_old/s*(sigma_f+sigma_v*C_PI))\(sigma_f+sigma_v*C_PI) );
% Hbf(2) 
% 
% figure
% bode(Hbf_tmp)
% hold on
% bode(H_BF)
% 
% figure
% bode(Hbf_tmp, 'b', Hbf(2), 'r--', Hbf_tmp_2, 'y:')

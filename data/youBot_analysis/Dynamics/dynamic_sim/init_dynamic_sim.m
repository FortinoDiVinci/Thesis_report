% Init sim
clear all

time_start = 25;
time_end = 150;
dt = 1e-3;

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
Tc = [0.0335;0.0335;0.051]; % Torque constants: conversion from current to torque
R = [156;100;71]; % Gear ratios: conversion from motor torque to joint torque
%position controller
Kx = [20;0;0]; 
Kxi = [0;0;0];
%joint position controller
Kj = [1;1;1]*5;
Kjd = [1;1;1]*0.1;

Fv = diag([0.5,0.37,0.7]); % viscous frictions
Fs = [0.9, 1.3, 0.5];%[0.97571; 0.65131; 0.25819];  % static frictions
Fc = Fs; % Coulomb friction
dv = 1e-3; % velocity bound to avoid unstable behaviour (static frictions)
% external force to robot torque transmission efficiency
nu = [0.8554, 0.5780, 0.9997]; %[0.9488; 0.9216; 0.8345];
tau_0 = [1.0000; 0.6119; 0.8268];%[1.0234;1.0516;1.0901]; %

fz0 = 0;
% Cartesian flexibilities
%[filt_num,filt_den] = butter(2,2*pi*14,'low','s'); % filter around 14Hz
w0 = 2*pi*14;
xi = sqrt(2)/2;
filt_num = 1;
filt_den = [1/w0^2 2*xi/w0 1];
% Environment
K_env = [0;350;0]; %350%400%500%300
B_env = [0;15;0]; %20%15
M_env = [0;0.5;0]; %0.7%0.8%0.5

x0 = 0;
le = 0.1;
[p0, r, ~, jp] = DGM_youBot(th0, x0, le);

simulateYouBotKinematics([0,th0',0]);

x0 = p0(1);
z0 = p0(3);
ry0 = acos(r(1,1));

is_real_data = 0; % no real force input
is_pos_input = 0; % no real position input

% dummy var (run next section and change manual switches in simulink for 
% real input test)
real_fz.signals.values = zeros(size(time_start:dt:time_end))';
real_fz.time = (time_start:dt:time_end)';
real_fx.signals.values = zeros(size(time_start:dt:time_end))';
real_fx.time = (time_start:dt:time_end)';
real_fry.signals.values = zeros(size(time_start:dt:time_end))';
real_fry.time = (time_start:dt:time_end)';

return % This other section can be run after this one

%% Loading real data, run this section to test simulation with real force data

load('..\..\..\ball_bouncing_experiment\experimental_bench_pert\data_2020_Nov_17\data_without_impacts_2020_11_17.mat')
addpath('..\..\..\force_torque_sensor')
exp_nb = 2;
is_real_data = 1;

real_x = mocap_marker_robot_base{exp_nb}(:,1);
real_z = mocap_marker_robot_base{exp_nb}(:,3);

[real_f,real_tau,~] = forces_filtering(forces_unf{exp_nb}', torques_unf{exp_nb}', ...
            thetas{exp_nb}', t{exp_nb});

% for simulink from workspace
real_fz.signals.values = real_f(3,:)'; % fsens = f(e->r)
real_fz.time = (t{exp_nb} - t{exp_nb}(1));
real_fx.signals.values = real_f(1,:)';
real_fx.time = (t{exp_nb} - t{exp_nb}(1));
real_fry.signals.values = real_tau(2,:)';
real_fry.time = (t{exp_nb} - t{exp_nb}(1));

real_ft = [real_f(1,:);real_f(3,:);real_tau(2,:)];

% To try to accout for the fact that during the phy. interaction, the user's
% hand maintains the position along z with its impedance relation, a gain
% is added in the position control (which does not exist in the real exp.)
Kx = [20;0;0]; % z gain to maintain z position

% figure
% lsim(Hz,real_fz.signals.values,t{exp_nb})
% 
% figure
% lsim(linsys1,real_ft,t{exp_nb})

out = sim('dynamic_discrete_simulation_control',time_end);

%% After runnning simulink with real force input, run this section

sim_z = pos_rec.signals.values(:,2);
sim_flex_z = posf_rec.signals.values(:,2);
n = length(sim_z);

idx_exp = time_start/dt + (1:n);

% position
figure('DefaultAxesFontSize',14)
hold on
plot(real_fz.time(idx_exp), sim_z)
plot(real_fz.time(idx_exp), sim_flex_z)
plot(real_fz.time(idx_exp),real_z(idx_exp))
legend('Simulated stiff position', 'Simulated flex position', 'Real mocap position')
xlabel('Time (s)')
ylabel('Position (m)')
title("Robot real and simulated z endpoint position, nu=[" + ...
    num2str(reshape(nu', 1, [])) + "]")

sim_vz = vel_rec.signals.values(:,2);
addpath('../../Utils/')
% velocity
figure('DefaultAxesFontSize',14)
hold on
plot(real_fz.time(idx_exp), sim_vz)
plot(real_fz.time(idx_exp), Iu_diffcent(real_z(idx_exp), real_fz.time(idx_exp)))
legend('Simulated velocity', 'Mocap position num. derivated')
xlabel('Time (s)')
ylabel('Velocity (m.s^{-1})')
title("Robot real and simulated z endpoint velocity, nu=[" + ...
    num2str(reshape(nu', 1, [])) + "]")

th_real = theta_DH' - (thetas{exp_nb} - KUKA_offset');
th_sim = [zeros(size(th_rec.signals.values,1),1), ...
    th_rec.signals.values, zeros(size(th_rec.signals.values,1),1)];
simulateDualYouBotKinematics(th_sim, th_real,dt, real_f(:,idx_exp)');

return

%% Loading real data, run this section to test simulation with real position

load('..\..\..\ball_bouncing_experiment\experimental_bench_pert\data_2020_Nov_17\data_without_impacts_2020_11_17.mat')
addpath('..\..\..\force_torque_sensor')
exp_nb = 2;
is_pos_input = 1;

real_z = mocap_marker_robot_base{exp_nb}(:,3);

% for simulink from workspace
input_pos.signals.values = real_z;
input_pos.time = (t{exp_nb} - t{exp_nb}(1));

out = sim('dynamic_discrete_simulation_control', time_end);

%% After simulink finished execution, run this section for behaviour display

th_sim = [zeros(size(th_rec.signals.values,1),1), ...
    th_rec.signals.values, zeros(size(th_rec.signals.values,1),1)];
f_sim = [fe_rec.signals.values(:,1),zeros(size(fe_rec.signals.values,1),1),...
    fe_rec.signals.values(:,2)];
p0_sim = [p0_rec.signals.values(:,1),zeros(size(p0_rec.signals.values,1),1),...
    p0_rec.signals.values(:,2)];

tau_p = taup_rec.signals.values;

is_pert = logical(tau_p ~= 0);

limited_idx = (1:8/dt); % to avoid simulating more than 30 sec
% comment next line not to record the video of simulation
record_name = "youBot_video_K" + string(K_env(2)) + "_B" + string(B_env(2));
if exist('record_name', 'var')
    answer = questdlg('Do you want to record simulation ?', ...
	'', 'Yes','No','Cancel','Cancel');
    if strcmp(answer, 'Yes')
    simulateYouBotKinematics(th_sim(limited_idx,:), dt, f_sim(limited_idx,:), ...
        p0_sim(limited_idx,:), is_pert(limited_idx,:), record_name);
    elseif strcmp(answer, 'No')
        simulateYouBotKinematics(th_sim(limited_idx,:), dt, f_sim(limited_idx,:), ...
            p0_sim(limited_idx,:), is_pert(limited_idx,:));
    end
else
    simulateYouBotKinematics(th_sim(limited_idx,:), dt, f_sim(limited_idx,:), ...
        p0_sim(limited_idx,:), is_pert(limited_idx,:));
end

time = time_start:dt:time_end;
figure('DefaultAxesFontSize',14)
subplot(2,1,1)
hold on
plot(time, pos_rec.signals.values(:,2))
p1 = plot(time(any(is_pert,2)), pos_rec.signals.values(any(is_pert,2),2), 'o');
%p1(1).Color(4) = 0.1;<
title('Endpoint Position')
legend('z', 'z pert')
ylabel('(m)')
xlabel('(s)')
subplot(2,1,2)
hold on
plot(time, f_sim(:,3))
p2 = plot(time(any(is_pert,2)), f_sim(any(is_pert,2),3), 'o');
%p2(1).Color(4) = 0.1;
title('Endpoint Force')
legend('z', 'z pert')
ylabel('(N)')
xlabel('(s)')

figure('DefaultAxesFontSize',14)
plot(tau_ctrl)
legend('\tau_2', '\tau_3', '\tau_4')
return

%% After linear analysis using control design tool & loading linsys model
% This section is outdated...
ssH = ss(linsys1.A, linsys1.B, linsys1.C, linsys1.D,dt);
H_ctrl = tf(ssH);

figure('DefaultAxesFontSize',14)
hold on, grid on
bode(linsys1(2,2))
bode(linsys2(2,2))
bode(Hz_rob{3}/s)
legend('Simulink LA, complete','Simulink LA, z ctrl','Analytical TF')
title('Robot close loop along the z axis: Pz = Hz*Fz')

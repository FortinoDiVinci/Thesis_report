% This script reproduce the methodology for impedance estimation for the
% experiment conducted on the robot but this time for the simulation
% KBM_sim.slx
% This is meant to validate the methodology in an ideal case scenario with
% known impedance parameters

clear all
addpath('../utils')

%% Simulation for data generation
% simulation parameters
Mv = 0.1;
Bv = 5;
Kv = 200;
dt = 1e-3;
pert_mag = 1;
pert_space = ceil(3.3/dt); % samples
pert_duration = 0.060/dt; % samples
t_max = (101*pert_space -1)*dt;

% to use real data set ext_signal to 1
ext_signal = 1; % set to 0 to use default sine wave for force
if ext_signal
    exp_nb = 2;
    load('..\data_2020_Nov_17\data_without_impacts_2020_11_17.mat')
    addpath('..\..\..\force_torque_sensor')
    real_f = forces_filtering(forces_unf{exp_nb}', torques_unf{exp_nb}', ...
            thetas{exp_nb}', t{exp_nb});
    input_force.signals.values = -real_f(3,:)';
    input_force.time = (t{exp_nb} - t{exp_nb}(1));
    if t{exp_nb}(end) < t_max
        t_max = t{exp_nb}(end);
    end
else
    input_force.signals.values = zeros(size(0:dt:t_max))';
    input_force.time = (0:dt:t_max)';
end

% launch sim
mdl = 'KBM_sim';
out = sim(mdl,t_max);

% get relevant data
fz = out.force.data;
z = out.position.data;
t = out.force.Time;
% get the rising edges indexes of the perturbations
pert_idx = find(diff(out.perturbations.data) > 0)';
pert_val = pert_mag.*ones(size(pert_idx)); 

%% Impedance estimation algorithm
% PARAMETERS
idx_wndw_virt_traj   = ceil(0.200/dt); % 200ms (position)
idx_wndw_virt_f_traj = ceil(0.065/dt); % 65ms  (force) 
idx_wndw_imp_eval    = ceil(0.200/dt); % 200ms       
idx_delay            = ceil(0.000/dt); % 0ms
idx_window = max(idx_wndw_imp_eval, idx_wndw_virt_traj);
nb_param = 3; % K B M

% DATA PRE-PROCESSING
delta_z = DIFF_TRAJECT(idx_window, idx_wndw_virt_traj, z, t, pert_idx, ...
    pert_val, idx_delay);
delta_fz = DIFF_TRAJECT(idx_window, idx_wndw_virt_f_traj, fz, t, pert_idx, ...
    pert_val, idx_delay);

delta_z.computeDiffTraject('VirtTrajMethod', 'spline');
delta_fz.computeDiffTraject('VirtTrajMethod', 'filterPlus');
delta_z.computeDerivatives(); 

% IMPEDANCE EVAL
impedance = IMPEDANCE_DATA(nb_param, length(pert_idx), idx_wndw_imp_eval);
impedance.init_phi(delta_z.diff_traject(1:idx_wndw_imp_eval,:), ...
    delta_z.d_diff_traject(1:idx_wndw_imp_eval,:), ... % speed
    delta_z.dd_diff_traject(1:idx_wndw_imp_eval,:)); % acceleration
impedance.init_y(delta_fz.diff_traject(1:idx_wndw_imp_eval,:));
impedance.lsq(); % least square optimization evaluation

%% Data display

mean_stiff = nanmean(impedance.xi(1,:));
mean_damp = mean(impedance.xi(2,:));
mean_mass = mean(impedance.xi(3,:));

disp("Mean Stiffness: " + string(mean_stiff))
disp("Mean Damping: " + string(mean_damp))
disp("Mean Mass: " + string(mean_mass))

% RELATIVE ERRORS
disp("Mean Stiffness rel. error: " + string(100*abs(mean_stiff - K)/K) ...
+ "%")
disp("Mean Damping rel. error: " + string(100*abs(mean_damp - B)/B) ...
+ "%")
disp("Mean Mass rel. error: " + string(100*abs(mean_mass - M)/M) ...
+ "%")

% Standard deviations
std_stiff = nanstd(impedance.xi(1,:));
std_damp = nanstd(impedance.xi(2,:));
std_mass = nanstd(impedance.xi(3,:));

% relative standard deviations
disp("Stiffness rel. std error: " + string(100*std_stiff/abs(mean_stiff)) ...
+ "%")
disp("Damping rel. std error: " + string(100*std_damp/abs(mean_damp)) ...
+ "%")
disp("Mass rel. std error: " + string(100*std_mass/abs(mean_mass)) ...
+ "%")

% Statistics

figure('DefaultAxesFontSize',14)
subplot(3,1,1)
hold on
plot(impedance.rel_std(1,:))
plot(abs(impedance.xi(1,:)-K)/K)
title('Stiffness')
legend('\sigma_x%','\epsilon%')
subplot(3,1,2)
hold on
plot(impedance.rel_std(2,:))
plot(abs(impedance.xi(2,:)-B)/B)
title('Damping')
legend('\sigma_x%','\epsilon%')
subplot(3,1,3)
hold on
plot(impedance.rel_std(3,:))
plot(abs(impedance.xi(3,:)-M)/M)
title('Mass')
legend('\sigma_x%','\epsilon%')

clear all

addpath('utils')
addpath('../../force_torque_sensor')
addpath('../../utils')

%% PARAMETERS
% MACRO
FILE_NAME = 'preliminary_experimental_data/data_vfo_3_phases.mat';
% PARAMS
% data processing
low_pass_cutoff_freq = 50; % Input signal are lp filt. before computation
filter_order = 2;
% impedance and trajectory windows 
% (the following param consider a sampling frequency of 1kHz)
wndw_virt_traj   = 200; % 200ms (position)
wndw_virt_f_traj = 065; % 65ms  (force) 
wndw_imp_eval    = 200; % 200ms       
p_delay            = 0; % 0ms
window = max(wndw_imp_eval, wndw_virt_traj);
%
nb_param = 3; % K B M

%% DATA LOADING

load(FILE_NAME);
var_list = who;
necessary_var = ["t","thetas", "forces_unf", "mocap_marker_robot_base", ...
    "t_dist", "dist"];

for i_var = necessary_var
    if ~any(strcmp(i_var,var_list))
        error("The data in " + FILE_NAME + " does not have all the " + ...
            "required variables. " + string(i_var) + " is missing.")
    end
end

if ~exist('dt', 'var')
    dt = 1e-3;
end

tot_nb_exp = length(t);

%% DATA PRE-PROCESSING

z = {};
fz = {};

[b,a] = butter(filter_order,low_pass_cutoff_freq/(1/(2*dt)),'low'); 

% force and position pre-processing
for exp_nb = tot_nb_exp:-1:1
    
    torques_unf = forces_unf{exp_nb}; % the torque is irrelevant here
    f_tmp = forces_filtering(forces_unf{exp_nb}', torques_unf', ...
        thetas{exp_nb}', t{exp_nb}); 
    
    z{exp_nb} = filtfilt(b,a,mocap_marker_robot_base{exp_nb}(:,3));
    fz{exp_nb} = -filtfilt(b,a,f_tmp(3,:))'; % f(r->e) = -f(e->r) = -fsens
    
end

% disturbance timing extractions
for exp_nb = tot_nb_exp:-1:1
    % get only the rising edges of perturbations
    dist_timings = t_dist{exp_nb}(1:2:end);
    dist_val{exp_nb} = dist{exp_nb}(1:2:end);
    for pert_idx = 1:length(dist_timings)
        idx_perts{exp_nb}(pert_idx) = find(t{exp_nb} >= ...
            dist_timings(pert_idx), 1, 'first');
    end
    % if the experiment was interrupted during the last perturbation
    if ( idx_perts{exp_nb}(end) + window + p_delay)  > length(t{exp_nb} )
        % the last perturbation will no be use for impedance estimation
        idx_perts{exp_nb} = idx_perts{exp_nb}(1:end-1);
        dist_val{exp_nb} = dist_val{exp_nb}(1:end-1);
    end
end

%% DATA PROCESSING 

delta_z = {};
delta_fz = {};
impedance = {};
impedance_arx = {};

% extraction of the delta of position and force
for exp_nb = tot_nb_exp:-1:1
    % delta z and its derivatives
    delta_z{exp_nb} = DIFF_TRAJECT(window, wndw_virt_traj, z{exp_nb}, ...
        t{exp_nb}, idx_perts{exp_nb}, dist_val{exp_nb}, p_delay);
    delta_z{exp_nb}.computeDiffTraject('VirtTrajMethod', 'spline');
    delta_z{exp_nb}.computeDerivatives();
    % delta fz
    delta_fz{exp_nb} = DIFF_TRAJECT(window, wndw_virt_f_traj, fz{exp_nb},...
        t{exp_nb}, idx_perts{exp_nb}, dist_val{exp_nb}, p_delay);
    delta_fz{exp_nb}.computeDiffTraject('VirtTrajMethod', 'filterPlus'); 
end

% impedance computation
for exp_nb = tot_nb_exp:-1:1   
    impedance{exp_nb} = IMPEDANCE_DATA(nb_param, length(idx_perts{exp_nb}),...
        wndw_imp_eval);
    impedance{exp_nb}.init_phi(delta_z{exp_nb}.diff_traject(1:wndw_imp_eval,:), ...
        delta_z{exp_nb}.d_diff_traject(1:wndw_imp_eval,:), ... % speed
        delta_z{exp_nb}.dd_diff_traject(1:wndw_imp_eval,:)); % acceleration
    impedance{exp_nb}.init_y(delta_fz{exp_nb}.diff_traject(1:wndw_imp_eval,:));
    impedance_arx{exp_nb} = copyObj(impedance{exp_nb});
    impedance{exp_nb}.lsq(); % least square optimization evaluation
    impedance_arx{exp_nb}.arx();
end

%% DATA POST-PROCESSING (statistical analysis)

stiff = [];
damp = [];
mass = [];
rho = [];

stiff_a = [];
damp_a = [];
mass_a = [];
rho_a = [];

for exp_nb = 1:tot_nb_exp  
    stiff = [stiff, impedance{exp_nb}.xi(1,:)];
    damp = [damp, impedance{exp_nb}.xi(2,:)];
    mass = [mass, impedance{exp_nb}.xi(3,:)];
    rho = [rho, impedance{exp_nb}.xi(4,:)];   
    stiff_a = [stiff_a, impedance_arx{exp_nb}.xi(1,:)];
    damp_a = [damp_a, impedance_arx{exp_nb}.xi(2,:)];
    mass_a = [mass_a, impedance_arx{exp_nb}.xi(3,:)];
    rho_a = [rho_a, impedance_arx{exp_nb}.xi(4,:)];   
end

%% Display
exp_nb = 1;
% trajectory estimation
figure
ax(1) = subplot(2,1,1);
hold on
plot(delta_z{exp_nb}.time, delta_z{exp_nb}.complete_traject)
plot(delta_z{exp_nb}.t_traject, delta_z{exp_nb}.virt_traject, 'r')
title('Position')
legend('meas.', 'virtual')
xlabel('Time (s)')
ylabel('Distance (m)')
ax(2) = subplot(2,1,2);
hold on
plot(delta_fz{exp_nb}.time, delta_fz{exp_nb}.complete_traject)
plot(delta_fz{exp_nb}.t_traject, delta_fz{exp_nb}.virt_traject, 'r')
plot(delta_z{exp_nb}.t_traject(3:end-2,:), impedance{exp_nb}.rec_y + delta_fz{exp_nb}.virt_traject(3:end-2,:), 'c--')
plot(delta_z{exp_nb}.t_traject(3:end-2,:), impedance_arx{exp_nb}.rec_y + delta_fz{exp_nb}.virt_traject(3:end-2,:), 'm--')
title('Force')
legend('meas.', 'virtual')
xlabel('Time (s)')
ylabel('Force (N)')
linkaxes(ax,'x')

% parameters identification
figure('DefaultAxesFontSize',14)
subplot(3,1,1)
hold on
plot(stiff)
plot(stiff_a)
legend('LSQ', 'ARX')
subplot(3,1,2)
hold on
plot(damp)
plot(damp_a)
legend('LSQ', 'ARX')
subplot(3,1,3)
hold on
plot(mass)
plot(mass_a)
legend('LSQ', 'ARX')

[mean(rmoutliers(stiff)), mean(rmoutliers(stiff_a))]
[mean(rmoutliers(damp)), mean(rmoutliers(damp_a))]
[mean(rmoutliers(mass)), mean(rmoutliers(mass_a))]

all_imp_arx = [impedance_arx{:}];
all_imp_lsq = [impedance{:}];

% 
figure
subplot(2,1,1)
hold on
p1 = plot([all_imp_lsq.rmse]);
p2 = plot([all_imp_arx.rmse]);
ylim([0,50]);
title('RMSE reconstruction errors')
legend([p1,p2],{'LSQ', 'ARX'})
subplot(2,1,2)
hold on
p1 = plot([all_imp_lsq.r_2]);
yyaxis right
p2 = plot([all_imp_arx.r_2]);
title('R^2 scores')
legend([p1,p2],{'LSQ', 'ARX'})
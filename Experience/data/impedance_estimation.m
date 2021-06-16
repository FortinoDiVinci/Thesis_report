clear all

addpath('../../data/ball_bouncing_experiment/experimental_bench_pert/utils')
addpath('../../data/utils')
addpath('../../data/force_torque_sensor')

%% PARAMETERS
% MACRO
% FILE_NAME = 'preliminary_experimental_data/data_vfo_3_phases.mat';
FILE_NAME = 'SB2021_new_data_v2.mat';
SAVED_FILE_NAME = 'SB2021_new_data_impedance_v2.mat';
% PARAMS
% data processing
low_pass_cutoff_freq = 50; % Input signal are lp filt. before computation
filter_order = 2;
% impedance and trajectory windows 
% (the following param consider a sampling frequency of 1kHz)
wndw_virt_traj   = 200; % 200ms (position)
wndw_virt_f_traj = 100; % 65ms  (force) 
wndw_imp_eval    = 200; % 200ms       
p_delay          = 12; % 0ms
f_delay          = 0;
window = max(wndw_imp_eval, wndw_virt_traj);
%
nb_param = 3; % K B M

%% DATA LOADING

load(FILE_NAME);
var_list = who;
necessary_var = ["t","q", "ft_sensor", "mocap_robot_endpoint", ...
    "t_dist", "dist_val"];

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
    
    f_tmp = forces_filtering(ft_sensor{exp_nb}(:,1:3)', ft_sensor{exp_nb}(:,4:6)', ...
        q{exp_nb}', t{exp_nb}); 
    
    z{exp_nb} = filtfilt(b,a,mocap_robot_endpoint{exp_nb}(:,3));
    fz{exp_nb} = -filtfilt(b,a,f_tmp(3,:))'; % f(r->e) = -f(e->r) = -fsens
    
end
dist_val_raw = dist_val;
clear dist_val;
% disturbance timing extractions
for exp_nb = tot_nb_exp:-1:1
    % get only the rising edges of perturbations
    dist_timings = t_dist{exp_nb}(1:2:end);
    dist_val{exp_nb} = dist_val_raw{exp_nb}(1:2:end);
    for pert_idx = 1:length(dist_timings)
        idx_perts{exp_nb}(pert_idx) = find(t{exp_nb} >= ...
            dist_timings(pert_idx), 1, 'first');
    end
    % if the experiment was interrupted during the last perturbation
    if (idx_perts{exp_nb}(end) + window + max(p_delay,f_delay)) > length(t{exp_nb})
        % the last perturbation will no be use for impedance estimation
        idx_perts{exp_nb} = idx_perts{exp_nb}(1:end-1);
        dist_val{exp_nb} = dist_val{exp_nb}(1:end-1);
    end
end

for i = 1:length(t)
    t{i} = t{i} - t{i}(1);
end

%% DATA PROCESSING 
delta_z = {};
delta_fz = {};
impedance = {};

% extraction of the delta of position and force
for exp_nb = tot_nb_exp:-1:1
    % delta z and its derivatives
    delta_z{exp_nb} = DIFF_TRAJECT(window, wndw_virt_traj, z{exp_nb}, ...
        t{exp_nb}, idx_perts{exp_nb}, dist_val{exp_nb}, p_delay);
    delta_z{exp_nb}.computeDiffTraject('VirtTrajMethod', 'spline');
    delta_z{exp_nb}.computeDerivatives();
    % delta fz
    delta_fz{exp_nb} = DIFF_TRAJECT(window, wndw_virt_f_traj, fz{exp_nb},...
        t{exp_nb}, idx_perts{exp_nb}, dist_val{exp_nb}, f_delay);
    delta_fz{exp_nb}.computeDiffTraject('VirtTrajMethod', 'sineOptM', ...
        'OptSolverName', 'lsqnonlin', 'OptNbSine', 3, 'OptlinearComp', 0); 
end

% impedance computation
for exp_nb = tot_nb_exp:-1:1
    impedance{exp_nb} = IMPEDANCE_DATA(nb_param, length(idx_perts{exp_nb}(1:size(delta_z{exp_nb}.diff_traject,2))),...
        wndw_imp_eval);
    impedance{exp_nb}.init_phi(delta_z{exp_nb}.diff_traject(1:wndw_imp_eval,:), ...
        delta_z{exp_nb}.d_diff_traject(1:wndw_imp_eval,:), ... % speed
        delta_z{exp_nb}.dd_diff_traject(1:wndw_imp_eval,:)); % acceleration
    impedance{exp_nb}.init_y(delta_fz{exp_nb}.diff_traject(1:wndw_imp_eval,:));
    impedance{exp_nb}.arx('NulInitialCond');
    % position reconstruction from force input (causal sim)
    impedance{exp_nb}.causalSim(dt,'NulInitialCond'); 
end

save(SAVED_FILE_NAME, 'delta_fz', 'delta_z', 'impedance',...
    'bounce_err', 'z_b', 'z_p', 'exp_parameters', 'idx_ball_off_ramp');
clear all

addpath('utils')
addpath('../../force_torque_sensor')

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
p_delay            = 0;   % 0ms
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
    impedance{exp_nb}.lsq(); % least square optimization evaluation
end

%% DATA POST-PROCESSING (statistical analysis)

stiff = [];
damp = [];
mass = [];
rho = [];

for exp_nb = 1:tot_nb_exp  
    stiff = [stiff, impedance{exp_nb}.xi(1,:)];
    damp = [damp, impedance{exp_nb}.xi(2,:)];
    mass = [mass, impedance{exp_nb}.xi(3,:)];
    rho = [rho, impedance{exp_nb}.xi(4,:)];   
end
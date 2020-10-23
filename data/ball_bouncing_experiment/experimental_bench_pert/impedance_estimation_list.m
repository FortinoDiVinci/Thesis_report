%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   IMPEDANCE ESTIMATION
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% This script make estimation of the virtual trajectory of the arm after
% a perturbation occured. To be able to estimate the impedance, as stated
% in [ref papier CASE], the virtual trajectory need to be computed.
% Here, the virtual trajectories and forces are approached using cubic 
% spline interpolation at 1 kHz. The estimation is based on a trajectory
% of 200 ms, using 100 ms both before and after the estimated time window,
% for the interpolation. A delay of 15 ms is injected to abide by the
% latency of the system ? 

clear all
close all

addpath('../../force_torque_sensor')
addpath('../../youBot_analysis/Utils')

%file_name = 'successful_exp_data.mat';
%file_name = 'data_impact2.mat';
%file_name = 'preliminary_data.mat';
%file_name = 'calibration_bench_data_26_08_20';
%file_name = 'data_eval_pert.mat';
file_name = 'data_vfo_3_phases.mat';
%file_name = 'data_mso_3_phases.mat';

load(file_name);
STATIC_EXP = cell(size(folder_names));
STATIC_EXP(:,:) = {0};
%STATIC_EXP(4:end) = {1};

%%%%%%%%%%%%%%%%%%
%% MACROS & variables
%%%%%%%%%%%%%%%%%%

DISP_STIFFNESS_DISTRIBUTION = 0;
SORT_PERT_BY_PHASE = 1; 

if ~exist('dt', 'var')
    dt = 1e-3;
end
% time evaluation variables
idx_window_virt_traj= ceil(0.200/dt); % 200ms
idx_window_imp_eval = ceil(0.200/dt); % 200ms       
idx_delay           = ceil(0.000/dt); % 015ms

nb_param            = 3; % for the impedance model, should be between 1 & 3
                         % - 1: F = Kx + e
                         % - 2: F = Kx + Bdx + e
                         % - 3: F = Kx + Bdx + Iddx + e

min_r2              = 0.75;
                         
% time of the end and start of each experiment need to be entered manually 
% if necessary else zeros need to be filled 
%T_END = [130,190,173.5,190,234,166.5,210.5,209,206,182,0,171.5];
%T_START = [0,0,0,0,0,98,0,173,0,0,0,0];     
%T_START = [138,2,2,2]; 
%T_END = [150,20,20,20]; 
%T_START = [0,0,0,0]; 
%T_END = [0,0,0,0]; 

z = {};
fz = {};

delta_z = {};
delta_fz = {};
impedance = {};

r_sq_all = [];
%r_sq_all_fit = [];
sign_all = [];
stiff_all = [];
damp_all = [];
mass_all = [];
rho_all = [];

if ~exist("NO_TRQ_CMD_DIST", "var")
    NO_TRQ_CMD_DIST = cell(size(folder_names));
    NO_TRQ_CMD_DIST (:,:) = {1};
end

% Z force and position extraction and low pass filtering (25 Hz)
for exp_nb = 1:length(folder_names)

    % data filtering
    if NO_MOCAP{exp_nb}
        z_temp = z_p{exp_nb};
    else
        z_temp = mocap_marker_robot_base{exp_nb}(:,3); % position of the motion capture
    end
    % the low pass filtering is not done is the following function
    [f,~,~]= forces_filtering(forces_unf{exp_nb}', torques_unf{exp_nb}', ...
        thetas{exp_nb}', t{exp_nb});
    fz_temp = f(3,:);
    fc = 25; % cut off frequency
    [b,a] = butter(2,fc/(1/(2*dt)),'low'); 

    z{exp_nb} = filtfilt(b,a,z_temp);
    fz{exp_nb} = -1*filtfilt(b,a,fz_temp);
    
end
    
for exp_nb = 1:length(folder_names)
    
    %% Perturbation indexes extraction 
    
    clear idx_perts dist_timings
    % case with perturbation introduced using rqt, in a static config
    if NO_DISTURBANCE{exp_nb} && ~NO_TRQ_CMD_DIST{exp_nb}
        dist_timings_tmp = t_trq_cmd{exp_nb}( sign(diff(val_trq_cmd{exp_nb}))~=0 );
        dist_timings = dist_timings_tmp(1:2:end-6);
        for pert_idx = 1:length(dist_timings)
            idx_perts(pert_idx) = find(t{exp_nb} >= dist_timings(pert_idx), 1, 'first');
        end
    elseif NO_DISTURBANCE{exp_nb}
        dist_duration = NaN;
        dist_timings = [];
        idx_perts = [];
    % nominal case (expected for the normal behaviour of the experiment)
    else
        % set the disturbances by couples, compute diff and then the average
        dist_duration = mean(diff(reshape(t_dist{exp_nb}, 2, length(t_dist{exp_nb})/2)));
        dist_timings = t_dist{exp_nb}(1:2:end); % extract only the start of the disturbance
        dist_val{exp_nb} = dist{exp_nb}(1:2:end);
        % extraction of the perturbation indexes
        for pert_idx = 1:length(dist_timings)
            idx_perts(pert_idx) = find(t{exp_nb} >= dist_timings(pert_idx), 1, 'first');
        end
    end
    %size(idx_perts)
    %size(dist_timings)
    % if the experiment was interrupted during the last perturbation
    reduced_dist{exp_nb} = 0;
    if ~isempty(idx_perts)
        if ( idx_perts(end) + idx_window_virt_traj + idx_delay)  > length(t{exp_nb})
            idx_perts = idx_perts(1:end-1);
            dist_val{exp_nb} = dist_val{exp_nb}(1:end-1);
            dist_timings = dist_timings(1:end-1);
            reduced_dist{exp_nb} = 1;
        end
    end
    %disp("pert size: " + string(size(idx_perts)))
    
    %% Cycle extraction and processing
    
    
    
    %% Trajectories extraction & estimation
    
    if STATIC_EXP{exp_nb}
        delta_z{exp_nb} = DIFF_TRAJECT(idx_window_virt_traj, idx_window_virt_traj, ...
            z{exp_nb}, t{exp_nb}, idx_perts, dist_val{exp_nb}, idx_delay);
        delta_fz{exp_nb} = DIFF_TRAJECT(idx_window_virt_traj, idx_window_virt_traj, ...
            fz{exp_nb}, t{exp_nb}, idx_perts, dist_val{exp_nb}, idx_delay);
     
        delta_z{exp_nb}.computeDiffTraject('VirtTrajMethod', 'static');
        delta_fz{exp_nb}.computeDiffTraject('VirtTrajMethod', 'static');
        disp('static exp: ' + string(exp_nb))
    else % nominal case
        delta_z{exp_nb} = DIFF_TRAJECT(idx_window_virt_traj, idx_window_virt_traj, ...
            z{exp_nb}, t{exp_nb}, idx_perts, dist_val{exp_nb}, idx_delay);
        delta_fz{exp_nb} = DIFF_TRAJECT(idx_window_virt_traj, idx_window_virt_traj, ...
            fz{exp_nb}, t{exp_nb}, idx_perts, dist_val{exp_nb}, idx_delay);
        
        delta_z{exp_nb}.computeDiffTraject('VirtTrajMethod', 'spline'); % z0 - z
        delta_fz{exp_nb}.computeDiffTraject('VirtTrajMethod', 'spline', 'DiffDirection', 'neg'); % fz - fz0
    end

    delta_z{exp_nb}.computeDerivatives();   
    
    %% Impedance estimation 
    
    impedance{exp_nb} = IMPEDANCE_DATA(nb_param, length(dist_timings), idx_window_imp_eval);
    impedance{exp_nb}.init_phi(delta_z{exp_nb}.diff_traject(1:idx_window_imp_eval,:), ...
        delta_z{exp_nb}.d_diff_traject(1:idx_window_imp_eval,:), ... % speed
        delta_z{exp_nb}.dd_diff_traject(1:idx_window_imp_eval,:)); % acceleration
    impedance{exp_nb}.init_y(delta_fz{exp_nb}.diff_traject(1:idx_window_imp_eval,:));
    impedance{exp_nb}.lsq(); % least square optimization evaluation
    
    if NO_MOCAP{exp_nb}
        continue
    end
    
    %% Data concatenation
    
    sign_all = [sign_all, sign(delta_fz{exp_nb}.pert_val)'];
    r_sq_all = [r_sq_all, impedance{exp_nb}.r_2'];
    %r_sq_all_fit = [r_sq_all_fit, impedance{exp_nb}.r_2_fit'];
    stiff_all = [stiff_all, impedance{exp_nb}.xi(1,:)];  
    
    switch nb_param
        case 1
            rho_all = [rho_all, impedance{exp_nb}.xi(2,:)]; 
        case 2
            damp_all = [damp_all, impedance{exp_nb}.xi(2,:)]; 
            rho_all = [rho_all, impedance{exp_nb}.xi(3,:)]; 
        case 3
            damp_all = [damp_all, impedance{exp_nb}.xi(2,:)]; 
            mass_all = [mass_all, impedance{exp_nb}.xi(3,:)]; 
            rho_all = [rho_all, impedance{exp_nb}.xi(4,:)]; 
    end
       
end

%% Post evaluation processing 

%stiff_all_cln = stiff_all( (stiff_all>0) & (damp_all>0));
%damp_all_cln = damp_all( (stiff_all>0) & (damp_all>0));

stiff_all_cln = stiff_all( (stiff_all>0) & (stiff_all<3e3));
damp_all_cln = damp_all( (stiff_all>0));

stiff_r2_sup = stiff_all(r_sq_all > min_r2);

%damp_all_cln = damp_all_cln(stiff_all_cln<2e3);
%stiff_all_cln = stiff_all_cln(stiff_all_cln<2e3);

std_stif = nanstd(stiff_all);
med_stif = nanmedian(stiff_all);
std_stif_cln = nanstd(stiff_all_cln);
med_stif_cln = nanmedian(stiff_all_cln);

std_damp = nanstd(damp_all);
med_damp = nanmedian(damp_all);
std_damp_cln = nanstd(damp_all_cln);
med_damp_cln = nanmedian(damp_all_cln);

%% Display

if DISP_STIFFNESS_DISTRIBUTION

    figure
    subplot(2,1,1)
    plot(stiff_all, 'o')
    hold on
    line([1 size(stiff_all,2)], [med_stif, med_stif], 'Color','red','LineStyle','--','linewidth',2);
    line([1 size(stiff_all,2)], [med_stif+std_stif, med_stif+std_stif], 'Color','black','LineStyle','--','linewidth',1);
    line([1 size(stiff_all,2)], [med_stif-std_stif, med_stif-std_stif], 'Color','black','LineStyle','--','linewidth',1);
    title('Unclean Stiffness, std = ' + string(std_stif) + ', mediane = ' + string(med_stif))
    subplot(2,1,2)
    plot(damp_all, 'x')
    hold on
    line([1 size(damp_all,2)], [med_damp, med_damp], 'Color','red','LineStyle','--','linewidth',2);
    line([1 size(damp_all,2)], [med_damp+std_damp, med_damp+std_damp], 'Color','black','LineStyle','--','linewidth',1);
    line([1 size(damp_all,2)], [med_damp-std_damp, med_damp-std_damp], 'Color','black','LineStyle','--','linewidth',1);
    title('Damping, std = ' + string(std_damp) + ', mediane = ' + string(med_damp))

    figure
    subplot(2,1,1)
    plot(stiff_all_cln, 'o')
    hold on
    line([1 size(stiff_all_cln,2)], [med_stif_cln, med_stif_cln], 'Color','red','LineStyle','--','linewidth',2);
    line([1 size(stiff_all_cln,2)], [med_stif_cln+std_stif_cln, med_stif_cln+std_stif_cln], 'Color','black','LineStyle','--','linewidth',1);
    line([1 size(stiff_all_cln,2)], [med_stif_cln-std_stif_cln, med_stif_cln-std_stif_cln], 'Color','black','LineStyle','--','linewidth',1);
    title('Stiffness, std = ' + string(std_stif_cln) + ', mediane = ' + string(med_stif_cln))
    subplot(2,1,2)
    plot(damp_all_cln, 'x')
    hold on
    line([1 size(stiff_all_cln,2)], [med_damp_cln, med_damp_cln], 'Color','red','LineStyle','--','linewidth',2);
    line([1 size(stiff_all_cln,2)], [med_damp_cln+std_damp_cln, med_damp_cln+std_damp_cln], 'Color','black','LineStyle','--','linewidth',1);
    line([1 size(stiff_all_cln,2)], [med_damp_cln-std_damp_cln, med_damp_cln-std_damp_cln], 'Color','black','LineStyle','--','linewidth',1);
    title('Damping, std = ' + string(std_damp_cln) + ', mediane = ' + string(med_damp_cln))

end

if SORT_PERT_BY_PHASE
    
    % In the main program the timing of the perturbation is given by  
    % decimal values of the disturbance : DIST_VAL + 0.1*cycle_nb
    % cycle 1 = 25%, cycle 2 = 50% cycle 3 = 75%
    
    epsilon = 0.001;
    
    idx_pert_cycle_1 = [];
    idx_pert_cycle_2 = [];
    idx_pert_cycle_3 = [];
    
    idx_pert_cycle_1_pos = [];
    idx_pert_cycle_2_pos = [];
    idx_pert_cycle_3_pos = [];
    
    idx_pert_cycle_1_neg = [];
    idx_pert_cycle_2_neg = [];
    idx_pert_cycle_3_neg = [];
    
    %data = [stiff_all, damp_all, mass_all, rho_all, r_sq_all];
    
    data_new(1) = STATISTIC_DATA(stiff_all, 'Stiffness', 'N/m');
    data_new(2) = STATISTIC_DATA(damp_all, 'Damping', 'N.s/m');
    data_new(3) = STATISTIC_DATA(mass_all, 'Mass', 'kg');
    data_new(4) = STATISTIC_DATA(mass_all, 'Rho', 'N');
    data_new(5) = STATISTIC_DATA(r_sq_all, 'R^2', '');
    
    for exp_nb = 1:length(folder_names)
    
        idx_pert_cycle_1 = [idx_pert_cycle_1; (compare2eps(dist_val{exp_nb}, 10.1, epsilon) | compare2eps(dist_val{exp_nb}, -9.9, epsilon))];
        idx_pert_cycle_2 = [idx_pert_cycle_2; (compare2eps(dist_val{exp_nb}, 10.2, epsilon) | compare2eps(dist_val{exp_nb}, -9.8, epsilon))];
        idx_pert_cycle_3 = [idx_pert_cycle_3; (compare2eps(dist_val{exp_nb}, 10.3, epsilon) | compare2eps(dist_val{exp_nb}, -9.7, epsilon))];

        idx_pert_cycle_1_pos = [idx_pert_cycle_1_pos; compare2eps(dist_val{exp_nb}, 10.1, epsilon)];
        idx_pert_cycle_2_pos = [idx_pert_cycle_2_pos; compare2eps(dist_val{exp_nb}, 10.2, epsilon)];
        idx_pert_cycle_3_pos = [idx_pert_cycle_3_pos; compare2eps(dist_val{exp_nb}, 10.3, epsilon)];

        idx_pert_cycle_1_neg = [idx_pert_cycle_1_neg; compare2eps(dist_val{exp_nb}, -9.9, epsilon)];
        idx_pert_cycle_2_neg = [idx_pert_cycle_2_neg; compare2eps(dist_val{exp_nb}, -9.8, epsilon)];
        idx_pert_cycle_3_neg = [idx_pert_cycle_3_neg; compare2eps(dist_val{exp_nb}, -9.7, epsilon)];
        
    end
    
    for data_nb = 1:length(data_new)
        %
        data_new(data_nb).clear_statistics(); % if data was not cleared properly
        data_new(data_nb).compute_new_statistics();
        %
        data_new(data_nb).compute_new_statistics(logical(idx_pert_cycle_1));
        data_new(data_nb).compute_new_statistics(logical(idx_pert_cycle_2));
        data_new(data_nb).compute_new_statistics(logical(idx_pert_cycle_3));
        %
        data_new(data_nb).compute_new_statistics(logical(idx_pert_cycle_1_pos));
        data_new(data_nb).compute_new_statistics(logical(idx_pert_cycle_2_pos));
        data_new(data_nb).compute_new_statistics(logical(idx_pert_cycle_3_pos));
        %
        data_new(data_nb).compute_new_statistics(logical(idx_pert_cycle_1_neg));
        data_new(data_nb).compute_new_statistics(logical(idx_pert_cycle_2_neg));
        data_new(data_nb).compute_new_statistics(logical(idx_pert_cycle_3_neg));
    end
    
    analysis_type = ["All data", "Cycle 1", "Cycle 2", "Cycle 3", "Cycle 1+", ...
        "Cycle 2+", "Cycle 3+", "Cycle 1-", "Cycle 2-", "Cycle 3-"]; 
    
    for data_nb = 1:length(data_new)
        
        figure
        subplot(2,2,1)
        histogram(data_new(data_nb).data, 15, 'Normalization','probability')
        legend(analysis_type(1))
        title(data_new(data_nb).name)
        %
        subplot(2,2,2)
        histogram(data_new(data_nb).data(data_new(data_nb).idx{2}), 10, 'Normalization','probability')
        hold on
        histogram(data_new(data_nb).data(data_new(data_nb).idx{3}), 10, 'Normalization','probability')
        histogram(data_new(data_nb).data(data_new(data_nb).idx{4}), 10, 'Normalization','probability')
        legend(analysis_type(2:4))
        title(data_new(data_nb).name)
        %
        subplot(2,2,3)
        histogram(data_new(data_nb).data(data_new(data_nb).idx{5}), 10, 'Normalization','probability')
        hold on
        histogram(data_new(data_nb).data(data_new(data_nb).idx{6}), 10, 'Normalization','probability')
        histogram(data_new(data_nb).data(data_new(data_nb).idx{7}), 10, 'Normalization','probability')
        legend(analysis_type(5:7))
        title(data_new(data_nb).name)
        %
        subplot(2,2,4)
        histogram(data_new(data_nb).data(data_new(data_nb).idx{8}), 10, 'Normalization','probability')
        hold on
        histogram(data_new(data_nb).data(data_new(data_nb).idx{9}), 10, 'Normalization','probability')
        histogram(data_new(data_nb).data(data_new(data_nb).idx{10}), 10, 'Normalization','probability')
        legend(analysis_type(8:10))
        title(data_new(data_nb).name)
        %
        for j = 1:length(analysis_type)
            disp("Average " + data_new(data_nb).name + " for " + analysis_type(j) + ...
                ": " + num2str(data_new(data_nb).mean(j),4) + data_new(data_nb).unit)
            disp("Median " + data_new(data_nb).name + " for " + analysis_type(j) + ...
                ": " + num2str(data_new(data_nb).median(j),4) + data_new(data_nb).unit)
            disp("Standart deviation " + data_new(data_nb).name + " for " + analysis_type(j) + ...
                ": " + num2str(data_new(data_nb).std_dev(j),4) + data_new(data_nb).unit)
            disp("%%%%%%%")
        end
    end  
end


function ret = compare2eps(a,b, eps)

    ret = logical(abs(a - b) < eps);

end
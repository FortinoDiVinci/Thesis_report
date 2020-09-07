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
file_name = 'data_eval_pert.mat';

load(file_name);
STATIC_EXP = cell(size(folder_names));
STATIC_EXP(:,:) = {0};
STATIC_EXP(4:end) = {1};

%%%%%%%%%%%%%%%%%%
%% MACROS & variables
%%%%%%%%%%%%%%%%%%
% display macros

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

stiff_all = [];
damp_all = [];
mass_all = [];
rho_all = [];

if ~exist("NO_TRQ_CMD_DIST", "var")
    NO_TRQ_CMD_DIST = cell(size(folder_names));
    NO_TRQ_CMD_DIST (:,:) = {1};
end

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
    fz{exp_nb} = filtfilt(b,a,fz_temp);
    
end
    
for exp_nb = 1:length(folder_names)
    clear idx_perts dist_timings
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
    else
        dist_duration = t_dist{exp_nb}(2) - t_dist{exp_nb}(1);
        dist_timings = t_dist{exp_nb}(1:2:end);
        for pert_idx = 1:length(dist_timings)
            idx_perts(pert_idx) = find(t{exp_nb} >= dist_timings(pert_idx), 1, 'first');
        end
    end
    %size(idx_perts)
    %size(dist_timings)
    % if the experiment was interrupted during the last perturbation
    if ~isempty(idx_perts)
        if ( idx_perts(end) + idx_window_virt_traj + idx_delay)  > length(t{exp_nb})
            idx_perts = idx_perts(1:end-1);
            dist_timings = dist_timings(1:end-1);
        end
    end
    %disp("pert size: " + string(size(idx_perts)))
    
%     if ~NO_TRQ_CMD_DIST{exp_nb}
%         delta_z{exp_nb} = DIFF_TRAJECT(idx_window_virt_traj, idx_window_virt_traj, z{exp_nb}, t{exp_nb}, idx_perts, idx_delay);
%         delta_fz{exp_nb} = DIFF_TRAJECT(idx_window_virt_traj, idx_window_virt_traj, -1.*fz{exp_nb}, t{exp_nb}, idx_perts, idx_delay);
%     
%         computeDiffTraject(delta_z{exp_nb}, 'VirtTrajMethod', 'static');
%         computeDiffTraject(delta_fz{exp_nb}, 'VirtTrajMethod', 'static');
%     else
%         delta_z{exp_nb} = DIFF_TRAJECT(idx_window_virt_traj, idx_window_virt_traj, z{exp_nb}, t{exp_nb}, idx_perts, idx_delay);
%         delta_fz{exp_nb} = DIFF_TRAJECT(idx_window_virt_traj, 100, -1.*fz{exp_nb}, t{exp_nb}, idx_perts, idx_delay);
%  
%         computeDiffTraject(delta_z{exp_nb}, 'VirtTrajMethod', 'static');
%         computeDiffTraject(delta_fz{exp_nb}, 'VirtTrajMethod', 'static');
%     end
    
    if STATIC_EXP{exp_nb}
        delta_z{exp_nb} = DIFF_TRAJECT(idx_window_virt_traj, idx_window_virt_traj, z{exp_nb}, t{exp_nb}, idx_perts, idx_delay);
        delta_fz{exp_nb} = DIFF_TRAJECT(idx_window_virt_traj, idx_window_virt_traj, -1.*fz{exp_nb}, t{exp_nb}, idx_perts, idx_delay);
     
        delta_z{exp_nb}.computeDiffTraject('VirtTrajMethod', 'static');
        delta_fz{exp_nb}.computeDiffTraject('VirtTrajMethod', 'static');
    else
        delta_z{exp_nb} = DIFF_TRAJECT(idx_window_virt_traj, idx_window_virt_traj, z{exp_nb}, t{exp_nb}, idx_perts, idx_delay);
        delta_fz{exp_nb} = DIFF_TRAJECT(idx_window_virt_traj, idx_window_virt_traj, -1.*fz{exp_nb}, t{exp_nb}, idx_perts, idx_delay);
        
        delta_z{exp_nb}.computeDiffTraject('VirtTrajMethod', 'spline');
        delta_fz{exp_nb}.computeDiffTraject('VirtTrajMethod', 'spline');
    end

    delta_z{exp_nb}.computeDerivatives();   
    
    impedance{exp_nb} = IMPEDANCE_DATA(nb_param, length(dist_timings), idx_window_imp_eval);
    impedance{exp_nb}.init_phi(delta_z{exp_nb}.diff_traject(1:idx_window_imp_eval,:), delta_z{exp_nb}.d_diff_traject(1:idx_window_imp_eval,:), delta_z{exp_nb}.dd_diff_traject(1:idx_window_imp_eval,:));
    impedance{exp_nb}.init_y(delta_fz{exp_nb}.diff_traject(1:idx_window_imp_eval,:));
    impedance{exp_nb}.lsq();
    
    if NO_MOCAP{exp_nb}
        continue
    end
    
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

stiff_all_cln = stiff_all( (stiff_all>0) & (damp_all>0));
damp_all_cln = damp_all( (stiff_all>0) & (damp_all>0));

stiff_all_cln = stiff_all( (stiff_all>0));
damp_all_cln = damp_all( (stiff_all>0));

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
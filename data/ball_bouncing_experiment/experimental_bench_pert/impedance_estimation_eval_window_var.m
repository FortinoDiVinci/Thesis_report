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
%close all

addpath('../../force_torque_sensor')
addpath('../../youBot_analysis/Utils')

%file_name = 'successful_exp_data.mat';
%file_name = 'data_impact2.mat';
%file_name = 'preliminary_data.mat';
%file_name = 'calibration_bench_data_26_08_20';
%file_name = 'data_eval_pert.mat';
file_name = 'preliminary_experimental_data/data_vfo_3_phases.mat';
%file_name = 'data_mso_3_phases.mat';

load(file_name);
STATIC_EXP = cell(size(folder_names));
STATIC_EXP(:,:) = {0};

%%%%%%%%%%%%%%%%%%
%% MACROS & variables
%%%%%%%%%%%%%%%%%%

DISP_STIFFNESS_DISTRIBUTION = 0;
SORT_PERT_BY_PHASE = 1; 
VIRTUAL_FILTERED = 0; % Virtual force = low pass filt. of original force
LOW_PASS_FREQ = 25; % Input signal are lp filtered before any computation
FILTER_ORDER = 2;

if ~exist('dt', 'var')
    dt = 1e-3;
end
% time evaluation variables
idx_window_virt_traj= ceil(0.065/dt); % 200ms
idx_window_imp_eval = ceil([0.065:0.005:0.30]./dt); % 200ms     
idx_wdw_virt_traj_pos= ceil(0.200/dt); % 200ms
idx_wdw_imp_eval_pos = ceil(0.300/dt); % 300ms, this is the max value
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

% Z force and position extraction and low pass filtering 
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
    [b,a] = butter(FILTER_ORDER,LOW_PASS_FREQ/(1/(2*dt)),'low'); 

    z{exp_nb} = filtfilt(b,a,z_temp);
    fz{exp_nb} = -1*filtfilt(b,a,fz_temp);
    
end

for wdw_idx = length(idx_window_imp_eval):-1:1
    % Virtual trajectories and impedance estimation
    for exp_nb = length(folder_names):-1:1

        %% Perturbation indexes extraction 

        clear idx_perts dist_timings
        if NO_DISTURBANCE{exp_nb}
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
        % if the experiment was interrupted during the last perturbation
        reduced_dist{exp_nb} = 0;
        wdw = max(idx_window_virt_traj, idx_window_imp_eval(wdw_idx));
        wdw = max(wdw, idx_wdw_imp_eval_pos);
        if ~isempty(idx_perts)
            if ( t{exp_nb}(idx_perts(end)) + (wdw + idx_delay)*dt )  >= t{exp_nb}(end)
                idx_perts = idx_perts(1:end-1);
                dist_val{exp_nb} = dist_val{exp_nb}(1:end-1);
                dist_timings = dist_timings(1:end-1);
                reduced_dist{exp_nb} = 1;
            end
        end
        %disp("pert size: " + string(size(idx_perts)))

        %% Trajectories extraction & estimation

        delta_z = DIFF_TRAJECT(idx_wdw_imp_eval_pos, idx_wdw_virt_traj_pos, ...
            z{exp_nb}, t{exp_nb}, idx_perts, dist_val{exp_nb}, idx_delay);
        delta_fz = DIFF_TRAJECT(idx_window_imp_eval(wdw_idx), idx_window_virt_traj, ...
            fz{exp_nb}, t{exp_nb}, idx_perts, dist_val{exp_nb}, idx_delay);

        delta_z.computeDiffTraject('VirtTrajMethod', 'spline'); % z0 - z
        % low pass filtered at 2Hz
        if VIRTUAL_FILTERED
            delta_fz.computeDiffTraject('VirtTrajMethod', ...
            'filter', 'DiffDirection', 'neg'); % fz - fz0
        else
            delta_fz.computeDiffTraject('VirtTrajMethod', 'spline', ...
            'DiffDirection', 'neg'); % fz - fz0
        end    

        delta_z.computeDerivatives();   

        %% Impedance estimation 

        impedance = IMPEDANCE_DATA(nb_param, length(dist_timings), idx_window_imp_eval(wdw_idx));
        impedance.init_phi(delta_z.diff_traject(1:idx_window_imp_eval(wdw_idx),:), ...
            delta_z.d_diff_traject(1:idx_window_imp_eval(wdw_idx),:), ... % speed
            delta_z.dd_diff_traject(1:idx_window_imp_eval(wdw_idx),:)); % acceleration
        impedance.init_y(delta_fz.diff_traject(1:idx_window_imp_eval(wdw_idx),:));
        impedance.lsq(); % least square optimization evaluation

        data(wdw_idx, exp_nb).impedance = impedance;
        data(wdw_idx, exp_nb).delta_z = delta_z;
        data(wdw_idx, exp_nb).delta_fz = delta_fz;

    end
end
R2 = [];  
K = [];
B = [];
M = [];
for ii = 1:length(idx_window_imp_eval)
    R2_tmp = [];
    K_tmp = [];
    B_tmp = [];
    M_tmp = [];
    for jj = 1:length(folder_names)
        R2_tmp = [R2_tmp, data(ii, jj).impedance.r_2'];
        K_tmp = [K_tmp, data(ii, jj).impedance.xi(1,:)];
        B_tmp = [B_tmp, data(ii, jj).impedance.xi(2,:)];
        M_tmp = [M_tmp, data(ii, jj).impedance.xi(3,:)];
    end
    if size(R2,2) == 0 || size(R2_tmp,2) == size(R2,2)
        R2 = [R2; R2_tmp];
        K = [K; K_tmp];
        B = [B; B_tmp];
        M = [M; M_tmp];
    elseif size(R2,2) - size(R2_tmp,2) > 0
        fill = NaN(1, size(R2,2) - size(R2_tmp,2));
        R2 = [R2; [R2_tmp, fill]];
        K = [K; K_tmp, fill];
        B = [B; B_tmp, fill];
        M = [M; M_tmp, fill];
    else
        warning('TODO: explain error')
    end        
end

% plot average R2, and impedance against the evaluation windows
figure
subplot(3,1,1)
plot(idx_window_imp_eval, nanmean(R2,2)')
hold on
plotStdSurface(nanmean(R2,2), nanstd(R2,0,2), idx_window_imp_eval, [0, 0.4470, 0.7410], 1)
xlim([min(idx_window_imp_eval), max(idx_window_imp_eval)])
xlabel('Time (ms)') 
title('mean R2 score against evaluation window')
subplot(3,1,2)
plot(idx_window_imp_eval, nanmean(K,2)')
hold on
plotStdSurface(nanmean(K,2), nanstd(K,0,2), idx_window_imp_eval, [0, 0.4470, 0.7410], 1)
xlim([min(idx_window_imp_eval), max(idx_window_imp_eval)])
xlabel('Time (ms)') 
ylabel('Stiffness (N/m)') 
title('mean Stiffness against evaluation window')
subplot(3,1,3)
plot(idx_window_imp_eval, nanmean(B,2)')
hold on
plotStdSurface(nanmean(B,2), nanstd(B,0,2), idx_window_imp_eval, [0, 0.4470, 0.7410], 1)
xlim([min(idx_window_imp_eval), max(idx_window_imp_eval)])
xlabel('Time (ms)') 
ylabel('Damping (N.s/m)') 
yyaxis right
plot(idx_window_imp_eval, nanmean(M,2)')
plotStdSurface(nanmean(M,2), nanstd(M,0,2), idx_window_imp_eval, [0.8500, 0.3250, 0.0980], 1)
ylabel('Mass (kg)') 
title('mean Damping & Mass against evaluation window')

% plot an example of trajectory
figure('DefaultAxesFontSize',14)
subplot(2,1,1)
hold on
plot(data(end, 7).delta_fz.time, data(end, 7).delta_fz.complete_traject)
plot(data(end, 7).delta_fz.t_traject(:,9), data(end, 7).delta_fz.virt_traject(:,9))
plot(data(end, 7).delta_fz.t_traject(3:end-2,9), data(end, 7).impedance.rec_y(:,9) +...
    data(end, 7).delta_fz.virt_traject(3:end-2,9))
plot(data(12, 7).delta_fz.t_traject(3:end-2,9), data(12, 7).impedance.rec_y(:,9) +...
    data(12, 7).delta_fz.virt_traject(3:end-2,9))
xlim([52.9, 55.4])
xlabel('Time (s)')
ylabel('Force (N)')
legend('Meas.', 'Virtual', 'Reconstr. 1', 'Reconstr. 2')
title({["Reconstr. 1) R2: " + num2str(data(end, 7).impedance.r_2(9)) + ", K: " + ...
    num2str(data(end, 7).impedance.xi(1,9)) + ", B: " + ...
    num2str(data(end, 7).impedance.xi(2,9)) + ", M: " + ...
    num2str(data(end, 7).impedance.xi(3,9))], ...
    ["Reconstr. 2) R2: " + num2str(data(12, 7).impedance.r_2(9)) + ", K: " + ...
    num2str(data(12, 7).impedance.xi(1,9)) + ", B: " + ...
    num2str(data(12, 7).impedance.xi(2,9)) + ", M: " + ...
    num2str(data(12, 7).impedance.xi(3,9))]})
subplot(2,1,2)
hold on
plot(data(end, 7).delta_z.time, data(end, 7).delta_z.complete_traject.*1e2)
plot(data(end, 7).delta_z.t_traject(:,9), data(end, 7).delta_z.virt_traject(:,9).*1e2)
xlim([52.9, 55.4])
xlabel('Time (s)')
ylabel('Position (cm)')
legend('Meas.', 'Virtual')

function ret = compare2eps(a,b, eps)

    ret = logical(abs(a - b) < eps);

end
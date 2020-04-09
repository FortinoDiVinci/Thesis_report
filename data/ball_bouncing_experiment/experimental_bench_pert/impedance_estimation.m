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

file_name = 'successful_exp_data.mat';
load(file_name);

%%%%%%%%%%%%%%%%%%
%% MACROS & variables
%%%%%%%%%%%%%%%%%%

DISP_NORMALIZED_TRAJECTORIES = 1


% time evaluation variables
idx_traj_fit        = ceil(0.100/dt); % 100ms before and after the window
idx_window          = ceil(0.200/dt); % 200ms
idx_delay           = ceil(0.015/dt); % 015ms


exp_nb = 2; % select the experience
% timing of the perturbations introduced
t_on_dist{exp_nb} = t_dist{exp_nb}(find((dist{exp_nb}(1:end-2) ~= 0))');
t = t{exp_nb};

%%%%%%%%%%%%%%%%%%
%% Signal Pre-Processing
%%%%%%%%%%%%%%%%%%

% data filtering
z{exp_nb} = mocap_marker_robot_base{exp_nb}(:,3); % position of the motion capture
% the low pass filtering is not done is the following function
[f{exp_nb},~,~]= forces_filtering(forces_unf{exp_nb}', torques_unf{exp_nb}', ...
    thetas{exp_nb}', t);
fz{exp_nb} = f{exp_nb}(3,:);
fc = 25; % cut off frequency
[b,a] = butter(2,fc/(1/(2*dt)),'low'); 

z{exp_nb} = filtfilt(b,a,z{exp_nb});
fz{exp_nb} = filtfilt(b,a,fz{exp_nb});

% centered derivates (W. Khalil and E. Dombre, 2002) eq. [12.16]
dz{exp_nb} = Iu_diffcent(z{exp_nb},t);
ddz{exp_nb} = Iu_diffcent(dz{exp_nb},t);

% peak detection
idx_peaks = crossing(dz{exp_nb});
idx_peaks = idx_peaks(diff(idx_peaks)>100);

t_1st_peak = 73.5; % approx time for selection of relevant data
t_last_peak = 115;
idx_peaks = idx_peaks(t(idx_peaks)>t_1st_peak & t(idx_peaks)<t_last_peak);
t_on_dist{exp_nb} = t_on_dist{exp_nb}(t_on_dist{exp_nb} > t_1st_peak & t_on_dist{exp_nb} < t_last_peak);

t_on_dist{exp_nb} = t_on_dist{exp_nb}(t_on_dist{exp_nb}>t_1st_peak & t_on_dist{exp_nb}<t_last_peak);
idx_on_dist = zeros(size(t_on_dist{exp_nb}));
% perturbation indexes
for pert_idx = 1:length(t_on_dist{exp_nb})
    idx_on_dist(pert_idx) = find(t >= t_on_dist{exp_nb}(pert_idx), 1, 'first');
end

% % Check peak detection
% figure()
% plot(t{exp_nb}, z{exp_nb},t(idx_peaks), z{exp_nb}(idx_peaks),'k*')

% % plot hand position and interaction forces with the perturbations timing
% figure()
% hold on, grid on
% plot(t, z{exp_nb}, t(idx_on_dist), z{exp_nb}(idx_on_dist), '^')
% yyaxis right
% plot(t, -fz{exp_nb}, t(idx_on_dist), -fz{exp_nb}(idx_on_dist), 'r^')

%%%%%%%%%%%%%%%%%%
%% Signal Processing
%%%%%%%%%%%%%%%%%%

%% Normalization

% half cycles
t_cycle = cell(1, length(idx_peaks)-1);
z_cycle = cell(size(t_cycle));
fz_cycle = cell(size(t_cycle));
% normalized cycles
zn_cycle = cell(size(t_cycle));
fzn_cycle = cell(size(t_cycle));

% half cycle selection and normalization 
for pk_idx = 1:length(idx_peaks)-1
    idx_cycle = idx_peaks(pk_idx):idx_peaks(pk_idx+1);
    t_cycle{pk_idx} = t(idx_cycle);
    z_cycle{pk_idx} = z{exp_nb}(idx_cycle);
    fz_cycle{pk_idx} = fz{exp_nb}(idx_cycle);
    %normalization: signal/amp(signal) - min(signal/amp(signal))
    zn_cycle{pk_idx} = z_cycle{pk_idx}/(max(z_cycle{pk_idx}) - min(z_cycle{pk_idx}));
    zn_cycle{pk_idx} = zn_cycle{pk_idx} - min(zn_cycle{pk_idx});
    fzn_cycle{pk_idx} = fz_cycle{pk_idx}/(max(fz_cycle{pk_idx}) - min(fz_cycle{pk_idx}));
    fzn_cycle{pk_idx} = fzn_cycle{pk_idx} - min(fzn_cycle{pk_idx});
end


cyc_np_idx = 1;
z_cycle_type = zeros(size(t_cycle));
idx_pert_cycle = zeros(size(t_cycle));

% sort normalized cycles according to the presence of perturbation or not 
% and if they are rising or decreasing (for position only)
for cyc_idx = 1:length(zn_cycle)
    if ~any( (min(t_cycle{cyc_idx}) <= t_on_dist{exp_nb}) & ...
            (max(t_cycle{cyc_idx}) >= t_on_dist{exp_nb}) )%% no perturbations
        zn_cycle_np{cyc_np_idx} = zn_cycle{cyc_idx};
        fzn_cycle_np{cyc_np_idx} = fzn_cycle{cyc_idx};
        t_cycle_np{cyc_np_idx} = t_cycle{cyc_idx};
        
        cyc_np_idx = cyc_np_idx + 1;
    else
        idx_pert_cycle(cyc_idx) = 1;
    end
    
    % rising or decreasing phase 
    if zn_cycle{cyc_idx}(1) < zn_cycle{cyc_idx}(end)
        z_cycle_type(cyc_idx) = 1; % rising
    else
        z_cycle_type(cyc_idx) = 0; % decreasing
    end
    
end

z_cycle_type_np = z_cycle_type(logical(~idx_pert_cycle)); % for non perturbed
% trajectories only


% time normalization
cycle_sizes = cellfun('size',zn_cycle,1);
t_norm = ((1:min(cycle_sizes))./min(cycle_sizes))';

zn2_cycle = zeros(length(t_norm), length(zn_cycle)); % magnitude and time normalization
dzn2_cycle = zeros(size(zn2_cycle)); % ""
fzn2_cycle = zeros(size(zn2_cycle)); % ""
% non perturbed trajectory only 
zn2_cycle_np = zeros(length(t_norm), length(zn_cycle_np));
dzn2_cycle_np = zeros(size(zn2_cycle_np));
fzn2_cycle_np = zeros(size(zn2_cycle_np));

cyc_np_idx = 1;
for cyc_idx = 1:length(cycle_sizes)
    t_norm_tmp = (1:cycle_sizes(cyc_idx))./cycle_sizes(cyc_idx);
    if (idx_pert_cycle(cyc_idx) == 0) % data without perturbation
        zn2_cycle_np(:, cyc_np_idx) = interp1(t_norm_tmp, zn_cycle{cyc_idx}, t_norm);
        dzn2_cycle_np(:, cyc_np_idx) = Iu_diffcent(zn2_cycle(:, cyc_idx), t_norm);
        fzn2_cycle_np(:, cyc_np_idx) = interp1(t_norm_tmp, fzn_cycle{cyc_idx}, t_norm);
        cyc_np_idx = cyc_np_idx + 1;
    end
    % all data
    zn2_cycle(:, cyc_idx) = interp1(t_norm_tmp, zn_cycle{cyc_idx}, t_norm);
    dzn2_cycle(:, cyc_idx) = Iu_diffcent(zn2_cycle(:, cyc_idx), t_norm);
    fzn2_cycle(:, cyc_idx) = interp1(t_norm_tmp, fzn_cycle{cyc_idx}, t_norm);
end

% statistics are made with cycle without perturbation only and sorting
% rising and decreasing phase of the cycle
zn2_up_idx_std = std(zn2_cycle_np(:, logical(z_cycle_type_np)), [], 2);
dzn2_up_idx_std = std(dzn2_cycle_np(:, logical(z_cycle_type_np)), [], 2);
fzn2_up_idx_std = std(fzn2_cycle_np(:, logical(z_cycle_type_np)), [], 2);

zn2_dw_idx_std = std(zn2_cycle_np(:, logical(~z_cycle_type_np)), [], 2);
dzn2_dw_idx_std = std(dzn2_cycle_np(:, logical(~z_cycle_type_np)), [], 2);
fzn2_dw_idx_std = std(fzn2_cycle_np(:, logical(~z_cycle_type_np)), [], 2);

zn2_up_idx_avg = mean(zn2_cycle_np(:, logical(z_cycle_type_np)), 2);
dzn2_up_idx_avg = mean(dzn2_cycle_np(:, logical(z_cycle_type_np)), 2);
fzn2_up_idx_avg = mean(fzn2_cycle_np(:, logical(z_cycle_type_np)), 2);

zn2_dw_idx_avg = mean(zn2_cycle_np(:, logical(~z_cycle_type_np)), 2);
dzn2_dw_idx_avg = mean(dzn2_cycle_np(:, logical(~z_cycle_type_np)), 2);
fzn2_dw_idx_avg = mean(fzn2_cycle_np(:, logical(~z_cycle_type_np)), 2);

curve_up_1 = zn2_up_idx_avg + zn2_up_idx_std;
curve_up_2 = zn2_up_idx_avg - zn2_up_idx_std;
curve_dw_1 = zn2_dw_idx_avg + zn2_dw_idx_std;
curve_dw_2 = zn2_dw_idx_avg - zn2_dw_idx_std;
% 3 std
curve_up_1b = zn2_up_idx_avg + 3*zn2_up_idx_std;
curve_up_2b = zn2_up_idx_avg - 3*zn2_up_idx_std;
curve_dw_1b = zn2_dw_idx_avg + 3*zn2_dw_idx_std;
curve_dw_2b = zn2_dw_idx_avg - 3*zn2_dw_idx_std;

if DISP_NORMALIZED_TRAJECTORIES
% plot normalized trajectories, perturbed trajectories are in red
    figure()
    % 3*std and std
    subplot(1,2,1)
    hold on, grid on
    fill([t_norm' fliplr(t_norm')],[curve_up_1b' fliplr(curve_up_2b')], ...
        [0.5 0.5 0.5], 'FaceAlpha', 0.25,'linestyle','none')
    fill([t_norm' fliplr(t_norm')],[curve_up_1' fliplr(curve_up_2')], ...
        [0.5 0.5 0.5], 'FaceAlpha', 0.75,'linestyle','none')  
    subplot(1,2,2)
    hold on, grid on
    fill([t_norm' fliplr(t_norm')],[curve_dw_1b' fliplr(curve_dw_2b')], ...
        [0.5 0.5 0.5], 'FaceAlpha', 0.25,'linestyle','none')
    fill([t_norm' fliplr(t_norm')],[curve_dw_1' fliplr(curve_dw_2')], ...
        [0.5 0.5 0.5], 'FaceAlpha', 0.75,'linestyle','none')
    % curves
    subplot(1,2,1)
    plot(t_norm, zn2_cycle(:,logical(~idx_pert_cycle)&logical(z_cycle_type)), ':k')
    plot(t_norm, zn2_cycle(:,logical(idx_pert_cycle)&logical(z_cycle_type)), ':r')
    subplot(1,2,2)
    plot(t_norm, zn2_cycle(:,logical(~idx_pert_cycle)&logical(~z_cycle_type)), ':k')
    plot(t_norm, zn2_cycle(:,logical(idx_pert_cycle)&logical(~z_cycle_type)), ':r')
    % average
    subplot(1,2,1)
    plot(t_norm, zn2_up_idx_avg, 'k', 'linewidth', 2)
    subplot(1,2,2)
    plot(t_norm, zn2_dw_idx_avg, 'k', 'linewidth', 2)
end

%% perturbed/unpertubed behaviour compared

idx_perts = zeros(size(t_on_dist{exp_nb})); % global indexes of all perturbations
for pert_idx = 1:length(t_on_dist{exp_nb})  
    idx_perts(pert_idx) = find(t >= t_on_dist{exp_nb}(pert_idx), 1, 'first');
end

for pert_idx = 1:length(idx_perts) % indexes whithin the perturbations
    
    idx_pert = idx_perts(pert_idx); % global index of the current pert
    % find closer peak
    next_peak_idx = find(t(idx_peaks) >= t(idx_pert), 1, 'first');
    previous_peak_idx = find(t(idx_peaks) <= t(idx_pert), 1, 'last');
    
    if abs(next_peak_idx - idx_pert) >= abs(idx_pert - previous_peak_idx)
        closest_peak_idx = previous_peak_idx;
    else
        closest_peak_idx = next_peak_idx;
    end
    
    % find its relative position on the 1/2 cycle    
    relative_pos = ( idx_pert - idx_peaks(previous_peak_idx) ) / ...
       (idx_peaks(next_peak_idx) - idx_peaks(previous_peak_idx) );
    
    % find the equivalent unperturbed instant on the next complete cycle
    idx_unpert_next_cycle = idx_peaks(previous_peak_idx+2) + ...
        round(relative_pos*( idx_peaks(previous_peak_idx+3) - idx_peaks(previous_peak_idx+2) ));
    
    % check if a perturbation occured in the 1/2 cycle or less than 150ms
    % before or after, if so pass on to next perturbation
    if any(abs(idx_perts-idx_unpert_next_cycle)*dt <= 0.150)
        idx_unpert_next_cycle = NaN;
        % continue;
    elseif idx_pert_cycle(previous_peak_idx+3) %there is a perturbation 
        % whithin the half cycle
        idx_unpert_next_cycle = NaN;
        % continue;
    end
   
    % perturbed cases for the next cycle have been discarded 
    % definition of the time windows for the virtual trajectories
    
    idx_fit = [ (idx_pert - idx_traj_fit+1) : (idx_pert) ,...
        (idx_pert + idx_window) : (idx_pert + idx_window + idx_traj_fit) ]...
        + idx_delay;  % index outside framing the window of interest
    % starting at the time of perturbation + delay
    idx_tot = [ (idx_pert - idx_traj_fit+1) : (idx_pert + idx_window + idx_traj_fit)]...
        + idx_delay; % index of fitting time + window of interst 
    
    if (~isnan(idx_unpert_next_cycle))
        % same indexes but the the cycle just after (to validate methodology)
        idx_fit_val = [ (idx_unpert_next_cycle - idx_traj_fit+1) : (idx_unpert_next_cycle) ,...
            (idx_unpert_next_cycle + idx_window) :  ...
            (idx_unpert_next_cycle + idx_window + idx_traj_fit) ]+ idx_delay;  
        idx_tot_val = [ (idx_unpert_next_cycle - idx_traj_fit+1) : ...
            (idx_unpert_next_cycle + idx_window + idx_traj_fit)]...
            + idx_delay;   
    end
    
    
    
%     if z_cycle_type(previous_peak_idx) % rising part of the cycle
%         
%     else % decreasing part of the cycle
%         
%     end
    
end

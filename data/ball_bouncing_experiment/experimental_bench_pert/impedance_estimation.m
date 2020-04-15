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
% display macros
DISP_NORMALIZED_TRAJECTORIES = 0
DISP_TIME_FRAMES_OF_INTEREST = 0
DISP_RECONSTRUCTED_FORCES    = 1
DISP_NORMAL_ERRORS_HISTOGRAM = 1

ONLY_DECREASING_CYCLES_PERT  = 1

min_time_distance_to_peak = 0.100;    % 050ms 
% time evaluation variables
idx_traj_fit        = ceil(0.100/dt); % 100ms before and after the window
idx_window          = ceil(0.200/dt); % 200ms
idx_delay           = ceil(0.015/dt); % 015ms

nb_param            = 4; % for the impedance model, should be between 2 & 4
                         % - 2: F = Kx + e
                         % - 3: F = Kx + Bdx + e
                         % - 4: F = Kx + Bdx + Iddx + e

exp_nb = 2; % select the experience
% timing of the perturbations introduced
t_on_dist = t_dist{exp_nb}(find((dist{exp_nb}(1:end-2) ~= 0))');
t = t{exp_nb};

%%%%%%%%%%%%%%%%%%
%% Signal Pre-Processing
%%%%%%%%%%%%%%%%%%

% data filtering
z = mocap_marker_robot_base{exp_nb}(:,3); % position of the motion capture
% the low pass filtering is not done is the following function
[f,~,~]= forces_filtering(forces_unf{exp_nb}', torques_unf{exp_nb}', ...
    thetas{exp_nb}', t);
fz = f(3,:);
fc = 25; % cut off frequency
[b,a] = butter(2,fc/(1/(2*dt)),'low'); 

z = filtfilt(b,a,z);
fz = filtfilt(b,a,fz);

% centered derivates (W. Khalil and E. Dombre, 2002) eq. [12.16]
dz = Iu_diffcent(z,t);
ddz = Iu_diffcent(dz,t);

% peak detection
idx_peaks = crossing(dz);
idx_peaks = idx_peaks(diff(idx_peaks)>100);

t_1st_peak = 73.5; % approx time for selection of relevant data
t_last_peak = 115;
idx_peaks = idx_peaks(t(idx_peaks)>t_1st_peak & t(idx_peaks)<t_last_peak);
t_on_dist = t_on_dist(t_on_dist > t_1st_peak & t_on_dist < t_last_peak);

t_on_dist = t_on_dist(t_on_dist>t_1st_peak & t_on_dist<t_last_peak);
idx_perts = zeros(size(t_on_dist)); % global indexes of all perturbations
% perturbation indexes
for pert_idx = 1:length(t_on_dist)
    idx_perts(pert_idx) = find(t >= t_on_dist(pert_idx), 1, 'first');
end

if DISP_TIME_FRAMES_OF_INTEREST
    figure(1)
    hold on, grid on
    title('Trajectories: Red = perturbed, Green = validation')
    % peak detection
    plot(t, z, t(idx_peaks), z(idx_peaks),'k*', 'MarkerSize', 10);
    % yyaxis right
    % plot(t, -fz{exp_nb}, t(idx_on_dist), -fz{exp_nb}(idx_on_dist), 'r^')
end

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
    z_cycle{pk_idx} = z(idx_cycle);
    fz_cycle{pk_idx} = fz(idx_cycle);
    %normalization: signal/amp(signal) - min(signal/amp(signal))
    zn_cycle{pk_idx} = z_cycle{pk_idx}/(max(z_cycle{pk_idx}) - min(z_cycle{pk_idx}));
    zn_cycle{pk_idx} = zn_cycle{pk_idx} - min(zn_cycle{pk_idx});
    fzn_cycle{pk_idx} = fz_cycle{pk_idx}/(max(fz_cycle{pk_idx}) - min(fz_cycle{pk_idx}));
    fzn_cycle{pk_idx} = fzn_cycle{pk_idx} - min(fzn_cycle{pk_idx});
end


cyc_np_idx = 1;
z_cycle_type = zeros(size(t_cycle)); % classify 1/2 by type (/ or \)
idx_pert_cycle = zeros(size(t_cycle)); % classify 1/2 cycles by perturbation
pert_cycle_type = zeros(length(t_on_dist), 1); % classify perturbation (/ or \)
pert_idx = 0; % start at zero because of the position of the increment 

% sort normalized cycles according to the presence of perturbation or not 
% and if they are rising or decreasing (for position only)
for cyc_idx = 1:length(zn_cycle)
    if ~any( (min(t_cycle{cyc_idx}) <= t_on_dist) & ...
            (max(t_cycle{cyc_idx}) >= t_on_dist) )%% no perturbations
        zn_cycle_np{cyc_np_idx} = zn_cycle{cyc_idx};
        fzn_cycle_np{cyc_np_idx} = fzn_cycle{cyc_idx};
        t_cycle_np{cyc_np_idx} = t_cycle{cyc_idx};
        
        cyc_np_idx = cyc_np_idx + 1;
    else
        idx_pert_cycle(cyc_idx) = 1;
        pert_idx = pert_idx + 1;
    end
    
    % rising or decreasing phase 
    if zn_cycle{cyc_idx}(1) < zn_cycle{cyc_idx}(end)
        z_cycle_type(cyc_idx) = 1; % rising
        if idx_pert_cycle(cyc_idx)
            pert_cycle_type(pert_idx) = 1;
        end           
    else
        z_cycle_type(cyc_idx) = 0; % decreasing
        if idx_pert_cycle(cyc_idx)
            pert_cycle_type(pert_idx) = 0;
        end
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
    figure(2)
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

tot_size = 2*idx_traj_fit + idx_window;
% real and virtual trajectories and forces for the estimation
z_fit = NaN(tot_size, length(idx_perts));
dz_fit = NaN(size(z_fit));
ddz_fit = NaN(size(z_fit));
fz_fit = NaN(size(z_fit));
t_fit = NaN(size(z_fit));
z_virt = NaN(size(z_fit));
dz_virt = NaN(size(z_fit));
ddz_virt = NaN(size(z_fit));
fz_virt = NaN(size(z_fit));
% real and virtual trajectories and forces for the validation
z_fit_val = NaN(size(z_fit));
dz_fit_val = NaN(size(z_fit));
ddz_fit_val = NaN(size(z_fit));
fz_fit_val = NaN(size(z_fit));
t_fit_val = NaN(size(z_fit));
z_virt_val = NaN(size(z_fit));
dz_virt_val = NaN(size(z_fit));
ddz_virt_val = NaN(size(z_fit));
fz_virt_val = NaN(size(z_fit));

for pert_idx = 1:length(idx_perts) % indexes whithin the perturbations
    
    idx_pert = idx_perts(pert_idx); % global index of the current pert
    % find closer peak
    next_peak_idx = find(t(idx_peaks) >= t(idx_pert), 1, 'first');
    previous_peak_idx = find(t(idx_peaks) <= t(idx_pert), 1, 'last');
    
    if abs(idx_peaks(next_peak_idx) - idx_pert) >= abs(idx_pert - idx_peaks(previous_peak_idx))
        closest_peak_idx = previous_peak_idx;
    else
        closest_peak_idx = next_peak_idx;
    end
    
    % time interval to closest peak, if a perturbation is too close, for
    % now it is discarded
    time_distance_to_peak = abs(t(idx_peaks(closest_peak_idx)) - t(idx_pert));
    if time_distance_to_peak < min_time_distance_to_peak        
        if DISP_TIME_FRAMES_OF_INTEREST
            figure(1)      
            % perturbation not considered
            plot(t(idx_pert), z(idx_pert), 'kv', 'MarkerSize', 10); 
        end      
        continue
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
        
        if DISP_TIME_FRAMES_OF_INTEREST
            figure(1)      
            % perturbation not considered
            plot(t(idx_unpert_next_cycle), z(idx_unpert_next_cycle), 'gx', 'MarkerSize', 10); 
        end
        idx_unpert_next_cycle = NaN;
        % continue;
    elseif logical(idx_pert_cycle(previous_peak_idx+3)) %there is a perturbation 
        % whithin the half cycle
        
        if DISP_TIME_FRAMES_OF_INTEREST
            figure(1)      
            % perturbation not considered
            plot(t(idx_unpert_next_cycle), z(idx_unpert_next_cycle), 'gx', 'MarkerSize', 10); 
            idx_unpert_next_cycle = NaN;
        end
        % continue;
    end
   
    % perturbed cases for the next cycle have been discarded 
    % definition of the time windows for the virtual trajectories
    
    idx_fit = [ (idx_pert - idx_traj_fit+1) : (idx_pert) ,...
        (idx_pert + idx_window) : (idx_pert + idx_window + idx_traj_fit) ]...
        + idx_delay;  % index outside framing the window of interest
    % starting at the time of perturbation + delay
    idx_tot = [ (idx_pert - idx_traj_fit+1) : (idx_pert + idx_window + idx_traj_fit) ]...
        + idx_delay; % index of fitting time + window of interst 
    
    if (~isnan(idx_unpert_next_cycle))
        % same indexes but the the cycle just after (to validate methodology)
        idx_fit_val = [ (idx_unpert_next_cycle - idx_traj_fit+1) : (idx_unpert_next_cycle) ,...
            (idx_unpert_next_cycle + idx_window) :  ...
            (idx_unpert_next_cycle + idx_window + idx_traj_fit) ]+ idx_delay;  
        idx_tot_val = [ (idx_unpert_next_cycle - idx_traj_fit+1) : ...
            (idx_unpert_next_cycle + idx_window + idx_traj_fit) ]...
            + idx_delay;   
    end
    
    %%%%%%%%%%%
    % estimation set
    t_interp = t(idx_fit);
    z_interp = z(idx_fit);
    fz_interp = fz(idx_fit);

    % perturbed data
    z_fit(:, pert_idx) = z(idx_tot);
    dz_fit(:, pert_idx) = dz(idx_tot);
    ddz_fit(:, pert_idx) = ddz(idx_tot);
    fz_fit(:, pert_idx) = fz(idx_tot);
    t_fit(:, pert_idx) = t(idx_tot);
    
    % virtual estimation (using splines)
    z_virt(:, pert_idx) = interp1(t_interp, z_interp, t_fit(:, pert_idx), 'spline');
    dz_virt(:, pert_idx) = Iu_diffcent(z_virt(:, pert_idx), t_fit(:, pert_idx));
    ddz_virt(:, pert_idx) = Iu_diffcent(dz_virt(:, pert_idx), t_fit(:, pert_idx));
    fz_virt(:, pert_idx) = interp1(t_interp, fz_interp, t_fit(:, pert_idx), 'spline');
    
    %%%%%%%%%%%
    % validation set at next cycle
    if (~isnan(idx_unpert_next_cycle))
    
        t_interp_val = t(idx_fit_val);
        z_interp_val = z(idx_fit_val);
        fz_interp_val = fz(idx_fit_val);

        % perturbed data
        z_fit_val(:, pert_idx) = z(idx_tot_val);
        dz_fit_val(:, pert_idx) = dz(idx_tot_val);
        ddz_fit_val(:, pert_idx) = ddz(idx_tot_val);
        fz_fit_val(:, pert_idx) = fz(idx_tot_val);
        t_fit_val(:, pert_idx) = t(idx_tot_val);

        % virtual estimation (using splines)
        z_virt_val(:, pert_idx) = interp1(t_interp_val, z_interp_val, t_fit_val(:, pert_idx), 'spline');
        dz_virt_val(:, pert_idx) = Iu_diffcent(z_virt_val(:, pert_idx), t_fit_val(:, pert_idx));
        ddz_virt_val(:, pert_idx) = Iu_diffcent(dz_virt_val(:, pert_idx), t_fit_val(:, pert_idx));
        fz_virt_val(:, pert_idx) = interp1(t_interp_val, fz_interp_val, t_fit_val(:, pert_idx), 'spline');
        
    end
    
    if DISP_TIME_FRAMES_OF_INTEREST
        figure(1)
        plot(t(idx_tot), z(idx_tot), 'r:','linewidth',2)
        plot(t(idx_fit), z(idx_fit), 'ro')
        
        plot(t(idx_tot_val), z(idx_tot_val), 'g:','linewidth',2)
        plot(t(idx_fit_val), z(idx_fit_val), 'go')       
        % perturbation 
        plot(t(idx_pert), z(idx_pert), 'k^', 'MarkerSize', 10); 
    end
    
%     if z_cycle_type(previous_peak_idx) % rising part of the cycle
%         
%     else % decreasing part of the cycle
%         
%     end
    
end

%%%%%%%%%%%%%%%%%%
%% Impedance estimation
%%%%%%%%%%%%%%%%%%

nb = length(idx_perts);

impedance = NaN(nb_param, nb);
impedance_val = NaN(nb_param, nb);

% input vector for least square identification
phi = zeros(idx_window, nb_param);
phi_val = zeros(idx_window, nb_param);

% differences between real and virtual trajectories
delta_z = zeros(idx_window, 1);
delta_z_val = zeros(idx_window, 1);
delta_dz = zeros(idx_window, 1);
delta_dz_val = zeros(idx_window, 1);
delta_ddz = zeros(idx_window, 1);
delta_ddz_val = zeros(idx_window, 1);
delta_fz = zeros(idx_window, nb);
delta_fz_val = zeros(idx_window, nb);
% r squares
r2 = NaN(nb,1);
r2_val = NaN(nb,1);
% reconstruction forces
delta_fz_rec = NaN(idx_window, nb);
delta_fz_rec_val = NaN(idx_window, nb);
% reconstruction errors
err_rec_n = NaN(idx_window, nb);
err_rec_n_val = NaN(idx_window, nb);

% indexes to select only the intersection between idx_tot and idx_fit, 
% that is the window of interest
idx1 = idx_traj_fit + 1;
idx2 = idx1 + idx_window - 1;

for pert_idx = 1:length(idx_perts)
    
    if pert_cycle_type(pert_idx) && ONLY_DECREASING_CYCLES_PERT
        disp("Pert N°"+pert_idx+" skipped")
        continue
    end
    
    delta_z = z_virt(idx1:idx2, pert_idx) - z_fit(idx1:idx2, pert_idx);
    delta_z_val = z_virt_val(idx1:idx2, pert_idx) - z_fit_val(idx1:idx2, pert_idx);
    delta_dz = dz_virt(idx1:idx2, pert_idx) - dz_fit(idx1:idx2, pert_idx);
    delta_dz_val = dz_virt_val(idx1:idx2, pert_idx) - dz_fit_val(idx1:idx2, pert_idx);
    delta_ddz = ddz_virt(idx1:idx2, pert_idx) - ddz_fit(idx1:idx2, pert_idx);
    delta_ddz_val = ddz_virt_val(idx1:idx2, pert_idx) - ddz_fit_val(idx1:idx2, pert_idx);
    
    delta_fz(:, pert_idx) = -(fz_fit(idx1:idx2, pert_idx) - fz_virt(idx1:idx2, pert_idx));
    delta_fz_val(:, pert_idx) = -(fz_fit_val(idx1:idx2, pert_idx) - fz_virt_val(idx1:idx2, pert_idx));
    
    switch nb_param
        case 2 % stiffness and artifacts
            phi = [delta_z, ones(size(delta_z))];
            phi_val = [delta_z_val, ones(size(delta_z_val))];
        case 3 % stiffness, damping and artifacts
            phi = [delta_z, delta_dz, ones(size(delta_z))];
            phi_val = [delta_z_val, delta_dz_val, ones(size(delta_z_val))];
        case 4 % stiffness, damping, inertia and artifacts
            phi = [delta_z, delta_dz, delta_ddz, ones(size(delta_z))];
            phi_val = [delta_z_val, delta_dz_val, delta_ddz_val, ones(size(delta_z_val))];
        otherwise
            warning('Number of parameters not implemented')
    end
    
    impedance(:, pert_idx) = phi\delta_fz(:, pert_idx);
    impedance_val(:, pert_idx) = phi_val\delta_fz_val(:, pert_idx);

    mdl = fitlm(phi,delta_fz(:, pert_idx));
    mdl_val = fitlm(phi_val,delta_fz_val(:, pert_idx));
    
    r2(pert_idx) = mdl.Rsquared.Adjusted;
    r2_val(pert_idx) = mdl_val.Rsquared.Adjusted;
    
    % reconstruction for quality evaluation    
    delta_fz_rec(:, pert_idx) = phi*impedance(:, pert_idx);
    delta_fz_rec_val(:, pert_idx) = phi_val*impedance_val(:, pert_idx);
    
    % reconstruction error
    mag_fz = abs(max(delta_fz(:, pert_idx)) - min(delta_fz(:, pert_idx))); % force magnitude
    mag_fz_val = abs(max(delta_fz_val(:, pert_idx)) - min(delta_fz_val(:, pert_idx)));
    err_rec_n(:, pert_idx) = (delta_fz_rec(:, pert_idx) - delta_fz(:, pert_idx)) / mag_fz;   
    err_rec_n_val(:, pert_idx) = (delta_fz_rec_val(:, pert_idx) - delta_fz_val(:, pert_idx)) / mag_fz_val;
end


% Display results
if DISP_RECONSTRUCTED_FORCES
    figure(3)
    for pert_idx = 1:nb
        subplot(4, ceil(nb/4), pert_idx)
        hold on, grid on
        plot(delta_fz(:, pert_idx), 'Color', [0    0.4470    0.7410], 'linewidth', 2)
        plot(delta_fz_rec(:, pert_idx), 'Color', [0.8500    0.3250    0.0980], 'linewidth', 2)        
        plot(delta_fz_val(:, pert_idx), 'Color', [0    0.4470    0.7410])
        plot(delta_fz_rec_val(:, pert_idx), 'Color', [0.8500    0.3250    0.0980])
        if isnan(r2(pert_idx))
            str_r2 = "NaN";
        else
            str_r2 = num2str(r2(pert_idx), '%1.3f');
        end
        title("Pert n°" + pert_idx + ", R^2=" + str_r2)
    end
    
end

if DISP_NORMAL_ERRORS_HISTOGRAM
    figure(4)
    histHandle = histogram(err_rec_n(:),50);
    hold on, grid on
    avg_tot = nanmean(err_rec_n(:));
    std_tot = nanstd(err_rec_n(:));
    line([avg_tot+std_tot, avg_tot+std_tot], [0, max(histHandle.Values)], 'Color','black','LineStyle','--','linewidth',2);
    line([avg_tot-std_tot, avg_tot-std_tot], [0, max(histHandle.Values)], 'Color','black','LineStyle','--','linewidth',2);
    line([avg_tot, avg_tot], [0, max(histHandle.Values)], 'Color','red','LineStyle','--','linewidth',2);
end

if nb_param > 1
    K_max = max(impedance(1,:));
    K_min = min(impedance(1,:));
    K_mean = nanmean(impedance(1,:));
    K_std = nanstd(impedance(1,:));
    disp('------- K --------')
    disp('------------------')
    disp("min: " + num2str(K_min, '%4.1f') + "N/m")
    disp("max: " + num2str(K_max, '%4.1f') + "N/m")
    disp("mean: " + num2str(K_mean, '%4.1f') + "N/m")
    disp("relative std: " + num2str(round(100*K_std/K_mean),'%i') + "%")
end
if nb_param > 2
    B_max = max(impedance(2,:));
    B_min = min(impedance(2,:));
    B_mean = nanmean(impedance(2,:));
    B_std = nanstd(impedance(2,:));
    disp('------- B --------')
    disp('------------------')
    disp("min: " + num2str(B_min, '%2.2f') + "N.s/m")
    disp("max: " + num2str(B_max, '%2.2f') + "N.s/m")
    disp("mean: " + num2str(B_mean, '%2.2f') + "N.s/m")
    disp("relative std: " + num2str(round(100*B_std/B_mean),'%i') + "%")
end
if nb_param > 3
    I_max = max(impedance(3,:));
    I_min = nanmin(impedance(3,:));
    I_mean = nanmean(impedance(3,:));
    I_std = nanstd(impedance(3,:));
    disp('------- I --------')
    disp('------------------')
    disp("min: " + num2str(I_min, '%1.3f') + "kg")
    disp("max: " + num2str(I_max, '%1.3f') + "kg")
    disp("mean: " + num2str(I_mean, '%1.3f') + "kg")
    disp("relative std: " + num2str(round(100*I_std/I_mean),'%i') + "%")
end

r2_mean = nanmean(r2);
r2_val_mean = nanmean(r2_val);

disp('------- R^2 -------')
disp('------------------')
disp("perturbed mean: " + num2str(r2_mean, '%1.3f'))
disp("non perturbed mean: " + num2str(r2_val_mean, '%1.3f'))

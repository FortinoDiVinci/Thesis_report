%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Normalized trajectory evaluation for trajectory prediction
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% This script evaluates the ability of normalized trajectories to approach 
% a human arm rhythmic trajectory (half cycles are used here)

clear all
close all

file_name = 'successful_exp_data.mat';
load(file_name);

%%%%%%%%%%%%%%%%%%
%% MACROS & variables
%%%%%%%%%%%%%%%%%%
% display macros
DISP_ERRORS_HISTOGRAM = 1

%selection of the experimental time
T_START                      = 73.5
T_END                        = 115

idx_start_pt = ceil(0.100/dt);        % 100ms 
% time evaluation variables
idx_traj_fit        = ceil(0.100/dt); % 100ms before and after the window
idx_window          = ceil(0.200/dt); % 200ms

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

t_1st_peak = T_START;
t_last_peak = T_END;
idx_peaks = idx_peaks(t(idx_peaks)>t_1st_peak & t(idx_peaks)<t_last_peak);
t_on_dist = t_on_dist(t_on_dist > t_1st_peak & t_on_dist < t_last_peak);

t_on_dist = t_on_dist(t_on_dist>t_1st_peak & t_on_dist<t_last_peak);
idx_perts = zeros(size(t_on_dist)); % global indexes of all perturbations
% perturbation indexes
for pert_idx = 1:length(t_on_dist)
    idx_perts(pert_idx) = find(t >= t_on_dist(pert_idx), 1, 'first');
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
        z_cycle_np{cyc_np_idx} = z_cycle{cyc_idx};
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

% learns cycle norms
learning_cycles = zn2_cycle_np(:, 1:floor(length(zn2_cycle_np(1,:))/2));
% set to validate the data
validation_cycles = zn2_cycle_np(:, floor(length(zn2_cycle_np(1,:))/2)+1:end);

idx_traj_fit_n = idx_traj_fit*length(t_norm)/(idx_window + 2*idx_traj_fit);
idx_fit = [1:ceil(idx_traj_fit_n), length(t_norm)-floor(idx_traj_fit_n):length(t_norm)];

cycle_fit = cell(length(validation_cycles(1,:)),1);
cycle_real = cell(length(validation_cycles(1,:)),1);

% Best fit matching from the first half of the cycles (only the idx_fit 
% ratio is used to match the equivalent of 100ms before and after the window)
for cycle_idx = 1:length(validation_cycles(1,:))
    fit_err = validation_cycles(idx_fit,cycle_idx) - learning_cycles(idx_fit,:);
    fit_err_avg = mean(fit_err);
    fit_err_std = std(fit_err);
    fit_rmse = rms(fit_err);
    fit_err_score = rms(fit_err);%abs(fit_err_avg) + fit_err_std;
    [~,min_score_idx] = min(abs(fit_err_score));

    val_cycle_idx = floor(length(zn2_cycle_np(1,:))/2)+cycle_idx;
    cycle = z_cycle_np{val_cycle_idx};
    cycle_fit{cycle_idx} = interp1(t_norm, learning_cycles(:,min_score_idx), (1:length(cycle))./length(cycle));
    mag_coef = max(cycle) - min(cycle);
    offset = min(cycle);
    cycle_fit{cycle_idx} = mag_coef.*cycle_fit{cycle_idx} + offset;
    cycle_real{cycle_idx} = cycle';
end

error = cellfun(@minus, cycle_real, cycle_fit ,'UniformOutput', false);

if DISP_ERRORS_HISTOGRAM
    figure(1)
    subplot(1,2,1)
    histHandle = histogram([error{:}],50);
    hold on, grid on
    avgn_tot = nanmean([error{:}]);
    stdn_tot = nanstd([error{:}]);
    line([avgn_tot+stdn_tot, avgn_tot+stdn_tot], [0, max(histHandle.Values)], 'Color','black','LineStyle','--','linewidth',2);
    line([avgn_tot-stdn_tot, avgn_tot-stdn_tot], [0, max(histHandle.Values)], 'Color','black','LineStyle','--','linewidth',2);
    line([avgn_tot, avgn_tot], [0, max(histHandle.Values)], 'Color','red','LineStyle','--','linewidth',2);
end

% Bennet average position centered on velocity maximum method
bennet_rising_data = [];
bennet_decrea_data = [];
for pk_idx = 1:length(idx_peaks)-1
    
    if (idx_pert_cycle(pk_idx) ) % perturbed cycle
        %bennet_meth_rising_data = [bennet_meth_rising_data, NaN(200,1)];
        %bennet_meth_decrea_data = [bennet_meth_decrea_data, NaN(200,1)];
        continue
    end
    
    idx_cycle = idx_peaks(pk_idx):idx_peaks(pk_idx+1);

    [~,vel_peak] = max(abs(dz(idx_cycle)));
    vel_peak = vel_peak + idx_peaks(pk_idx);
    
    amp = abs(max(z(vel_peak-100+1:vel_peak+100)) - min(z(vel_peak-100+1:vel_peak+100)));
    
    if(z_cycle_type(pk_idx) == 1) % rising
        bennet_rising_data = [bennet_rising_data, ...
            (z(vel_peak-100+1:vel_peak+100) - z(vel_peak-100+1))/amp];
    else
        bennet_decrea_data = [bennet_decrea_data, ...
            (z(vel_peak-100+1:vel_peak+100) - z(vel_peak-100+1))/amp];
    end
    
end

bennet_mean_rising = mean(bennet_rising_data,2);
bennet_err_rising = bennet_mean_rising - bennet_rising_data;

figure
hold on
plot(bennet_rising_data)
plot(bennet_mean_rising, '--k', 'LineWidth',2)

% Abe & Yamada cosine fitting method
%ft = fittype('cosine_fit_abe_yamada(t,f, a,b,c)');
abe_rising_data = [];
abe_decrea_data = [];
abe_err_rising = [];
for pk_idx = 1:length(idx_peaks)-1
    
    if (idx_pert_cycle(pk_idx) ) % perturbed cycle
        continue
    end
    
    idx_cycle = idx_peaks(pk_idx):idx_peaks(pk_idx+1);
    f_m = 0.5/(t(idx_cycle(end)) - t(idx_cycle(1)));
    idx_eval = (idx_cycle(100)+1:idx_cycle(100)+200)';
    idx_tot = (idx_cycle(1)-99:idx_cycle(100)+200)';
    %f_abe_fit = 2*pi*f_m*1e-3;
    t_abe_fit = t(idx_cycle(1)-99:idx_cycle(100));
    z_abe_fit = z(idx_cycle(1)-99:idx_cycle(100));
    f_abe_fit = ones(size(t_abe_fit))*f_m*1e-3;
    ft = fittype( @(a,b,c,t,f) a + b * cos(2*pi*f .* t - c),'independent', {'t', 'f'} );
    %ft = fit(t_abe_fit, {z_abe_fit, 'a + b * cos(',num2str(f_abe_fit),' * x - c)'}, ...
    %    'StartPoint', [mean(z(idx_cycle)), abs(max(z(idx_cycle))-min(z(idx_cycle))), 0]);
    f = fit([t_abe_fit, f_abe_fit], z_abe_fit, ft, 'StartPoint', ...
        [mean(z(idx_cycle)), abs(max(z(idx_cycle))-min(z(idx_cycle))), 0]);
    f_abe_fit = ones(size(idx_cycle))*f_m;
    coef = coeffvalues(f);
    if(z_cycle_type(pk_idx) == 1) % rising
        abe_rising_data = [abe_rising_data, ...
            cosine_fit_abe_yamada(t(idx_tot), f_abe_fit(1), coef(1), coef(2), coef(3))];        
        abe_err_rising = [abe_err_rising, ...
            z(idx_eval) - abe_rising_data(200:end,end)];  
%         figure()
%         plot(abe_rising_data(:,end))
%         hold on
%         plot(z(idx_tot))
    else
        abe_decrea_data = [abe_decrea_data, ...
            cosine_fit_abe_yamada(t(idx_eval), f_abe_fit(1), coef(1), coef(2), coef(3))];  
    end
    
end

f1_err = [];
f2_err = [];
f3_err = [];
for pk_idx = 1:length(idx_peaks)-1
    
    if (idx_pert_cycle(pk_idx) ) % perturbed cycle
        continue
    end
    
    idx_cycle = idx_peaks(pk_idx):idx_peaks(pk_idx+1);
    f_m = 0.5/(t(idx_cycle(end)) - t(idx_cycle(1)));
    idx_eval = (idx_cycle(100)+1:idx_cycle(100)+200)';
    idx_tot = (idx_cycle(1):idx_cycle(100)+300)';
    idx_fit = [idx_cycle(1):idx_cycle(100), idx_cycle(100)+200:idx_cycle(100)+300]';
  
%     t_abe_fit = t(idx_cycle(1)-99:idx_cycle(100));
%     z_abe_fit = z(idx_cycle(1)-99:idx_cycle(100)) - mean(z(idx_cycle));
    t_abe_fit = t(idx_fit);
    z_abe_fit = z(idx_fit) - mean(z(idx_cycle));

    f1 = fit(t_abe_fit,z_abe_fit,'sin1');
    f2 = fit(t_abe_fit,z_abe_fit,'sin2');
    f3 = fit(t_abe_fit,z_abe_fit,'sin3');
    
%     z_eval = z(idx_eval);    
%     f1_err = [f1_err, ( z_eval - mean(z(idx_cycle)) ) - feval(f1,t(idx_eval))];
%     f2_err = [f2_err, ( z_eval - mean(z(idx_cycle)) ) - feval(f2,t(idx_eval))];
%     f3_err = [f3_err, ( z_eval - mean(z(idx_cycle)) ) - feval(f3,t(idx_eval))];
    
%     figure
%     plot(t(idx_tot), z(idx_tot) - mean(z(idx_cycle)), 'LineWidth',1.75)
%     hold on
%     plot(f1, '--r')
%     plot(f2, '--g')
%     plot(f3, '--b')
%     legend('real', 'sin1', 'sin2', 'sin3')
end
    
unp_pk_idx = 0;
% spline methodology on the same learning data
for pk_idx = 1:length(idx_peaks)-1
    
    % getting rid of perturbed data
    if(idx_pert_cycle(pk_idx))
        continue
    else
        unp_pk_idx = unp_pk_idx + 1;
        % only using the same validation data
        if(unp_pk_idx < floor(length(zn2_cycle_np(1,:))/2)+1)
            continue
        end
        
        % see impedance estimation file for more details about this methods
        fit_idx = idx_peaks(pk_idx) + idx_start_pt;
        idx_fit = [ (fit_idx - idx_traj_fit+1) : (fit_idx) ,...
        (fit_idx + idx_window) : (fit_idx + idx_window + idx_traj_fit) ];
    	idx_tot = [ (fit_idx - idx_traj_fit+1) : (fit_idx + idx_window + idx_traj_fit) ];    
    
        t_interp = t(idx_fit);
        z_interp = z(idx_fit);
        dz_interp = dz(idx_fit);
        
        idx = unp_pk_idx - floor(length(zn2_cycle_np(1,:))/2);
        
        % real data
        z_fit(:, idx) = z(idx_tot);
        dz_fit(:, idx) = dz(idx_tot);
        t_fit(:, idx) = t(idx_tot);
        % spline estimation
        z_virt(:, idx) = interp1(t_interp, z_interp, t_fit(:, idx), 'spline');
        dz_virt(:, idx) = Iu_diffcent(z_virt(:, idx),t_fit(:, idx));
        dz_virt_vel(:, idx) = interp1(t_interp, dz_interp, t_fit(:, idx), 'spline');
        % error
        spline_error(:, idx) = z_fit(:, idx) - z_virt(:, idx);
        dspline_error(:, idx) = dz_fit(:, idx) - dz_virt(:, idx);
        spline_vel_error(:, idx) = dz_fit(:, idx) - dz_virt_vel(:, idx);
    end
end

figure
plot(dz_fit(:, 8))
hold on
plot(dz_virt(:, 8))
plot(dz_virt_vel(:, 8))
legend('real', 'dPspline', 'Vspline')

if DISP_ERRORS_HISTOGRAM
    figure(1)
    subplot(1,2,2)
    prediction_errors = spline_error(idx_traj_fit+1:idx_traj_fit + idx_window,:);
    histHandle = histogram(prediction_errors(:),50);
    hold on, grid on
    avg_tot = nanmean(prediction_errors(:));
    std_tot = nanstd(prediction_errors(:));
    line([avg_tot+std_tot, avg_tot+std_tot], [0, max(histHandle.Values)], 'Color','black','LineStyle','--','linewidth',2);
    line([avg_tot-std_tot, avg_tot-std_tot], [0, max(histHandle.Values)], 'Color','black','LineStyle','--','linewidth',2);
    line([avg_tot, avg_tot], [0, max(histHandle.Values)], 'Color','red','LineStyle','--','linewidth',2);
end
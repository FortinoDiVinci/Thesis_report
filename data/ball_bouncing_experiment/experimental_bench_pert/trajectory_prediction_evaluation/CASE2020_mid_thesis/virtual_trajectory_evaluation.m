%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   SPLINE ESTIMATION
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% The script proposes to approach trajectories of human rhythmic movement 
% using normalized previous trajectories.

clear all
close all

addpath('../../Other data')

file_name = 'successful_exp_data.mat';
load(file_name);

%%%%%%%%%%%%%%%%%%
%% MACROS & variables
%%%%%%%%%%%%%%%%%%
% display macros
DISP_TIME_FRAMES_OF_INTEREST = 0
DISP_SPLINE_FITS_VS_REAL     = 1   
DISP_NORMAL_ERRORS_HISTOGRAM = 0
DISP_TOTAL_ERRORS_HISTOGRAM  = 1
% macros
SPLINE_FIT_USING_VELOCITIES  = 0

%selection of the experimental time
T_START                      = 73.5
T_END                        = 115

% time evaluation variables
idx_start_pt        = ceil(0.155/dt); % 
%idx_start_pt        = ceil(-0.10/dt); % determines the starting point after
% peak, the value can be negative to evaluate splines during peaks  
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

t_1st_peak = T_START; % not really the first peak...
t_last_peak = T_END;
idx_peaks = idx_peaks(t(idx_peaks)>t_1st_peak & t(idx_peaks)<t_last_peak);
t_on_dist = t_on_dist(t_on_dist > t_1st_peak & t_on_dist < t_last_peak);

t_on_dist = t_on_dist(t_on_dist>t_1st_peak & t_on_dist<t_last_peak);
idx_perts = zeros(size(t_on_dist)); % global indexes of all perturbations
% perturbation indexes
for cycle_idx = 1:length(t_on_dist)
    idx_perts(cycle_idx) = find(t >= t_on_dist(cycle_idx), 1, 'first');
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
dz_cycle = cell(size(t_cycle));
ddz_cycle = cell(size(t_cycle));
fz_cycle = cell(size(t_cycle));

% half cycle selection
for pk_idx = 1:length(idx_peaks)-1
    idx_cycle = idx_peaks(pk_idx):idx_peaks(pk_idx+1);
    t_cycle{pk_idx} = t(idx_cycle);
    z_cycle{pk_idx} = z(idx_cycle);
    dz_cycle{pk_idx} = dz(idx_cycle);
    ddz_cycle{pk_idx} = ddz(idx_cycle);
    fz_cycle{pk_idx} = fz(idx_cycle);
end

z_cycle_type = zeros(size(t_cycle)); % classify 1/2 by type (/ or \)
idx_pert_cycle = zeros(size(t_cycle)); % classify 1/2 cycles by perturbation

% sort cycles according to the presence of perturbation or not 
% and if they are rising or decreasing (for position only)
for cyc_idx = 1:length(z_cycle)
    if any( (min(t_cycle{cyc_idx}) <= t_on_dist) & ...
            (max(t_cycle{cyc_idx}) >= t_on_dist) ) %% perturbation detected
        idx_pert_cycle(cyc_idx) = 1;
    end
    
    % rising or decreasing phase 
    if z_cycle{cyc_idx}(1) < z_cycle{cyc_idx}(end)
        z_cycle_type(cyc_idx) = 1; % rising         
    else
        z_cycle_type(cyc_idx) = 0; % decreasing
    end
    
end

% for non perturbed trajectories only
z_cycle_type_np = z_cycle_type(logical(~idx_pert_cycle)); 

%% unpertubed behaviour analysis

tot_size = 2*idx_traj_fit + idx_window;
% real and virtual trajectories and forces for the estimation
z_fit = NaN(tot_size, length(z_cycle_type_np));
dz_fit = NaN(size(z_fit));
ddz_fit = NaN(size(z_fit));
fz_fit = NaN(size(z_fit));
t_fit = NaN(size(z_fit));
z_virt = NaN(size(z_fit));
dz_virt = NaN(size(z_fit));
ddz_virt = NaN(size(z_fit));
fz_virt = NaN(size(z_fit));
% errors
z_error = NaN(idx_window, length(z_cycle_type_np));
dz_error = NaN(size(z_error));
ddz_error = NaN(size(z_error));
fz_error = NaN(size(z_error));
% normalized errors
zn_error = NaN(size(z_error));
dzn_error = NaN(size(z_error));
ddzn_error = NaN(size(z_error));
fzn_error = NaN(size(z_error));

i = 0; % idx correction to compensate perturbed cycles

for cycle_idx = 1:length(t_cycle) % indexes whithin the cycles
    
    % check if a perturbation occured in the 1/2 cycle 
    if idx_pert_cycle(cycle_idx)
        i = i + 1;
        continue
    end
    
    % definition of the time windows for the virtual trajectories
    % index outside framing the window of interest 
    idx_fit = [ (-idx_traj_fit+1) : (0) , (idx_window) :...
       (idx_window + idx_traj_fit) ] + idx_start_pt + idx_peaks(cycle_idx);  
    % index of fitting time + window of interest
    idx_tot = [ (-idx_traj_fit+1) : (idx_window + idx_traj_fit) ]...
        + idx_start_pt + idx_peaks(cycle_idx); 
    
    %%%%%%%%%%%
    % estimation set
    t_interp = t(idx_fit);
    z_interp = z(idx_fit);
    dz_interp = dz(idx_fit);
    fz_interp = fz(idx_fit);

    % perturbed data
    z_fit(:, cycle_idx-i) = z(idx_tot);
    dz_fit(:, cycle_idx-i) = dz(idx_tot);
    ddz_fit(:, cycle_idx-i) = ddz(idx_tot);
    fz_fit(:, cycle_idx-i) = fz(idx_tot);
    t_fit(:, cycle_idx-i) = t(idx_tot);
    
    if SPLINE_FIT_USING_VELOCITIES
        % virtual estimation (using splines)
        dz_virt(:, cycle_idx-i) = interp1(t_interp, dz_interp, t_fit(:, cycle_idx-i), 'spline');
        z_virt(:, cycle_idx-i) = Iu_intcent(dz_virt(:, cycle_idx-i), t_fit(:, cycle_idx-i), z_fit(1, cycle_idx-i));
        ddz_virt(:, cycle_idx-i) = Iu_diffcent(dz_virt(:, cycle_idx-i), t_fit(:, cycle_idx-i));
    else
        % virtual estimation (using splines)
        z_virt(:, cycle_idx-i) = interp1(t_interp, z_interp, t_fit(:, cycle_idx-i), 'spline');
        dz_virt(:, cycle_idx-i) = Iu_diffcent(z_virt(:, cycle_idx-i), t_fit(:, cycle_idx-i));
        ddz_virt(:, cycle_idx-i) = Iu_diffcent(dz_virt(:, cycle_idx-i), t_fit(:, cycle_idx-i));
    end
    fz_virt(:, cycle_idx-i) = interp1(t_interp, fz_interp, t_fit(:, cycle_idx-i), 'spline'); 
    
    % error between real and estimated trajectories
    eval_window = [1:idx_window]+idx_traj_fit;
    z_error(:, cycle_idx-i) = z_fit(eval_window, cycle_idx-i) - z_virt(eval_window, cycle_idx-i);
    dz_error(:, cycle_idx-i) = dz_fit(eval_window, cycle_idx-i) - dz_virt(eval_window, cycle_idx-i);
    ddz_error(:, cycle_idx-i) = ddz_fit(eval_window, cycle_idx-i) - ddz_virt(eval_window, cycle_idx-i);
    fz_error(:, cycle_idx-i) = fz_fit(eval_window, cycle_idx-i) - fz_virt(eval_window, cycle_idx-i);   
    
    zn_error(:, cycle_idx-i) = z_error(:, cycle_idx-i)./abs(max(z_fit(:, cycle_idx-i)) - min(z_fit(:, cycle_idx-i)));
    dzn_error(:, cycle_idx-i) = dz_error(:, cycle_idx-i)./abs(max(dz_fit(:, cycle_idx-i)) - min(dz_fit(:, cycle_idx-i)));
    ddzn_error(:, cycle_idx-i) = ddz_error(:, cycle_idx-i)./abs(max(ddz_fit(:, cycle_idx-i)) - min(ddz_fit(:, cycle_idx-i)));
    fzn_error(:, cycle_idx-i) = fz_error(:, cycle_idx-i)./abs(max(fz_fit(:, cycle_idx-i)) - min(fz_fit(:, cycle_idx-i)));
    
    if DISP_SPLINE_FITS_VS_REAL
        figure(3)
        subplot(5, ceil((length(t_cycle)-length(t_on_dist))/5), cycle_idx-i)
        hold on, grid on
        %plot(z_fit(:, cycle_idx-i))
        %plot(z_virt(:, cycle_idx-i), ':', 'lineWidth', 2) 
        plot(z_error(:, cycle_idx-i))
    end
    
end

if DISP_NORMAL_ERRORS_HISTOGRAM
    figure(4)
    subplot(2,2,1)
    histHandle = histogram(zn_error(:,logical(z_cycle_type_np)),50);
    hold on, grid on
    avg_zn_error_r = mean2(zn_error(:,logical(z_cycle_type_np)));
    std_zn_error_r = std2(zn_error(:,logical(z_cycle_type_np)));   
    line([0, 0]+(avg_zn_error_r+std_zn_error_r), [0, max(histHandle.Values)], 'Color','black','LineStyle','--','linewidth',2);
    line([0, 0]+(avg_zn_error_r-std_zn_error_r), [0, max(histHandle.Values)], 'Color','black','LineStyle','--','linewidth',2);
    line([0, 0]+avg_zn_error_r, [0, max(histHandle.Values)], 'Color','red','LineStyle','--','linewidth',2);
    title('Rising cycle normalized position error')
    subplot(2,2,2)
    histHandle = histogram(zn_error(:,logical(~z_cycle_type_np)),50);
    hold on, grid on
    avg_zn_error_d = mean2(zn_error(:,~logical(z_cycle_type_np)));
    std_zn_error_d = std2(zn_error(:,~logical(z_cycle_type_np)));   
    line([0, 0]+(avg_zn_error_d+std_zn_error_d), [0, max(histHandle.Values)], 'Color','black','LineStyle','--','linewidth',2);
    line([0, 0]+(avg_zn_error_d-std_zn_error_d), [0, max(histHandle.Values)], 'Color','black','LineStyle','--','linewidth',2);
    line([0, 0]+avg_zn_error_d, [0, max(histHandle.Values)], 'Color','red','LineStyle','--','linewidth',2);
    title('Decreasing cycle normalized position error')
    subplot(2,2,3)
    histHandle = histogram(fzn_error(:,logical(z_cycle_type_np)),50);
    hold on, grid on
    avg_fzn_error_r = mean2(fzn_error(:,logical(z_cycle_type_np)));
    std_fzn_error_r = std2(fzn_error(:,logical(z_cycle_type_np)));   
    line([0, 0]+(avg_fzn_error_r+std_fzn_error_r), [0, max(histHandle.Values)], 'Color','black','LineStyle','--','linewidth',2);
    line([0, 0]+(avg_fzn_error_r-std_fzn_error_r), [0, max(histHandle.Values)], 'Color','black','LineStyle','--','linewidth',2);
    line([0, 0]+avg_fzn_error_r, [0, max(histHandle.Values)], 'Color','red','LineStyle','--','linewidth',2);
    title('Rising cycle normalized force error')
    subplot(2,2,4)
    histHandle = histogram(fzn_error(:,logical(~z_cycle_type_np)),50);
    hold on, grid on
    avg_fzn_error_d = mean2(fzn_error(:,logical(~z_cycle_type_np)));
    std_fzn_error_d = std2(fzn_error(:,logical(~z_cycle_type_np)));   
    line([0, 0]+(avg_fzn_error_d+std_fzn_error_d), [0, max(histHandle.Values)], 'Color','black','LineStyle','--','linewidth',2);
    line([0, 0]+(avg_fzn_error_d-std_fzn_error_d), [0, max(histHandle.Values)], 'Color','black','LineStyle','--','linewidth',2);
    line([0, 0]+avg_fzn_error_d, [0, max(histHandle.Values)], 'Color','red','LineStyle','--','linewidth',2);
    title('Decreasing cycle normalized force error')
end

if DISP_TOTAL_ERRORS_HISTOGRAM
    figure(4)
    subplot(2,2,1)
    histHandle = histogram(z_error(:),50);
    hold on, grid on
    avg_z_error_r = mean(z_error(:)); % mean2 ?
    std_z_error_r = std(z_error(:)); % std2 ?
    line([0, 0]+(avg_z_error_r+std_z_error_r), [0, max(histHandle.Values)], 'Color','black','LineStyle','--','linewidth',2);
    line([0, 0]+(avg_z_error_r-std_z_error_r), [0, max(histHandle.Values)], 'Color','black','LineStyle','--','linewidth',2);
    line([0, 0]+avg_z_error_r, [0, max(histHandle.Values)], 'Color','red','LineStyle','--','linewidth',2);
    title('Position errors')
    
    subplot(2,2,2)
    histHandle = histogram(dz_error(:),50);
    hold on, grid on
    avg_dz_error_r = mean(dz_error(:));
    std_dz_error_r = std(dz_error(:));   
    line([0, 0]+(avg_dz_error_r+std_dz_error_r), [0, max(histHandle.Values)], 'Color','black','LineStyle','--','linewidth',2);
    line([0, 0]+(avg_dz_error_r-std_dz_error_r), [0, max(histHandle.Values)], 'Color','black','LineStyle','--','linewidth',2);
    line([0, 0]+avg_dz_error_r, [0, max(histHandle.Values)], 'Color','red','LineStyle','--','linewidth',2);
    title('Velocity errors')
    
    subplot(2,2,3)
    histHandle = histogram(ddz_error(:),50);
    hold on, grid on
    avg_ddz_error_r = mean(ddz_error(:));
    std_ddz_error_r = std(ddz_error(:));   
    line([0, 0]+(avg_ddz_error_r+std_ddz_error_r), [0, max(histHandle.Values)], 'Color','black','LineStyle','--','linewidth',2);
    line([0, 0]+(avg_ddz_error_r-std_ddz_error_r), [0, max(histHandle.Values)], 'Color','black','LineStyle','--','linewidth',2);
    line([0, 0]+avg_ddz_error_r, [0, max(histHandle.Values)], 'Color','red','LineStyle','--','linewidth',2);
    title('Acceleration errors')
    
    subplot(2,2,4)
    histHandle = histogram(fz_error(:),50);
    hold on, grid on
    avg_fz_error_r = mean(fz_error(:));
    std_fz_error_r = std(fz_error(:));   
    line([0, 0]+(avg_fz_error_r+std_fz_error_r), [0, max(histHandle.Values)], 'Color','black','LineStyle','--','linewidth',2);
    line([0, 0]+(avg_fz_error_r-std_fz_error_r), [0, max(histHandle.Values)], 'Color','black','LineStyle','--','linewidth',2);
    line([0, 0]+avg_fz_error_r, [0, max(histHandle.Values)], 'Color','red','LineStyle','--','linewidth',2);
    title('Force errors')
    
end

%% data saving
% errors
zn_error_r_save = fzn_error(:,logical(z_cycle_type_np));
zn_error_d_save = fzn_error(:,logical(~z_cycle_type_np));
%zn_error_save = {zn_error_r_save(:), zn_error_d_save(:)};
%zn_error_save_table = cell2table(zn_error_save,'VariableNames',{'i','ii'});
%writetable(zn_error_save_table, 'spline_z_norm_error_histograms.csv');
dlmwrite('tempf1.csv', zn_error_r_save(:),'delimiter',',','precision',5);
dlmwrite('tempf2.csv', zn_error_d_save(:),'delimiter',',','precision',5);

%trajectories examples
% dlmwrite('temp_traj_virt_r.csv', [z_fit(:,6), z_virt(:,6)],'delimiter',',','precision',5);
% dlmwrite('temp_traj_virt_d.csv', [z_fit(:,42), z_virt(:,42)],'delimiter',',','precision',5);
% dlmwrite('temp_traj_virt_iv.csv', [z_fit(:,36), z_virt(:,36)],'delimiter',',','precision',5);
% dlmwrite('temp_traj_virt_iii.csv', [z_fit(:,37), z_virt(:,37)],'delimiter',',','precision',5);

% down sampling at 50Hz
% t_50hz = (75:0.02:110)';
% z_50hz = interp1(t, (z-0.3)*3, t_50hz, 'linear');
% zb_50hz = interp1(t, z_b{2}, t_50hz, 'linear');
% dlmwrite('ball_boucing_exp2.csv', [t_50hz, z_50hz, zb_50hz],'delimiter',',','precision',5);

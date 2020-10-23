clear all
close all

load('data_vfo_3_phases.mat')

addpath('../../force_torque_sensor')
addpath('../../youBot_analysis/Utils')

% i = 3;
% idx_st = 2.1e4;
% idx_end = 1.1e5; 
% 
% i = 7;
% idx_st = 4.2e4;
% idx_end = 1.6e5; 
%
% i = 10;
% idx_st = 1.45e4;
% idx_end = 1.1e5; 
%
% i = 1; % martin
% idx_st = 1.42e4;
% idx_end = 3.2e5; 
%
idx_st = [1.2e4, 1.22e4, 1.31e4, 1.15e4, 1.08e4, 1.1e4, 1e4];
idx_end = [1.2e5, 1.0e5, 1.09e5, 1.071e5, 9.8e4, 1.05e5, 1.026e5]; 

target_height = 1.7;
paddle_offset = 0.3;
kinematic_coeff = 6;
global_cycles = [];

for i = 1:length(idx_st)

    %% ball signal processing
    
    fc = 25; % cut off frequency
    [b,a] = butter(2,fc/(1/(2*dt)),'low'); 

    zb = filtfilt(b,a,z_b{i});
    dzb = Iu_diffcent(zb,t{i});
    zb_old = zb;
    zb = zb(idx_st(i):idx_end(i));
    dzb = dzb(idx_st(i):idx_end(i));

    impact = crossing(dzb);
    idx_imp = impact(diff(impact)>100);
    idx_imp_inf = idx_imp(zb(idx_imp) < 0.5);
    idx_apex = idx_imp(zb(idx_imp) > 0.5);

    t_new = t{i}(idx_st(i):idx_end(i));

    % bouncing error 
    rms_be = sqrt(mean((zb(idx_apex) - target_height).^2))/target_height; 
    
    figure()
    plot(t{i}, zb_old)
    hold on
    plot(t_new, zb)
    plot(t_new(idx_imp_inf), zb(idx_imp_inf), '*k',  'MarkerSize', 10)
    line([t_new(1), t_new(end)], [target_height, target_height], 'Color','red','LineStyle','--','linewidth',2)
    title('Bouncing RMSE: ' + string(rms_be*1e2) + '%')

    %% disturbance data processing

    t_dist_on = t_dist{i}(1:2:end);
    val_dist_on = dist{i}(1:2:end);
    t_dist_off = t_dist{i}(2:2:end);
    val_dist_off = dist{i}(2:2:end);
    t_dist_avg = mean([t_dist_on, t_dist_off], 2); % middle of the perturbation

    pert_idx = [];
    pert_val = [];
    for dist_idx = 1:length(t_dist_avg)
        if t_dist_avg(dist_idx) < min(t_new)
            continue;
        end
        pert_idx = [pert_idx, find(t_new >= t_dist_avg(dist_idx) , 1, 'first')];   
        pert_val = [pert_val, val_dist_on(dist_idx)];
    end

    %% hand position signal processing

    zh = filtfilt(b,a,mocap_marker_robot_base{i}(:,3)); %z{i}
    dzh = Iu_diffcent(zh,t{i});
    zh = zh(idx_st(i):idx_end(i))*1e2; % conversion to cm
    dzh = dzh(idx_st(i):idx_end(i));
    
    zp = (zh*1e-2 - paddle_offset) * kinematic_coeff;
    
    plot(t_new, zp, 'color', [0.4940, 0.1840, 0.5560])
    plot(t_new(pert_idx), zp(pert_idx), 'pk', 'MarkerFaceColor', [0.6350, 0.0780, 0.1840],  'MarkerSize', 10)
    
    fc = 0.1; % cut off frequency
    [b2,a2] = butter(2,fc/(1/(2*dt)),'low'); 
    z_avg = filtfilt(b2,a2,zh);
 
    figure()
    plot(zh-z_avg, dzh)
    hold on
    plot(zh(idx_imp_inf)-z_avg(idx_imp_inf), dzh(idx_imp_inf), '.r', 'MarkerSize',30)
    plot(zh(pert_idx)-z_avg(pert_idx), dzh(pert_idx), '.m', 'MarkerSize',30)
    xlim([-8, 8])
    title('Phase diagram ' + string(i))
    
    %% Per cycle analysis
    
    % cycle initialization
    for cyc_nb = length(idx_imp_inf):-1:2 % this way the object vector is preallocated       
        i_cyc = (idx_imp_inf(cyc_nb-1):idx_imp_inf(cyc_nb))';
        t_cyc = t_new(i_cyc);
        z_cyc = zh(i_cyc);
        vz_cyc = dzh(i_cyc);        
        cycles(cyc_nb-1) = CYCLE_DATA(t_cyc, z_cyc, vz_cyc, i_cyc);      
        % did a perturbation occured in this cycle ?
        bool_pert_cyc = logical(t_cyc(1) < t_new(pert_idx)) & logical(t_cyc(end) > t_new(pert_idx));
        if(any(bool_pert_cyc))   
            cycles(cyc_nb-1).setPerturbation(pert_val(bool_pert_cyc), t_new(pert_idx(bool_pert_cyc)));
        end
    end

    % cycle normalization    
    cycles_norm = cycle_data_norm(cycles);
    % cycle statistics computation
    
    plot(cycles_norm_mean.time, cycles_norm_mean.velocity, 'k', 'linewidth', 2)
    fill([cycles_norm_mean.time fliplr(cycles_norm_mean.time)], ...
        [(cycles_norm_mean.velocity+cycles_norm_std.velocity)' ...
        fliplr((cycles_norm_mean.velocity-cycles_norm_std.velocity)')], ...
        [0.25, 0.25, 0.25], 'FaceAlpha', 0.3,'linestyle','-','edgecolor',[0.25, 0.25, 0.25])
    
    global_cycles = [global_cycles, cycles];
end


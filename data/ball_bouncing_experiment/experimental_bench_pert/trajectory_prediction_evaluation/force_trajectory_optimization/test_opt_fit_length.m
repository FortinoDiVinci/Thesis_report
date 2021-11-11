clear all

load '../../data_2020_Nov_17/data_without_impacts_2020_11_17.mat' 't' 'dt' ...
     'thetas' 'forces_unf' 'idx_ball_off_ramp'
addpath('../../utils') % 
addpath('../../../../force_torque_sensor') % for force pre-processing
addpath('../../../../utils') % 

DO_BOTH_DIRECTION = 1; % set to zero for only positive perturbation, 
                       % and to 1 to to have positive & negative pert.
REAL_VIRT_POSITION = 1; % either simulate virtual position using KBM model, 
                        % or use a measured unperturbed trajectory
HEAVY_DATA = 1; % when testing numerous configuration, only the median and 
                % the quartile error of K, B, M and R2 are kept to avoid 
                % crashing                        
                        
base_file_name = "test_opt_force_window_a_";
                        
if DO_BOTH_DIRECTION
    pert_dir = -1;
else
    pert_dir = 1;
end

M_list = [0.6];%[0.6, 2.8];%[2.8];%
B_list = [12];%[12, 44];%[44];%
K_list = [280];%[280, 539];%[539];%

exp_nb = 7;
t = t{exp_nb} - t{exp_nb}(1); % t0 = 0s
[b,a] = butter(2,50/(1/(2*dt)),'low'); % BW 2nd order low pass filter (cutoff freq. 50 Hz)
idx_s = idx_ball_off_ramp{exp_nb};

f_tmp = forces_filtering(forces_unf{exp_nb}', forces_unf{exp_nb}', thetas{exp_nb}', t); % from sensor base to robot base
f_tmp = -f_tmp(3,:)'; % fz conversion from f(e->r) to f(r->e), robot force on the environment
f = filtfilt(b,a,f_tmp); % zero phase digital filtering
clear forces_unf thetas f_tmp idx_ball_off_ramp

input_force.signals.values = f;
input_force.time = t;

% perturbation introduced in simulation
ext_signal = 1;
pert_mag = 5;
pert_space = ceil(2.1/dt); % samples
pert_duration = 0.030/dt;  % samples
pert_init_delay = 5/dt;    % initial delay
% perturbation filter
xi = sqrt(2)/2;
w0 = 40*pi*1;
t_max = t(end);

alt_direction = zeros(size(t));
j = 1;
for i = 1:pert_space:(length(t) - pert_init_delay)
    idx = pert_init_delay + i + (0:pert_duration);
    alt_direction(idx) = (pert_dir)^j;
    pert_idx(j) = idx(1);
    j = j + 1;
end
input_direction.signals.values = alt_direction;
input_direction.time = t;

% delete perturbations that are introduced before rythmic behaviour
del_idx = find(pert_idx <= idx_s, 1, 'last');
if ~isempty(del_idx)
    pert_idx(1:del_idx) = [];
end
% delete last perturbation to avoid overflow if the perturbation occurs in
% the last samples
pert_idx(end) = [];

for config_nb = 1:length(M_list)

    file_name = base_file_name+string(config_nb);
    
    Mv = M_list(config_nb);
    Bv = B_list(config_nb);
    Kv = K_list(config_nb);
        
    K_f = 5*w0^2;%20*Kv;

    out = sim('../../Impedance_env_simulation/KBM_sim',t_max);
    
    fz = out.force.data;
    fz0 = out.virt_force.data;
    z = out.position.data;
    z0 = out.virt_position.data;
    t = out.force.Time;
    % get the rising edges indexes of the perturbations
    % This method seems to induce errors...   
    pert_val = pert_mag.*alt_direction(pert_idx);

    % PARAMETERS
    idx_wndw_fit_b_min = ceil(0.030/dt); % 30ms 
    idx_wndw_fit_b_max = ceil(0.120/dt); % 120ms 
    idx_wndw_fit_a_min = ceil(0.080/dt); % 80ms 
    idx_wndw_fit_a_max = ceil(0.200/dt); % 200ms 
    mask_size = ceil(0.100/dt); % 100ms 
    STEP = 10;
    idx_wndw_imp_eval_list = ceil([.1,.15,.2,.25,.3]./dt); % 100ms - 300ms 
    idx_delay            = ceil(0.000/dt); % 0ms
    %idx_window = max(idx_wndw_imp_eval, idx_wndw_virt_traj);
    nb_param = 3; % K B M

    idx_lw = (idx_wndw_fit_b_min:STEP:idx_wndw_fit_b_max)';
    idx_up = (idx_wndw_fit_a_min:STEP:idx_wndw_fit_a_max)';
    % DATA PRE-PROCESSING
    kj = 1;
    ki = 1;
    for i = idx_lw'
        for j = idx_up'
            tic
%             if ki == 1 && kj == 1
%                 continue
%             end
            delta_fz{ki,kj} = DIFF_TRAJECT(max(idx_wndw_imp_eval_list), mask_size, ...
                fz, t, pert_idx, pert_val, idx_delay);
            delta_fz{ki,kj}.computeDiffTraject('VirtTrajMethod', 'sineOptM', ...
                'LowerFitLen', i,'UpperFitLen', j);
            timeElapsed(kj,ki) = toc;
            timeElapsed(kj,ki)
            kj = kj + 1;
        end        
        ki = ki + 1;
        kj = 1;
    end
    
    % real differential force and position
    for i = 1:length(pert_idx)
        idx = pert_idx(i) + (0:max(idx_wndw_imp_eval_list)-1);
        diff_pos(:,i) = z(idx) - z0(idx);
        diff_for(:,i) = fz(idx) - fz0(idx);
    end

    % trajectory error in force
    kj = 1;
    ki = 1;
    for i = 1:length(idx_lw)
        for j = 1:length(idx_up)
            error_force(:,:,ki,kj) = single(diff_for - delta_fz{ki,kj}.diff_traject);%delta_fz{ki,kj}.diff_traject);
            kj = kj + 1;
        end
        ki = ki + 1;
        kj = 1;
    end
    
    for i = 1:size(diff_pos,2)
        d_diff_pos(:,i) = Iu_diffcent(diff_pos(:,i), delta_fz{1,1}.t_traject(3:end-2,i));
        dd_diff_pos(:,i) = Iu_diffcent(d_diff_pos(:,i), delta_fz{1,1}.t_traject(3:end-2,i));
    end
    
    nb_id_wdw = length(idx_wndw_imp_eval_list);
    K_all = zeros(length(pert_idx),nb_id_wdw,size(delta_fz,1),size(delta_fz,2),'single');
    B_all = zeros(size(K_all),'single');
    M_all = zeros(size(K_all),'single');
    r2_all = zeros(size(K_all),'single');
    for j = nb_id_wdw:-1:1
        idx_wndw_imp_eval = idx_wndw_imp_eval_list(j);
        impedance = IMPEDANCE_DATA(3, length(pert_idx), idx_wndw_imp_eval);
        impedance.init_phi(diff_pos, d_diff_pos, dd_diff_pos);
        tic
        for i = size(delta_fz,1):-1:1
            for j = size(delta_fz,2):-1:1
                impedance.init_y(delta_fz{i,j}.diff_traject(1:idx_wndw_imp_eval,:) - ...
                    delta_fz{i,j}.diff_traject(1,:));
                impedance.arx('NulInitialCond');
                impedance.causalSim(dt,'NulInitialCond'); 
                K_all(:,j,i) = impedance.xi(1,:);
                B_all(:,j,i) = impedance.xi(2,:);
                M_all(:,j,i) = impedance.xi(3,:);
                r2_all(:,j,i) = impedance.r2_pos;  
            end
        end        
        timeElapsedImp(j) = toc;
        timeElapsedImp(j)
    end

     save(file_name+".mat", 'delta_fz', 'diff_force', 'diff_pos', 'r2_all', ...
         'idx_wndw_fit_b_min', 'idx_wndw_fit_b_max', 'idx_wndw_fit_a_min', ...
         'idx_wndw_fit_a_max', 'mask_size', 'Mv', 'Bv', 'Kv', 'STEP', ...
         'K_all', 'B_all', 'M_all', 'error_force', '-v7.3');    
     
%     acc = cellfun(@(x) prctile(real(x.r2_pos), [25,50,75])', impedance, 'UniformOutput', false);
%     quartiles_r2 = cell2mat(acc);
%     acc = cellfun(@(x) prctile(abs(real(x.xi(1,:))-Kv)/Kv, [25,50,75])', impedance, 'UniformOutput', false);
%     k_rel_err = cell2mat(acc);
%     acc = cellfun(@(x) prctile(abs(real(x.xi(2,:))-Bv)/Bv, [25,50,75])', impedance, 'UniformOutput', false);
%     b_rel_err = cell2mat(acc);
%     acc = cellfun(@(x) prctile(abs(real(x.xi(3,:))-Mv)/Mv, [25,50,75])', impedance, 'UniformOutput', false);
%     m_rel_err = cell2mat(acc);
%     acc = cellfun(@(x) prctile(real(x.xi(1,:)), [25,50,75])', impedance, 'UniformOutput', false);
%     quartiles_k = cell2mat(acc);
%     acc = cellfun(@(x) prctile(real(x.xi(2,:)), [25,50,75])', impedance, 'UniformOutput', false);
%     quartiles_b = cell2mat(acc);
%     acc = cellfun(@(x) prctile(real(x.xi(3,:)), [25,50,75])', impedance, 'UniformOutput', false);
%     quartiles_m = cell2mat(acc);
% 
%     colors = lines(3);
% 
%     figure
%     subplot(3,1,1)
%     plot(idx_samples, quartiles_r2(2,:), 'Color', colors(1,:))
%     hold on
%     plot(idx_samples, quartiles_r2(1,:), ':', 'Color', colors(1,:))
%     plot(idx_samples, quartiles_r2(3,:), ':', 'Color', colors(1,:))
%     subplot(3,1,2)
%     plot(idx_samples, k_rel_err(2,:), 'Color', colors(1,:))
%     hold on
%     plot(idx_samples, k_rel_err(1,:), ':', 'Color', colors(1,:))
%     plot(idx_samples, k_rel_err(3,:), ':', 'Color', colors(1,:))
%     plot(idx_samples, b_rel_err(2,:), 'Color', colors(2,:))
%     plot(idx_samples, b_rel_err(1,:), ':', 'Color', colors(2,:))
%     plot(idx_samples, b_rel_err(3,:), ':', 'Color', colors(2,:))
%     plot(idx_samples, m_rel_err(2,:), 'Color', colors(3,:))
%     plot(idx_samples, m_rel_err(1,:), ':', 'Color', colors(3,:))
%     plot(idx_samples, m_rel_err(3,:), ':', 'Color', colors(3,:))
%     subplot(3,3,7)
%     plot(idx_samples, quartiles_k(2,:))
%     hold on
%     plot([idx_wndw_virt_traj_min,idx_wndw_virt_traj_max], [Kv, Kv])
%     plot(idx_samples, quartiles_k(1,:), ':', 'Color', lines(1))
%     plot(idx_samples, quartiles_k(3,:), ':', 'Color', lines(1))
%     subplot(3,3,8)
%     plot(idx_samples, quartiles_b(2,:))
%     hold on
%     plot([idx_wndw_virt_traj_min,idx_wndw_virt_traj_max], [Bv, Bv])
%     plot(idx_samples, quartiles_b(1,:), ':', 'Color', lines(1))
%     plot(idx_samples, quartiles_b(3,:), ':', 'Color', lines(1))
%     subplot(3,3,9)
%     plot(idx_samples, quartiles_m(2,:))
%     hold on
%     plot([idx_wndw_virt_traj_min,idx_wndw_virt_traj_max], [Mv, Mv])
%     plot(idx_samples, quartiles_m(1,:), ':', 'Color', lines(1))
%     plot(idx_samples, quartiles_m(3,:), ':', 'Color', lines(1))
% 
% 
%     tmp = cellfun(@(x) [x.xi(1,:)]', impedance, 'UniformOutput', false);
%     K_all = cell2mat(tmp);
% 
%     med_rel_err_K = abs(quartiles_k(2,:) - Kv)/Kv;
%     med_rel_err_B = abs(quartiles_b(2,:) - Bv)/Bv;
%     med_rel_err_M = abs(quartiles_m(2,:) - Mv)/Mv;
% 
%     figure
%     plot(idx_samples, med_rel_err_K)
%     hold on
%     plot(idx_samples, med_rel_err_B)
%     plot(idx_samples, med_rel_err_M)
% 
%     Ke = med_rel_err_K';
%     Be = med_rel_err_B';
%     Me = med_rel_err_M';
%     time_window = idx_samples;
% 
%     table_rel_err = table(time_window, Ke, Be, Me);
%     write(table_rel_err,file_name+".csv",'Delimiter',',');

end



clear all

load '../data_2020_Nov_17/data_without_impacts_2020_11_17.mat' 't' 'dt' ...
     'thetas' 'forces_unf' 'mocap_marker_robot_base' 'idx_ball_off_ramp'
addpath('../utils') % 
addpath('../../../force_torque_sensor') % for force pre-processing
addpath('../../../utils') % 

DO_BOTH_DIRECTION = 1; % set to zero for only positive perturbation, 
                       % and to 1 to to have positive & negative pert.
REAL_VIRT_POSITION = 1; % either simulate virtual position using KBM model, 
                        % or use a measured unperturbed trajectory
HEAVY_DATA = 1; % when testing numerous configuration, only the median and 
                % the quartile error of K, B, M and R2 are kept to avoid 
                % crashing                        
                        
base_file_name = "test_spline_interp_window_500ms_k_";
                        
if DO_BOTH_DIRECTION
    pert_dir = -1;
else
    pert_dir = 1;
end

M_list = [0.6, 2.8];
B_list = [12, 44];
K_list = [280, 539];

exp_nb = 7;
t = t{exp_nb} - t{exp_nb}(1); % t0 = 0s
[b,a] = butter(2,50/(1/(2*dt)),'low'); % BW 2nd order low pass filter (cutoff freq. 50 Hz)
idx_s = idx_ball_off_ramp{exp_nb};

if REAL_VIRT_POSITION
    p_tmp = mocap_marker_robot_base{exp_nb};
    p = filtfilt(b,a,p_tmp(:,3));
    f = zeros(size(p));
    clear forces_unf thetas p_tmp mocap_marker_robot_base idx_ball_off_ramp

    input_position.signals.values = p;
    input_position.time = t;
else
    f_tmp = forces_filtering(forces_unf{exp_nb}', forces_unf{exp_nb}', thetas{exp_nb}', t); % from sensor base to robot base
    f_tmp = -f_tmp(3,:)'; % fz conversion from f(e->r) to f(r->e), robot force on the environment
    f = filtfilt(b,a,f_tmp); % zero phase digital filtering
    clear forces_unf thetas f_tmp mocap_marker_robot_base idx_ball_off_ramp

    input_force.signals.values = f;
    input_force.time = t;
end

% perturbation introduced in simulation
ext_signal = 1;
pert_mag = 10;
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
        
    K_f = 10*Kv;
    
    if REAL_VIRT_POSITION
        out = sim('../Impedance_env_simulation/KBM_sim_real_position',t_max);
    else
        out = sim('../Impedance_env_simulation/KBM_sim',t_max);
    end
    
    fz = out.force.data;
    z = out.position.data;
    t = out.force.Time;
    % get the rising edges indexes of the perturbations
    % This method seems to induce errors...   
    pert_val = pert_mag.*ones(size(pert_idx));

    % PARAMETERS
    idx_wndw_virt_traj_min = ceil(0.200/dt); % 200ms (position)
    idx_wndw_virt_traj_max = ceil(0.450/dt); % 500ms (position)
    STEP = 3;
    idx_wndw_imp_eval_min = ceil(0.100/dt); % 100ms
    idx_wndw_imp_eval_max = ceil(0.300/dt); % 300ms
    %idx_wndw_imp_eval    = ceil(0.200/dt); % 200ms       
    idx_delay            = ceil(0.000/dt); % 0ms
    %idx_window = max(idx_wndw_imp_eval, idx_wndw_virt_traj);
    nb_param = 3; % K B M

    idx_samples = (idx_wndw_virt_traj_min:STEP:idx_wndw_virt_traj_max)';
    % DATA PRE-PROCESSING
    delta_z{1} = DIFF_TRAJECT(idx_wndw_imp_eval_max, idx_wndw_virt_traj_min, z, t, pert_idx, pert_val, idx_delay);
    for i = idx_samples'
        delta_z{(i-idx_wndw_virt_traj_min)/STEP+1} = DIFF_TRAJECT(idx_wndw_imp_eval_max, i, z, t, pert_idx, pert_val, idx_delay);
        delta_z{(i-idx_wndw_virt_traj_min)/STEP+1}.computeDiffTraject('VirtTrajMethod', 'spline');
        delta_z{(i-idx_wndw_virt_traj_min)/STEP+1}.computeDerivatives();
    end
    %delta_fz = DIFF_TRAJECT(idx_wndw_imp_eval, idx_wndw_virt_f_traj, fz, t, pert_idx, pert_val, idx_delay);
    %delta_fz.computeDiffTraject('VirtTrajMethod', 'sineOptM', 'OptNbSine', 3, 'OptlinearComp', 0); % this step might take few seconds

    for i = 1:length(pert_idx)
        idx = pert_idx(i) + (0:idx_wndw_imp_eval_max-1);
        diff_force(:,i) = fz(idx) - f(idx);
    end

    %impedance{1} = IMPEDANCE_DATA(3, length(pert_idx), idx_wndw_imp_eval);
    %impedance{1}.init_y(delta_fz.diff_traject(1:idx_wndw_imp_eval,:) - ...
    %    delta_fz.diff_traject(1,:));
    if HEAVY_DATA
        nb_id_wdw = length(idx_wndw_imp_eval_min:STEP:idx_wndw_imp_eval_max);
        K_all = zeros(length(pert_idx),nb_id_wdw,length(delta_z),'single');
        impedance = IMPEDANCE_DATA(3, length(pert_idx), 300);
        impedance.init_y(diff_force);
        for j = nb_id_wdw:-1:1
            idx_wndw_imp_eval = idx_wndw_imp_eval_min + (j-1)*STEP;
            impedance.id_size = idx_wndw_imp_eval;
            for i = length(delta_z):-1:1
                tic
                impedance.init_phi(delta_z{i}.diff_traject(1:idx_wndw_imp_eval,:) - ...
                    delta_z{i}.diff_traject(1,:), ...
                    delta_z{i}.d_diff_traject(1:idx_wndw_imp_eval,:), ... % speed
                    delta_z{i}.dd_diff_traject(1:idx_wndw_imp_eval,:));
                impedance.arx('NulInitialCond');
                impedance.causalSim(dt,'NulInitialCond'); 
                K_all(:,j,i) = impedance.xi(1,:);
                B_all(:,j,i) = impedance.xi(2,:);
                M_all(:,j,i) = impedance.xi(3,:);
                r2_all(:,j,i) = impedance.r2_pos;                
                timeElapsed(j,i) = toc;
                timeElapsed(j,i)
            end
        end
    else
        for i = length(delta_z):-1:1
            tic
            for j = length(idx_wndw_imp_eval_min:STEP:idx_wndw_imp_eval_max):-1:1
                idx_wndw_imp_eval = idx_wndw_imp_eval_min + (j-1)*STEP;
                impedance{i,j} = IMPEDANCE_DATA(3, length(pert_idx), idx_wndw_imp_eval);
                impedance{i,j}.init_y(diff_force);
                impedance{i,j}.init_phi(delta_z{i}.diff_traject(1:idx_wndw_imp_eval,:) - ...
                    delta_z{i}.diff_traject(1,:), ...
                    delta_z{i}.d_diff_traject(1:idx_wndw_imp_eval,:), ... % speed
                    delta_z{i}.dd_diff_traject(1:idx_wndw_imp_eval,:)); % acc
                impedance{i,j}.arx('NulInitialCond');
                impedance{i,j}.causalSim(dt,'NulInitialCond'); 
            end
            timeElapsed(i) = toc;
            timeElapsed(i)
        end
    end

%     impedance{1}.init_phi(delta_z{1}.diff_traject(1:idx_wndw_imp_eval,:) - ...
%         delta_z{1}.diff_traject(1,:), ...
%             delta_z{1}.d_diff_traject(1:idx_wndw_imp_eval,:), ... % speed
%             delta_z{1}.dd_diff_traject(1:idx_wndw_imp_eval,:)); % acc
%     impedance{1}.arx('NulInitialCond');
%     impedance{1}.causalSim(dt,'NulInitialCond'); 
    if HEAVY_DATA
         save(file_name+".mat", 'delta_z', 'diff_force', 'r2_all', ...
             'idx_wndw_virt_traj_min', 'idx_wndw_virt_traj_max',...
             'idx_wndw_imp_eval_min', 'idx_wndw_imp_eval_max', 'Mv', 'Bv', ...
             'Kv', 'STEP', 'K_all', 'B_all', 'M_all', '-v7.3');
         clear K_all B_all M_all r2_all
    else
        save(file_name+".mat", 'impedance', 'idx_wndw_imp_eval_min', ...
        'delta_z', 'diff_force', 'idx_wndw_virt_traj_min', 'idx_wndw_virt_traj_max',...
        'Mv', 'Bv', 'Kv', 'STEP', 'idx_wndw_imp_eval_max', '-v7.3');
    end

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



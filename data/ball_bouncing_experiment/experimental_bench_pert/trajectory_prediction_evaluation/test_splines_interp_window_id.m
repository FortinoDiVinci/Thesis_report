clear all

load '../data_2020_Nov_17/data_without_impacts_2020_11_17.mat' 't' 'thetas' 'forces_unf' ...
    'dt'
addpath('../utils') % 
addpath('../../../force_torque_sensor') % for force pre-processing
addpath('../../../utils') % 

M_list = [0.6, 2.8];
B_list = [12, 44];
K_list = [280, 539];

exp_nb = 7;
t = t{exp_nb} - t{exp_nb}(1); % t0 = 0s
f_tmp = forces_filtering(forces_unf{exp_nb}', forces_unf{exp_nb}', thetas{exp_nb}', t); % from sensor base to robot base
%
[b,a] = butter(2,50/(1/(2*dt)),'low'); % BW 2nd order low pass filter (cutoff freq. 50 Hz)
f_tmp = -f_tmp(3,:)'; % fz conversion from f(e->r) to f(r->e), robot force on the environment
f = filtfilt(b,a,f_tmp); % zero phase digital filtering
clear forces_unf thetas f_tmp % clear unecessary data

for config_nb = 1:length(M_list)

    Mv = M_list(config_nb);
    Bv = B_list(config_nb);
    Kv = K_list(config_nb);

    input_force.signals.values = f;
    input_force.time = t;
    % perturbation introduced in simulation
    ext_signal = 1;
    pert_mag = 10;
    pert_space = ceil(1.4/dt); % samples
    pert_duration = 0.030/dt;  % samples
    % perturbation filter
    xi = sqrt(2)/2;
    w0 = 40*pi*1;
    K_f = 3e3;
    t_max = t(end);
    out = sim('../Impedance_env_simulation/KBM_sim',t_max);

    fz = out.force.data;
    z = out.position.data;
    t = out.force.Time;
    % get the rising edges indexes of the perturbations
    pert_idx = find(diff(out.perturbations.data) > 0)';
    pert_val = pert_mag.*ones(size(pert_idx));

    % PARAMETERS
    idx_wndw_virt_traj_min   = ceil(0.200/dt); % 200ms (position)
    idx_wndw_virt_traj_max   = ceil(0.500/dt); % 500ms (position)
    STEP = 3;
    idx_wndw_virt_f_traj = ceil(0.100/dt); % 100ms  (force) 
    idx_wndw_imp_eval    = ceil(0.200/dt); % 200ms       
    idx_delay            = ceil(0.000/dt); % 0ms
    %idx_window = max(idx_wndw_imp_eval, idx_wndw_virt_traj);
    nb_param = 3; % K B M

    idx_samples = (idx_wndw_virt_traj_min:STEP:idx_wndw_virt_traj_max)';
    % DATA PRE-PROCESSING
    delta_z{1} = DIFF_TRAJECT(idx_wndw_imp_eval, idx_wndw_virt_traj_min, z, t, pert_idx, pert_val, idx_delay);
    for i = idx_samples'
        delta_z{(i-idx_wndw_virt_traj_min)/STEP+1} = DIFF_TRAJECT(idx_wndw_imp_eval, i, z, t, pert_idx, pert_val, idx_delay);
        delta_z{(i-idx_wndw_virt_traj_min)/STEP+1}.computeDiffTraject('VirtTrajMethod', 'spline');
        delta_z{(i-idx_wndw_virt_traj_min)/STEP+1}.computeDerivatives();
    end
    %delta_fz = DIFF_TRAJECT(idx_wndw_imp_eval, idx_wndw_virt_f_traj, fz, t, pert_idx, pert_val, idx_delay);
    %delta_fz.computeDiffTraject('VirtTrajMethod', 'sineOptM', 'OptNbSine', 3, 'OptlinearComp', 0); % this step might take few seconds

    diff_force = repmat(fz(pert_idx:pert_idx+199),1,length(pert_idx));

    tic
    impedance{1} = IMPEDANCE_DATA(3, length(pert_idx), idx_wndw_imp_eval);
    %impedance{1}.init_y(delta_fz.diff_traject(1:idx_wndw_imp_eval,:) - ...
    %    delta_fz.diff_traject(1,:));
    impedance{1}.init_y(diff_force);
    for i = 2:length(delta_z)
        impedance{i} = copyObj(impedance{1});
        impedance{i}.init_phi(delta_z{i}.diff_traject(1:idx_wndw_imp_eval,:) - ...
            delta_z{i}.diff_traject(1,:), ...
            delta_z{i}.d_diff_traject(1:idx_wndw_imp_eval,:), ... % speed
            delta_z{i}.dd_diff_traject(1:idx_wndw_imp_eval,:)); % acc
        impedance{i}.arx('NulInitialCond');
        impedance{i}.causalSim(dt,'NulInitialCond'); 

    end

    impedance{1}.init_phi(delta_z{1}.diff_traject(1:idx_wndw_imp_eval,:) - ...
        delta_z{1}.diff_traject(1,:), ...
            delta_z{1}.d_diff_traject(1:idx_wndw_imp_eval,:), ... % speed
            delta_z{1}.dd_diff_traject(1:idx_wndw_imp_eval,:)); % acc
    impedance{1}.arx('NulInitialCond');
    impedance{1}.causalSim(dt,'NulInitialCond'); 
    timeElapsed = toc

    save("test_spline_interp_window_500ms_"+string(config_nb)+".mat", 'impedance', ...
        'delta_z', 'diff_force', 'idx_wndw_virt_traj_min', 'idx_wndw_virt_traj_max',...
        'idx_wndw_imp_eval', 'Mv', 'Bv', 'Kv', '-v7.3');

    acc = cellfun(@(x) prctile(x.r2_pos, [25,50,75])', impedance, 'UniformOutput', false);
    quartiles_r2 = cell2mat(acc);
    acc = cellfun(@(x) prctile(abs(x.xi(1,:)-Kv)/Kv, [25,50,75])', impedance, 'UniformOutput', false);
    k_rel_err = cell2mat(acc);
    acc = cellfun(@(x) prctile(abs(x.xi(2,:)-Bv)/Bv, [25,50,75])', impedance, 'UniformOutput', false);
    b_rel_err = cell2mat(acc);
    acc = cellfun(@(x) prctile(abs(x.xi(3,:)-Mv)/Mv, [25,50,75])', impedance, 'UniformOutput', false);
    m_rel_err = cell2mat(acc);
    acc = cellfun(@(x) prctile(x.xi(1,:), [25,50,75])', impedance, 'UniformOutput', false);
    quartiles_k = cell2mat(acc);
    acc = cellfun(@(x) prctile(x.xi(2,:), [25,50,75])', impedance, 'UniformOutput', false);
    quartiles_b = cell2mat(acc);
    acc = cellfun(@(x) prctile(x.xi(3,:), [25,50,75])', impedance, 'UniformOutput', false);
    quartiles_m = cell2mat(acc);

    colors = lines(3);

    figure
    subplot(3,1,1)
    plot(idx_samples, quartiles_r2(2,:), 'Color', colors(1,:))
    hold on
    plot(idx_samples, quartiles_r2(1,:), ':', 'Color', colors(1,:))
    plot(idx_samples, quartiles_r2(3,:), ':', 'Color', colors(1,:))
    subplot(3,1,2)
    plot(idx_samples, k_rel_err(2,:), 'Color', colors(1,:))
    hold on
    plot(idx_samples, k_rel_err(1,:), ':', 'Color', colors(1,:))
    plot(idx_samples, k_rel_err(3,:), ':', 'Color', colors(1,:))
    plot(idx_samples, b_rel_err(2,:), 'Color', colors(2,:))
    plot(idx_samples, b_rel_err(1,:), ':', 'Color', colors(2,:))
    plot(idx_samples, b_rel_err(3,:), ':', 'Color', colors(2,:))
    plot(idx_samples, m_rel_err(2,:), 'Color', colors(3,:))
    plot(idx_samples, m_rel_err(1,:), ':', 'Color', colors(3,:))
    plot(idx_samples, m_rel_err(3,:), ':', 'Color', colors(3,:))
    subplot(3,3,7)
    plot(idx_samples, quartiles_k(2,:))
    hold on
    plot([idx_wndw_virt_traj_min,idx_wndw_virt_traj_max], [Kv, Kv])
    plot(idx_samples, quartiles_k(1,:), ':', 'Color', lines(1))
    plot(idx_samples, quartiles_k(3,:), ':', 'Color', lines(1))
    subplot(3,3,8)
    plot(idx_samples, quartiles_b(2,:))
    hold on
    plot([idx_wndw_virt_traj_min,idx_wndw_virt_traj_max], [Bv, Bv])
    plot(idx_samples, quartiles_b(1,:), ':', 'Color', lines(1))
    plot(idx_samples, quartiles_b(3,:), ':', 'Color', lines(1))
    subplot(3,3,9)
    plot(idx_samples, quartiles_m(2,:))
    hold on
    plot([idx_wndw_virt_traj_min,idx_wndw_virt_traj_max], [Mv, Mv])
    plot(idx_samples, quartiles_m(1,:), ':', 'Color', lines(1))
    plot(idx_samples, quartiles_m(3,:), ':', 'Color', lines(1))


    tmp = cellfun(@(x) [x.xi(1,:)]', impedance, 'UniformOutput', false);
    K_all = cell2mat(tmp);

    med_rel_err_K = abs(quartiles_k(2,:) - Kv)/Kv;
    med_rel_err_B = abs(quartiles_b(2,:) - Bv)/Bv;
    med_rel_err_M = abs(quartiles_m(2,:) - Mv)/Mv;

    figure
    plot(idx_samples, med_rel_err_K)
    hold on
    plot(idx_samples, med_rel_err_B)
    plot(idx_samples, med_rel_err_M)

    Ke = med_rel_err_K';
    Be = med_rel_err_B';
    Me = med_rel_err_M';
    time_window = idx_samples;

    table_rel_err_Kv539 = table(time_window, Ke, Be, Me);
    write(table_rel_err_Kv539,"spline_window_interp_id_500ms"+string(config_nb)+".csv",'Delimiter',',');

end
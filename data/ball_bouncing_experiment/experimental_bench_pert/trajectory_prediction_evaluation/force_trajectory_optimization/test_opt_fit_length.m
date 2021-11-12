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

M_list = [2.8];%[0.6];%[0.6, 2.8];%
B_list = [44];%[12];%[12, 44];%
K_list = [539];%[280];%[280, 539];%

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
    for k = nb_id_wdw:-1:1
        idx_wndw_imp_eval = idx_wndw_imp_eval_list(k);
        impedance = IMPEDANCE_DATA(3, length(pert_idx), idx_wndw_imp_eval);
        impedance.init_phi(diff_pos, d_diff_pos, dd_diff_pos);
        tic
        for i = size(delta_fz,1):-1:1
            for j = size(delta_fz,2):-1:1
                impedance.init_y(delta_fz{i,j}.diff_traject(1:idx_wndw_imp_eval,:) - ...
                    delta_fz{i,j}.diff_traject(1,:));
                impedance.arx('NulInitialCond');
                impedance.causalSim(dt,'NulInitialCond'); 
                K_all(:,j,i,k) = impedance.xi(1,:);
                B_all(:,j,i,k) = impedance.xi(2,:);
                M_all(:,j,i,k) = impedance.xi(3,:);
                r2_all(:,j,i,k) = impedance.r2_pos;  
            end
        end        
        timeElapsedImp(j) = toc;
        timeElapsedImp(j)
    end

    save(file_name+".mat", 'delta_fz', 'diff_for', 'diff_pos', 'r2_all', ...
         'idx_wndw_fit_b_min', 'idx_wndw_fit_b_max', 'idx_wndw_fit_a_min', ...
         'idx_wndw_fit_a_max', 'mask_size', 'Mv', 'Bv', 'Kv', 'STEP', ...
         'K_all', 'B_all', 'M_all', 'error_force', '-v7.3');    
     
    for i = 1:size(error_force,3)
        for j = 1:size(error_force,4)
            tmp = error_force(1:100,:,i,j);
            rmse_for_100ms(i,j) = rms(tmp(:));
            tmp = error_force(1:200,:,i,j);
            rmse_for_200ms(i,j) = rms(tmp(:));
            tmp = error_force(:,:,i,j);
            rmse_for_300ms(i,j) = rms(tmp(:));
        end
    end
     
    figure
    subplot(1,3,1)
    surf(idx_up,idx_lw,rmse_for_300ms)
    colorbar
    view(2)
    subplot(1,3,2)
    surf(idx_up,idx_lw,rmse_for_200ms)
    colorbar
    view(2)
    subplot(1,3,3)
    surf(idx_up,idx_lw,rmse_for_100ms)
    colorbar
    view(2)
    
    if SAVE_TRAJ_ERROR_CSV
        rmse_300ms = reshape(rmse_for_300ms,[],1);
        rmse_100ms = reshape(rmse_for_100ms,[],1);
        lower_window = repmat(idx_lw',1,size(rmse_for_300ms,2))';
        upper_window = reshape(repmat(idx_up',size(rmse_for_300ms,1),1),[],1);
        table_csv = table(lower_window, upper_window, rmse_100ms, rmse_300ms);
        write(table_csv,'sine_opt_err_traject.csv','Delimiter',',');
    end
    
    for i = 1:5
        K_prc = prctile(K_all(:,:,:,i), [25,50,75], 1);
        K_med = squeeze(K_prc(2,:,:));
        K_med_err = abs(K_med - Kv)./Kv;
        K_q_e = squeeze(K_prc(3,:,:) - K_prc(1,:,:));
         
        B_prc = prctile(B_all(:,:,:,i), [25,50,75], 1);
        B_med = squeeze(B_prc(2,:,:));
        B_med_err = abs(B_med - Bv)./Bv;
        B_q_e = squeeze(B_prc(3,:,:) - B_prc(1,:,:));
        
        M_prc = prctile(M_all(:,:,:,i), [25,50,75], 1);
        M_med = squeeze(M_prc(2,:,:));
        M_med_err = abs(M_med - Mv)./Mv;
        M_q_e = squeeze(M_prc(3,:,:) - M_prc(1,:,:));
        
        figure
        subplot(2,3,1)
        surf(idx_lw, idx_up, K_med_err)
        view(2)
        colorbar
        title('K median relative error')
        ylabel('Indentif. window')
        subplot(2,3,4)
        surf(idx_lw, idx_up, K_q_e)
        view(2)
        colorbar
        title('K quartile distribution')
        xlabel('Spline interp window')
        ylabel('Indentif. window')
        subplot(2,3,2)
        surf(idx_lw, idx_up, B_med_err)
        view(2)
        colorbar
        title('B median relative error')
        ylabel('Indentif. window')
        subplot(2,3,5)
        surf(idx_lw, idx_up, B_q_e)
        view(2)
        colorbar
        title('B quartile distribution')
        xlabel('Spline interp window')
        ylabel('Indentif. window')
        subplot(2,3,3)
        surf(idx_lw, idx_up, M_med_err)
        view(2)
        colorbar
        title('M median relative error')
        ylabel('Indentif. window')
        subplot(2,3,6)
        %surf(idx_interp_spl(1:35), idx_id_wdw, M_q_e(:,1:35))
        surf(idx_lw, idx_up, M_q_e)
        view(2)
        colorbar
        title('M quartile distribution')
        xlabel('Spline interp window')
        ylabel('Indentif. window')  
    end
    
    if SAVE_PARAM_ERROR_CSV
        K_med_err_csv = reshape(K_med_err,[],1);
        B_med_err_csv = reshape(B_med_err,[],1);
        M_med_err_csv = reshape(M_med_err,[],1);
        lower_window = repmat(idx_lw',1,size(rmse_for_300ms,2))';
        upper_window = reshape(repmat(idx_up',size(rmse_for_300ms,1),1),[],1);
        table_csv = table(lower_window, upper_window, K_med_err_csv, B_med_err_csv, M_med_err_csv);
        write(table_csv,'sine_opt_id_error_200ms_param_1.csv','Delimiter',',');
    end
    
end



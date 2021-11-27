clear all

addpath('../../data/ball_bouncing_experiment/experimental_bench_pert/utils')
addpath('../../data/utils')
addpath('../../data/force_torque_sensor')

%% PARAMETERS
% MACRO
%FILE_NAMES = ["exp_complement_2021.mat","exp_june_2021_ter.mat"];
FILE_NAME_BASE = "exp_june_2021_9ms_delay_";
NB_FILES = 6;
SAVED_FILE_NAME = 'exp_delta_z_2021';
SAVED_FOLDER_NAME = "users_pz/";
% PARAMS
% data processing
low_pass_cutoff_freq = 25; % Input signal are lp filt. before computation
filter_order = 2;
% trajectory windows 
% (the following param consider a sampling frequency of 1kHz)
wndw_virt_traj   = 350; % 200ms
wndw_imp_eval    = 300;
p_delay          = 0;
window = max(wndw_imp_eval, wndw_virt_traj);
%

%load("users_fz/experiments_list.mat", "users_idx_list");

for file_nb = 1:NB_FILES
    load(FILE_NAME_BASE + string(file_nb) + ".mat", ...
        "t", "mocap_robot_endpoint", "t_d", "dist_val", "exp_parameters", "dt");

    if ~exist('dt', 'var')
        dt = 1e-3;
    end

    tot_nb_exp = length(t);

    %% DATA PRE-PROCESSING

    z = {};

    [b,a] = butter(filter_order,low_pass_cutoff_freq/(1/(2*dt)),'low'); 

    % position pre-processing
    for exp_nb = tot_nb_exp:-1:1
        if strcmp(exp_parameters(exp_nb).experience, "calib") || strcmp(exp_parameters(exp_nb).experience, "unknown")
            z{exp_nb} = [];
        else
            z{exp_nb} = filtfilt(b,a,mocap_robot_endpoint{exp_nb}(:,3));
        end
    end
    dist_val_raw = dist_val;
    clear dist_val q ft_sensor f_tmp;
    % disturbance timing extractions
    for exp_nb = tot_nb_exp:-1:1
        if strcmp(exp_parameters(exp_nb).experience, "calib") || strcmp(exp_parameters(exp_nb).experience, "l_phri")
            idx_perts{exp_nb} = [];
            dist_val{exp_nb} = [];
        else
            % get only the rising edges of perturbations
            dist_timings = t_d{exp_nb}(1:2:end);
            dist_val{exp_nb} = dist_val_raw{exp_nb}(1:2:end);
            for pert_idx = 1:length(dist_timings)
                idx_perts{exp_nb}(pert_idx) = find(t{exp_nb} >= ...
                    dist_timings(pert_idx), 1, 'first');
            end
            % if the experiment was interrupted during the last perturbation
            if (idx_perts{exp_nb}(end) + window) > length(t{exp_nb})
                % the last perturbation will no be used for impedance estimation
                idx_perts{exp_nb} = idx_perts{exp_nb}(1:end-1);
                dist_val{exp_nb} = dist_val{exp_nb}(1:end-1);
            end
        end
    end
    clear dist_val_raw
    
    for i = 1:length(t)
        t{i} = t{i} - t{i}(1);
    end

    %% DATA PROCESSING 
    delta_z = {};
    % extraction of the delta of position and force
    %[~, user_idx] = sort(string(vertcat(exp_parameters.user)));
%     if file_nb == 1
%         user_idx = users_idx_list(1:tot_nb_exp);
%         tot_nb_exp_old = tot_nb_exp;
%     else
%         user_idx = users_idx_list(tot_nb_exp_old+1:end);
%         p_delay = 9;
%     end

    list_name = "users_fz/list_" + string(file_nb) + ".mat";
    load(list_name)
    last_user = exp_parameters(user_idx(1)).user;
    ii = 1;
    for exp_nb = 1:tot_nb_exp
        usr_idx = user_idx(exp_nb);
        if strcmp(exp_parameters(usr_idx).experience, "calib") || strcmp(exp_parameters(usr_idx).experience, "l_phri") || ...
            strcmp(exp_parameters(usr_idx).experience, "unknown")
            continue
        end
        current_user = exp_parameters(usr_idx).user;
        if ~strcmp(current_user, last_user)
            str_us = "_" + string(last_user);
            save_file_name = strcat(string(SAVED_FILE_NAME), str_us);
            % to avoid unvoluntary data erasing
            k = 1;
            while exist(strcat(SAVED_FOLDER_NAME, save_file_name) + ".mat", "file")
                warning('The file ' + save_file_name + ".mat, already exists.")
                save_file_name = strcat(save_file_name, "_" + string(k));
                k = k + 1;
            end
            save(strcat(SAVED_FOLDER_NAME, save_file_name + ".mat"), 'delta_z', ...
                'wndw_virt_traj', 'p_delay', 'wndw_imp_eval', '-v7.3');
            last_user = current_user;
            ii = 1;
            delta_z = {};
        end
        % delta z
        header = exp_parameters(usr_idx).experience;
        delta_z{ii} = DIFF_TRAJECT(window, wndw_virt_traj, z{usr_idx},...
            t{usr_idx}, idx_perts{usr_idx}, dist_val{usr_idx}, p_delay, header);
        delta_z{ii}.computeDiffTraject('VirtTrajMethod', 'spline'); 
        ii = ii + 1;
    end
    
    str_us = "_" + string(last_user);
    save_file_name = strcat(string(SAVED_FILE_NAME), str_us);
    k = 1;
    % to avoid unvoluntary data erasing
    while exist(strcat(SAVED_FOLDER_NAME, save_file_name) + ".mat", "file")
        warning('The file ' + save_file_name + ".mat, already exists.")
        save_file_name = strcat(save_file_name, "_"+string(k));
        k = k + 1;
    end
    save(strcat(SAVED_FOLDER_NAME, save_file_name + ".mat"), 'delta_z', ...
        'wndw_virt_traj', 'p_delay', 'wndw_imp_eval', '-v7.3');
    
    clear t mocap_robot_endpoint t_dist dist_val exp_parameters dt delta_z ...
        user_idx dist_val idx_perts z
    
end % file_nb

return

load(SAVED_FOLDER_NAME + string(SAVED_FILE_NAME) + "_851" + ".mat");
out = load(SAVED_FOLDER_NAME + string(SAVED_FILE_NAME) + "_851_1" + ".mat");
delta_z = [delta_z,out.delta_z];
save(SAVED_FOLDER_NAME + string(SAVED_FILE_NAME) + "_851" + ".mat", "delta_z", '-append');

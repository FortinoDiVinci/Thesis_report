clear all

addpath('../../data/ball_bouncing_experiment/experimental_bench_pert/utils')
addpath('../../data/utils')
addpath('../../data/force_torque_sensor')

%% PARAMETERS
% MACRO
%FILE_NAMES = ["exp_complement_2021.mat","exp_june_2021_ter.mat"];
FILE_NAME_BASE = "exp_june_2021_9ms_delay_";
NB_FILES = 6;
SAVED_FILE_NAME = 'exp_ball_bouncing_2021';
SAVED_FOLDER_NAME = "users_bb/";
% PARAMS
% data processing
low_pass_cutoff_freq = 25; % Input signal are lp filt. before computation
filter_order = 2;
% trajectory windows 
% (the following param consider a sampling frequency of 1kHz)

%load("users_fz/experiments_list.mat", "users_idx_list");

for file_nb = 1:NB_FILES
    out = load(FILE_NAME_BASE + string(file_nb) + ".mat", ...
        "t", "bounce_err", "idx_apex", "idx_ball_off_ramp", "z_b", "z_p");
    load(FILE_NAME_BASE + string(file_nb) + ".mat","exp_parameters")
    
    if ~exist('dt', 'var')
        dt = 1e-3;
    end

    tot_nb_exp = length(out.t);

    %% DATA PRE-PROCESSING

    zb = {};
    zp = {};

    [b,a] = butter(filter_order,low_pass_cutoff_freq/(1/(2*dt)),'low'); 

    %% DATA PROCESSING 

    list_name = "users_fz/list_" + string(file_nb) + ".mat";
    load(list_name)
    last_user = exp_parameters(user_idx(1)).user;
    ii = 1;
    for exp_nb = 1:tot_nb_exp
        usr_idx = user_idx(exp_nb);
        current_exp = exp_parameters(usr_idx).experience;
        if strcmp(current_exp, "calib") || strcmp(current_exp, "l_phri") || ...
            strcmp(current_exp, "unknown")
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
            save(strcat(SAVED_FOLDER_NAME, save_file_name + ".mat"), 'zb',...
                'zp','idx_ball_off_ramp','bounce_err','idx_apex','-v7.3');
            last_user = current_user;
            ii = 1;
            zb = {};
            zp = {};
            idx_ball_off_ramp = [];
            idx_apex = {};
            clear bounce_err
        end
        % 
        zb{ii} = out.z_b{usr_idx};
        zp{ii} = out.z_p{usr_idx};
        idx_ball_off_ramp(ii) = out.idx_ball_off_ramp(usr_idx);
        bounce_err(ii) = out.bounce_err{usr_idx};
        idx_apex{ii} = out.idx_apex{usr_idx};
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
    save(strcat(SAVED_FOLDER_NAME, save_file_name + ".mat"), 'zb',...
                'zp','idx_ball_off_ramp','bounce_err','idx_apex','-v7.3');
    
    clear t zb zp idx_ball_off_ramp exp_parameters bounce_err idx_apex out
    
end % file_nb

return

% ref = "922";
% load(SAVED_FOLDER_NAME + string(SAVED_FILE_NAME) + "_" + ref + ".mat");
% out = load(SAVED_FOLDER_NAME + string(SAVED_FILE_NAME) + "_"+ref+"_1" + ".mat");
% idx_ball_off_ramp = [idx_ball_off_ramp,out.idx_ball_off_ramp];
% bounce_err = [bounce_err,out.bounce_err];
% zb = [zb,out.zb];
% zp = [zp,out.zp];
% idx_apex = [idx_apex,out.idx_apex];
% save(SAVED_FOLDER_NAME + string(SAVED_FILE_NAME) + "_" + ref + ".mat", ...
%     "idx_ball_off_ramp", "bounce_err", "zb", "zp", "idx_apex", '-append');
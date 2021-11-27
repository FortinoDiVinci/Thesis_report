clear all
close all

addpath('utils/')
% general functions
addpath('../../data/utils/')
% dedicated classes, functions...
addpath('../../data/ball_bouncing_experiment/experimental_bench_pert/utils/')

%% final experiment
%load('SB2021_new_data_impedance_v2.mat')
load('users_fz/experiments_list.mat') % for the link between the experiments and users
% FILE_NAME_BASE = "exp_june_2021_9ms_delay_";
% NB_FILES = 6;

FORCE_FILE_NAME = 'exp_delta_fz_2021_';
FORCE_FOLDER_NAME = "users_fz/";

POSIT_FILE_NAME = 'exp_delta_z_2021_';
POSIT_FOLDER_NAME = "users_pz/";

BALLB_FILE_NAME = 'exp_ball_bouncing_2021_';
BALLB_FOLDER_NAME = "users_bb/";

% exp_parameters = [];
% for nb_raw_file = 1:length(raw_data_file_list)
%     out = load(raw_data_file_list(nb_raw_file), 'exp_parameters');
%     exp_parameters = [exp_parameters; out.exp_parameters];
% end

load("users_impedance.mat");

users_list = unique(string(vertcat(exp_parameters.user)));
nb_users = length(users_list);

DISP = 0;
USER_REF_NAMES = [];
NB_PHASES = [];
dt = 1e-3;

% 50 Hz filter
[b,a] = butter(2,50/(1/(2*dt)),'low'); 

for user_nb = 1:nb_users
    
    user_name = users_list(user_nb);
    load(FORCE_FOLDER_NAME + FORCE_FILE_NAME + user_name + ".mat");
    load(POSIT_FOLDER_NAME + POSIT_FILE_NAME + user_name + ".mat");
    load(BALLB_FOLDER_NAME + BALLB_FILE_NAME + user_name + ".mat");
    impedance_data_user = impedance_data(strcmp(vertcat(impedance_data.user), user_name));
    idx_exp = strcmp(string(vertcat(exp_parameters.user)), user_name);
    exp_names = [exp_parameters(idx_exp).experience];
    %target_height = exp_parameters(idx_exp).target_height;
    exp_parameters_user_i = exp_parameters(idx_exp);
    del_idx = [];
    for k = 1:length(exp_names)
        if ~(strcmp(exp_names(k), "exp_1") || strcmp(exp_names(k), "exp_2")  || strcmp(exp_names(k), "l_bb"))
            del_idx = [del_idx, k];
        end
    end
    % delete data not corresponding to experiments 1,2 or to learning
    % ball bouncing
    exp_names(del_idx) = [];
    %target_height(del_idx) = [];
    exp_parameters_user_i(del_idx) = [];
    
    le = length(exp_names);
    %lth = length(target_height);
    lfz = length(delta_fz);
    lz = length(delta_fz);
    
    if ~isequal(le,lfz,lz)
        error("Dimension of data does not match for user " + user_name + ...
            ", check dimensions of differential trajectories and the parameters of the experiments");
    end
    
    for exp_nb = 1:le
        
        dz = delta_z{exp_nb};
        dfz = delta_fz{exp_nb};
        idx_p = [dz.pert_ind];
        i_imp = detectBallImpacts(dz.time, zb{exp_nb}, DISP);
        
        if DISP
            plot(dz.time, zp{exp_nb}, 'Color', [0.4940,0.1840,0.5560], 'Linewidth', 2)
            plot([dz.time(1), dz.time(end)], [th, th], 'k--', 'Linewidth', 1.5)
        end
        
        % if the experiment start with a ball on the paddle, the previous 
        % function will detect the initial static instants as impacts
        if isempty(idx_ball_off_ramp(exp_nb))
            idx_ball_off_ramp(exp_nb) = offRampIdx(zb{exp_nb}, dt);
        end
        i_imp = i_imp(i_imp >= idx_ball_off_ramp(exp_nb));
        first_impact = 4; % no perturbation before the 5th impact

        [i_red,idx_i_red] = findRelevantData(dz.time, dz.complete_traject, i_imp, first_impact); 
        if DISP
            plot(dz.time(idx_p), zp{exp_nb}(idx_p), 'rp', 'MarkerFaceColor', 'r',... 
                'Markersize',15);
            plot(dz.time(idx_i_red), zb{exp_nb}(idx_i_red), 'o', 'color', ...
                [0.9290,0.6940,0.1250], 'Markersize', 15, 'Linewidth', 1.5)
        %legend('ball','impacts','paddle', 'target', 'perturb.')
        end
        
%         cycles{exp_nb} = cycleExtraction(idx_i_red, dt, dfz, dz, zb{exp_nb}, ...
%             zp{exp_nb}, exp_parameters_user_i(exp_nb), DISP);
        cycles{exp_nb} = cycleExtractionImpedance(idx_i_red, dt, dfz, dz, zb{exp_nb}, ...
            zp{exp_nb}, exp_parameters_user_i(exp_nb), impedance_data_user(exp_nb), DISP);
        
%         cycles{exp_nb}.K = [experience_data_user(exp_nb).K];
%         cycles{exp_nb}.B = [experience_data_user(exp_nb).B];
%         cycles{exp_nb}.M = [experience_data_user(exp_nb).M];
%         cycles{exp_nb}.r2 = [experience_data_user(exp_nb).r2];
        
    end

end

cycles_norm = cycleNormalisation(cycles, dt);
[center_ratio, count_ratio, idx_ratio, cycles_class] = sortPerturbations(cycles_norm, max(NB_PHASES), DISP);

% regroup users data
for exp_nb = 1:nb_exp
    impedance{exp_nb}.init_t(delta_z{exp_nb}.t_traject(3:end-2,:), delta_z{exp_nb}.time(delta_z{exp_nb}.pert_ind));
end
cycles_imp = addImpedance(cycles_class, impedance);
cycles_regr = regroupCycles(cycles_imp);
% sort outliers according to phase and perturbation direction
cycles_id = clearOutliers(cycles_regr);

% bouncing error variations
plotBouncingError(cycles_id);
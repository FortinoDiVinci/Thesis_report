clear all
close all

addpath('../ROMAN_2021/fct_maria')
addpath('../utils/')
addpath('../../../youBot_analysis/Utils')
addpath('../../../force_torque_sensor')
% 

VERSION = 2;

% load('imp_data_exp1.mat')
% clear impedance
% %load('exp_1_impedance_id/imp_data_exp1_2.mat', 'impedance')
% %load('exp_1_impedance_id/f_wdw_85ms/imp_data_exp1_b9.mat', 'impedance')
% load('exp_1.mat', 'z_p')
%% first SB2021 submission
if VERSION == 1
    load('exp_1_impedance_id/imp_data_exp1_nom_cond.mat', 'impedance', 'delta_fz',...
        'delta_z')
    load('exp_1_nom_cond.mat', 'z_p', 'z_b', 'idx_ball_off_ramp', 'bounc_err')
    USER_REF_NAMES = ["user1";"user2";"user3";"user1"];
    NB_PHASES = 3;
%% corrected SB2021 submission
elseif VERSION == 2
    load('../../../../Experience/data/SB2021_new_data_impedance_v2.mat')
    USER_REF_NAMES = [string(vertcat(exp_parameters.user))];
    NB_PHASES = 3;
    class_order = ["c_1", "c_2", "c_3"];
    expertise_order = ["novice", "intermediate", "advanced"];
end

nb_exp = length(z_b); 
dt = 1e-3;

% 50 Hz filter
[b,a] = butter(2,50/(1/(2*dt)),'low'); 
% cycle_shift = -350; % cycles are shifted by xxx samples so that the impact 
% is no longer the starting point. 

for exp_nb = 1:nb_exp
    
    dz = delta_z{exp_nb};
    dfz = delta_fz{exp_nb};
    idx_p = [delta_z{exp_nb}.pert_ind];
    if VERSION == 1
        i_imp = impacts_extraction2(dz.time, z_b{exp_nb}, 0.6);   
    elseif VERSION == 2
        i_imp = impacts_extraction2(dz.time, z_b{exp_nb}); 
    end
    plot(dz.time, z_p{exp_nb}, 'Color', [0.4940,0.1840,0.5560], 'Linewidth', 2)
    plot([dz.time(1), dz.time(end)], [1.75, 1.75], 'k--', 'Linewidth', 1.5)
    % if the experiment start with a ball on the paddle, the previous 
    % function will detect the initial static instants as impacts
    if VERSION == 1
        if ~isempty(idx_ball_off_ramp{exp_nb})
            i_imp = i_imp(i_imp >= idx_ball_off_ramp{exp_nb});
        end
    elseif VERSION == 2
        if ~isempty(idx_ball_off_ramp(exp_nb))
            i_imp = i_imp(i_imp >= idx_ball_off_ramp(exp_nb));
        end
    end
        
    first_impact = 4; % no perturbation before the 5th impact
    
    [i_red,idx_i_red] = find_relevant_data(dz.time, dz.complete_traject, i_imp, first_impact);
    plot(dz.time(idx_p), z_p{exp_nb}(idx_p), 'rp', 'MarkerFaceColor', 'r',... 
        'Markersize',15);
    plot(dz.time(idx_i_red), z_b{exp_nb}(idx_i_red), 'o', 'color', ...
        [0.9290,0.6940,0.1250], 'Markersize', 15, 'Linewidth', 1.5)
    %legend('ball','impacts','paddle', 'target', 'perturb.')
    
    display = 0;    
    cycles{exp_nb} = cycle_extraction(idx_i_red, dt, dfz, dz, z_b{exp_nb}, z_p{exp_nb}, display);
    
    traject(exp_nb).time = dz.time;
    traject(exp_nb).position = dz.complete_traject;
    traject(exp_nb).velocity = Iu_diffcent(traject(exp_nb).position, traject(exp_nb).time);
    traject(exp_nb).force = dfz.complete_traject;
    traject(exp_nb).yank = Iu_diffcent(traject(exp_nb).force, traject(exp_nb).time);

end

cycles_norm = cycle_normalisation(cycles,dt);
[center_ratio, count_ratio, idx_ratio, cycles_class] = dist_sorting(cycles_norm, NB_PHASES);

cycles_class_regr = disp_temporal_norm_multp_cond(cycles_class, USER_REF_NAMES);

%disp_impedance_per_cycle(cycles_class, impedance, USER_REF_NAMES);
data_sorted = disp_impedance_against_frequency(cycles_class, impedance, USER_REF_NAMES);

%% task error histograms
[user_err_med, user_err_25prc, user_err_75prc] = error_histograms(bounce_err, USER_REF_NAMES);

%%
% user_stiff_med = cellfun(@(x) median(x), reshape([data_sorted.stiffness],[],3));
% user_stiff_25prc = cellfun(@(x) x(1), cellfun(@(x) [prctile(x,[25,50,75])],...
% reshape([data_sorted.stiffness],[],3), 'UniformOutput', false));
% user_stiff_75prc = cellfun(@(x) x(3), cellfun(@(x) [prctile(x,[25,50,75])],...
% reshape([data_sorted.stiffness],[],3), 'UniformOutput', false));

for ui = 1:size(data_sorted,1)
    for ci = 1:size(data_sorted,2)
        tmp = prctile([data_sorted(ui,ci).stiffness],[25,50,75]);
        user_stiff_25prc(ui,ci) = tmp(1);
        user_stiff_med(ui,ci) = tmp(2);
        user_stiff_75prc(ui,ci) = tmp(3);
    end
end

color = hsv(length(user_stiff_med));
markers = ['o','s','^'];
figure('DefaultAxesFontSize',16)
hold on
for i = 1:3
    errorbar(user_stiff_med(:,i),user_err_med,user_err_25prc,user_err_75prc,...
        user_stiff_med(:,i) - user_stiff_25prc(:,i), ...
        user_stiff_med(:,i) - user_stiff_75prc(:,i), markers(i))
end

%% anova analysis
% anova_analysis(cycles_class_regr, data_sorted, user_err_75prc - user_err_25prc);
% users expertise
[idx_usr_lvl, C_ratio] = kmeans((user_err_75prc - user_err_25prc)', 3); 
[~, sort_idx] = sort(C_ratio);
sort_lvl = ["advanced", "intermediate", "novice"];
usr_lvl = string.empty;
for i = idx_usr_lvl'
    usr_lvl(end+1) = sort_lvl(sort_idx == i);
end

anova_analysis(data_sorted, usr_lvl);

% disp_temporal_norm_multp_cond(cycles_class_regr, usr_lvl');
% 3 examples
[~,idx_max] = max(user_err_75prc - user_err_25prc);
idx_med = ceil(length(user_err_med)/2);
[~,idx_min] = min(user_err_75prc - user_err_25prc);
disp_temporal_norm_multp_cond(cycles_class_regr([idx_max, idx_med, idx_min]), usr_lvl([idx_max, idx_med, idx_min])');
% clean impedance cell array from non cyclic identifications
% impedance{1}.arx_id = impedance{1}.arx_id(:,1); % TODO: check for anomaly
impedance_cln = delete_noncyclic_id(cycles_class, impedance);
% regroup data by expertise for stiffness against frequency plot
impedance_regr_usr = regroup_impedance(impedance_cln, USER_REF_NAMES);
% impedance_regr_lvl = regroup_impedance(impedance_regr_usr, usr_lvl');
% res = disp_impedance_against_frequency(cycles_class_regr, impedance_regr_usr, usr_lvl');
disp_impedance_against_frequency2(data_sorted, usr_lvl, class_order, expertise_order);
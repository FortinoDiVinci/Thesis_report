clear all
close all

%addpath('../ROMAN_2021/fct_maria')
%addpath('../data_2021_April')
%addpath('../utils/')
%addpath('../../../youBot_analysis/Utils')
%addpath('../../../force_torque_sensor')
%addpath('../../../../Experience/data/utils')
addpath('utils/')
addpath('../../data/utils/')


%% second ROMAN submission
load('SB2021_new_data_impedance_v2.mat')

nb_exp = length(z_b); 

DISP = 1;
USER_REF_NAMES = [];
NB_PHASES = [];
dt = 1e-3;

% 50 Hz filter
[b,a] = butter(2,50/(1/(2*dt)),'low'); 
% cycle_shift = -350; % cycles are shifted by xxx samples so that the impact 
% is no longer the starting point. 

for exp_nb = 1:nb_exp
    
    USER_REF_NAMES(exp_nb) = string(exp_parameters(exp_nb).user);
    if strcmp(exp_parameters(exp_nb).experience, "exp_1") || ...
        strcmp(exp_parameters(exp_nb).experience, "l_bb")
        NB_PHASES(exp_nb) = 3;
    elseif strcmp(exp_parameters(exp_nb).experience, "exp_2")
        NB_PHASES(exp_nb) = 3;
    else % calibration or physical interaction learning phase
        NB_PHASES(exp_nb) = 0;
        continue
    end
    
    th = exp_parameters(exp_nb).target_height;
    
    dz = delta_z{exp_nb};
    dfz = delta_fz{exp_nb};
    idx_p = [dz.pert_ind];
    i_imp = detectBallImpacts(dz.time, z_b{exp_nb}, DISP); 
    
    if DISP
        plot(dz.time, z_p{exp_nb}, 'Color', [0.4940,0.1840,0.5560], 'Linewidth', 2)
        plot([dz.time(1), dz.time(end)], [th, th], 'k--', 'Linewidth', 1.5)
    end
    
    % if the experiment start with a ball on the paddle, the previous 
    % function will detect the initial static instants as impacts
    if isempty(idx_ball_off_ramp(exp_nb))
        idx_ball_off_ramp(exp_nb) = offRampIdx(z_b{exp_nb}, dt);
    end
    i_imp = i_imp(i_imp >= idx_ball_off_ramp(exp_nb));
    first_impact = 4; % no perturbation before the 5th impact
    
    [i_red,idx_i_red] = findRelevantData(dz.time, dz.complete_traject, i_imp, first_impact); 
    if DISP
        plot(dz.time(idx_p), z_p{exp_nb}(idx_p), 'rp', 'MarkerFaceColor', 'r',... 
            'Markersize',15);
        plot(dz.time(idx_i_red), z_b{exp_nb}(idx_i_red), 'o', 'color', ...
            [0.9290,0.6940,0.1250], 'Markersize', 15, 'Linewidth', 1.5)
    %legend('ball','impacts','paddle', 'target', 'perturb.')
    end
      
    cycles{exp_nb} = cycleExtraction(idx_i_red, dt, dfz, dz, z_b{exp_nb}, ...
        z_p{exp_nb}, exp_parameters(exp_nb), DISP);
    
    traject(exp_nb).time = dz.time;
    traject(exp_nb).position = dz.complete_traject;
    traject(exp_nb).velocity = Iu_diffcent(traject(exp_nb).position, traject(exp_nb).time);
    traject(exp_nb).force = dfz.complete_traject;
    traject(exp_nb).yank = Iu_diffcent(traject(exp_nb).force, traject(exp_nb).time);

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


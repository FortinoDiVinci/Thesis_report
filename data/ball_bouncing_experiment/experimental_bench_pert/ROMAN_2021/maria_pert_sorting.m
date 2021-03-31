clear all
close all

addpath('../')
addpath('fct_maria')
addpath('../utils/')
addpath('../../../youBot_analysis/Utils')
% EXP 1 is with the 7 impedance experiences
% EXP 2 is with the 14 experiences without perturbations nor force feedback
EXP = 2;
% 
if EXP == 1
    load('../imp_test_data/imp_test_08.mat')
    load('../preliminary_experimental_data/data_vfo_3_phases.mat', 'z_p', 'z_b')
    shift = 0;
elseif EXP == 2
    load('../trajectory_prediction_evaluation/force_trajectory_optimization/test_sine_opt_6.mat')
    load('../data_2020_Nov_17/data_without_impacts_2020_11_17.mat', 'z_p', 'z_b')
    shift = 1;
end
dt = 1e-3;
nb_exp = length(z_b); 


for exp_nb = 1:nb_exp
    
    if EXP == 1
        dfz = data(1).delta_fz{exp_nb,1};
        dz = data(1).delta_z{1,exp_nb};
    elseif EXP == 2
        if exp_nb == nb_exp % first exp was calibr° and was not considered
            break;
        end
        dfz = delta_fz{exp_nb+shift,9};
        dz = DIFF_TRAJECT(200, 200, z{exp_nb+shift}, dfz.time, [], ...
            [], 12, '12 ms delay position');
    end
    
    if EXP == 1
        i_imp = impacts_extraction(dz.time, z_b{exp_nb+shift}, z_p{exp_nb+shift}, -0.07);
    elseif EXP == 2
        i_imp = impacts_extraction2(dz.time, z_b{exp_nb+shift}, 0.5);
    end
        
    first_impact = 2; 
    fct_time_ratio = 0.98;
    
    [i_red,ind_i_red]  = boundaries_suppr(dz.time, i_imp, first_impact, fct_time_ratio);
    
    display = 1;
    cycles{exp_nb} = cycle_extraction(ind_i_red, dt, dfz, dz, z_b{exp_nb+shift}, z_p{exp_nb+shift}, display);
    
    traject(exp_nb).time = dz.time;
    traject(exp_nb).position = dz.complete_traject;
    traject(exp_nb).velocity = Iu_diffcent(dz.complete_traject, dz.time);
    traject(exp_nb).force = dfz.complete_traject;
    traject(exp_nb).yank = Iu_diffcent(dfz.complete_traject, dz.time);

end

close all

cycles_norm2 = cycle_normalisation(cycles,dt);
[center_ratio, count_ratio, idx_ratio, cycles_class] = dist_sorting(cycles_norm2);

disp_phase_diagram(cycles_class, traject);
%disp_temporal_norm(cycles_class);

save('cyclic_data_21_03_31.mat', 'cycles_class')

return

%%
exp_nb = 1;
figure
subplot(2,1,1)
hold on
plot(traject(exp_nb).time, traject(exp_nb).position)
idx1 = [cycles_class{exp_nb}([[cycles_class{exp_nb}.type_dist] == 1]).glob_ind];
idx2 = [cycles_class{exp_nb}([[cycles_class{exp_nb}.type_dist] == 2]).glob_ind];
idx3 = [cycles_class{exp_nb}([[cycles_class{exp_nb}.type_dist] == 3]).glob_ind];
plot(traject(exp_nb).time(idx1), traject(exp_nb).position(idx1), 'ro')
plot(traject(exp_nb).time(idx2), traject(exp_nb).position(idx2), 'bo')
plot(traject(exp_nb).time(idx3), traject(exp_nb).position(idx3), 'go')
subplot(2,1,2)
hold on
plot(traject(exp_nb).time, traject(exp_nb).force)
plot(traject(exp_nb).time(idx1), traject(exp_nb).force(idx1), 'ro')
plot(traject(exp_nb).time(idx2), traject(exp_nb).force(idx2), 'bo')
plot(traject(exp_nb).time(idx3), traject(exp_nb).force(idx3), 'go')
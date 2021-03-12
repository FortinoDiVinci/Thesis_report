%clear 
%close all

% addpath('../')
% addpath('fct_maria')
% addpath('../utils/')
% addpath('../../../youBot_analysis/Utils')
% 
% load('../imp_test_data/imp_test_08.mat')
% load('../data_2020_Nov_17/data_without_impacts_2020_11_17.mat', 'z_p', 'z_b')
load('../preliminary_experimental_data/data_vfo_3_phases.mat', 'z_p', 'z_b')
dt = 1e-3;
nb_exp = length(z_b); 

for exp_nb = 1:nb_exp
    
    dfz = data(1).delta_fz{exp_nb,1};
    dz = data(1).delta_z{1,exp_nb};
    
    i_imp = impacts_extraction(dz.time, z_b{exp_nb}, z_p{exp_nb}, -0.07);
    
    first_impact = 2; 
    fct_time_ratio = 0.98;
    
    [i_red,ind_i_red]  = boundaries_suppr(dz.time, i_imp, first_impact, fct_time_ratio);
    
    display = 1;
    cycles{exp_nb} = cycle_extraction(ind_i_red, dt, dfz, dz, z_b{exp_nb}, z_p{exp_nb}, display);
    
    traject(exp_nb).time = dz.time;
    traject(exp_nb).position = dz.complete_traject;
    traject(exp_nb).force = dfz.complete_traject;

end

close all

cycles_norm2 = cycle_normalisation(cycles,dt);
[center_ratio, count_ratio, idx_ratio, cycles_class] = dist_sorting(cycles_norm2);

disp_phase_diagram(cycles_class, traject);

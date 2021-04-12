clear all
close all

addpath('../ROMAN_2021/fct_maria')
addpath('../utils/')
addpath('../../../youBot_analysis/Utils')
addpath('../../../force_torque_sensor')
% 

load('exp_2.mat')

nb_exp = length(z_b); 

% 50 Hz filter
[b,a] = butter(2,50/(1/(2*dt)),'low'); 
% cycle_shift = -350; % cycles are shifted by xxx samples so that the impact 
% is no longer the starting point. 

for exp_nb = 1:nb_exp
    
    f_tmp = forces_filtering(forces_unf{exp_nb}', torques_unf{exp_nb}', ...
        thetas{exp_nb}', t{exp_nb});
    z{exp_nb} = filtfilt(b,a,mocap_marker_robot_base{exp_nb}(:,3));
    fz{exp_nb} = -filtfilt(b,a,f_tmp(3,:))';
    
    
    dfz = DIFF_TRAJECT(200, 100, fz{exp_nb}, t{exp_nb}, [], ...
        [], 0, 'force');
    dz = DIFF_TRAJECT(200, 200, z{exp_nb}, t{exp_nb}, [], ...
        [], 12, '12 ms delayed position');
    
    i_imp = impacts_extraction2(dz.time, z_b{exp_nb}, 0.6);
        
    first_impact = 4; % no perturbation before the 5th impact
    
    [i_red,idx_i_red]  = find_relevant_data(t{exp_nb}, z{exp_nb}, i_imp, first_impact);
    plot(t{exp_nb}(idx_i_red), z_b{exp_nb}(idx_i_red), 'o')
    
    display = 1;
    cycles{exp_nb} = cycle_extraction(idx_i_red, dt, dfz, dz, z_b{exp_nb}, z_p{exp_nb}, display);
    
    traject(exp_nb).time = t{exp_nb};
    traject(exp_nb).position = z{exp_nb};
    traject(exp_nb).velocity = Iu_diffcent(z{exp_nb}, t{exp_nb});
    traject(exp_nb).force = fz{exp_nb};
    traject(exp_nb).yank = Iu_diffcent(fz{exp_nb}, t{exp_nb});

end

cycles_norm2 = cycle_normalisation(cycles,dt);
cycles_ghost = get_ghost_impacts(cycles_norm2, t_ghost_impulse);
% The ghost impulse were not indicated in one of the experiments, they are
% integrated manualy after observing the cycles, ambiguous cycle will be
% considered in neither in ghost impulse nor in normal force feeback
ghost_impact_list = [15,24,25,43,64,65,70,88,122,126,129,134,139,172,182,...
    202,204,208,222,239,253,261,262,273];
ambiguous_impact_list = [106,152,153,157,159,162,191,219,207,272];
for i = 1:length(cycles_ghost{1})
    cycles_ghost{1}(i).is_ghost_impact = double(any(i == ghost_impact_list));
    if any(i == ambiguous_impact_list)
        cycles_ghost{1}(i).is_ghost_impact = -1;
    end
end

% disp ghosted VS non ghosted
% 1: 10% ghost., 2: 90% ghost., 3: 1 with cocontrac., 4: 2 with cocontrac.
exp_class = [1,1,2,2,1,4,3];
disp_ghosted_norm(cycles_ghost, exp_class);

%[center_ratio, count_ratio, idx_ratio, cycles_class] = dist_sorting(cycles_norm2);

%disp_phase_diagram(cycles_class, traject);
disp_temporal_norm(cycles_class);

%save('cyclic_data_21_03_31.mat', 'cycles_class')

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
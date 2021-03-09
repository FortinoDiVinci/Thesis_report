clear all

addpath('../')
addpath('../utils/')
addpath('../../../youBot_analysis/Utils')

load('imp_test_05.mat')

% original online phase sorting (intended phase spread)
for exp_nb = 1:length(data(1).delta_z)
    idx_25{:,exp_nb} = (round(10.*data(1).delta_z{exp_nb}.pert_val) == 101 | ...
        round(10.*data(1).delta_z{exp_nb}.pert_val) == -99);
    idx_50{:,exp_nb} = (round(10.*data(1).delta_z{exp_nb}.pert_val) == 102 | ...
        round(10.*data(1).delta_z{exp_nb}.pert_val) == -98);
    idx_75{:,exp_nb} = (round(10.*data(1).delta_z{exp_nb}.pert_val) == 103 | ...
        round(10.*data(1).delta_z{exp_nb}.pert_val) == -97);
end

%% classic phase diagram

figure('DefaultAxesFontSize',14)
for ii = 1:4
    subplot(2,4,ii)
    hold on
    time = data(1).delta_z{ii}.time;
    z = data(1).delta_z{ii}.complete_traject;
    dz = Iu_diffcent(z, time);
    idx_pert = data(1).delta_z{ii}.pert_ind;
    plot(z, dz);
    plot(z(idx_pert(idx_25{:,ii})), dz(idx_pert(idx_25{:,ii})), 'o', ...
        'Color', [0.8500, 0.3250, 0.0980], 'MarkerFaceColor', [0.8500, 0.3250, 0.0980]);
    plot(z(idx_pert(idx_50{:,ii})), dz(idx_pert(idx_50{:,ii})), 'o', ...
        'Color', [0.9290, 0.6940, 0.1250], 'MarkerFaceColor', [0.9290, 0.6940, 0.1250]);
    plot(z(idx_pert(idx_75{:,ii})), dz(idx_pert(idx_75{:,ii})), 'o', ...
        'Color', [0.4940, 0.1840, 0.5560], 'MarkerFaceColor', [0.4940, 0.1840, 0.5560]);
end
for ii = 5:7
    subplot(2,3,ii-1)
    hold on
    time = data(1).delta_z{ii}.time;
    z = data(1).delta_z{ii}.complete_traject;
    dz = Iu_diffcent(z, time);
    idx_pert = data(1).delta_z{ii}.pert_ind;
    plot(z, dz);
    plot(z(idx_pert(idx_25{:,ii})), dz(idx_pert(idx_25{:,ii})), 'o', ...
        'Color', [0.8500, 0.3250, 0.0980], 'MarkerFaceColor', [0.8500, 0.3250, 0.0980]);
    plot(z(idx_pert(idx_50{:,ii})), dz(idx_pert(idx_50{:,ii})), 'o', ...
        'Color', [0.9290, 0.6940, 0.1250], 'MarkerFaceColor', [0.9290, 0.6940, 0.1250]);
    plot(z(idx_pert(idx_75{:,ii})), dz(idx_pert(idx_75{:,ii})), 'o', ...
        'Color', [0.4940, 0.1840, 0.5560], 'MarkerFaceColor', [0.4940, 0.1840, 0.5560]);
end

%% force phase diagram

figure('DefaultAxesFontSize',14)
for ii = 1:4
    subplot(2,4,ii)
    hold on
    time = data(1).delta_fz{ii}.time;
    fz = data(1).delta_fz{ii}.complete_traject;
    dfz = Iu_diffcent(fz, time);
    idx_pert = data(1).delta_fz{ii}.pert_ind;
    plot(fz, dfz);
    plot(fz(idx_pert(idx_25{:,ii})), dfz(idx_pert(idx_25{:,ii})), 'o', ...
        'Color', [0.8500, 0.3250, 0.0980], 'MarkerFaceColor', [0.8500, 0.3250, 0.0980]);
    plot(fz(idx_pert(idx_50{:,ii})), dfz(idx_pert(idx_50{:,ii})), 'o', ...
        'Color', [0.9290, 0.6940, 0.1250], 'MarkerFaceColor', [0.9290, 0.6940, 0.1250]);
    plot(fz(idx_pert(idx_75{:,ii})), dfz(idx_pert(idx_75{:,ii})), 'o', ...
        'Color', [0.4940, 0.1840, 0.5560], 'MarkerFaceColor', [0.4940, 0.1840, 0.5560]);
end
for ii = 5:7
    subplot(2,3,ii-1)
    hold on
    time = data(1).delta_fz{ii}.time;
    fz = data(1).delta_fz{ii}.complete_traject;
    dfz = Iu_diffcent(fz, time);
    idx_pert = data(1).delta_fz{ii}.pert_ind;
    plot(fz, dfz);
    plot(fz(idx_pert(idx_25{:,ii})), dfz(idx_pert(idx_25{:,ii})), 'o', ...
        'Color', [0.8500, 0.3250, 0.0980], 'MarkerFaceColor', [0.8500, 0.3250, 0.0980]);
    plot(fz(idx_pert(idx_50{:,ii})), dfz(idx_pert(idx_50{:,ii})), 'o', ...
        'Color', [0.9290, 0.6940, 0.1250], 'MarkerFaceColor', [0.9290, 0.6940, 0.1250]);
    plot(fz(idx_pert(idx_75{:,ii})), dfz(idx_pert(idx_75{:,ii})), 'o', ...
        'Color', [0.4940, 0.1840, 0.5560], 'MarkerFaceColor', [0.4940, 0.1840, 0.5560]);
end

%%

figure('DefaultAxesFontSize',14)
subplot(1,2,1)
hold on
for ii = 1:7
    time = data(1).delta_z{ii}.time;
    pz = data(1).delta_z{ii}.complete_traject;
    dpz = Iu_diffcent(pz, time);
    idx_pert = data(1).delta_z{ii}.pert_ind;
    plot(pz, dpz, 'Color', [0, 0.4470, 0.7410,0.4]);   
end
for ii = 1:7
    time = data(1).delta_z{ii}.time;
    z = data(1).delta_z{ii}.complete_traject;
    dz = Iu_diffcent(z, time);
    idx_pert = data(1).delta_z{ii}.pert_ind;
    plot(z(idx_pert(idx_25{:,ii})), dz(idx_pert(idx_25{:,ii})), 'o', ...
        'Color', [0.8500, 0.3250, 0.0980], 'MarkerFaceColor', [0.8500, 0.3250, 0.0980]);
    plot(z(idx_pert(idx_50{:,ii})), dz(idx_pert(idx_50{:,ii})), 'o', ...
        'Color', [0.9290, 0.6940, 0.1250], 'MarkerFaceColor', [0.9290, 0.6940, 0.1250]);
    plot(z(idx_pert(idx_75{:,ii})), dz(idx_pert(idx_75{:,ii})), 'o', ...
        'Color', [0.4940, 0.1840, 0.5560], 'MarkerFaceColor', [0.4940, 0.1840, 0.5560]);
end
title('Position')
subplot(1,2,2)
hold on
for ii = 1:7
    time = data(1).delta_fz{ii}.time;
    fz = data(1).delta_fz{ii}.complete_traject;
    dfz = Iu_diffcent(fz, time);
    idx_pert = data(1).delta_fz{ii}.pert_ind;
    plot(fz, dfz, 'Color', [0, 0.4470, 0.7410,0.4]);   
end
for ii = 1:7
    time = data(1).delta_fz{ii}.time;
    fz = data(1).delta_fz{ii}.complete_traject;
    dfz = Iu_diffcent(fz, time);
    idx_pert = data(1).delta_fz{ii}.pert_ind;
    plot(fz(idx_pert(idx_25{:,ii})), dfz(idx_pert(idx_25{:,ii})), 'o', ...
        'Color', [0.8500, 0.3250, 0.0980], 'MarkerFaceColor', [0.8500, 0.3250, 0.0980]);
    plot(fz(idx_pert(idx_50{:,ii})), dfz(idx_pert(idx_50{:,ii})), 'o', ...
        'Color', [0.9290, 0.6940, 0.1250], 'MarkerFaceColor', [0.9290, 0.6940, 0.1250]);
    plot(fz(idx_pert(idx_75{:,ii})), dfz(idx_pert(idx_75{:,ii})), 'o', ...
        'Color', [0.4940, 0.1840, 0.5560], 'MarkerFaceColor', [0.4940, 0.1840, 0.5560]);
end
title('Force')
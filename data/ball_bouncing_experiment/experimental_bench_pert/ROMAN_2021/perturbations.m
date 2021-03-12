clear all

addpath('../')
addpath('../imp_test_data')
addpath('../utils/')
addpath('../../../youBot_analysis/Utils')

load('imp_test_07.mat')

method_i = [1,11];

figure('DefaultAxesFontSize',14)

for i = 1:2
    subplot(2,1,i)
    hold on

    fz = [data(1).delta_fz{:,method_i(i)}];
    pert_fz = [fz.diff_traject];
    pos_pert = vertcat(fz.pert_val) > 0;
    neg_pert = vertcat(fz.pert_val) < 0;
    
    plot(pert_fz(:,pos_pert), 'g')
    plot(pert_fz(:,neg_pert), 'r')
end

return

%% unperturbed trajectories

load('../trajectory_prediction_evaluation/force_trajectory_optimization/test_sine_opt_5.mat')

method_i = [8,11];

figure('DefaultAxesFontSize',14)
for i = 1:2
    subplot(2,1,i)
    hold on

    fz = [delta_fz{:,method_i(i)}];
    pert_fz = [fz.diff_traject];
    avg_err = mean(pert_fz,2);
    std_err = std(pert_fz,1,2);
    
    plot(pert_fz, 'Color', [0.3010, 0.7450, 0.9330, 0.04])
    plot((1:200), avg_err, '--', 'Color', [0.3010, 0.7450, 0.9330], 'Linewidth', 1.8)
    plotStdSurface(avg_err, std_err, (1:200), [0.3010, 0.7450, 0.9330], 1)
end

figure('DefaultAxesFontSize',14)
subplot(2,1,1)
hold on
for i = 1:length(method_i)
    fz = [delta_fz{:,method_i(i)}];
    pert_fz = [fz.diff_traject];
    std_err = std(pert_fz,1,2); 
    plot(std_err);
end
subplot(2,1,2)
hold on
for i = 1:length(method_i)
    fz = [delta_fz{:,method_i(i)}];
    pert_fz = [fz.diff_traject];
    avg_err = mean(pert_fz,2);
    plot(avg_err);
end
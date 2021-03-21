clear all

addpath('../')
addpath('../imp_test_data')
addpath('../utils/')
addpath('../../../youBot_analysis/Utils')

load('imp_test_08.mat')
load('delta_z_no_delay_spline_200ms.mat')

% method comparison
method_i = [8,11];
figure('DefaultAxesFontSize',14)
for i = 1:2
    subplot(2,1,i)
    hold on

    fz = [data(1).delta_fz{:,method_i(i)}];
    pert_fz = [fz.diff_traject];
    pos_pert = vertcat(fz.pert_val) > 0;
    neg_pert = vertcat(fz.pert_val) < 0;
    
    plot(pert_fz(:,pos_pert), 'Color', [0.8500, 0.3250, 0.0980, 0.3])
    plot(pert_fz(:,neg_pert), 'Color', [0.4660, 0.6740, 0.1880, 0.3])
end

% selected method position VS force
sel_met = 8;
figure('DefaultAxesFontSize',14)
subplot(2,1,1)
hold on
fz = [data(1).delta_fz{:,sel_met}];
pert_fz = [fz.diff_traject];
pos_pert = vertcat(fz.pert_val) > 0;
neg_pert = vertcat(fz.pert_val) < 0;
plot(pert_fz(:,pos_pert), 'Color', [0.8500, 0.3250, 0.0980, 0.3])
plot(pert_fz(:,neg_pert), 'Color', [0.4660, 0.6740, 0.1880, 0.3])
subplot(2,1,2)
hold on
z = [delta_z{:}];
pert_fz = [z.diff_traject];
pos_pert = vertcat(z.pert_val) > 0;
neg_pert = vertcat(z.pert_val) < 0;
plot(pert_fz(:,pos_pert), 'Color', [0.8500, 0.3250, 0.0980, 0.3])
plot(pert_fz(:,neg_pert), 'Color', [0.4660, 0.6740, 0.1880, 0.3])

% avg avd std
% selected method position VS force
sel_met = 8;

fz = [data(1).delta_fz{:,sel_met}];
pert_fz = [fz.diff_traject];
pos_pert_idx = vertcat(fz.pert_val) > 0;
neg_pert_idx  = vertcat(fz.pert_val) < 0;
fz_pos_pert_avg = mean(pert_fz(:,pos_pert_idx),2);
fz_pos_pert_std = std(pert_fz(:,pos_pert_idx),1,2);
fz_neg_pert_avg = mean(pert_fz(:,neg_pert_idx),2);
fz_neg_pert_std = std(pert_fz(:,neg_pert_idx),1,2);

z = [delta_z{:}];
pert_z = [z.diff_traject];
pos_pert_idx = vertcat(z.pert_val) > 0;
neg_pert_idx  = vertcat(z.pert_val) < 0;
z_pos_pert_avg = mean(pert_z(:,pos_pert_idx),2);
z_pos_pert_std = std(pert_z(:,pos_pert_idx),1,2);
z_neg_pert_avg = mean(pert_z(:,neg_pert_idx),2);
z_neg_pert_std = std(pert_z(:,neg_pert_idx),1,2);

figure('DefaultAxesFontSize',14)
subplot(2,1,1)
hold on
plotStdSurface(z_pos_pert_avg, z_pos_pert_std, (1:200), [0.8500, 0.3250, 0.0980], 1)
plotStdSurface(z_neg_pert_avg, z_neg_pert_std, (1:200), [0.4660, 0.6740, 0.1880], 1)
plot(z_neg_pert_avg, '--', 'Color', [0.4660, 0.6740, 0.1880], 'Linewidth', 1.5)
plot(z_pos_pert_avg, '--', 'Color', [0.8500, 0.3250, 0.0980], 'Linewidth', 1.5)
xlabel(' Time (ms)')
ylabel('Position (m)')
subplot(2,1,2)
hold on
plotStdSurface(fz_pos_pert_avg, fz_pos_pert_std, (1:200), [0.8500, 0.3250, 0.0980], 1)
plotStdSurface(fz_neg_pert_avg, fz_neg_pert_std, (1:200), [0.4660, 0.6740, 0.1880], 1)
plot(fz_pos_pert_avg, '--', 'Color', [0.8500, 0.3250, 0.0980], 'Linewidth', 1.5)
plot(fz_neg_pert_avg, '--', 'Color', [0.4660, 0.6740, 0.1880], 'Linewidth', 1.5)
xlabel(' Time (ms)')
ylabel('Force (N)')

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
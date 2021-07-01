clear all

addpath('../')
addpath('../imp_test_data')
addpath('../utils/')
addpath('../../../youBot_analysis/Utils')

load('imp_test_08.mat')
load('delta_z_no_delay_spline_200ms.mat')

% method comparison
method_i = [8,11];
%figure('DefaultAxesFontSize',14)
for i = 1:2
    %subplot(2,1,i)
    %hold on

    fz = [data(1).delta_fz{:,method_i(i)}];
    pert_fz = [fz.diff_traject];
    pos_pert = vertcat(fz.pert_val) > 0;
    neg_pert = vertcat(fz.pert_val) < 0;
    
    %plot(pert_fz(:,pos_pert), 'Color', [0.8500, 0.3250, 0.0980, 0.3])
    %plot(pert_fz(:,neg_pert), 'Color', [0.4660, 0.6740, 0.1880, 0.3])
end

% selected method position VS force
sel_met = 8;
%figure('DefaultAxesFontSize',14)
%subplot(2,1,1)
%hold on
fz = [data(1).delta_fz{:,sel_met}];
pert_fz = [fz.diff_traject];
pos_pert = vertcat(fz.pert_val) > 0;
neg_pert = vertcat(fz.pert_val) < 0;
%plot(pert_fz(:,pos_pert), 'Color', [0.8500, 0.3250, 0.0980, 0.3])
%plot(pert_fz(:,neg_pert), 'Color', [0.4660, 0.6740, 0.1880, 0.3])
%subplot(2,1,2)
%hold on
z = [delta_z{:}];
pert_fz = [z.diff_traject];
pos_pert = vertcat(z.pert_val) > 0;
neg_pert = vertcat(z.pert_val) < 0;
%plot(pert_fz(:,pos_pert), 'Color', [0.8500, 0.3250, 0.0980, 0.3])
%plot(pert_fz(:,neg_pert), 'Color', [0.4660, 0.6740, 0.1880, 0.3])

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
subplot(1,2,1)
hold on
plotStdSurface(z_pos_pert_avg, z_pos_pert_std, (1:200), [0.8500, 0.3250, 0.0980], 1)
plotStdSurface(z_neg_pert_avg, z_neg_pert_std, (1:200), [0.4660, 0.6740, 0.1880], 1)
plot(z_neg_pert_avg, '--', 'Color', [0.4660, 0.6740, 0.1880], 'Linewidth', 1.5)
plot(z_pos_pert_avg, '--', 'Color', [0.8500, 0.3250, 0.0980], 'Linewidth', 1.5)
xlabel(' Time (ms)')
ylabel('Position (m)')
subplot(1,2,2)
hold on
plotStdSurface(fz_pos_pert_avg, fz_pos_pert_std, (1:200), [0.8500, 0.3250, 0.0980], 1)
plotStdSurface(fz_neg_pert_avg, fz_neg_pert_std, (1:200), [0.4660, 0.6740, 0.1880], 1)
plot(fz_pos_pert_avg, '--', 'Color', [0.8500, 0.3250, 0.0980], 'Linewidth', 1.5)
plot(fz_neg_pert_avg, '--', 'Color', [0.4660, 0.6740, 0.1880], 'Linewidth', 1.5)
xlabel(' Time (ms)')
ylabel('Force (N)')

figure('DefaultAxesFontSize',26)
tiledlayout(1,2,'TileSpacing','Compact', 'Padding','Compact');
nexttile
hold on
plotStdSurface(z_pos_pert_avg.*100, z_pos_pert_std.*100, (1:200), [0.8500, 0.3250, 0.0980], 1)
plotStdSurface(z_neg_pert_avg.*100, z_neg_pert_std.*100, (1:200), [0.4660, 0.6740, 0.1880], 1)
plot(z_neg_pert_avg.*100, '--', 'Color', [0.4660, 0.6740, 0.1880], 'Linewidth', 1.5)
plot(z_pos_pert_avg.*100, '--', 'Color', [0.8500, 0.3250, 0.0980], 'Linewidth', 1.5)
xlabel(' Time (ms)')
ylabel('Position (cm)')
nexttile
hold on
plotStdSurface(fz_pos_pert_avg, fz_pos_pert_std, (1:200), [0.8500, 0.3250, 0.0980], 1)
plotStdSurface(fz_neg_pert_avg, fz_neg_pert_std, (1:200), [0.4660, 0.6740, 0.1880], 1)
plot(fz_pos_pert_avg, '--', 'Color', [0.8500, 0.3250, 0.0980], 'Linewidth', 1.5)
plot(fz_neg_pert_avg, '--', 'Color', [0.4660, 0.6740, 0.1880], 'Linewidth', 1.5)
xlabel(' Time (ms)')
ylabel('Force (N)')

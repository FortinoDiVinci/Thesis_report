clear all

addpath('../')
addpath('../utils/')
addpath('../../../youBot_analysis/Utils')

%% force traject

load('../trajectory_prediction_evaluation/force_trajectory_optimization/test_sine_opt_5.mat')

fz_data = [delta_fz{2:15,8}]; % (8) 3sineOptM
tmp = [fz_data(:).traject];
r_traj = tmp(3:end-2,:);
epsilon = [fz_data(:).diff_traject];
mean_traj = mean(r_traj);

SE = sum(epsilon);  % sum errors
[SE_clc, out_idx] = rmoutliers(SE,'ThresholdFactor', 3);

itv = (1:200);
r2 = 1 - sum(epsilon(itv,:).^2)./sum((r_traj(itv,:) - mean_traj).^2);
r2_clc = 1 - sum(epsilon(itv,~out_idx).^2)./sum((r_traj(itv,~out_idx) - mean_traj(~out_idx)).^2);

n = size(r_traj,1);
p = 3*3 + 0;
r2adj = 1 - (1 - r2)*(n-1)/(n-p-1);

% 
% mean(r2adj)
% 
% std(r2adj)

mean(r2)
mean(rmoutliers(r2, 'ThresholdFactor', 5))
mean(rmoutliers(r2adj, 'ThresholdFactor', 5))
mean(r2_clc)
std(r2)
std(rmoutliers(r2, 'ThresholdFactor', 5))
std(rmoutliers(r2adj, 'ThresholdFactor', 5))
std(r2_clc)

figure
hold on
plot(r2)
plot(r2adj)

figure('DefaultAxesFontSize',13)
hold on
histogram(epsilon, 'Normalization', 'probability', 'BinWidth', 0.01, 'EdgeColor', 'none')
histogram(epsilon(1:100,:), 'Normalization', 'probability', 'BinWidth', 0.01, 'EdgeColor', 'none')
xlabel('Error (N)')

eps_masked = epsilon(1:100,:);
[h,p] = ttest(epsilon(:)) % rmoutliers(epsilon(:), 'ThresholdFactor', 5)
[h,p] = ttest(eps_masked(:)) % rmoutliers(eps_masked(:), 'ThresholdFactor', 5)

[h, p] = adtest(rmoutliers(epsilon(:), 'ThresholdFactor',5), 'Alpha', 0.0005)
[h, p] = adtest(eps_masked(:))

% test = [];
% for ii = 1:size(epsilon, 2)
%     test = [test, adtest(epsilon(:,ii), 'Alpha', 0.0005)];
% end

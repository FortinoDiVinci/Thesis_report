clear all

load('../imp_test_data/imp_test_09.mat')

n = 200;
p = 3; % K B M ?

for i = 1:length(data)
    tmp_imp = [data(i).impedance{:}];
    mean_nrmse(i) = mean([tmp_imp(:).nrmse_pos]);
    std_nrmse(i) = std([tmp_imp(:).nrmse_pos]);
    [clc_data, bool_out] = rmoutliers([tmp_imp(:).nrmse_pos]);
    mean_nrmse_clc(i) = mean(clc_data);
    std_nrmse_clc(i) = std(clc_data);
    count_outlier(i) = sum(bool_out);
    
    phi = cat(3,tmp_imp.phi);
    r2(:,i) = 1 - sum(([tmp_imp.rec_pos_err]).^2)./...
    sum((squeeze(phi(:,1,:)) - mean(squeeze(phi(:,1,:)))).^2);
    r2_adj(:,i) = 1 - (1 - r2(:,i))*(n-1)/(n-p-1);    
    %xi_all(:,:,i) = [tmp_imp.xi];
end

figure
hold on
%errorbar((5:15), mean_nrmse, std_nrmse)
errorbar((5:15), mean_nrmse_clc, std_nrmse_clc)


figure
plot((5:15), count_outlier)

% best data for delay = 11ms
imp_data_best = [data(11-5+1).impedance{:}];
xi = [imp_data_best(:).xi];
mean(rmoutliers(xi(1,:)))
std(rmoutliers(xi(1,:)))
mean(rmoutliers(xi(2,:)))
std(rmoutliers(xi(2,:)))
mean(rmoutliers(xi(3,:)))
std(rmoutliers(xi(3,:)))

for i = 1:size(r2_adj, 2)
    r2_adj_clc{i} = rmoutliers(r2_adj(:,i), 'ThresholdFactor', 5);
end

figure
errorbar((5:15), cellfun(@(x) mean(x), r2_adj_clc), cellfun(@(x) std(x), r2_adj_clc))

% this method deletes too many "outliers"...
anova1(rmoutliers(r2_adj, 'ThresholdFactor', 5), {'5','6','7','8','9','10','11','12','13','14','15'})
ax1 = gca; 

% without outliers
figure
s1 = subplot(2,2,1);
% errorbar((5:15), mean(rmoutliers(r2_adj)),  std(rmoutliers(r2_adj)))
fig1 = get(ax1,'children');
copyobj(fig1,s1);
xticks(([1,6,11]))
xticklabels({'5','10','15'})
xlabel('Delay (ms)')
ylabel('R_{adj}^2 score')
subplot(2,2,2)
errorbar((5:15), mean(rmoutliers(squeeze(xi_all(1,:,:)))),  std(rmoutliers(squeeze(xi_all(1,:,:)))))
xlabel('Delay (ms)')
ylabel('Stiffness (N.m^{-1})')
subplot(2,2,3)
errorbar((5:15), mean(rmoutliers(squeeze(xi_all(2,:,:)))),  std(rmoutliers(squeeze(xi_all(2,:,:)))))
xlabel('Delay (ms)')
ylabel('Dampin (N.s.m^{-1})')
subplot(2,2,4)
errorbar((5:15), mean(rmoutliers(squeeze(xi_all(3,:,:)))),  std(rmoutliers(squeeze(xi_all(3,:,:)))))
xlabel('Delay (ms)')
ylabel('Mass (kg)')

% with outliers
figure
subplot(2,2,1)
errorbar((5:15), mean(r2_adj),  std(r2_adj))
xlabel('Delay (ms)')
ylabel('R^2_{adj} score')
subplot(2,2,2)
errorbar((5:15), mean(squeeze(xi_all(1,:,:))),  std(squeeze(xi_all(1,:,:))))
xlabel('Delay (ms)')
ylabel('Stiffness (N.m^{-1})')
subplot(2,2,3)
errorbar((5:15), mean(squeeze(xi_all(2,:,:))),  std(squeeze(xi_all(2,:,:))))
xlabel('Delay (ms)')
ylabel('Dampin (N.s.m^{-1})')
subplot(2,2,4)
errorbar((5:15), mean(squeeze(xi_all(3,:,:))),  std(squeeze(xi_all(3,:,:))))
xlabel('Delay (ms)')
ylabel('Mass (kg)')

%% anova test for delay

anova1(rmoutliers(r2_adj, 'ThresholdFactor', 5), {'5','6','7','8','9','10','11','12','13','14','15'})
xlabel('Delay (ms)')
ylabel('R^2_{adj} score')

anova1(rmoutliers(r2_adj(:,6:8), 'ThresholdFactor', 5), {'10','11','12'})
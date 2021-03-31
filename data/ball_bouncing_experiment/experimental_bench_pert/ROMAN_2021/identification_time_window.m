clear all

load('../imp_test_data/imp_test_12.mat')
load('cyclic_data.mat')

addpath('../utils')
addpath('../../../utils')

% perturbation classification
for i = 1:length(cycles_class)
    tmp = [cycles_class{i}.type_dist];
    tmp_val = [cycles_class{i}.dist_val];
    perturbation_class{i} = tmp([cycles_class{i}.type_dist] ~= 0);
    perturbation_dir{i} = sign(tmp_val([cycles_class{i}.type_dist] ~= 0));
end

% unclassified data
glob_idx_suppr = [];
glob_len = 0;
for i = 1:length(cycles_class)
    tmp = cycles_class{i};
    pert_class_idx{i} = [cycles_class{i}.glob_ind];
    pert_imp_idx{i} = [data.delta_fz{i}.pert_ind];
    idx_suppr{i} = [];
    for j = 1:length(pert_imp_idx{i})
        t = find(pert_imp_idx{i}(j) == pert_class_idx{i});
        if isempty(t)
            idx_suppr{i} = [idx_suppr{i}, j];
            glob_idx_suppr(j + glob_len) = 1;
        else
            glob_idx_suppr(j + glob_len) = 0;
        end
    end
    glob_len = glob_len + length(pert_imp_idx{i});
end

% 2 methods for removing outliers are compared here
% 1) remove data with R^2<50%
% 2) removing R^2 idx outside 5 sMAD 
% 3) removing K idx outside 5 sMAD 
% 4) Combination of 2) and 3)
% 5) Combination of 1) and 3)
% 6) Combination of 1) and removing M idx outside 5 sMAD

p = 3;
for i = 1:1:size(data.impedance,2)
    imp_data = [data.impedance{:,i}];
    phi = cat(3,imp_data.phi);
    n = size(phi,1);
    r2(i,:) = 1 - squeeze(sum(([imp_data.rec_pos_err]).^2))./...
            squeeze(sum((mean(phi(:,1,:)) - phi(:,1,:)).^2))';
    r2_adj(i,:) = 1 - (1 - r2(i,:))*(n-1)/(n-p-1);
    
    xi = [imp_data.xi];
    
    % method 1
    idx_50p(i,:) = r2_adj(i, ~glob_idx_suppr) > 0.5; % keep only data with R2 > 50%
    [~, idx_clc{i}] = rmoutliers(xi(1, :), 'ThresholdFactor', 5);
    
    r2_adj_clc{i} = r2_adj(i, ~idx_clc{i}); %
    
    r2_adj_mean(i) = mean(r2_adj_clc{i});
    r2_adj_std(i) = std(r2_adj_clc{i});
    r2_suppr(i) = sum(idx_clc{i}); % total of outliers removed method 1)
    
    % method 2
    [r2_adj_clc2{i}, idx_clc2(i,:)] = rmoutliers(r2_adj(i, :), 'ThresholdFactor', 5); % no specific sorting
    
    r2_adj_mean2(i) = mean(r2_adj_clc2{i});
    r2_adj_std2(i) = std(r2_adj_clc2{i});
    r2_suppr2(i) = sum(idx_clc2(i,:)); % total of outliers removed method 2)
    
    K(i,:) = xi(1,:);
    B(i,:) = xi(2,:);
    M(i,:) = xi(3,:);
    
    K_mean(i) = mean(K(i,idx_clc{i}));
    K_std(i) = std(K(i,idx_clc{i}));
    B_mean(i) = mean(B(i,idx_clc{i}));
    B_std(i) = std(B(i,idx_clc{i}));
    M_mean(i) = mean(M(i,idx_clc{i}));
    M_std(i) = std(M(i,idx_clc{i}));
    
    K_mean2(i) = mean(K(i,~idx_clc2(i,:)));
    K_std2(i) = std(K(i,~idx_clc2(i,:)));
    B_mean2(i) = mean(B(i,~idx_clc2(i,:)));
    B_std2(i) = std(B(i,~idx_clc2(i,:)));
    M_mean2(i) = mean(M(i,~idx_clc2(i,:)));
    M_std2(i) = std(M(i,~idx_clc2(i,:)));
    
    % method 3 (outlier only with stiffness)
    [K_clc, K_outl_idx] = rmoutliers(K(i,:), 'ThresholdFactor', 5);
    K_mean3(i) = mean(K_clc);
    K_std3(i) = std(K_clc);
    K_suppr3(i) = sum(K_outl_idx);
    
    r2_adj_mean3(i) = mean(r2_adj(i, ~K_outl_idx));
    r2_adj_std3(i) = std(r2_adj(i, ~K_outl_idx));
    
    % method 4 or combination of method 2 and 3
    idx_4 = (K_outl_idx) | (idx_clc2(i,:) | glob_idx_suppr ); % combining both sorting
    K_mean4(i) = mean(K(i, ~idx_4));
    K_std4(i) = std(K(i, ~idx_4));
    B_mean4(i) = mean(B(i, ~idx_4));
    B_std4(i) = std(B(i, ~idx_4));
    M_mean4(i) = mean(M(i, ~idx_4));
    M_std4(i) = std(M(i, ~idx_4));
    
    r2_adj_mean4(i) = mean(r2_adj(i, ~idx_4));
    r2_adj_std4(i) = std(r2_adj(i, ~idx_4));
    r2_suppr4(i) = sum(idx_4);
    
    % method 5 R2 > 50% or stiffness outlier
    idx_5 = K_outl_idx | glob_idx_suppr | (r2_adj(i, :) < 0.5);
    r2_adj_mean5(i) = mean(r2_adj(i, ~idx_5));
    r2_adj_std5(i) = std(r2_adj(i, ~idx_5));
    r2_suppr5(i) = sum(idx_5);
    
    K_mean5(i) = mean(K(i, ~idx_5));
    K_std5(i) = std(K(i, ~idx_5));
    B_mean5(i) = mean(B(i, ~idx_5));
    B_std5(i) = std(B(i, ~idx_5));
    M_mean5(i) = mean(M(i, ~idx_5));
    M_std5(i) = std(M(i, ~idx_5));
    
    % method 6 R2 > 50% or mass outlier
    [M_clc, M_outl_idx] = rmoutliers(M(i,:), 'ThresholdFactor', 5);
    idx_6 = M_outl_idx | glob_idx_suppr | (r2_adj(i, :) < 0.5);
    r2_adj_mean6(i) = mean(r2_adj(i, ~idx_6));
    r2_adj_std6(i) = std(r2_adj(i, ~idx_6));
    r2_suppr6(i) = sum(idx_6);
    
    K_mean6(i) = mean(K(i, ~idx_6));
    K_std6(i) = std(K(i, ~idx_6));
    B_mean6(i) = mean(B(i, ~idx_6));
    B_std6(i) = std(B(i, ~idx_6));
    M_mean6(i) = mean(M(i, ~idx_6));
    M_std6(i) = std(M(i, ~idx_6));
end

%r2 = r2(~cellfun('isempty',r2));

% r2_mean = cellfun(@(x) mean(rmoutliers(x, 'ThresholdFactor', 5)), r2);
% r2_std = cellfun(@(x) std(rmoutliers(x, 'ThresholdFactor', 5)), r2);
% r2_suppr = cellfun(@(x) sum(out2(@() rmoutliers(x, 'ThresholdFactor', 5))), r2);

figure
subplot(3,1,1)
hold on
%errorbar((50:1:200), r2_adj_mean, r2_adj_std);
errorbar((50:1:200), r2_adj_mean2, r2_adj_std2);
%errorbar((50:1:200), r2_adj_mean3, r2_adj_std3);
errorbar((50:1:200), r2_adj_mean4, r2_adj_std4);
errorbar((50:1:200), r2_adj_mean5, r2_adj_std5);
errorbar((50:1:200), r2_adj_mean6, r2_adj_std6);
legend('M2', 'M4', 'M5', 'M6')
ylabel('R^2 score')
xlabel('Identification time window (ms)')
subplot(3,2,3)
hold on
errorbar((50:1:200), K_mean2, K_std2)
%errorbar((50:1:200), K_mean3, K_std3)
errorbar((50:1:200), K_mean4, K_std4)
errorbar((50:1:200), K_mean5, K_std5)
errorbar((50:1:200), K_mean6, K_std6)
xlabel('Identification time window (ms)')
ylabel('Stiffness (Nm^{-1})')
subplot(3,2,4)
hold on
errorbar((50:1:200), B_mean2, B_std2)
%errorbar((50:1:200), K_mean3, K_std3)
errorbar((50:1:200), B_mean4, B_std4)
errorbar((50:1:200), B_mean5, B_std5)
errorbar((50:1:200), B_mean6, B_std6)
xlabel('Identification time window (ms)')
ylabel('Damping (Nsm^{-1})')
subplot(3,2,5)
hold on
errorbar((50:1:200), M_mean2, M_std2)
%errorbar((50:1:200), K_mean3, K_std3)
errorbar((50:1:200), M_mean4, M_std4)
errorbar((50:1:200), M_mean5, M_std5)
errorbar((50:1:200), M_mean6, M_std6)
xlabel('Identification time window (ms)')
ylabel('Damping (Nsm^{-1})')
subplot(3,2,6)
hold on
plot((50:1:200), r2_suppr2)
%plot((50:1:200), K_suppr3)
plot((50:1:200), r2_suppr4)
plot((50:1:200), r2_suppr5)
plot((50:1:200), r2_suppr6)
xlabel('Identification time window (ms)')
ylabel('Nb of outliers')

%%
figure
subplot(2,2,1)
hold on
%errorbar((50:1:200), r2_adj_mean2, r2_adj_std2);
plotStdSurface(r2_adj_mean2', r2_adj_std2', (50:1:200), [0, 0.4470, 0.7410], 1)
plot((50:1:200), r2_adj_mean2, '--', 'Linewidth', 1.5, 'Color', [0, 0.4470, 0.7410])
plot([50, 200], r2_adj_mean2(end)/100*105+[0,0], '--', 'Color', [0.8500, 0.3250, 0.0980])
plot([50, 200], r2_adj_mean2(end)/100*95+[0,0], '--', 'Color', [0.8500, 0.3250, 0.0980])
xlabel('Nb samples')
ylabel('R^2_{adj} score')
subplot(2,2,2)
hold on
%errorbar((50:1:200), K_mean4, K_std4)
plotStdSurface(K_mean4', K_std4', (50:1:200), [0, 0.4470, 0.7410], 1)
plot((50:1:200), K_mean4, '--', 'Linewidth', 1.5, 'Color', [0, 0.4470, 0.7410])
plot([50, 200], K_mean4(end)/100*105+[0,0], '--', 'Color', [0.8500, 0.3250, 0.0980])
plot([50, 200], K_mean4(end)/100*95+[0,0], '--', 'Color', [0.8500, 0.3250, 0.0980])
xlabel('Nb samples')
ylabel('Stiffness (Nm^{-1})')
subplot(2,2,3)
hold on
%errorbar((50:1:200), B_mean4, B_std4)
plotStdSurface(B_mean4', B_std4', (50:1:200), [0, 0.4470, 0.7410], 1)
plot((50:1:200), B_mean4, '--', 'Linewidth', 1.5, 'Color', [0, 0.4470, 0.7410])
plot([50, 200], B_mean4(end)/100*105+[0,0], '--', 'Color', [0.8500, 0.3250, 0.0980])
plot([50, 200], B_mean4(end)/100*95+[0,0], '--', 'Color', [0.8500, 0.3250, 0.0980])
xlabel('Nb samples')
ylabel('Damping (Nsm^{-1})')
subplot(2,2,4)
hold on
%errorbar((50:1:200), M_mean4, M_std4)
plotStdSurface(M_mean4', M_std4', (50:1:200), [0, 0.4470, 0.7410], 1)
plot((50:1:200), M_mean4, '--', 'Linewidth', 1.5, 'Color', [0, 0.4470, 0.7410])
plot([50, 200], M_mean4(end)/100*105+[0,0], '--', 'Color', [0.8500, 0.3250, 0.0980])
plot([50, 200], M_mean4(end)/100*95+[0,0], '--', 'Color', [0.8500, 0.3250, 0.0980])
xlabel('Nb samples')
ylabel('Mass (kg)')

%%
anova1(r2_adj)
ax1 = gca; 

% without outliers
figure
s1 = subplot(2,2,1);
% errorbar((5:15), mean(rmoutliers(r2_adj)),  std(rmoutliers(r2_adj)))
fig1 = get(ax1,'children');
copyobj(fig1,s1);
xticks(([51,101,151,201]))
xticklabels({'50','100','150','200'})
xlabel('Delay (ms)')
ylabel('R_{adj}^2 score')
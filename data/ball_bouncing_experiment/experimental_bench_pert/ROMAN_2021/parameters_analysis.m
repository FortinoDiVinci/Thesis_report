clear all

load('cyclic_data.mat')
addpath('../utils/')
addpath('../../../utils')

time_wdw = 200;

if time_wdw == 200
    load('../imp_test_data/imp_test_09.mat')
elseif time_wdw == 150
    load('../imp_test_data/imp_test_12.mat')
end
    
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
    if time_wdw == 200
        pert_imp_idx{i} = [data(8).delta_fz{i}.pert_ind];
    elseif time_wdw == 150
        pert_imp_idx{i} = [data.delta_fz{i}.pert_ind];
    end

    idx_suppr{i} = [];
    for j = 1:length(pert_imp_idx{i})
        t = find(pert_imp_idx{i}(j) == pert_class_idx{i});
        if isempty(t)
            idx_suppr{i} = [idx_suppr{i}, j];
            %glob_idx_suppr = [glob_idx_suppr, j + glob_len];
            glob_idx_suppr(j + glob_len) = 1;
        else
            glob_idx_suppr(j + glob_len) = 0;
        end
    end
    glob_len = glob_len + length(pert_imp_idx{i});
end

for i = 1:length(data)
    if time_wdw == 200
        selec_imp_data = [data(i).impedance{:}]; % 12(ms) - 5(ms) + 1
    elseif time_wdw == 150
        selec_imp_data = [data.impedance{:,101}];
    end
    
    n = selec_imp_data(:).id_size; % sum([selec_imp_data(:).nb_id]);
    p = 3;

    xi(:,:,i) = [selec_imp_data(:).xi];
    phi = cat(3, selec_imp_data.phi);
    epsilon = [selec_imp_data(:).rec_pos_err];
    r2_tot(:,i) = 1 - sum(epsilon.^2) ./ ...
        sum((mean(squeeze(phi(:,1,:))) - squeeze(phi(:,1,:))).^2);
    r2(:,i) = 1 - sum(epsilon(~glob_idx_suppr).^2) ./ ...
        sum((mean(squeeze(phi(:,1,~glob_idx_suppr))) - squeeze(phi(:,1,~glob_idx_suppr))).^2);
    
    r2_adj_tot(:,i) = 1 - (1 - r2_tot(:,i)).*(n-1)/(n-p-1);
    r2_adj(:,i) = 1 - (1 - r2(:,i)).*(n-1)/(n-p-1);
end

avg_r2_adj_tot = mean(rmoutliers(r2_adj_tot, 'ThresholdFactor', 5));
avg_r2_adj = mean(rmoutliers(r2_adj, 'ThresholdFactor', 5));

if time_wdw == 200
    K = squeeze(xi(1,~glob_idx_suppr,:));
    B = squeeze(xi(2,~glob_idx_suppr,:));
    M = squeeze(xi(3,~glob_idx_suppr,:));
elseif time_wdw == 150
    K = squeeze(xi(1,~glob_idx_suppr,:))';
    B = squeeze(xi(2,~glob_idx_suppr,:))';
    M = squeeze(xi(3,~glob_idx_suppr,:))';
end

%%
display1 = 0;
if display1 == 1
    anova1(rmoutliers(r2_adj_tot, 'ThresholdFactor', 5), {'5','6','7','8','9','10','11','12','13','14','15'})
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
    errorbar((5:15), mean(rmoutliers(K)),  std(rmoutliers(K)))
    xlabel('Delay (ms)')
    ylabel('Stiffness (N.m^{-1})')
    subplot(2,2,3)
    errorbar((5:15), mean(rmoutliers(B)),  std(rmoutliers(B)))
    xlabel('Delay (ms)')
    ylabel('Dampin (N.s.m^{-1})')
    subplot(2,2,4)
    errorbar((5:15), mean(rmoutliers(M)),  std(rmoutliers(M)))
    xlabel('Delay (ms)')
    ylabel('Mass (kg)')
end

%% Sorting by type of perturbation

for i = 1:3
    idx_pert_type_pos = ([perturbation_class{:}] == i) & ([perturbation_dir{:}] == 1);
    idx_pert_type_neg = ([perturbation_class{:}] == i) & ([perturbation_dir{:}] == -1);
    data_sorted(i).pos.count = sum(idx_pert_type_pos);
    data_sorted(i).neg.count = sum(idx_pert_type_neg);
    data_sorted(i).pos.r2 = r2(idx_pert_type_pos,:);
    data_sorted(i).neg.r2 = r2(idx_pert_type_neg,:);
    data_sorted(i).pos.K = K(idx_pert_type_pos,:);
    data_sorted(i).neg.K = K(idx_pert_type_neg,:);
    data_sorted(i).pos.B = B(idx_pert_type_pos,:);
    data_sorted(i).neg.B = B(idx_pert_type_neg,:);
    data_sorted(i).pos.M = M(idx_pert_type_pos,:);
    data_sorted(i).neg.M = M(idx_pert_type_neg,:);
end

Kp = [];
Kn = [];
Bp = [];
Bn = [];
Mp = [];
Mn = [];
%
Kp2 = [];
Kn2 = [];
Bp2 = [];
Bn2 = [];
Mp2 = [];
Mn2 = [];

if time_wdw == 200
    dly = 8; % delay
elseif time_wdw == 150
    dly = 1; % delay
end  

for i = 1:3
    idx1 = 2*i-1;
    idx2 = 2*i;
    
    idx_neg_r2{idx1} = data_sorted(i).pos.r2(:,dly) < 0.5;
    idx_neg_r2{idx2} = data_sorted(i).neg.r2(:,dly) < 0.5;
    idx_k_out{idx1} = out2(@() rmoutliers(data_sorted(i).pos.K(:,dly), 'ThresholdFactor', 5));
    idx_k_out{idx2} = out2(@() rmoutliers(data_sorted(i).neg.K(:,dly), 'ThresholdFactor', 5));
    idx_m_out{idx1} = out2(@() rmoutliers(data_sorted(i).pos.M(:,dly), 'ThresholdFactor', 5));
    idx_m_out{idx2} = out2(@() rmoutliers(data_sorted(i).neg.M(:,dly), 'ThresholdFactor', 5));

    % K and R^2 method
    % total outliers
    idx_tot{idx1} = idx_neg_r2{idx1} | idx_k_out{idx1};
    idx_tot{idx2} = idx_neg_r2{idx2} | idx_k_out{idx2};
    % common outliers
    idx_inter{idx1} = idx_neg_r2{idx1} & idx_k_out{idx1};
    idx_inter{idx2} = idx_neg_r2{idx2} & idx_k_out{idx2};
    % outliers solely in K
    idx_k_out_o{idx1} = xor(idx_k_out{idx1}, idx_inter{idx1});
    idx_k_out_o{idx2} = xor(idx_k_out{idx2}, idx_inter{idx2});
    % outliers solely in R^2
    idx_neg_r2_o{idx1} = xor(idx_neg_r2{idx1}, idx_inter{idx1});
    idx_neg_r2_o{idx2} = xor(idx_neg_r2{idx2}, idx_inter{idx2});
    % non outliers count
    counts(idx1) = data_sorted(i).pos.count - sum(idx_tot{idx1});
    counts(idx2) = data_sorted(i).neg.count - sum(idx_tot{idx2});
    % data
    %R^2
    mean_r2(idx1) = mean(data_sorted(i).pos.r2(~idx_tot{idx1},dly));
    mean_r2(idx2) = mean(data_sorted(i).neg.r2(~idx_tot{idx2},dly));
    std_r2(idx1) = std(data_sorted(i).pos.r2(~idx_tot{idx1},dly));
    std_r2(idx2) = std(data_sorted(i).neg.r2(~idx_tot{idx2},dly));
    % K
    mean_K(idx1) = mean(data_sorted(i).pos.K(~idx_tot{idx1},dly));
    mean_K(idx2) = mean(data_sorted(i).neg.K(~idx_tot{idx2},dly));
    std_K(idx1) = std(data_sorted(i).pos.K(~idx_tot{idx1},dly));
    std_K(idx2) = std(data_sorted(i).neg.K(~idx_tot{idx2},dly));
    % B
    mean_B(idx1) = mean(data_sorted(i).pos.B(~idx_tot{idx1},dly));
    mean_B(idx2) = mean(data_sorted(i).neg.B(~idx_tot{idx2},dly));
    std_B(idx1) = std(data_sorted(i).pos.B(~idx_tot{idx1},dly));
    std_B(idx2) = std(data_sorted(i).neg.B(~idx_tot{idx2},dly));
    % M
    mean_M(idx1) = mean(data_sorted(i).pos.M(~idx_tot{idx1},dly));
    mean_M(idx2) = mean(data_sorted(i).neg.M(~idx_tot{idx2},dly));
    std_M(idx1) = std(data_sorted(i).pos.M(~idx_tot{idx1},dly));
    std_M(idx2) = std(data_sorted(i).neg.M(~idx_tot{idx2},dly));
    % +
    Kp = [Kp; data_sorted(i).pos.K(~idx_tot{idx1},dly)];
    Kn = [Kn; data_sorted(i).neg.K(~idx_tot{idx2},dly)];
    Bp = [Bp; data_sorted(i).pos.B(~idx_tot{idx1},dly)];
    Bn = [Bn; data_sorted(i).neg.B(~idx_tot{idx2},dly)];
    Mp = [Mp; data_sorted(i).pos.M(~idx_tot{idx1},dly)];
    Mn = [Mn; data_sorted(i).neg.M(~idx_tot{idx2},dly)];
    
    % M and R^2 method
    % total outliers
    idx_tot2{idx1} = idx_neg_r2{idx1} | idx_m_out{idx1};
    idx_tot2{idx2} = idx_neg_r2{idx2} | idx_m_out{idx2};
    % common outliers
    idx_inter2{idx1} = idx_neg_r2{idx1} & idx_m_out{idx1};
    idx_inter2{idx2} = idx_neg_r2{idx2} & idx_m_out{idx2};
    % outliers solely in K
    idx_m_out_o2{idx1} = xor(idx_m_out{idx1}, idx_inter2{idx1});
    idx_m_out_o2{idx2} = xor(idx_m_out{idx2}, idx_inter2{idx2});
    % outliers solely in R^2
    idx_neg_r2_o2{idx1} = xor(idx_neg_r2{idx1}, idx_inter2{idx1});
    idx_neg_r2_o2{idx2} = xor(idx_neg_r2{idx2}, idx_inter2{idx2});
    % non outliers count
    counts2(idx1) = data_sorted(i).pos.count - sum(idx_tot2{idx1});
    counts2(idx2) = data_sorted(i).neg.count - sum(idx_tot2{idx2});
    % data
    %R^2
    mean_r22(idx1) = mean(data_sorted(i).pos.r2(~idx_tot2{idx1},dly));
    mean_r22(idx2) = mean(data_sorted(i).neg.r2(~idx_tot2{idx2},dly));
    std_r22(idx1) = std(data_sorted(i).pos.r2(~idx_tot2{idx1},dly));
    std_r22(idx2) = std(data_sorted(i).neg.r2(~idx_tot2{idx2},dly));
    % K
    mean_K2(idx1) = mean(data_sorted(i).pos.K(~idx_tot2{idx1},dly));
    mean_K2(idx2) = mean(data_sorted(i).neg.K(~idx_tot2{idx2},dly));
    std_K2(idx1) = std(data_sorted(i).pos.K(~idx_tot2{idx1},dly));
    std_K2(idx2) = std(data_sorted(i).neg.K(~idx_tot2{idx2},dly));
    % B
    mean_B2(idx1) = mean(data_sorted(i).pos.B(~idx_tot2{idx1},dly));
    mean_B2(idx2) = mean(data_sorted(i).neg.B(~idx_tot2{idx2},dly));
    std_B2(idx1) = std(data_sorted(i).pos.B(~idx_tot2{idx1},dly));
    std_B2(idx2) = std(data_sorted(i).neg.B(~idx_tot2{idx2},dly));
    % M
    mean_M2(idx1) = mean(data_sorted(i).pos.M(~idx_tot2{idx1},dly));
    mean_M2(idx2) = mean(data_sorted(i).neg.M(~idx_tot2{idx2},dly));
    std_M2(idx1) = std(data_sorted(i).pos.M(~idx_tot2{idx1},dly));
    std_M2(idx2) = std(data_sorted(i).neg.M(~idx_tot2{idx2},dly));
    % +
    Kp2 = [Kp2; data_sorted(i).pos.K(~idx_tot2{idx1},dly)];
    Kn2 = [Kn2; data_sorted(i).neg.K(~idx_tot2{idx2},dly)];
    Bp2 = [Bp2; data_sorted(i).pos.B(~idx_tot2{idx1},dly)];
    Bn2 = [Bn2; data_sorted(i).neg.B(~idx_tot2{idx2},dly)];
    Mp2 = [Mp2; data_sorted(i).pos.M(~idx_tot2{idx1},dly)];
    Mn2 = [Mn2; data_sorted(i).neg.M(~idx_tot2{idx2},dly)];
end

for i = 1:3
%     bar_count(1,2*i-1) = counts(2*i-1); 
%     bar_count(2,2*i-1) = sum(idx_neg_r2{2*i-1});
%     bar_count(3,2*i-1) = sum(id{2*i-1});
%     bar_count(1,2*i) = counts(2*i); 
%     bar_count(2,2*i) = sum(idx_neg_r2{2*i});
%     bar_count(3,2*i) = sum(id{2*i});
    for j = [-1,0]
        bar_count(1,2*i+j) = counts(2*i+j); 
        bar_count(2,2*i+j) = sum(idx_neg_r2_o{2*i+j});
        bar_count(3,2*i+j) = sum(idx_inter{2*i+j});
        bar_count(4,2*i+j) = sum(idx_k_out_o{2*i+j});
    end
    % method 2
    for j = [-1,0]
        bar_count2(1,2*i+j) = counts2(2*i+j); 
        bar_count2(2,2*i+j) = sum(idx_neg_r2_o2{2*i+j});
        bar_count2(3,2*i+j) = sum(idx_inter2{2*i+j});
        bar_count2(4,2*i+j) = sum(idx_m_out_o2{2*i+j});
    end    
end

figure
subplot(2,2,1)
title('Counts')
bar(bar_count','stacked')
set(gca, 'XTickLabel', {'c_1+', 'c_1-', 'c_2+', 'c_2-', 'c_3+', 'c_3-'})
ylabel('Nb identifications')
xlabel('Class')
subplot(2,2,2)
errorbar(mean_K, std_K)
set(gca, 'XTick', [1 3 5])
set(gca, 'XTickLabel', {'c_1+', 'c_2+', 'c_3+'})
ylabel('Stiffness (N m^{-1})')
xlabel('Class')
subplot(2,2,3)
errorbar(mean_B, std_B)
set(gca, 'XTick', [1 3 5])
set(gca, 'XTickLabel', {'c_1+', 'c_2+', 'c_3+'})
ylabel('Damping (Ns m^{-1})')
xlabel('Class')
subplot(2,2,4)
errorbar(mean_M, std_M)
set(gca, 'XTick', [1 3 5])
set(gca, 'XTickLabel', {'c_1+', 'c_2+', 'c_3+'})
ylabel('Mass (kg)')
xlabel('Class')


figure
subplot(2,2,1)
title('Counts')
bar(bar_count2','stacked')
set(gca, 'XTickLabel', {'c_1+', 'c_1-', 'c_2+', 'c_2-', 'c_3+', 'c_3-'})
ylabel('Nb identifications')
xlabel('Class')
subplot(2,2,2)
errorbar(mean_K2, std_K2)
set(gca, 'XTick', [1 3 5])
set(gca, 'XTickLabel', {'c_1+', 'c_2+', 'c_3+'})
ylabel('Stiffness (N m^{-1})')
xlabel('Class')
subplot(2,2,3)
errorbar(mean_B2, std_B2)
set(gca, 'XTick', [1 3 5])
set(gca, 'XTickLabel', {'c_1+', 'c_2+', 'c_3+'})
ylabel('Damping (Ns m^{-1})')
xlabel('Class')
subplot(2,2,4)
errorbar(mean_M2, std_M2)
set(gca, 'XTick', [1 3 5])
set(gca, 'XTickLabel', {'c_1+', 'c_2+', 'c_3+'})
ylabel('Mass (kg)')
xlabel('Class')

return

%% group anova
% R2
r2_a =[data_sorted(1).pos.r2(~idx_tot{1},dly); data_sorted(1).neg.r2(~idx_tot{2},dly); ...
    data_sorted(2).pos.r2(~idx_tot{3},dly); data_sorted(2).neg.r2(~idx_tot{4},dly); ...
    data_sorted(3).pos.r2(~idx_tot{5},dly); data_sorted(3).neg.r2(~idx_tot{6},dly)];

r2_g =[ones(size(data_sorted(1).pos.r2(~idx_tot{1},dly))); 
    2.*ones(size(data_sorted(1).neg.r2(~idx_tot{2},dly))); ...
    3.*ones(size(data_sorted(2).pos.r2(~idx_tot{3},dly))); ...
    4.*ones(size(data_sorted(2).neg.r2(~idx_tot{4},dly))); ...
    5.*ones(size(data_sorted(3).pos.r2(~idx_tot{5},dly))); ...
    6.*ones(size(data_sorted(3).neg.r2(~idx_tot{6},dly)))];

[p,t,stats] = anova1(r2_a, r2_g);
[c,m,h,nms] = multcompare(stats);

% K
K_a =[data_sorted(1).pos.K(~idx_tot{1},dly); data_sorted(1).neg.K(~idx_tot{2},dly); ...
    data_sorted(2).pos.K(~idx_tot{3},dly); data_sorted(2).neg.K(~idx_tot{4},dly); ...
    data_sorted(3).pos.K(~idx_tot{5},dly); data_sorted(3).neg.K(~idx_tot{6},dly)];

K_g =[ones(size(data_sorted(1).pos.K(~idx_tot{1},dly))); 
    2.*ones(size(data_sorted(1).neg.K(~idx_tot{2},dly))); ...
    3.*ones(size(data_sorted(2).pos.K(~idx_tot{3},dly))); ...
    4.*ones(size(data_sorted(2).neg.K(~idx_tot{4},dly))); ...
    5.*ones(size(data_sorted(3).pos.K(~idx_tot{5},dly))); ...
    6.*ones(size(data_sorted(3).neg.K(~idx_tot{6},dly)))];

[p,t,stats] = anova1(K_a, K_g);
[c,m,h,nms] = multcompare(stats);

% B
B_a =[data_sorted(1).pos.B(~idx_tot{1},dly); data_sorted(1).neg.B(~idx_tot{2},dly); ...
    data_sorted(2).pos.B(~idx_tot{3},dly); data_sorted(2).neg.B(~idx_tot{4},dly); ...
    data_sorted(3).pos.B(~idx_tot{5},dly); data_sorted(3).neg.B(~idx_tot{6},dly)];

B_g =[ones(size(data_sorted(1).pos.B(~idx_tot{1},dly))); 
    2.*ones(size(data_sorted(1).neg.B(~idx_tot{2},dly))); ...
    3.*ones(size(data_sorted(2).pos.B(~idx_tot{3},dly))); ...
    4.*ones(size(data_sorted(2).neg.B(~idx_tot{4},dly))); ...
    5.*ones(size(data_sorted(3).pos.B(~idx_tot{5},dly))); ...
    6.*ones(size(data_sorted(3).neg.B(~idx_tot{6},dly)))];

[p,t,stats] = anova1(B_a, B_g);
[c,m,h,nms] = multcompare(stats);

% M
M_a =[data_sorted(1).pos.M(~idx_tot{1},dly); data_sorted(1).neg.M(~idx_tot{2},dly); ...
    data_sorted(2).pos.M(~idx_tot{3},dly); data_sorted(2).neg.M(~idx_tot{4},dly); ...
    data_sorted(3).pos.M(~idx_tot{5},dly); data_sorted(3).neg.M(~idx_tot{6},dly)];

M_g =[ones(size(data_sorted(1).pos.M(~idx_tot{1},dly))); 
    2.*ones(size(data_sorted(1).neg.M(~idx_tot{2},dly))); ...
    3.*ones(size(data_sorted(2).pos.M(~idx_tot{3},dly))); ...
    4.*ones(size(data_sorted(2).neg.M(~idx_tot{4},dly))); ...
    5.*ones(size(data_sorted(3).pos.M(~idx_tot{5},dly))); ...
    6.*ones(size(data_sorted(3).neg.M(~idx_tot{6},dly)))];

[p,t,stats] = anova1(M_a, M_g);
[c,m,h,nms] = multcompare(stats);

% data200ms.r2_a = r2_a;
% data200ms.r2_g = r2_g;
% data200ms.K_a = K_a;
% data200ms.K_g = K_g;
% data200ms.B_a = B_a;
% data200ms.B_g = B_g;
% data200ms.M_a = M_a;
% data200ms.M_g = M_g;
% 
% save('data_anova_200ms.mat', 'data200ms')

% data100ms.r2_a = r2_a;
% data100ms.r2_g = r2_g;
% data100ms.K_a = K_a;
% data100ms.K_g = K_g;
% data100ms.B_a = B_a;
% data100ms.B_g = B_g;
% data100ms.M_a = M_a;
% data100ms.M_g = M_g;
% 
% save('data_anova_100ms.mat', 'data100ms')
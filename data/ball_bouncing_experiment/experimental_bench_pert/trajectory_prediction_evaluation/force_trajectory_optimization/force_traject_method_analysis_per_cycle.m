clear all

addpath('../../utils/')

load('test_sine_opt_3.mat')

for exp_nb = 2:size(delta_fz,1)
    idx_25{:,exp_nb} = (strcmp(phase{exp_nb}, "upper pk"));
    idx_50{:,exp_nb} = (strcmp(phase{exp_nb}, "decreas"));
    idx_75{:,exp_nb} = (strcmp(phase{exp_nb}, "lower pk"));
end

%
figure
for method_i = 1:size(delta_fz,2)
    tmp25_nrmse = [];
    tmp25_r2 = [];
    tmp50_nrmse = [];
    tmp50_r2 = [];
    tmp75_nrmse = [];
    tmp75_r2 = [];
    err25{method_i} = [];
    tra25{method_i} = [];
    err50{method_i} = [];
    tra50{method_i} = [];
    err75{method_i} = [];
    tra75{method_i} = [];
    subplot(3,4,method_i)
    hold on
    for exp_nb = 2:size(delta_fz,1)
        err25{method_i} = [err25{method_i}, ...
            delta_fz{exp_nb,method_i}.diff_traject(:,idx_25{:,exp_nb})];
        tra25{method_i} = [tra25{method_i}, ...
            delta_fz{exp_nb,method_i}.traject(:,idx_25{:,exp_nb})];
        err50{method_i} = [err50{method_i}, ...
            delta_fz{exp_nb,method_i}.diff_traject(:,idx_50{:,exp_nb})];
        tra50{method_i} = [tra50{method_i}, ...
            delta_fz{exp_nb,method_i}.traject(:,idx_50{:,exp_nb})];
        err75{method_i} = [err75{method_i}, ...
            delta_fz{exp_nb,method_i}.diff_traject(:,idx_75{:,exp_nb})];
        tra75{method_i} = [tra75{method_i}, ...
            delta_fz{exp_nb,method_i}.traject(:,idx_75{:,exp_nb})];
    end
    histogram(err25{method_i}(1:100,:), 'BinWidth', 0.02, 'Normalization', 'probability')
    histogram(err50{method_i}(1:100,:), 'BinWidth', 0.02, 'Normalization', 'probability')
    histogram(err75{method_i}(1:100,:), 'BinWidth', 0.02, 'Normalization', 'probability')
    xlim([-1,1])
    legend('up', 'dec', 'low')
    title(delta_fz{2,method_i}.header)
    for ii = 1:size(err25{method_i},2)
        tmp25_nrmse = [tmp25_nrmse, 1 - norm(err25{method_i}(:,ii))/norm(...
            tra25{method_i}(3:end-2,ii) - mean(tra25{method_i}(3:end-2,ii)))];
        tmp25_r2 = [tmp25_r2, 1 - sum(err25{method_i}(:,ii).^2)/sum(...
            (tra25{method_i}(3:end-2,ii) - mean(tra25{method_i}(3:end-2,ii))).^2)];
    end
    for ii = 1:size(err50{method_i},2)
        tmp50_nrmse = [tmp50_nrmse, 1 - norm(err50{method_i}(:,ii))/norm(...
            tra50{method_i}(3:end-2,ii) - mean(tra50{method_i}(3:end-2,ii)))];
        tmp50_r2 = [tmp50_r2, 1 - sum(err50{method_i}(:,ii).^2)/sum(...
            (tra50{method_i}(3:end-2,ii) - mean(tra50{method_i}(3:end-2,ii))).^2)];
    end
    for ii = 1:size(err75{method_i},2)
        tmp75_nrmse = [tmp75_nrmse, 1 - norm(err75{method_i}(:,ii))/norm(...
            tra75{method_i}(3:end-2,ii) - mean(tra75{method_i}(3:end-2,ii)))];
        tmp75_r2 = [tmp75_r2, 1 - sum(err75{method_i}(:,ii).^2)/sum(...
            (tra75{method_i}(3:end-2,ii) - mean(tra75{method_i}(3:end-2,ii))).^2)];
    end
%         end
    nrmse_data{1, method_i} = tmp25_nrmse;
    nrmse_data{2, method_i} = tmp50_nrmse;
    nrmse_data{3, method_i} = tmp75_nrmse;
    r2_data{1, method_i} = tmp25_r2;
    r2_data{2, method_i} = tmp25_r2;
    r2_data{3, method_i} = tmp25_r2;
end

% colors(1,:) = [0, 0.4470, 0.7410];
% colors(2,:) = [0.8500, 0.3250, 0.0980];
% colors(3,:) = [0.9290, 0.6940, 0.1250];
% colors(4,:) = [0.4940, 0.1840, 0.5560];
% colors(5,:) = [0.4660, 0.6740, 0.1880];
% colors(6,:) = [0.3010, 0.7450, 0.9330];
% colors(7,:) = [0.6350, 0.0780, 0.1840];
% colors(8,:) = [0, 0.25, 0.25];          	
% colors(9,:) = [0.1, 0.5, 0.1];   
% colors(10,:) = [1, 0.7137, 0.7569];
% colors(11,:) = [0.545,0.271,0.075];

% write data to table (excel)
for method_i = 1:size(delta_fz,2)
    for pert_i = 1:3
        avg_i = nanmean(rmoutliers(nrmse_data{pert_i, method_i}));
        std_i = nanstd(rmoutliers(nrmse_data{pert_i, method_i}));
        avg_delay_data(method_i, pert_i) = avg_i;
        std_delay_data(method_i, pert_i) = std_i;
    end
end

table_d = table(avg_delay_data);
table_d2 = table(std_delay_data);
writetable(table_d,'avg_val.xlsx')
writetable(table_d2,'std_val.xlsx')

% for method_i = 1:size(delta_fz,2)
%     table_er25_data = table(err25{method_i});
%     writetable(table_er25_data, '_err25.xlsx', 'sheet', delta_fz{2,method_i}.header, 'Range', 'B3')
% end
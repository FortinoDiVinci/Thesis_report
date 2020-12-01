function plotBestCandidatesErrors(errors)
% error is an array or matrix of STATISTIC_MDATA objects regrouping the 
% difference between the actual signal and the best candidate choosen 
% starting 100 indexes before prediction and ending 200 indexes after, 
% for a total of 300 samples per candidate
%
% if error is an (m*n*p) matrix, 
% m: number of inputs (signals)
% n: number of configurations
% p: number of methods for candidate generation
%
% if error is an m*n matrix,
% m: number of inputs (signals)
% n: number of configurations
%
% if error is an m matrix,
% m: number of inputs (signals)
%
% WARNING: The function has been completed for (m*n*p) matrix only...
dims = size(errors);

if length(dims) > 3 || length(dims) < 1
    error('Wrong input dimension');
end

data = {};
data_name = [];
d_mean = {};
d_std = {};
d_med = {};

% Reshaping data
if length(dims) > 1  
for j = 1:dims(2)

    if length(dims) > 2
    for k = 1:dims(3)
        if isempty(errors(1,j,k).data) % this config and method combination does not exists
            continue
        end
        data(end+1) = {[errors(:,j,k).data]};
        %{reshape([errors(:,j,k).data], length(errors(1,j,k).data(1,:)), [])};
        data_name = [data_name; convertCharsToStrings(errors(1,j,k).name)];
        d_mean(end+1) = {nanmean(data{end},1)};
        d_med(end+1) = {nanmedian(data{end},1)};
        d_std(end+1) = {nanstd(data{end},0,1)};
    end
    else
    data = cat(3, data, reshape([errors(:,j).data], length(errors(1,j).data(1,:)), [])); 
    data_name = [data_name; convertCharsToStrings(errors(1,j).name)];
    end
end 
else
data = reshape([errors(:).data], length(errors(1).data(1,:)), []);
data_name = convertCharsToStrings(errors(1).name);
end
    
% Mean error plots +/- std
nb_data_type = length(data_name);
figure('DefaultAxesFontSize',14)
for i = 1:nb_data_type
    subplot(3,3,i)
    hold on
    errorbar(d_mean{i}, d_std{i});
    legend("Estimation error");
    if ~contains(data_name(i), "Spline")
        errorbar(d_mean{i}(1:100), d_std{i}(1:100), 'Color', [0.8500, 0.3250, 0.0980]);
        legend("Estimation error","Error minimization");
    end
    title(data_name(i))
    %xlabel('Normalized errors', 'FontSize', 12);
    %xlabel('Time (s)', 'FontSize', 12);
    %ax = gca
    %ax.XAxis.FontSize = 12;
    %ax.YAxis.FontSize = 12;
end

% Histogram plots
figure('DefaultAxesFontSize',14)
for i = 1:nb_data_type
    subplot(3,3,i)
    if ~contains(data_name(i), "Spline")
        histogram(data{i}(:,100:end));
    else
        histogram(data{i}(:,:));
    end
    legend("Estimation error");
    title(data_name(i))
end

% All errors plots
figure('DefaultAxesFontSize',14)
for i = 1:nb_data_type
    subplot(3,3,i)
    hold on
    p1 = plot(data{i}(:,:)', 'Color', [0, 0.4470, 0.7410]);
    for j=1:length(p1)
        p1(j).Color(4) = 0.3;
    end
    legend("Estimation error");
    if ~contains(data_name(i), "Spline")
        p2 = plot(data{i}(:,1:100)', 'Color', [0.8500, 0.3250, 0.0980]);
        for j=1:length(p2)
            p2(j).Color(4) = 0.3;
        end
        legend([p1(1), p2(1)], {"Estimation errors","Errors minimization"});
    end
    
    title(data_name(i))
end
% get relevant data
% for config_idx = 1:size(error1,2)
%     data_burdet(:,:,config_idx) = reshape([error1(:,config_idx).data], ...
%         [300, length([error1(:,config_idx).data])/300]);
%     data_burdet_mean(:,config_idx) = mean(data_burdet(:,:,config_idx),2);
%     data_burdet_std(:,config_idx) = std(data_burdet(:,:,config_idx),0,2);
%     
%     data_minRMSE(:,:,config_idx) = reshape([error2(:,config_idx).data], ...
%         [300, length([error2(:,config_idx).data])/300]);
%     data_minRMSE_mean(:,config_idx) = mean(data_minRMSE(:,:,config_idx),2);
%     data_minRMSE_std(:,config_idx) = std(data_minRMSE(:,:,config_idx),0,2);    
% end
% 
% std_times = 1; % for plot
% 
% colors = [[0, 0.4470, 0.7410]; [0.8500, 0.3250, 0.0980]; [0.4660, 0.6740, 0.1880]; [0.6350, 0.0780, 0.1840] ]; 	
% 
% figure
% subplot(2,1,1)
% hold on
% for config_idx = 1:size(error1,2)
%     p1 = plot(data_burdet(:,:,config_idx), 'Color', colors(config_idx,:), 'LineWidth', 0.5);
%     for ip = 1:length(p1)
%         p1(ip).Color(4) = 0.3;
%     end
%     p(config_idx) = p1(1);
%     plot(data_burdet_mean(:,config_idx), 'Color', colors(config_idx,:), 'LineWidth', 1.1)
%     plot(100, data_burdet_mean(100,config_idx), 'pk', 'Markersize', 15)
%     plotStdSurface(data_burdet_mean(:,config_idx), data_burdet_std(:,config_idx),...
%         [], colors(config_idx,:));
% end
% legend(p, {'Config 1', 'Config 2', 'Config 3'});
% title('Burdet (2000) candidates best choice error')
% 
% subplot(2,1,2)
% hold on
% for config_idx = 1:size(error1,2)
%     p1 = plot(data_minRMSE(:,:,config_idx), 'Color', colors(config_idx,:), 'LineWidth', 0.5);
%     for ip = 1:length(p1)
%         p1(ip).Color(4) = 0.3;
%     end
%     p(config_idx) = p1(1);
%     plot(data_minRMSE_mean(:,config_idx), 'Color', colors(config_idx,:), 'LineWidth', 1.1)
%     plot(100, data_minRMSE_mean(100,config_idx), 'pk', 'Markersize', 15)
%     plotStdSurface(data_minRMSE_mean(:,config_idx), ...
%         data_minRMSE_std(:,config_idx),...
%         [], colors(config_idx,:));
% end
% legend(p, {'Config 1', 'Config 2', 'Config 3'});
% title('minRMSE candidates best choice error')

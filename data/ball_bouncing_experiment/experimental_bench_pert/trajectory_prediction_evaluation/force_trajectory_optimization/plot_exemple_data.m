clear all

addpath('../../utils')
%addpath('../../../../utils')

load('test_sine_opt_6.mat')

delta_fz(1,:) = [];

delta_all_err = cell(size(delta_fz,2),1);
delta_all_param = cell(size(delta_fz,2),1);
for meth_nb = 1:size(delta_fz,2)
    for exp_nb = 1:size(delta_fz,1)
        delta_all_err{meth_nb} = horzcat(delta_all_err{meth_nb}, [delta_fz{exp_nb,meth_nb}.diff_traject]);
        delta_all_param{meth_nb} = horzcat(delta_all_param{meth_nb}, [delta_fz{exp_nb,meth_nb}.opt_param]);
    end
    names(meth_nb) = delta_fz{1,meth_nb}.header;
end

% error histogram
figure('DefaultAxesFontSize',14)
for i = 2:2:length(delta_all_err)
    subplot(3,2,i/2)
    hold on
    histogram(delta_all_err{1}, 'BinWidth', 0.1)
    histogram(delta_all_err{i}, 'BinWidth', 0.1)
    histogram(delta_all_err{i+1}, 'BinWidth', 0.1)
    title("Virtual force error histogram")
    legend(names(1), names(i), names(i+1))
end
% error histogram avg and std
for i = 1:length(delta_all_err)
    avg_ = nanmean(delta_all_err{i}(:));
    std_ = nanstd(delta_all_err{i}(:));
    fprintf('Method %s with an average mean profile error of %.3f and an average std of %.3f\n', names(i), avg_, std_);
end

% temporal representation of the error (mean and std)
figure('DefaultAxesFontSize',14)
for i = 2:2:length(delta_all_err)
    subplot(3,2,i/2)
    hold on
    plotStdSurface(nanmean(delta_all_err{1},2), nanstd(delta_all_err{1},0,2), (0:199), [0, 0.4470, 0.7410])
    plotStdSurface(nanmean(delta_all_err{i},2), nanstd(delta_all_err{i},0,2), (0:199), [0.8500, 0.3250, 0.0980])
    plotStdSurface(nanmean(delta_all_err{i+1},2), nanstd(delta_all_err{i+1},0,2), (0:199), [0.9290, 0.6940, 0.1250])
    plot((0:199), nanmean(delta_all_err{1},2), 'Linewidth', 1.5, 'Color', [0, 0.4470, 0.7410]) 
    plot((0:199), nanmean(delta_all_err{i},2), 'Linewidth', 1.5, 'Color', [0.8500, 0.3250, 0.0980]) 
    plot((0:199), nanmean(delta_all_err{i+1},2), 'Linewidth', 1.5, 'Color', [0.9290, 0.6940, 0.1250]) 
    title("Virtual force error mean and std for " + string(size(delta_all_err{1},2)) + " estimations")
    legend(names(1), names(i), names(i+1))
end

%%
colors = lines(7);

figure
plot(delta_fz{2,8}.time, delta_fz{2,8}.complete_traject)
hold on
plot([delta_fz{2,8}.t_traject], [delta_fz{2,8}.virt_traject], 'Color', colors(2,:))

t_min = 147.1;%204.72;%
t_max = 148.5;%206;%

idx1 = find(delta_fz{2,8}.time > t_min, 1, 'first');
idx2 = find(delta_fz{2,8}.time < t_max, 1, 'last');

idx_virt = find(delta_fz{2,8}.pert_ind > idx1, 1, 'first');
idx_p_in_chunck = find(delta_fz{2,8}.time(idx1:idx2) >= delta_fz{2,8}.time(delta_fz{2,8}.pert_ind(idx_virt)), 1, 'first');

% figure
% hold on
% plot(delta_fz{2,8}.time(idx1:idx2), delta_fz{2,8}.complete_traject(idx1:idx2))
% plot(delta_fz{2,8}.t_traject(3:end-2,idx_virt), delta_fz{2,8}.virt_traject(3:end-2,idx_virt), 'Color', colors(2,:))

time = delta_fz{2,8}.time(idx1:idx2);
force = delta_fz{2,8}.complete_traject(idx1:idx2);
virtual = NaN(size(time));
virtual(idx_p_in_chunck:idx_p_in_chunck+199) = delta_fz{2,8}.virt_traject(3:end-2,idx_virt);

figure
hold on
plot(time, force, time, virtual)

% table_chunk = table(time, force, virtual);
% write(table_chunk,'sine_optimisation_trajectory_2.csv','Delimiter',',');
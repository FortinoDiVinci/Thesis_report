%
clear all

addpath('../')
addpath('../utils/')
addpath('../utils/cycles')
addpath('../../../youBot_analysis/Utils')
% 
% load('../trajectory_prediction_evaluation/force_trajectory_optimization/test_sine_opt_5.mat')
% load('../data_2020_Nov_17/data_without_impacts_2020_11_17.mat', 'mocap_marker_robot_base', 'z_b', 'z_p')

load('cyclic_data_21_03_31_.mat')

tot_cyc = [cycles_class{:}];

% mean_ui = mean(vertcat(tot_cyc(:).ui_interp));
% mean_dui = mean(vertcat(tot_cyc(:).dui_interp));
% figure
% hold on
% plot(vertcat(tot_cyc(:).ui_interp), vertcat(tot_cyc(:).dui_interp), 'b')
% plot(mean_ui, mean_dui, 'k', 'Linewidth', 2)

figure
plot(vertcat(tot_cyc(:).ui)-mean(vertcat(tot_cyc(:).ui)), ...
    vertcat(tot_cyc(:).dui)-mean(vertcat(tot_cyc(:).dui)), 'b')


mean_pts = round(mean([tot_cyc.npts])); % mean nb of points
t_com = linspace(0,1,mean_pts);

for i = 1:length(tot_cyc)
	ui_interp(i,:) = interp1(tot_cyc(i).t_red, tot_cyc(i).ui, ...
        linspace(tot_cyc(i).t_red(1), tot_cyc(i).t_red(end), mean_pts));
	dui_interp(i,:) = interp1(tot_cyc(i).t_red, tot_cyc(i).dui, ...
        linspace(tot_cyc(i).t_red(1), tot_cyc(i).t_red(end), mean_pts));
end

ui_interp_avg = mean(ui_interp);
dui_interp_avg = mean(dui_interp);
ui_interp_std = std(ui_interp);
dui_interp_std = std(dui_interp);

figure
subplot(2,1,1)
hold on
plot(repmat(t_com, size(ui_interp,1), 1)', ui_interp', 'b');
plotStdSurface(ui_interp_avg', ui_interp_std', t_com, [0, 0.4470, 0.7410], 1)
plot(t_com, ui_interp_avg, 'Linewidth', 2);
subplot(2,1,2)
hold on
plot(repmat(t_com, size(ui_interp,1), 1)', dui_interp', 'b');
plotStdSurface(dui_interp_avg', dui_interp_std', t_com, [0, 0.4470, 0.7410], 1)
plot(t_com, dui_interp_avg, 'Linewidth', 2);


idx_0 = find(dui_interp_avg <= 0, 1, 'first');
t_circle = circshift(linspace(0,2*pi,mean_pts),idx_0);

figure
hold on
plot(ui_interp', dui_interp', 'Color', [0.3010, 0.7450, 0.9330, 0.1])
plot(ui_interp_avg, dui_interp_avg, '--', 'Linewidth', 2, 'Color', [0, 0.4470, 0.7410])
plot(ui_interp_avg-ui_interp_std.*cos(t_circle), ...
    dui_interp_avg+dui_interp_std.*sin(t_circle), 'Linewidth', 1.5, 'Color', [0, 0.4470, 0.7410])
plot(ui_interp_avg+ui_interp_std.*cos(t_circle), ...
    dui_interp_avg-dui_interp_std.*sin(t_circle), 'Linewidth', 1.5, 'Color', [0, 0.4470, 0.7410])
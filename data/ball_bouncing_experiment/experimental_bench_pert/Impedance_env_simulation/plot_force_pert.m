

% whole signal
figure('DefaultAxesFontSize',16)
hold on
p1 = plot(t,fz, 'Linewidth', 2);
p2 = plot(t,fz0, '--', 'Linewidth', 2);
p4 = plot(delta_fz_sine.t_traject, delta_fz_sine.virt_traject, 'Color', ...
    [0.4660, 0.6740, 0.1880], 'Linewidth', 2);
xlim([143.4,145.7])
ylabel('Force (N)')
xlabel('Time (s)')

% perturbation temporal errors
figure
hold on
p1 = plot(delta_fz_sine.virt_traject-delta_fz_sim.virt_traject, 'Color', [0.4660, 0.6740, 0.1880, 0.4]);
%p2 = plot(delta_fz_spline.virt_traject-delta_fz_sim.virt_traject, 'Color', [0.3010, 0.7450, 0.9330, 0.4]);
%legend([p1(1),p2(1)], {'sine', 'spline'})
xlim([1,200])
ylabel('Force (N)')
xlabel('Time (ms)')


avg_diff = mean(delta_fz_sine.diff_traject,2);
std_diff = std(delta_fz_sine.diff_traject,0,2);

% perturbation shape
figure(10)
hold on
p1 = plot(delta_fz_sine.diff_traject, 'Color', [0.4660, 0.6740, 0.1880, 0.2]);
plotStdSurface(avg_diff, std_diff, (1:200), [0.4660, 0.6740, 0.1880], 1);
p2 = plot(delta_fz_sim.diff_traject, '--k', 'Linewidth', 2);
p3 = plot(avg_diff, '--', 'Linewidth', 2, 'Color', [0.4660, 0.6740, 0.1880]);
legend([p1(1), p2(1)], {'$\hat{\delta f}$','$\delta f$'},'interpreter','latex')
xlim([1,200])
ylabel('Force (N)')
xlabel('Time (ms)')

% perturbation error distribution
figure('DefaultAxesFontSize',16)
hold on
histogram(delta_fz_sine.virt_traject(1:100,:)-delta_fz_sim.virt_traject(1:100,:), ...
    'Normalization', 'probability','BinWidth',0.02);
xlabel('Force estimation error (N)')
%histogram(delta_fz_spline.virt_traject(1:100,:)-delta_fz_sim.virt_traject(1:100,:), ...
%    'Normalization', 'probability','BinWidth',0.02)
%title('Force virtual trajectories errors')
%legend('sineOpt', 'spline')


% à ajuster selon les besoins
fontsize0 = 26; %30
fontsize1 = 20;
paperSizeX = 35; %cm %40
paperSizeY = 28; %cm %30

axis_affich = [1,200,-Inf,Inf];

figure(10)
set(gca,'Fontsize',fontsize0)
%ylabel('Axis 3 (rad)');
axis(axis_affich)

set(gcf, 'PaperUnits', 'centimeters', 'PaperSize', [paperSizeX paperSizeY]);
myfiguresize = [0, 0, paperSizeX, paperSizeY];
set(gcf,'PaperPositionMode','manual', 'PaperPosition', myfiguresize);
print -dpdf -f10 -r300 test_figure % -f<n° figure> -r<résolution>
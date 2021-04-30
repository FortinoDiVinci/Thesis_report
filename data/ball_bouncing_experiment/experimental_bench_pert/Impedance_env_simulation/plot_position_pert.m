

% whole signal
figure('DefaultAxesFontSize',16)
hold on
p1 = plot(t,(z-mean(z)).*100, 'Linewidth', 2);
%p2 = plot(t,z0.*100, '--', 'Linewidth', 2);
p4 = plot(delta_z.t_traject, (delta_z.virt_traject-mean(z)).*100, 'Color', ...
    [0.4660, 0.6740, 0.1880], 'Linewidth', 2);
xlim([122.4,124.5])
ylabel('Position (cm)')
xlabel('Time (s)')

% perturbation temporal errors
figure
hold on
p1 = plot(delta_z.virt_traject-delta_z_sim.virt_traject, 'Color', [0.4660, 0.6740, 0.1880, 0.4]);
%p2 = plot(delta_fz_spline.virt_traject-delta_fz_sim.virt_traject, 'Color', [0.3010, 0.7450, 0.9330, 0.4]);
%legend([p1(1),p2(1)], {'sine', 'spline'})
xlim([1,200])
ylabel('Force (N)')
xlabel('Time (ms)')


avg_diff = mean(delta_z.diff_traject,2);
std_diff = std(delta_z.diff_traject,0,2);

% perturbation shape
figure('DefaultAxesFontSize',16)%(10)
hold on
p1 = plot(delta_z.diff_traject.*100, 'Color', [0.4660, 0.6740, 0.1880, 0.2]);
plotStdSurface(avg_diff.*100, std_diff.*100, (1:200), [0.4660, 0.6740, 0.1880], 1);
p2 = plot(delta_z_sim.diff_traject.*100, '--k', 'Linewidth', 2);
p3 = plot(avg_diff.*100, '--', 'Linewidth', 2, 'Color', [0.4660, 0.6740, 0.1880]);
p4 = plot(1:200, (z(pert_idx:pert_idx+199)-z0(pert_idx:pert_idx+199)).*100);
legend([p1(1), p2(1)], {'$\hat{\delta z}$','$\delta z$'},'interpreter','latex')
xlim([1,200])
ylabel('Position (cm)')
xlabel('Time (ms)')

error_p = delta_z.virt_traject-delta_z_sim.virt_traject;

% perturbation error distribution
figure('DefaultAxesFontSize',16)
hold on
histogram(error_p(1:200,:).*100, ...
    'Normalization', 'probability','BinWidth',0.01, 'EdgeAlpha', 0.5);
xlabel('Position estimation error (cm)')
xlim([-0.10,0.10])
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


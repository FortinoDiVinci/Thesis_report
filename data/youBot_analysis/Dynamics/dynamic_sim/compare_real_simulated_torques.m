clear all

load('compare_real_simulated_torque.mat')

idx_simulation = find(t{exp_nb} <= (time_end + t{exp_nb}(1) + 0.0009));
t_real_red = t{exp_nb}(idx_simulation);

real_joint_effort = joint_eff{exp_nb}(idx_simulation,2:4);
sim_joint_effort = tau_ctrl.data;

fontsize0 = 26; %30
fontsize1 = 20;
paperSizeX = 35; %cm %40
paperSizeY = 28; %cm %30

figure('DefaultAxesFontSize',fontsize0);
tiledlayout(3,1, 'TileSpacing', 'compact', 'Padding', 'compact')
for i = 1:3
    nexttile
    hold on
    plot(t_real_red, real_joint_effort(:,i))
    plot(t_real_red, sim_joint_effort(:,i))
    xlim([15, 30])
    title("\tau_" + string(i))
    ylabel('torque (N.m)')
    if i == 3
        xlabel('Experimental time (s)')
    end
end

n = get(gcf,'Number');
%%
set(gcf, 'PaperUnits', 'centimeters', 'PaperSize', [paperSizeX paperSizeY]);
myfiguresize = [0, 0, paperSizeX, paperSizeY];
set(gcf,'PaperPositionMode','manual', 'PaperPosition', myfiguresize);
print -dpdf -f9 -r300 real_simulated_torque % -f<n° figure> -r<résolution>
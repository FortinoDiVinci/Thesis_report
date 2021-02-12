exp_nb = 1;
traj_nb = 1;
mean_sub = 1; % or 0

figure
ax(1) = subplot(2,1,1);
hold on
plot(delta_z{exp_nb}.time, delta_z{exp_nb}.complete_traject)
plot(delta_z{exp_nb}.t_traject, delta_z{exp_nb}.virt_traject, 'r')
title('Position')
legend('meas.', 'virtual')
xlabel('Time (s)')
ylabel('Distance (m)')
ax(2) = subplot(2,1,2);
hold on
plot(delta_fz{exp_nb}.time, delta_fz{exp_nb}.complete_traject)
plot(delta_fz{exp_nb}.t_traject, delta_fz{exp_nb}.virt_traject, 'r')
plot(delta_z{exp_nb}.t_traject(3:end-2,:), impedance{exp_nb}.rec_y + delta_fz{exp_nb}.virt_traject(3:end-2,:), 'c--')
plot(delta_z{exp_nb}.t_traject(3:end-2,:), impedance_arx{exp_nb}.rec_y + delta_fz{exp_nb}.virt_traject(3:end-2,:), 'm--')
title('Force')
legend('meas.', 'virtual')
xlabel('Time (s)')
ylabel('Force (N)')
linkaxes(ax,'x')

%
rec_y_stiff_alone = (delta_z{exp_nb}.diff_traject(:,traj_nb)-mean(delta_z{exp_nb}.diff_traject(:,traj_nb))).*impedance{exp_nb}.xi(1,traj_nb);
figure
hold on
subplot(2,2,1)
hold on
plot(delta_z{exp_nb}.t_traject(:,traj_nb), delta_z{exp_nb}.traject(:,traj_nb))
plot(delta_z{exp_nb}.t_traject(:,traj_nb), delta_z{exp_nb}.virt_traject(:,traj_nb), '--')
title('Meas VS virt position')
subplot(2,2,2)
hold on
plot(delta_z{exp_nb}.t_traject(3:end-2,traj_nb), delta_z{exp_nb}.diff_traject(:,traj_nb))%-mean(delta_z{exp_nb}.diff_traject(:,traj_nb)))
yyaxis right
plot(delta_z{exp_nb}.t_traject(3:end-2,traj_nb), delta_z{exp_nb}.d_diff_traject(:,traj_nb))%-mean(delta_z{exp_nb}.d_diff_traject(:,traj_nb)))
legend('dx', 'dv')
title('Differential position')
subplot(2,2,3)
hold on
plot(delta_fz{exp_nb}.t_traject(:,traj_nb), delta_fz{exp_nb}.traject(:,traj_nb))
plot(delta_fz{exp_nb}.t_traject(:,traj_nb), delta_fz{exp_nb}.virt_traject(:,traj_nb), '--')
title('Meas VS virt force')
subplot(2,2,4)
hold on
plot(delta_fz{exp_nb}.t_traject(3:end-2,traj_nb), delta_fz{exp_nb}.diff_traject(:,traj_nb))%-mean(delta_fz{exp_nb}.diff_traject(:,traj_nb)))
plot(delta_fz{exp_nb}.t_traject(3:end-2,traj_nb), impedance{exp_nb}.rec_y(:,traj_nb))%-mean(impedance{exp_nb}.rec_y(:,traj_nb)), '--')
yyaxis right
plot(delta_fz{exp_nb}.t_traject(3:end-2,traj_nb), rec_y_stiff_alone,'--')%-mean(rec_y_stiff_alone), '--')
legend('fz - fz0', 'Kdx + Bdv + Mda', 'Kdx')
title('Differential force')

%% detailed lsq identification for different models

ph1 = [delta_z{exp_nb}.diff_traject(:,traj_nb),...
       ones(size(delta_z{exp_nb}.diff_traject(:,traj_nb)))];
ph2 = [delta_z{exp_nb}.diff_traject(:,traj_nb),...
       delta_z{exp_nb}.d_diff_traject(:,traj_nb),...
       ones(size(delta_z{exp_nb}.diff_traject(:,traj_nb)))];
ph3 = [delta_z{exp_nb}.diff_traject(:,traj_nb),...
       delta_z{exp_nb}.d_diff_traject(:,traj_nb),...
       delta_z{exp_nb}.dd_diff_traject(:,traj_nb),...
       ones(size(delta_z{exp_nb}.diff_traject(:,traj_nb)))];

if mean_sub
    ph1 = ph1 - [mean(ph1(:,1)),0];
    ph2 = ph2 - [mean(ph2(:,1:2)),0];
    ph3 = ph3 - [mean(ph2(:,1:3)),0];
end
   
y = delta_fz{exp_nb}.diff_traject(:,traj_nb);
   
xi1 = (ph1'*ph1)\ph1'*y;
xi2 = (ph2'*ph2)\ph2'*y;
xi3 = (ph3'*ph3)\ph3'*y;

rec_y1 = ph1*xi1;
rec_y2 = ph2*xi2;
rec_y3 = ph3*xi3;

figure
hold on
plot(delta_fz{exp_nb}.t_traject(3:end-2,traj_nb), delta_fz{exp_nb}.diff_traject(:,traj_nb))
plot(delta_fz{exp_nb}.t_traject(3:end-2,traj_nb), rec_y1)
plot(delta_fz{exp_nb}.t_traject(3:end-2,traj_nb), rec_y2)
plot(delta_fz{exp_nb}.t_traject(3:end-2,traj_nb), rec_y3)

%% perturbation isolated

figure('DefaultAxesFontSize',14)
subplot(2,1,1)
hold on
title('Perturbation position')
plot(delta_z{exp_nb}.diff_traject(:,delta_z{exp_nb}.pert_val > 0), 'b')
plot(mean(delta_z{exp_nb}.diff_traject(:,delta_z{exp_nb}.pert_val > 0),2),'k--', 'LineWidth', 1.5)
plot(delta_z{exp_nb}.diff_traject(:,delta_z{exp_nb}.pert_val < 0), 'g')
plot(mean(delta_z{exp_nb}.diff_traject(:,delta_z{exp_nb}.pert_val < 0),2),'k--', 'LineWidth', 1.5)
ylabel('Distance (m)')
subplot(2,1,2)
hold on
title('Perturbation force')
plot(delta_fz{exp_nb}.diff_traject(:,delta_fz{exp_nb}.pert_val > 0), 'b')
plot(mean(delta_fz{exp_nb}.diff_traject(:,delta_fz{exp_nb}.pert_val > 0),2),'k--', 'LineWidth', 1.5)
plot(delta_fz{exp_nb}.diff_traject(:,delta_fz{exp_nb}.pert_val < 0), 'g')
plot(mean(delta_fz{exp_nb}.diff_traject(:,delta_fz{exp_nb}.pert_val < 0),2),'k--', 'LineWidth', 1.5)
xlabel('Time (s)')
ylabel('Force (N)')

%% Torque and joint position (youBot)
addpath('../../youBot_analysis/Utils/')
% for i = delta_z{exp_nb}.nb_traject:-1:1
%     x = delta_z{exp_nb}.pert_ind(i);
%     for j = 1:delta_z{exp_nb}.estim_window
%         % tau = J^t F => F = (J^t)^-1 tau (J is invertible)
%         force_from_joints(:,j,i) = Jacobian_tot_youbot_vf_transpose_3joints(...
%             thetas{exp_nb}(x+j-1,2), thetas{exp_nb}(x+j-1,3), ...
%             thetas{exp_nb}(x+j-1,3)) \ [joint_eff{exp_nb}(x+j-1,2);...
%             joint_eff{exp_nb}(x+j-1,3); joint_eff{exp_nb}(x+j-1,4)];
%         % p = mgd(th)
%         posi_from_joints(:,:,j,i) = MGD_T0handle(thetas{exp_nb}(x+j-1,1),...
%             thetas{exp_nb}(x+j-1,2),thetas{exp_nb}(x+j-1,3),...
%             thetas{exp_nb}(x+j-1,4),thetas{exp_nb}(x+j-1,5));
%     end  
%     fz_j(:,i) = force_from_joints(2,:,i)';
%     z_j(:,i) = posi_from_joints(3,4,:,i);
% end

for i = length(delta_z{exp_nb}.complete_traject):-1:1
    % tau = J^t F => F = (J^t)^-1 tau (J is invertible)
    jac = Jacobian_tot_youbot_vf_transpose_3joints(thetas{exp_nb}(i,2), ...
        thetas{exp_nb}(i,3), thetas{exp_nb}(i,3));
    force_from_joints(:,i) =  jac(:,[1,3,5])\ [joint_eff{exp_nb}(i,2);...
        joint_eff{exp_nb}(i,3); joint_eff{exp_nb}(i,4)];
    % p = mgd(th)
    posi_from_joints(:,:,i) = MGD_T0handle(thetas{exp_nb}(i,1),...
        thetas{exp_nb}(i,2),thetas{exp_nb}(i,3),...
        thetas{exp_nb}(i,4),thetas{exp_nb}(i,5));
end
%[b2,a2] = butter(20,100/(1/(2*dt)),'low'); 
% fz_j = filtfilt(b2,a2,force_from_joints(2,:)');
fz_j = force_from_joints(2,:)'./3;
z_j = squeeze(posi_from_joints(3,4,:));

figure('DefaultAxesFontSize',14)
ax(1) = subplot(2,1,1);
hold on
plot(delta_z{exp_nb}.time, delta_z{exp_nb}.complete_traject)
plot(delta_z{exp_nb}.time, z_j)
plot(delta_z{exp_nb}.t_traject, delta_z{exp_nb}.virt_traject, 'g')
title('Position')
legend('meas.', 'mgd(\theta)', 'virtual')
xlabel('Time (s)')
ylabel('Distance (m)')
ax(2) = subplot(2,1,2);
hold on
plot(delta_z{exp_nb}.time, delta_fz{exp_nb}.complete_traject)
plot(delta_z{exp_nb}.time, fz_j)
plot(delta_fz{exp_nb}.t_traject, delta_fz{exp_nb}.virt_traject, 'g')
title('Force')
legend('meas.', '(J^t)^{-1} \tau/3', 'virtual')
xlabel('Time (s)')
ylabel('Force (N)')
linkaxes(ax,'x')
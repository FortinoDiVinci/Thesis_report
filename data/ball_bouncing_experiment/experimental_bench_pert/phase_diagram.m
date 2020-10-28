
clear all
close all

addpath('../../force_torque_sensor'); % force filtering
addpath('../../youBot_analysis/Utils'); % MGD

%file_name = 'data_impact.mat'; % dist are in fact ball impact in this example
file_name = 'data_impact2.mat';

load(file_name);

for exp_nb = 1:length(folder_names)

    if ~NO_DISTURBANCE{exp_nb}
        idx_dist = find((dist{exp_nb} ~= 0))';
        t_on_dist = t_dist{exp_nb}(idx_dist);
        val_on_dist = dist{exp_nb}(idx_dist);
    end
    if ~NO_IMPULSE{exp_nb}    
        idx_impulse = find((imp{exp_nb} ~= 0))';
        t_on_impulse = t_impulse{exp_nb}(idx_impulse);
        val_on_impulse = imp{exp_nb}(idx_impulse);
    end
    
    t_exp = t{exp_nb};

    % data filtering
    z = mocap_marker_robot_base{exp_nb}(:,3); % position of the motion capture
    % the low pass filtering is not done is the following function
    [f,~,~]= forces_filtering(forces_unf{exp_nb}', torques_unf{exp_nb}', ...
        thetas{exp_nb}', t_exp);
    fz = f(3,:);
    fc = 25; % cut off frequency
    [b,a] = butter(2,fc/(1/(2*dt)),'low'); 

    z = filtfilt(b,a,z);
    zp = (z - 0.3)*3;
    fz = filtfilt(b,a,fz);

    % centered derivates (W. Khalil and E. Dombre, 2002) eq. [12.16]
    dz = Iu_diffcent(z,t_exp);
    ddz = Iu_diffcent(dz,t_exp);

    zb = z_b{exp_nb};
    dzb = Iu_diffcent(zb,t_exp);
    
    eff = [joint_eff{exp_nb}(:,2), joint_eff{exp_nb}(:,3), joint_eff{exp_nb}(:,4)];
    thetas_exp = thetas{exp_nb};
    f_cmd = zeros(size(eff));
    Gr = zeros(3,3); Gr(1,1) = 156; Gr(2,2) = 100; Gr(3,3) = 71; % gear ratios
    Tc = zeros(3,3); Tc(1,1) = 0.0335; Tc(2,2) = 0.0335; Tc(3,3) = 0.051; % torque constants
    
    for i = 1:length(t_exp)
        jac_3x6 = Jacobian_tot_youbot_vf_transpose_3joints(thetas_exp(i,2), ...
            thetas_exp(i,3), thetas_exp(i,4));
        %f_cmd(i,:) = (jac_3x6(:,[1,3,5]) \ Gr * Tc * i_m(i,:)')';
        f_cmd(i,:) = (jac_3x6(:,[1,3,5]) \ eff(i,:)')';
    end
    
    figure()
    subplot(3,1,1)
    plot(t_exp,zb, '-r', t_exp,zp, 'b')

    subplot(3,1,2)
    plotHandle = plot(t_exp,fz);
    if ~NO_DISTURBANCE{exp_nb}
        line([t_on_dist(:), t_on_dist(:)], [min(plotHandle.YData), max(plotHandle.YData)], 'Color','blue','LineStyle','--');
    end
    if ~NO_IMPULSE{exp_nb} 
        line([t_on_impulse(:), t_on_impulse(:)], [min(plotHandle.YData), max(plotHandle.YData)], 'Color','black','LineStyle','--');
    end
    subplot(3,1,3)
    %plot(t_exp, joint_eff{exp_nb}(:,2), t_exp, joint_eff{exp_nb}(:,3), t_exp, joint_eff{exp_nb}(:,4))
    %legend('\tau_2', '\tau_3', '\tau_4')
    plot(t_exp, f_cmd(:,2))
    legend('f_z')

    idx_dist_t = size(t_on_dist);
    for ii = 1:length(t_on_dist)
        idx_dist_t(ii) = find(t_exp >= t_on_dist(ii), 1, 'first');
    end
    idx_impulse_t = size(t_on_impulse);
    for ii = 1:length(t_on_impulse)
        idx_impulse_t(ii) = find(t_exp >= t_on_impulse(ii), 1, 'first');
    end
    
    % PHASE DIAGRAM
    figure()
    hang_plot = plot(z,dz);
    hold on
    hang_plot_dist = plot(z(idx_dist_t),dz(idx_dist_t), 'ro');
    hang_plot_impulse = plot(z(idx_impulse_t),dz(idx_impulse_t), 'bx');
    legend([hang_plot(1), hang_plot_dist(1), hang_plot_impulse(1)], 'phase cycle', 'distrurbance', 'impact')
    xlabel('z')
    ylabel('v_z')
    
    figure()
    hang_plot = plot(zb,dzb);
    hold on
    hang_plot_dist = plot(zb(idx_dist_t),dzb(idx_dist_t), 'ro');
    hang_plot_impulse = plot(zb(idx_impulse_t),dzb(idx_impulse_t), 'bx');
    legend([hang_plot(1), hang_plot_dist(1), hang_plot_impulse(1)], 'phase cycle', 'disturbance', 'impact')
    xlabel('zb')
    ylabel('v_{zb}')
    
end

% figure()
% subplot(3,1,1)
% plot(t_exp,zb, '-r', t_exp,zp, 'b')
% legend('ball', 'paddle')
% subplot(3,1,2)
% plotHandle = plot(t_exp,fz);
% line([t_on_impulse(:), t_on_impulse(:)], [min(plotHandle.YData), max(plotHandle.YData)], 'Color','black','LineStyle','--');
% legend('fz', 'impulse')
% subplot(3,1,3)
% plot(t_exp,f_cmd)
% legend('fx', 'fz', 'f\theta')
%% 

clear all
close all

addpath('../../../force_torque_sensor')
addpath('../../../youBot_analysis/Utils')

%load('haptic_feedback_validation.mat')
%load('data_16-Oct-2020_15h16/haptic_feedback_validation_2.mat')
load('data_20-Oct-2020_11h24/haptic_feedback_validation_3.mat')

%% Force computation using joint effort

J = NaN(3,3,length(t{1}));
gear_ratio = eye(3); gear_ratio(1,1) = 156; gear_ratio(2,2) = 100; gear_ratio(3,3) = 71;

for i = 1:length(t{1})
     J_tot = Jacobian_tot_youbot_vf_transpose_3joints(thetas{1}(i,2),thetas{1}(i,3),thetas{1}(i,4));
     J(:,:,i) = J_tot(:,[1, 3, 5]);
     force_from_joint_torques(:,i) = J(:,:,i)\[joint_eff{1}(i,2);joint_eff{1}(i,3);joint_eff{1}(i,4)];
end
    
fz_eff = force_from_joint_torques(2,:);

%% Interaction force transformation to robot base

[f,~,~]= forces_filtering(forces_unf{1}', torques_unf{1}', ...
        thetas{1}', t{1});
fz_int = f(3,:);

%% Find forces right before the perturbations and haptic feedbacks

%idx_pert = NaN(length(dist{1})/2+1,1);
idx_pert = NaN(length(dist{1}),1);
idx_imp = NaN(length(imp{1}),1);

%for i = 1:2:length(dist{1})+1 % only rising edges of the haptic feedback 
%    idx_pert((i-1)/2+1) = find(t{1} >= t_dist{1}((i-1)/2+1), 1, 'first');    
%end
for i = 1:length(dist{1}) % only rising edges of the haptic feedback 
    idx_pert(i) = find(t{1} >= t_dist{1}(i), 1, 'first');    
end
for i = 1:length(imp{1}) 
    idx_imp(i) = find(t{1} >= t_impulse{1}(i), 1, 'first'); 
end

%% Disp force

idx_pos_pert = find(dist{1} > 0);
idx_neg_pert = find(dist{1} < 0);

figure
subplot(2,1,1)
hold on
plot(t_dist{1}, dist{1}, '*')
for i = 1:length(idx_pos_pert)
    line([t_dist{1}(idx_pos_pert(i)), t_dist{1}(idx_pos_pert(i))], [0 20],'Color','green','linewidth',1)
    rectangle('Position',[t_dist{1}(idx_pos_pert(i)), fz_eff(idx_pert(idx_pos_pert(i))), 0.03, 10],'FaceColor',[0.4660 0.6740 0.1880], 'LineStyle', 'none')
end
for i = 1:length(idx_neg_pert)
    line([t_dist{1}(idx_neg_pert(i)), t_dist{1}(idx_neg_pert(i))], [0 -20],'Color','red','linewidth',1)
    rectangle('Position',[t_dist{1}(idx_neg_pert(i)), fz_eff(idx_pert(idx_neg_pert(i)))-10, 0.03, 10],'FaceColor',[0.6350 0.0780 0.1840], 'LineStyle', 'none')
end
for i = 1:length(t_impulse{1})
    line([t_impulse{1}(i), t_impulse{1}(i)], [0 imp{1}(i)],'Color','blue','linewidth',1)
    rectangle('Position',[t_impulse{1}(i), fz_eff(idx_imp(i)), 0.03, imp{1}(i)/3],'FaceColor',[0 0.4470 0.7410], 'LineStyle', 'none')
end
plot(t{1}, fz_eff, 'k')
title('Force computed from joint torque')
subplot(2,1,2)
hold on
for i = 1:length(idx_pos_pert)
    line([t_dist{1}(idx_pos_pert(i)), t_dist{1}(idx_pos_pert(i))], [0 20],'Color','green','linewidth',1)
    rectangle('Position',[t_dist{1}(idx_pos_pert(i)), -fz_int(idx_pert(idx_pos_pert(i))), 0.03, 10],'FaceColor',[0.4660 0.6740 0.1880], 'LineStyle', 'none')
end
for i = 1:length(idx_neg_pert)
    line([t_dist{1}(idx_neg_pert(i)), t_dist{1}(idx_neg_pert(i))], [0 -20],'Color','red','linewidth',1)
    rectangle('Position',[t_dist{1}(idx_neg_pert(i)), -fz_int(idx_pert(idx_neg_pert(i)))-10, 0.03, 10],'FaceColor',[0.6350 0.0780 0.1840], 'LineStyle', 'none')
end
for i = 1:length(t_impulse{1})
    line([t_impulse{1}(i), t_impulse{1}(i)], [0 imp{1}(i)],'Color','blue','linewidth',1)
    rectangle('Position',[t_impulse{1}(i), -fz_int(idx_imp(i)), 0.03, imp{1}(i)/3],'FaceColor',[0 0.4470 0.7410], 'LineStyle', 'none')
end
plot(t{1}, -fz_int, 'k')
title('Force from sensor')

%% Velocity command analysis (to try to explain the torque spike whyen switching from torque control to velocity control)

if NO_VEL_CMD{1} == 1 % no velocity command: we leave out this part
    % do nothing
    disp('No velocity command were provided ?')
else %
    
    % try to detect the velocity controlled parts
    diff_t_vel_cmd = diff(t_vel_cmd{1});
    epsilon = 0.003; % 3 ms jitter max, we are looking for 30ms switches
    idx_switch = find(diff_t_vel_cmd > ( mean(diff_t_vel_cmd) + epsilon)); 
    velocity_cmd = {};
    z_speed_cmd = {};
    idx_t = {};
    old_idx = 1;
    for i = 1:length(idx_switch)
        
        idx_1 = find(t_vel_cmd{1}(old_idx) <= t{1}, 1, 'first');
        %if i == length(idx_switch)
        %    idx_2 = length(t{1});
        %else
        idx_2 = find(t_vel_cmd{1}(idx_switch(i)) <= t{1}, 1, 'first');
        %end
                
        idx_t{i} = (idx_1:idx_2);
        velocity_cmd{i} = [...
        interp1(t{1}(idx_t{i}), val_vel_cmd{1}(old_idx:idx_switch(i),1), t_vel_cmd{1}(old_idx:idx_switch(i))), ...
        interp1(t{1}(idx_t{i}), val_vel_cmd{1}(old_idx:idx_switch(i),2), t_vel_cmd{1}(old_idx:idx_switch(i))), ...
        interp1(t{1}(idx_t{i}), val_vel_cmd{1}(old_idx:idx_switch(i),3), t_vel_cmd{1}(old_idx:idx_switch(i)))];
        old_idx = idx_switch(i)+1;
        
        for j = 1:length(idx_t{i})
            idx = idx_t{i}(j);
            J = Jacobian_tot_youbot_vf_3joints(thetas{1}(j,2),thetas{1}(j,3),thetas{1}(j,4));
            tmp = J([1,3,5],:) * velocity_cmd{i}(j,:)';
            z_speed_cmd{i}(j) = tmp(2);
        end
    end

    figure
    subplot(2,1,1)
    plot(t{1}, fz_eff)
    
    subplot(2,1,2)
    hold on
    for i = 1:length(idx_switch)
        plot(t{1}(idx_t{i}), z_speed_cmd{i}, 'color', [0.4660 0.6740 0.1880]);
        hold on
    end
    
end

%% Velocity command re-computation
% The idea here is to understand the origin of the velocity peak
% 
% cartesian_cmd(i,1) = Kx * (x0 - x);
% error = Fr + Fz;
% error_sum = error_sum + error;
% cartesian_cmd(i,2) = Kp * error + Ki * error_sum * (t - t_old);
% 
% joint_ctrl = qi_0 - [thetas{1}(j,2);thetas{1}(j,3);thetas{1}(j,4)];
% 
% temp_jac = Jacobian_tot_youbot_vf_3joints(thetas{1}(j,2),thetas{1}(j,3),thetas{1}(j,4));
% jac_z_task = temp_jac(3,:);
% zNullSpaceProjector = eye(3) - jac_z_task' * inv( jac_z_task' * ( jac_z_task * jac_z_task' ))';
% joint_ctrl = Kq*zNullSpaceProjector*joint_ctrl;
% 
% joint_ctrl = joint_ctrl + jacobian.inv_matrix * cartesian_cmd;
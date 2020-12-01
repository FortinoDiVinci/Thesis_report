clear all

%% DATA

folder_name = '2020_02_07/no_ramp/';

joint_set_points = readtable(strcat(folder_name, 'bagfile-_arm_1_joint_set_points.csv'));
joint_ramp_set_points = readtable(strcat(folder_name, 'bagfile-_arm_1_joint_ramp_set_points.csv'));
joint_states = readtable(strcat(folder_name, 'bagfile-_joint_states.csv'));

t = joint_states.x_time;
eq = joint_states.field_effort4;
vq = joint_states.field_velocity4;
q = joint_states.field_position4;

eq_sp = interp1(joint_set_points.x_time, joint_set_points.field_effort4, joint_states.x_time);
vq_sp = interp1(joint_set_points.x_time, joint_set_points.field_velocity4, joint_states.x_time);
vq_ramp_sp = interp1(joint_ramp_set_points.x_time, joint_ramp_set_points.field_velocity4, joint_states.x_time);
q_sp = interp1(joint_set_points.x_time, joint_set_points.field_position4, joint_states.x_time);
epsilon_vq = vq_ramp_sp - vq;

folder_name = '2020_02_07/ramp/';

joint_set_points_ramp = readtable(strcat(folder_name, 'bagfile-_arm_1_joint_set_points.csv'));
joint_ramp_set_points_ramp = readtable(strcat(folder_name, 'bagfile-_arm_1_joint_ramp_set_points.csv'));
joint_states_ramp = readtable(strcat(folder_name, 'bagfile-_joint_states.csv'));

t_r = joint_states_ramp.x_time;
eq_r = joint_states_ramp.field_effort4;
vq_r = joint_states_ramp.field_velocity4;
q_r = joint_states_ramp.field_position4;

eq_sp_r = interp1(joint_set_points_ramp.x_time, joint_set_points_ramp.field_effort4, joint_states_ramp.x_time);
vq_sp_r = interp1(joint_set_points_ramp.x_time, joint_set_points_ramp.field_velocity4, joint_states_ramp.x_time);
vq_ramp_sp_r = interp1(joint_ramp_set_points_ramp.x_time, joint_ramp_set_points_ramp.field_velocity4, joint_states_ramp.x_time);
q_sp_r = interp1(joint_set_points_ramp.x_time, joint_set_points_ramp.field_position4, joint_states_ramp.x_time);
epsilon_vq_r = vq_ramp_sp_r - vq_r;

p_gain = 800/256;
torque_constant = [0.0335, 0.0335, 0.0335, 0.051 ,0.049]; %Nm/A
gear_ratio = [1/156, 1/156, 1/100, 1/71 , 1/71];


%% PLOT

figure
subplot(2,1,1)
hold on, grid on
plot((t-t(1))*1e-9, vq_sp);
plot((t-t(1))*1e-9, vq_ramp_sp);
plot((t-t(1))*1e-9, vq);
legend({'$\dot{q}_{5 cons}$', '$\dot{q}_{5 ramp}$', '$\dot{q}_5$'}, 'Interpreter','latex')
title('Joint 5 velocity without ramp generator')
subplot(2,1,2)
hold on, grid on
plot((t_r-t_r(1))*1e-9, vq_sp_r);
plot((t_r-t_r(1))*1e-9, vq_ramp_sp_r);
plot((t_r-t_r(1))*1e-9, vq_r);
legend({'$\dot{q}_{5 cons}$', '$\dot{q}_{5 ramp}$', '$\dot{q}_5$'}, 'Interpreter','latex')
title('Joint 5 velocity with ramp generator')


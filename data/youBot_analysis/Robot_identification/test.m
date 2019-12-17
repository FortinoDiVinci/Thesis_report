%% trajectory data

traj_joint2 = readtable('traj_faible_vit1.csv');
traj_joint3 = readtable('traj_faible_vit2.csv');
traj_joint4 = readtable('traj_faible_vit3.csv');

figure()
hold on, grid on
plot(traj_joint2.joint_pos)
plot(traj_joint3.joint_pos)
plot(traj_joint4.joint_pos)
legend('joint2', 'joint3', 'joint4')

%% joint data

tc_jnt2 = 0.0335; % Nm/A
tc_jnt3 = 0.0335; % Nm/A
tc_jnt4 = 0.051; % Nm/A

%% experimental data

experimental_joint_positions = readtable('joint_state_pos_traj_0');
experimental_joint_sp_positions = readtable('joint_state_pos_traj_sp_0');

experimental_joint_velocities = readtable('joint_state_vel_traj_0');
experimental_joint_sp_velocities = readtable('joint_state_vel_traj_sp_0');

experimental_joint_efforts = readtable('joint_state_eff_traj_0');
experimental_joint_sp_efforts = readtable('joint_state_eff_traj_sp_0');

% measured joints positions

figure()
hold on, grid on
t = (experimental_joint_positions.time_stamp - experimental_joint_positions.time_stamp(1))*10e-10;
plot(t, experimental_joint_positions.arm_joint_2_pos)
plot(t, experimental_joint_positions.arm_joint_3_pos)
plot(t, experimental_joint_positions.arm_joint_4_pos)
legend('joint2', 'joint3', 'joint4')
title('Joint State')

% joints set point positions

figure()
hold on, grid on
t = (experimental_joint_sp_positions.time_stamp - experimental_joint_sp_positions.time_stamp(1))*10e-10;
plot(t, experimental_joint_sp_positions.arm_joint_2_pos)
plot(t ,experimental_joint_sp_positions.arm_joint_3_pos)
plot(t, experimental_joint_sp_positions.arm_joint_4_pos)
legend('joint2', 'joint3', 'joint4')
title('Joint Setpoint data')

% measured joints effort

figure()
hold on, grid on
t = (experimental_joint_efforts.time_stamp - experimental_joint_efforts.time_stamp(1))*10e-10;
plot(t,experimental_joint_efforts.arm_joint_2_eff)
plot(t,experimental_joint_efforts.arm_joint_3_eff)
plot(t,experimental_joint_efforts.arm_joint_4_eff)
legend('joint2', 'joint3', 'joint4')
title('Joint effort data')

% joints set point effort

figure()
hold on, grid on
t = (experimental_joint_sp_efforts.time_stamp - experimental_joint_sp_efforts.time_stamp(1))*10e-10;
plot(t,experimental_joint_sp_efforts.arm_joint_2_eff)
plot(t,experimental_joint_sp_efforts.arm_joint_3_eff)
plot(t,experimental_joint_sp_efforts.arm_joint_4_eff)
legend('joint2', 'joint3', 'joint4')
title('Joint effort set point')
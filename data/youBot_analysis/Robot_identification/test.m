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

% experimental_joint_positions = readtable('rosbag_vs_python_recording/joint_state_pos_traj_0');
% experimental_joint_sp_positions = readtable('rosbag_vs_python_recording/joint_state_pos_traj_sp_0');
% 
% experimental_joint_velocities = readtable('rosbag_vs_python_recording/joint_state_vel_traj_0');
% experimental_joint_sp_velocities = readtable('rosbag_vs_python_recording/joint_state_vel_traj_sp_0');
% 
% experimental_joint_efforts = readtable('rosbag_vs_python_recording/joint_state_eff_traj_0'); % signal should be filtered before recording
% experimental_joint_sp_efforts = readtable('rosbag_vs_python_recording/joint_state_eff_traj_sp_0'); % signal should be filtered before recording
% 
% rosbag_joint_states = readtable('rosbag_vs_python_recording/bagfile-_joint_states.csv');
% rosbag_joint_set_points = readtable('rosbag_vs_python_recording/bagfile-_arm_1_joint_set_points.csv');
% 
% pos_m = [rosbag_joint_states.field_position1 rosbag_joint_states.field_position2 rosbag_joint_states.field_position3];
% vel_m = [rosbag_joint_states.field_velocity1 rosbag_joint_states.field_velocity2 rosbag_joint_states.field_velocity3];
% eff_m = [rosbag_joint_states.field_effort1 rosbag_joint_states.field_effort2 rosbag_joint_states.field_effort3];
% 
% pos_sp = [rosbag_joint_set_points.field_position1 rosbag_joint_set_points.field_position2 rosbag_joint_set_points.field_position3];
% vel_sp = [rosbag_joint_set_points.field_velocity1 rosbag_joint_set_points.field_velocity2 rosbag_joint_set_points.field_velocity3];
% eff_sp = [rosbag_joint_set_points.field_effort1 rosbag_joint_set_points.field_effort2 rosbag_joint_set_points.field_effort3];

rosbag_joint_states = readtable('rosbag_1kHz/bagfile-_joint_states.csv');
rosbag_joint_set_points = readtable('rosbag_1kHz/bagfile-_arm_1_joint_set_points.csv');

%%%%%%%%%%%
%% DISPLAY
%%%%%%%%%%%

% measured joints positions

figure()
hold on, grid on
%t = (experimental_joint_positions.time_stamp - experimental_joint_positions.time_stamp(1))*10e-10;
%t_ros = (rosbag_joint_states.x_time - experimental_joint_positions.time_stamp(1))*10e-10;
t_ros = (rosbag_joint_states.x_time - rosbag_joint_states.x_time(1))*10e-10;
%plot(t, experimental_joint_positions.arm_joint_2_pos)
%plot(t, experimental_joint_positions.arm_joint_3_pos)
%plot(t, experimental_joint_positions.arm_joint_4_pos)
plot(t_ros, rosbag_joint_states.field_position1)
plot(t_ros, rosbag_joint_states.field_position2)
plot(t_ros, rosbag_joint_states.field_position3)

%legend('joint2', 'joint3', 'joint4', 'joint2 ROS', 'joint3 ROS', 'joint4 ROS')
legend('joint2 ROS', 'joint3 ROS', 'joint4 ROS')
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
t_ros = (rosbag_joint_states.x_time - experimental_joint_positions.time_stamp(1))*10e-10;
plot(t,experimental_joint_efforts.arm_joint_2_eff)
plot(t,experimental_joint_efforts.arm_joint_3_eff)
plot(t,experimental_joint_efforts.arm_joint_4_eff)
plot(t_ros, rosbag_joint_states.field_effort1)
plot(t_ros, rosbag_joint_states.field_effort2)
plot(t_ros, rosbag_joint_states.field_effort3)

legend('joint2', 'joint3', 'joint4', 'joint2 ROS', 'joint3 ROS', 'joint4 ROS')
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

%%%%%%%%%%%%%%%%%%%%
% set point VS meas.
%%%%%%%%%%%%%%%%%%%%

%% joint2

figure()
hold on, grid on
t_ros_js = (rosbag_joint_states.x_time - rosbag_joint_states.x_time(1))*10e-10;
t_ros_sp = (rosbag_joint_set_points.x_time - rosbag_joint_set_points.x_time(1))*10e-10;
plot(t_ros_js, rosbag_joint_states.field_position1)
plot(t_ros_sp, rosbag_joint_set_points.field_position1)
legend('meas', 'set point')
title('Joint 2 position')

figure()
hold on, grid on
t_ros_js = (rosbag_joint_states.x_time - rosbag_joint_states.x_time(1))*10e-10;
t_ros_sp = (rosbag_joint_set_points.x_time - rosbag_joint_set_points.x_time(1))*10e-10;
plot(t_ros_js, rosbag_joint_states.field_velocity1)
plot(t_ros_sp, rosbag_joint_set_points.field_velocity1)
legend('meas', 'set point')
title('Joint 2 velocity')

figure()
hold on, grid on
t_ros_js = (rosbag_joint_states.x_time - rosbag_joint_states.x_time(1))*10e-10;
t_ros_sp = (rosbag_joint_set_points.x_time - rosbag_joint_set_points.x_time(1))*10e-10;
plot(t_ros_js, rosbag_joint_states.field_effort1)
plot(t_ros_sp, rosbag_joint_set_points.field_effort1)
legend('meas', 'set point')
title('Joint 2 effort')

%% joint3

figure()
hold on, grid on
t_ros_js = (rosbag_joint_states.x_time - rosbag_joint_states.x_time(1))*10e-10;
t_ros_sp = (rosbag_joint_set_points.x_time - rosbag_joint_set_points.x_time(1))*10e-10;
plot(t_ros_js, rosbag_joint_states.field_position2)
plot(t_ros_sp, rosbag_joint_set_points.field_position2)
legend('meas', 'set point')
title('Joint 3 position')

figure()
hold on, grid on
t_ros_js = (rosbag_joint_states.x_time - rosbag_joint_states.x_time(1))*10e-10;
t_ros_sp = (rosbag_joint_set_points.x_time - rosbag_joint_set_points.x_time(1))*10e-10;
plot(t_ros_js, rosbag_joint_states.field_velocity2)
plot(t_ros_sp, rosbag_joint_set_points.field_velocity2)
legend('meas', 'set point')
title('Joint 3 velocity')

figure()
hold on, grid on
t_ros_js = (rosbag_joint_states.x_time - rosbag_joint_states.x_time(1))*10e-10;
t_ros_sp = (rosbag_joint_set_points.x_time - rosbag_joint_set_points.x_time(1))*10e-10;
plot(t_ros_js, rosbag_joint_states.field_effort2)
plot(t_ros_sp, rosbag_joint_set_points.field_effort2)
legend('meas', 'set point')
title('Joint 3 effort')

%%%
%% identification des gains du controlleur de vitesse
%%%

H = rosbag_joint_set_points.field_effort1./rosbag_joint_set_points.field_velocity1;

figure()
hold on, grid on
t_ros_sp = (rosbag_joint_set_points.x_time - rosbag_joint_set_points.x_time(1))*10e-10;
plot(t_ros_sp, H)

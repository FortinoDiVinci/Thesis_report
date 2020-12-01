clear all
close all

addpath('../../Utils');
addpath('../../Jacobian');
addpath('../../utils');

folder_name = 'calibration_set_3/';

joint_states = readtable(strcat(folder_name, 'bagfile-_joint_states.csv'));
joint_setpoints = readtable(strcat(folder_name, 'bagfile-_arm_1_joint_set_points.csv'));
mocap_joint5 = readtable(strcat(folder_name, 'bagfile-_vrpn_client_node_robot_joint_5_pose.csv'));

t = (joint_states.x_time - joint_states.x_time(1))*1e-9;
mocap5_t = (mocap_joint5.x_time - joint_states.x_time(1))*1e-9;
thetas = [joint_states.field_position0, joint_states.field_position1, joint_states.field_position2, joint_states.field_position3, joint_states.field_position4];
dthetas = [joint_states.field_velocity0, joint_states.field_velocity1, joint_states.field_velocity2, joint_states.field_velocity3, joint_states.field_velocity4];
thetas_sp = [interp1(joint_setpoints.x_time, joint_setpoints.field_position0, joint_states.x_time), interp1(joint_setpoints.x_time, joint_setpoints.field_position1, joint_states.x_time), interp1(joint_setpoints.x_time, joint_setpoints.field_position2, joint_states.x_time), interp1(joint_setpoints.x_time, joint_setpoints.field_position3, joint_states.x_time), interp1(joint_setpoints.x_time, joint_setpoints.field_position4, joint_states.x_time)];
mocap_joint5_xyz = [interp1(mocap_joint5.x_time, mocap_joint5.field_pose_position_x, joint_states.x_time), interp1(mocap_joint5.x_time, mocap_joint5.field_pose_position_y, joint_states.x_time), interp1(mocap_joint5.x_time, mocap_joint5.field_pose_position_z, joint_states.x_time)];


%% Check opti track interpolation

figure()
hold on, grid on
plot(t, mocap_joint5_xyz, 'o')
plot(mocap5_t, [mocap_joint5.field_pose_position_x, mocap_joint5.field_pose_position_y, mocap_joint5.field_pose_position_z], '*')
legend('interpolation x', 'interpolation y', 'interpolation z', 'original x', 'original y', 'original z')
title('Joint 5')

%% Optitrack VS Robot Endpoint

% endpoint computation
robot_joint5_xyz = zeros(length(t), 3);
robot_joint5_sp_xyz = zeros(length(t), 3);
alpha = zeros(length(t), 1);
thetas_offset = [169 65 -146 102.5 167.5].*pi/180;
for i=1:length(t)
    T = MGD_T0marker(thetas(i,1), thetas(i,2), thetas(i,3), thetas(i,4), thetas(i,5));
    T_sp = MGD_T0marker(thetas_sp(i,1), thetas_sp(i,2), thetas_sp(i,3), thetas_sp(i,4), thetas_sp(i,5));
    robot_joint5_xyz(i, :) = T(1:3,4);
    robot_joint5_sp_xyz(i, :) = T_sp(1:3,4);
    alpha(i) = -thetas(i,2) + thetas_offset(2) - thetas(i,3) + thetas_offset(3) - thetas(i,4) + thetas_offset(4);
end

% finding transformation matrix between optitrack coordinates and robot
% base coordinates (using non contact coordinates only)
%[R, Bfit, ErrorStats] = absor(mocap_joint5_xyz(10:5e3,:)', robot_joint5_xyz(10:5e3,:)', 'doScale', 'TRUE');
[R, Bfit, ErrorStats] = absor(mocap_joint5_xyz(10:5e4,:)', robot_joint5_xyz(10:5e4,:)');

mocap_joint5_robot_base_xyz = zeros(size(mocap_joint5_xyz));
for i=1:length(t)
    temp_hom = R.M * [mocap_joint5_xyz(i,:), 1]';
    mocap_joint5_robot_base_xyz(i,:) = temp_hom(1:3);
end

figure()
subplot(3,1,1)
hold on, grid on
%plot(t, opti_track_xyz(:,3));
plot(t, mocap_joint5_robot_base_xyz(:,1));
plot(t, robot_joint5_xyz(:,1));
legend('mocap x','robot x')
title('Motion capture VS Direct Kinematics')
subplot(3,1,2)
hold on, grid on
%plot(t, opti_track_xyz(:,3));
plot(t, mocap_joint5_robot_base_xyz(:,2));
plot(t, robot_joint5_xyz(:,2));
legend('mocap y', 'robot y')
subplot(3,1,3)
hold on, grid on
%plot(t, opti_track_xyz(:,3));
plot(t, mocap_joint5_robot_base_xyz(:,3));
plot(t, robot_joint5_xyz(:,3));
legend('mocap z', 'robot z')

%% Cartesian velocities and jacobian calibration

% filtering positions
freq = length(t)/t(end);
filter_order = 100;
fir_filter = fir1(filter_order, (20/(freq)), 'low');
mocap_joint5_robot_base_xyz_f = filter(fir_filter, 1, mocap_joint5_robot_base_xyz); 
mocap_joint5_robot_base_xyz_f = circshift(mocap_joint5_robot_base_xyz_f, length(mocap_joint5_robot_base_xyz_f)-filter_order/2, 1) ;
%robot_joint5_xyz_f = filter(fir_filter, 1, robot_joint5_xyz); 
%robot_joint5_xyz_f = circshift(robot_joint5_xyz_f, length(robot_joint5_xyz_f)-filter_order/2, 1) ;

% derivative
mocap_vel = zeros(size(mocap_joint5_robot_base_xyz));
robot_vel = zeros(size(robot_joint5_xyz));
for i = 2:length(mocap_joint5_robot_base_xyz) - 1
    mocap_vel(i, :) = ( mocap_joint5_robot_base_xyz_f(i + 1, :) - mocap_joint5_robot_base_xyz_f(i - 1, :) ) ./ ( 2*( t(i+1) - t(i-1) ) );
    robot_vel(i, :) = ( robot_joint5_xyz(i + 1, :) - robot_joint5_xyz(i - 1, :) ) ./ ( 2*( t(i+1) - t(i-1) ) );
end

% filtering robot velocities
robot_vel_f = filter(fir_filter, 2, robot_vel); 
robot_vel_f = circshift(robot_vel_f, length(robot_vel_f)-filter_order/2, 1);
dthetas_f = filter(fir_filter, 1, dthetas); 
dthetas_f = circshift(dthetas_f, length(dthetas_f)-filter_order/2, 1);

% jacobian
robot_vel_from_jacobian = zeros(size(robot_joint5_xyz));
for i = 1:length(mocap_joint5_robot_base_xyz)
   vels = Jacobian_tot_youbot_vf_3joints(thetas(i,2), thetas(i,3), thetas(i,4)) * [dthetas_f(i,2); dthetas_f(i,3); dthetas_f(i,4)];
   robot_vel_from_jacobian(i, :) = vels(1:3);
end

figure()
subplot(2,1,1)
hold on, grid on
plot(t, mocap_vel);
plot(t, robot_vel);
legend('mocap dx', 'mocap dy', 'mocap dz', 'robot dx', 'robot dy', 'robot dz')
title('Endpoint velocities')
subplot(2,1,2)
hold on, grid on
plot(t, robot_vel);
plot(t, robot_vel_from_jacobian);
legend('robot dx', 'robot dy', 'robot dz', 'J robot dx', 'J robot dy', 'J robot dz')
title('Endpoint velocities')

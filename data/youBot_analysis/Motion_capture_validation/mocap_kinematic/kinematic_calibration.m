clear all
close all

addpath('../../../Utils');
addpath('../../../Jacobian');
addpath('../utils');

joint_states = readtable('bagfile-_joint_states.csv');
joint_setpoints = readtable('bagfile-_arm_1_joint_set_points.csv');
mocap_joint1 = readtable('bagfile-_vrpn_client_node_robot_joint_1_pose.csv');
mocap_joint5 = readtable('bagfile-_vrpn_client_node_robot_joint_5_pose.csv');

t = (joint_states.x_time - joint_states.x_time(1))*1e-9;
mocap1_t = (mocap_joint1.x_time - joint_states.x_time(1))*1e-9;
mocap5_t = (mocap_joint5.x_time - joint_states.x_time(1))*1e-9;
thetas = [joint_states.field_position0, joint_states.field_position1, joint_states.field_position2, joint_states.field_position3, joint_states.field_position4];
thetas_sp = [interp1(joint_setpoints.x_time, joint_setpoints.field_position0, joint_states.x_time), interp1(joint_setpoints.x_time, joint_setpoints.field_position1, joint_states.x_time), interp1(joint_setpoints.x_time, joint_setpoints.field_position2, joint_states.x_time), interp1(joint_setpoints.x_time, joint_setpoints.field_position3, joint_states.x_time), interp1(joint_setpoints.x_time, joint_setpoints.field_position4, joint_states.x_time)];
mocap_joint1_xyz = [interp1(mocap_joint1.x_time, mocap_joint1.field_pose_position_x, joint_states.x_time), interp1(mocap_joint1.x_time, mocap_joint1.field_pose_position_y, joint_states.x_time), interp1(mocap_joint1.x_time, mocap_joint1.field_pose_position_z, joint_states.x_time)];
mocap_joint5_xyz = [interp1(mocap_joint5.x_time, mocap_joint5.field_pose_position_x, joint_states.x_time), interp1(mocap_joint5.x_time, mocap_joint5.field_pose_position_y, joint_states.x_time), interp1(mocap_joint5.x_time, mocap_joint5.field_pose_position_z, joint_states.x_time)];


%% Check opti track interpolation

figure()
subplot(2,1,1)
hold on, grid on
plot(t, mocap_joint1_xyz, 'o')
plot(mocap1_t, [mocap_joint1.field_pose_position_x, mocap_joint1.field_pose_position_y, mocap_joint1.field_pose_position_z], '*')
legend('interpolation x', 'interpolation y', 'interpolation z', 'original x', 'original y', 'original z')
title('Joint 1')
subplot(2,1,2)
hold on, grid on
plot(t, mocap_joint5_xyz, 'o')
plot(mocap5_t, [mocap_joint5.field_pose_position_x, mocap_joint5.field_pose_position_y, mocap_joint5.field_pose_position_z], '*')
legend('interpolation x', 'interpolation y', 'interpolation z', 'original x', 'original y', 'original z')
title('Joint 5')

%% Optitrack VS Robot Endpoint

% endpoint computation
robot_joint5_xyz = zeros(length(t), 3);
robot_joint5_sp_xyz = zeros(length(t), 3);
robot_joint1_xyz = zeros(length(t), 3);
alpha = zeros(length(t), 1);
thetas_offset = [169 65 -146 102.5 167.5].*pi/180;
for i=1:length(t)
    T = MGD_T05(thetas(i,1), thetas(i,2), thetas(i,3), thetas(i,4), thetas(i,5));
    T01 = MGD_T01(thetas(i,1));
    T_sp = MGD_T05(thetas_sp(i,1), thetas_sp(i,2), thetas_sp(i,3), thetas_sp(i,4), thetas_sp(i,5));
    robot_joint5_xyz(i, :) = T(1:3,4);
    robot_joint5_sp_xyz(i, :) = T_sp(1:3,4);
    robot_joint1_xyz(i, :) = T01(1:3,4);
    alpha(i) = -thetas(i,2) + thetas_offset(2) - thetas(i,3) + thetas_offset(3) - thetas(i,4) + thetas_offset(4);
end

% finding transformation matrix between optitrack coordinates and robot
% base coordinates (using non contact coordinates only)
[R, Bfit, ErrorStats] = absor(mocap_joint5_xyz(10:5e4,:)', robot_joint5_xyz(10:5e4,:)', 'doScale', 'TRUE');
[R2, Bfit2, ErrorStats2] = absor(mocap_joint1_xyz(10:5e4,:)', robot_joint1_xyz(10:5e4,:)', 'doScale', 'TRUE');

quat = tform2quat(R.M);

mocap_joint5_robot_base_xyz = zeros(size(mocap_joint5_xyz));
mocap_joint1_robot_base_xyz = zeros(size(mocap_joint1_xyz));
for i=1:length(t)
    temp_hom = R.M * [mocap_joint5_xyz(i,:), 1]';
    mocap_joint5_robot_base_xyz(i,:) = temp_hom(1:3);
    temp_hom = R.M * [mocap_joint1_xyz(i,:), 1]';
    mocap_joint1_robot_base_xyz(i,:) = temp_hom(1:3);
end

figure()
subplot(2,1,1)
hold on, grid on
%plot(t, opti_track_xyz(:,3));
plot(t, mocap_joint5_robot_base_xyz(:,:));
plot(t, robot_joint5_xyz(:,:));
legend('mocap x', 'mocap y', 'mocap z', 'robot x', 'robot y', 'robot z')
title('Joint 5')
subplot(2,1,2)
hold on, grid on
%plot(t, opti_track_xyz(:,3));
plot(t, mocap_joint1_robot_base_xyz(:,:));
plot(t, robot_joint1_xyz(:,:));
legend('mocap x', 'mocap y', 'mocap z', 'robot x', 'robot y', 'robot z')
title('Joint 1')

%% 
% 7.8cm offset from joint 1 sensor z, and 0

x_joint1 = nanmean(robot_joint1_xyz(:,1));
y_joint1 = nanmean(robot_joint1_xyz(:,2));
z_joint1 = nanmean(robot_joint1_xyz(:,3));

x_mocap_offset = x_joint1 - (nanmean(mocap_joint1_robot_base_xyz(:,1)));
y_mocap_offset = y_joint1 - (nanmean(mocap_joint1_robot_base_xyz(:,2)));
z_mocap_offset = z_joint1 - (nanmean(mocap_joint1_robot_base_xyz(:,3)) - 0.078);

figure()
subplot(2,1,1)
hold on, grid on
%plot(t, opti_track_xyz(:,3));
plot(t, mocap_joint5_robot_base_xyz(:,:) + [x_mocap_offset, y_mocap_offset, z_mocap_offset]);
plot(t, robot_joint5_xyz(:,:));
legend('mocap x', 'mocap y', 'mocap z', 'robot x', 'robot y', 'robot z')
title('Joint 5')
subplot(2,1,2)
hold on, grid on
%plot(t, opti_track_xyz(:,3));
plot(t, mocap_joint1_robot_base_xyz(:,:) + [x_mocap_offset, y_mocap_offset, z_mocap_offset+0.078]);
plot(t, robot_joint1_xyz(:,:));
legend('mocap x', 'mocap y', 'mocap z', 'robot x', 'robot y', 'robot z')
title('Joint 1')

tr = R.t;
q = R.q;

%% Rotation using robot 1st joint pose from mocap

r = quat2rotation([nanmean(mocap_joint1.field_pose_orientation_w), nanmean(mocap_joint1.field_pose_orientation_x), nanmean(mocap_joint1.field_pose_orientation_y), nanmean(mocap_joint1.field_pose_orientation_z)]); 

Tw1o = [r, [nanmean(mocap_joint1.field_pose_position_x); nanmean(mocap_joint1.field_pose_position_y); nanmean(mocap_joint1.field_pose_position_z)]];
Tw1o = [Tw1o; [0,0,0,1]];
T1o1 = [eye(3), [0;0;0.078] ; [0,0,0,1]]; %translation (the target, has an offset)
Tw1 = Tw1o*T1o1; 

%T01 mean computation
T01_sum = zeros(4,4);
for i=1:length(t)
    T01_sum = T01_sum + MGD_T01(thetas(i,1));
end
T01_mean = T01_sum./length(t);
T10 = inv(T01_mean);

Tw0 = Tw1*T10;
Trb_mc = zeros(length(t), 4);
for i=1:length(t)
    Trb_mc(i,:) = (Tw0*[mocap_joint1_xyz(i,:), 1]')';
end

figure
subplot(2,1,1)
plot(t, Trb_mc(:,1:3));
legend('x', 'y', 'z')
subplot(2,1,2)
plot(t, mocap_joint1_xyz);
legend('x', 'y', 'z')
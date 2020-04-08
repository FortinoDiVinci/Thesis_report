clear all
close all

addpath('../../Utils');
addpath('../../Jacobian');
addpath('../../Motion_capture_validation/utils');
addpath('../../../force_torque_sensor')
load('rotation_data.mat')

folder_name = 'calibration_set_1/';

joint_states = readtable(strcat(folder_name, 'bagfile-_joint_states.csv'));
joint_setpoints = readtable(strcat(folder_name, 'bagfile-_arm_1_joint_set_points.csv'));
mocap_marker = readtable(strcat(folder_name, 'bagfile-_vrpn_client_node_robot_joint_5_pose.csv'));
force_unf = readtable(strcat(folder_name, 'bagfile-_netft_data.csv'));
disturbance = readtable(strcat(folder_name, 'bagfile-_arm_1_disturbance_val.csv'));

joint_t = (joint_states.x_time - joint_states.x_time(1))*1e-9;
joint_sp_t = (joint_setpoints.x_time - joint_states.x_time(1))*1e-9;
mocap_t = (mocap_marker.x_time - joint_states.x_time(1))*1e-9;
force_unf_t = (force_unf.x_time - joint_states.x_time(1))*1e-9;
t_end = min([joint_t(end); joint_sp_t(end); mocap_t(end); force_unf_t(end)]);
t_start = max([joint_t(1); joint_sp_t(1); mocap_t(1); force_unf_t(1)]);
t = t_start:1e-3:t_end;
t = t';
t_free_mov = (max(joint_t(1), mocap_t(1)):1e-3:t_start)';
t_dist = (disturbance.x_time - joint_states.x_time(1))*1e-9;
t_dist = t_dist(1:end-2);
%
thetas = [interp1(joint_t, joint_states.field_position0, t), interp1(joint_t, joint_states.field_position1, t), interp1(joint_t, joint_states.field_position2, t), interp1(joint_t, joint_states.field_position3, t), interp1(joint_t, joint_states.field_position4, t)];
thetas_fm = [interp1(joint_t, joint_states.field_position0, t_free_mov), interp1(joint_t, joint_states.field_position1, t_free_mov), interp1(joint_t, joint_states.field_position2, t_free_mov), interp1(joint_t, joint_states.field_position3, t_free_mov), interp1(joint_t, joint_states.field_position4, t_free_mov)];
dthetas = [interp1(joint_t, joint_states.field_velocity0, t), interp1(joint_t, joint_states.field_velocity1, t), interp1(joint_t, joint_states.field_velocity2, t), interp1(joint_t, joint_states.field_velocity3, t), interp1(joint_t, joint_states.field_velocity4, t)];
thetas_sp = [interp1(joint_sp_t, joint_setpoints.field_position0, t), interp1(joint_sp_t, joint_setpoints.field_position1, t), interp1(joint_sp_t, joint_setpoints.field_position2, t), interp1(joint_sp_t, joint_setpoints.field_position3, t), interp1(joint_sp_t, joint_setpoints.field_position4, t)];
eff_sp = [interp1(joint_sp_t, joint_setpoints.field_effort0, t), interp1(joint_sp_t, joint_setpoints.field_effort1, t), interp1(joint_sp_t, joint_setpoints.field_effort2, t), interp1(joint_sp_t, joint_setpoints.field_effort3, t), interp1(joint_sp_t, joint_setpoints.field_effort4, t)];
mocap_marker_xyz = [interp1(mocap_t, mocap_marker.field_pose_position_x, t), interp1(mocap_t, mocap_marker.field_pose_position_y, t), interp1(mocap_t, mocap_marker.field_pose_position_z, t)];
mocap_marker_xyz_fm = [interp1(mocap_t, mocap_marker.field_pose_position_x, t_free_mov), interp1(mocap_t, mocap_marker.field_pose_position_y, t_free_mov), interp1(mocap_t, mocap_marker.field_pose_position_z, t_free_mov)];
force_unf_xyz = [interp1(force_unf_t, force_unf.field_wrench_force_x, t), interp1(force_unf_t, force_unf.field_wrench_force_y, t), interp1(force_unf_t, force_unf.field_wrench_force_z, t)];
torque_unf_xyz = [interp1(force_unf_t, force_unf.field_wrench_torque_x, t), interp1(force_unf_t, force_unf.field_wrench_torque_y, t), interp1(force_unf_t, force_unf.field_wrench_torque_z, t)];


%% Check opti track interpolation

figure()
hold on, grid on
plot(t, mocap_marker_xyz, 'o')
plot(mocap_t, [mocap_marker.field_pose_position_x, mocap_marker.field_pose_position_y, mocap_marker.field_pose_position_z], '*')
legend('interpolation x', 'interpolation y', 'interpolation z', 'original x', 'original y', 'original z')
title('Marker endpoint coordinates')

figure()
hold on, grid on
plot(t, force_unf_xyz, 'o')
plot(force_unf_t, [force_unf.field_wrench_force_x, force_unf.field_wrench_force_y, force_unf.field_wrench_force_z], '*')
legend('interpolation x', 'interpolation y', 'interpolation z', 'original x', 'original y', 'original z')
title('Sensor Forces')

%% Optitrack VS Robot Endpoint

% endpoint computation
robot_handle_xyz = zeros(length(t), 3);
robot_marker_xyz = zeros(length(t), 3);
robot_marker_xyz_fm = zeros(length(t_free_mov), 3);
robot_marker_sp_xyz = zeros(length(t), 3);
alpha = zeros(length(t), 1);
thetas_offset = [169 65 -146 102.5 167.5].*pi/180;
for i=1:length(t)
    Tend = MGD_T0handle(thetas(i,1), thetas(i,2), thetas(i,3), thetas(i,4), thetas(i,5));
    T = MGD_T0marker(thetas(i,1), thetas(i,2), thetas(i,3), thetas(i,4), thetas(i,5));
    T_sp = MGD_T0marker(thetas_sp(i,1), thetas_sp(i,2), thetas_sp(i,3), thetas_sp(i,4), thetas_sp(i,5));
    robot_handle_xyz(i, :) = Tend(1:3,4);
    robot_marker_xyz(i, :) = T(1:3,4);
    robot_marker_sp_xyz(i, :) = T_sp(1:3,4);
end
for i=1:length(t_free_mov)
    T_fm = MGD_T0marker(thetas_fm(i,1), thetas_fm(i,2), thetas_fm(i,3), thetas_fm(i,4), thetas_fm(i,5)); % free move
    robot_marker_xyz_fm(i, :) = T_fm(1:3,4);
end
% transformation recalibration is done using free movement
[R2, Bfit, ErrorStats] = absor(mocap_marker_xyz_fm', robot_marker_xyz_fm');

mocap_marker_robot_base_xyz = zeros(size(mocap_marker_xyz));
mocap_marker_robot_base_xyz2 = zeros(size(mocap_marker_xyz));
for i=1:length(t)
    temp_hom = R.M * [mocap_marker_xyz(i,:), 1]';
    mocap_marker_robot_base_xyz(i,:) = temp_hom(1:3);
    temp_hom = R2.M * [mocap_marker_xyz(i,:), 1]';
    mocap_marker_robot_base_xyz2(i,:) = temp_hom(1:3);
end

% We compare the robot endpoint coordinates at the marker point, using
% - the direct kinematic model and the robot joint data
% - the optitrack endpoint coordinates transformed to the robot base using
%       * the transformation matrix obtained from a previous experiment
%       * the transformation matrix recomputed
% The differences might be explained by the fact that the previous exper.
% occured only in the xz plan.
% The coordinates of the handle (real endpoint) are also provided, they 
% are computed from the same kinematic model, with a translation on z added

figure()
subplot(3,1,1)
hold on, grid on
%plot(t, opti_track_xyz(:,3));
plot(t, mocap_marker_robot_base_xyz2(:,1));
plot(t, robot_marker_xyz(:,1));
legend('mocap2 x', 'marker x')
title('Motion capture VS Direct Kinematics')
subplot(3,1,2)
hold on, grid on
%plot(t, opti_track_xyz(:,3));
plot(t, mocap_marker_robot_base_xyz2(:,2));
plot(t, robot_marker_xyz(:,2));
legend('mocap2 y', 'marker y')
subplot(3,1,3)
hold on, grid on
%plot(t, opti_track_xyz(:,3));
plot(t, mocap_marker_robot_base_xyz2(:,3));
plot(t, robot_marker_xyz(:,3));
legend('mocap2 z', 'marker z')

%% Velocities and accelerations computation

% filtering positions
freq = length(t)/t(end);
filter_order = 100;
fir_filter = fir1(filter_order, (20/(freq)), 'low');
% mocap with R 
mocap_marker_robot_base_xyz_f = filtfilt(fir_filter, 1, mocap_marker_robot_base_xyz);
% mocap with R2 
mocap_marker_robot_base_xyz2_f = filtfilt(fir_filter, 1, mocap_marker_robot_base_xyz2); 
% direct kinematic to marker 
robot_marker_xyz_f = filtfilt(fir_filter, 1, robot_marker_xyz); 
% direct kinematic to handle endpoint 
robot_handle_xyz_f = filtfilt(fir_filter, 1, robot_handle_xyz); 

% 2 points centered derivative
mocap_vel = zeros(size(mocap_marker_robot_base_xyz));
mocap_vel2 = zeros(size(mocap_marker_robot_base_xyz2));
robot_vel_m = zeros(size(robot_marker_xyz));
robot_vel_h = zeros(size(robot_handle_xyz));
for i = 2:length(t) - 1
    delta_t = 2*( t(i+1) - t(i-1) );
    mocap_vel(i, :) = ( mocap_marker_robot_base_xyz_f(i + 1, :) - mocap_marker_robot_base_xyz_f(i - 1, :) ) ./ delta_t;
    mocap_vel2(i, :) = ( mocap_marker_robot_base_xyz2_f(i + 1, :) - mocap_marker_robot_base_xyz2_f(i - 1, :) ) ./ delta_t;
    robot_vel_m(i, :) = ( robot_marker_xyz_f(i + 1, :) - robot_marker_xyz_f(i - 1, :) ) ./ delta_t;
    robot_vel_h(i, :) = ( robot_handle_xyz_f(i + 1, :) - robot_handle_xyz_f(i - 1, :) ) ./ delta_t;
end

figure()
hold on, grid on
plot(t, mocap_vel(:,3))
plot(t, mocap_vel2(:,3))
plot(t, robot_vel_m(:, 3))
plot(t, robot_vel_h(:, 3))
legend('mocap (R)', 'mocap (R2)', 'robot marker', 'robot handle')
title('Velocities on the z endpoint axis')

%% Estimation of time of interest (perturbation time list)


%% Force transformation

m = 0.1096;   %sensor mass (determined by least square method)
l = 0.0103;   %arm lever
% the arm remained without force contact for 6 secs after the begining of
% force measurement
[F_r, T_r, ft_bias] = forces_filtering(force_unf_xyz', torque_unf_xyz', thetas', t, 5e4, m, l, 'filtering', 'TRUE');

%% impedance estimation on z axis

t_eval = 200; %ms
Fz0 = zeros(length(t_dist),1);
z0_mocap2 = zeros(length(t_dist),1);
z0_rob_m = zeros(length(t_dist),1);
t0_v = zeros(length(t_dist),1);
z0_mocap2_virt = zeros(t_eval+1, length(t_dist)/2);
z0_rob_m_virt = zeros(t_eval+1, length(t_dist)/2);
Fz0_virt = zeros(t_eval+1, length(t_dist)/2);
i = 1;

for idx_pert = 1:2:length(t_dist)
    idx(i) = find(t >= t_dist(idx_pert), 1, 'first');
    Fz0(idx_pert) = F_r(3, idx(i));
    Fz0(idx_pert+1) = F_r(3, idx(i)+t_eval);
    z0_mocap2(idx_pert) = mocap_marker_robot_base_xyz2_f(idx(i), 3);
    z0_mocap2(idx_pert+1) = mocap_marker_robot_base_xyz2_f(idx(i)+t_eval, 3);
    z0_rob_m(idx_pert) = robot_marker_xyz_f(idx(i), 3);
    z0_rob_m(idx_pert+1) = robot_marker_xyz_f(idx(i)+t_eval, 3);
    t0_v(idx_pert) = t(idx(i));
    t0_v(idx_pert+1) = t(idx(i)+t_eval);
    
    % Virtual trajectory estimation, a basic interpolation is done for now
    z0_mocap2_virt(:,i) = interp1([t0_v(idx_pert), t0_v(idx_pert+1)], [z0_mocap2(idx_pert), z0_mocap2(idx_pert+1)], [t0_v(idx_pert):1e-3:t0_v(idx_pert+1)])';
    z0_rob_m_virt(:,i) = interp1([t0_v(idx_pert), t0_v(idx_pert+1)], [z0_rob_m(idx_pert), z0_rob_m(idx_pert+1)], [t0_v(idx_pert):1e-3:t0_v(idx_pert+1)])';
    Fz0_virt(:,i) = interp1([t0_v(idx_pert), t0_v(idx_pert+1)], [Fz0(idx_pert), Fz0(idx_pert+1)], [t0_v(idx_pert):1e-3:t0_v(idx_pert+1)])';
    i = i + 1;
end

% plot position, perturbation, virtual position estimation
figure()
subplot(2,1,1) 
hold on, grid on
plot(t, mocap_marker_robot_base_xyz2_f(:,3))
for i = 1:length(t_dist)
    plot([t0_v(i), t0_v(i)], [0.25, 0.4], 'k--');
end
for i = 1:length(z0_mocap2_virt(1,:))
    plot([t0_v(2*i-1):1e-3:t0_v(2*i)], z0_mocap2_virt(:,i), '*')
end
subplot(2,1,2) 
hold on, grid on
plot(t, F_r(3,:))
for i = 1:length(t_dist)
    plot([t0_v(i), t0_v(i)], [-5, 5], 'k--');
end
for i = 1:length(z0_mocap2_virt(1,:))
    plot([t0_v(2*i-1):1e-3:t0_v(2*i)], Fz0_virt(:,i), '*')
end

% phi = [Vz-Vz0, z-z0, 1] such as :
% Fz = phi * X (with X = [B; K; epsilon], the mechanical impedance param 
% of the paddle)

phi_mocap2 = {};
phi_rob_m = {};
dFz = [];

for i = 1:length(z0_mocap2_virt(1,:))
    phi_mocap2{i} = [z0_mocap2_virt(:,i) - mocap_marker_robot_base_xyz2_f(idx(i):idx(i)+t_eval, 3), ones(size(z0_mocap2_virt(:,i)))];
    phi_rob_m{i} = [z0_rob_m_virt(:,i) - robot_marker_xyz_f(idx(i):idx(i)+t_eval, 3), ones(size(z0_mocap2_virt(:,i)))];
    dFz(:,i) = F_r(3, idx:idx+t_eval)' - Fz0_virt(:,i);
end

impedance_mocap2 = zeros(2, length(z0_mocap2_virt(1,:)));
impedance_rob_m = zeros(2, length(z0_mocap2_virt(1,:))); 

for i = 1:length(phi_mocap2)
    %impedance(:,i) = prod(prod(inv(prod(phi{i}', phi{i}, 'omitnan')), phi{i}', 'omitnan'), Fz', 'omitnan');
    impedance_mocap2(:,i) =(phi_mocap2{i}'*phi_mocap2{i})\phi_mocap2{i}'*dFz(:,i);
    impedance_rob_m(:,i) = (phi_rob_m{i}'*phi_rob_m{i})\phi_rob_m{i}'*dFz(:,i);
end

% z force reconstruction
% Fz_rec = {[], [], [], []};
% for i = 1:length(phi)
%     Fz_rec{i} = phi{i}*impedance(:,i);
% end
% 
% titles = ["Motion Capture with R", "Motion Capture with R2", "Robot direct kinematic at marker", "Robot direct kinematic at handle"];
% 
% figure()
% for i = 1:length(phi)
%     subplot(length(phi), 1, i)
%     hold on, grid on
%     plot(t_rec, Fz, 'r')
%     plot(t_rec, Fz_rec{i})
%     legend('Meas.', 'Reconstr')
%     title(titles(i))
% end

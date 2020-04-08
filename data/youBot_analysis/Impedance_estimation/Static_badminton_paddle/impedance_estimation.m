clear all
close all

addpath('../../Utils');
addpath('../../Jacobian');
addpath('../../../force_torque_sensor')
addpath('../../Motion_capture_validation/utils');
load('rotation_data.mat')

folder_name = 'calibration_set_1/';

joint_states = readtable(strcat(folder_name, 'bagfile-_joint_states.csv'));
%joint_setpoints = readtable(strcat(folder_name, 'bagfile-_arm_1_joint_set_points.csv'));
mocap_marker = readtable(strcat(folder_name, 'bagfile-_vrpn_client_node_robot_joint_5_pose.csv'));
force_unf = readtable(strcat(folder_name, 'bagfile-_netft_data.csv'));
trq_cmd = readtable(strcat(folder_name, 'bagfile-_arm_1_arm_controller_torque_command.csv'));

joint_t = (joint_states.x_time - joint_states.x_time(1))*1e-9;
%joint_sp_t = (joint_setpoints.x_time - joint_states.x_time(1))*1e-9;
trq_cmd_t = (trq_cmd.x_time - joint_states.x_time(1))*1e-9;
mocap_t = (mocap_marker.x_time - joint_states.x_time(1))*1e-9;
force_unf_t = (force_unf.x_time - joint_states.x_time(1))*1e-9;
%t_end = min([joint_t(end); joint_sp_t(end); mocap_t(end); force_unf_t(end)]);
%t_start = max([joint_t(1); joint_sp_t(1); mocap_t(1); force_unf_t(1)]);
t_end = min([joint_t(end); mocap_t(end); force_unf_t(end)]);
t_start = max([joint_t(1); mocap_t(1); force_unf_t(1)]);
t = t_start:1e-3:t_end;
t = t';
%
thetas = [interp1(joint_t, joint_states.field_position0, t), interp1(joint_t, joint_states.field_position1, t), interp1(joint_t, joint_states.field_position2, t), interp1(joint_t, joint_states.field_position3, t), interp1(joint_t, joint_states.field_position4, t)];
dthetas = [interp1(joint_t, joint_states.field_velocity0, t), interp1(joint_t, joint_states.field_velocity1, t), interp1(joint_t, joint_states.field_velocity2, t), interp1(joint_t, joint_states.field_velocity3, t), interp1(joint_t, joint_states.field_velocity4, t)];
%thetas_sp = [interp1(joint_sp_t, joint_setpoints.field_position0, t), interp1(joint_sp_t, joint_setpoints.field_position1, t), interp1(joint_sp_t, joint_setpoints.field_position2, t), interp1(joint_sp_t, joint_setpoints.field_position3, t), interp1(joint_sp_t, joint_setpoints.field_position4, t)];
%eff_sp = [interp1(joint_sp_t, joint_setpoints.field_effort0, t), interp1(joint_sp_t, joint_setpoints.field_effort1, t), interp1(joint_sp_t, joint_setpoints.field_effort2, t), interp1(joint_sp_t, joint_setpoints.field_effort3, t), interp1(joint_sp_t, joint_setpoints.field_effort4, t)];
trq_cmd_jnt2 = trq_cmd.field_torques0_value;
mocap_marker_xyz = [interp1(mocap_t, mocap_marker.field_pose_position_x, t), interp1(mocap_t, mocap_marker.field_pose_position_y, t), interp1(mocap_t, mocap_marker.field_pose_position_z, t)];
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
%robot_marker_sp_xyz = zeros(length(t), 3);
alpha = zeros(length(t), 1);
thetas_offset = [169 65 -146 102.5 167.5].*pi/180;
for i=1:length(t)
    Tend = MGD_T0handle(thetas(i,1), thetas(i,2), thetas(i,3), thetas(i,4), thetas(i,5));
    T = MGD_T0marker(thetas(i,1), thetas(i,2), thetas(i,3), thetas(i,4), thetas(i,5));
    %T_sp = MGD_T0marker(thetas_sp(i,1), thetas_sp(i,2), thetas_sp(i,3), thetas_sp(i,4), thetas_sp(i,5));
    robot_handle_xyz(i, :) = Tend(1:3,4);
    robot_marker_xyz(i, :) = T(1:3,4);
    %robot_marker_sp_xyz(i, :) = T_sp(1:3,4);
end

% during the first moves, the robot endpoint was out of sight fom the motion
% capture, it happened shortly before 36s for calibration set 1
idx_in_reach = find(t >= mocap_t(find(mocap_t > 36, 1, 'first')), 1, 'first');
% idx_in_reach = 1;
% transformation recalibration is done using 34s of free movement for
% calibration set 1 else 100sec
[R2, Bfit, ErrorStats] = absor(mocap_marker_xyz(idx_in_reach:idx_in_reach+10e4,:)', robot_marker_xyz(idx_in_reach:idx_in_reach+10e4,:)');

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
plot(t, mocap_marker_robot_base_xyz(:,1));
plot(t, mocap_marker_robot_base_xyz2(:,1));
plot(t, robot_marker_xyz(:,1));
plot(t, robot_handle_xyz(:,1));
legend('mocap x', 'mocap2 x', 'marker x', 'handle x')
title('Motion capture VS Direct Kinematics')
subplot(3,1,2)
hold on, grid on
%plot(t, opti_track_xyz(:,3));
plot(t, mocap_marker_robot_base_xyz(:,2));
plot(t, mocap_marker_robot_base_xyz2(:,2));
plot(t, robot_marker_xyz(:,2));
plot(t, robot_handle_xyz(:,2));
legend('mocap y', 'mocap2 y', 'marker y', 'handle y')
subplot(3,1,3)
hold on, grid on
%plot(t, opti_track_xyz(:,3));
plot(t, mocap_marker_robot_base_xyz(:,3));
plot(t, mocap_marker_robot_base_xyz2(:,3));
plot(t, robot_marker_xyz(:,3));
plot(t, robot_handle_xyz(:,3));
legend('mocap z', 'mocap2 z', 'marker z', 'handle z')

%% Velocities and accelerations computation

% filtering positions
freq = length(t)/t(end);
filter_order = 100;
fir_filter = fir1(filter_order, (20/(freq)), 'low');
% mocap with R 
mocap_marker_robot_base_xyz_f = filtfilt(fir_filter, 1, mocap_marker_robot_base_xyz);
%mocap_marker_robot_base_xyz_f = filter(fir_filter, 1, mocap_marker_robot_base_xyz); 
%mocap_marker_robot_base_xyz_f = circshift(mocap_marker_robot_base_xyz_f, length(mocap_marker_robot_base_xyz_f)-filter_order/2, 1);
% mocap with R2 
mocap_marker_robot_base_xyz2_f = filtfilt(fir_filter, 1, mocap_marker_robot_base_xyz2); 
%mocap_marker_robot_base_xyz2_f = filter(fir_filter, 1, mocap_marker_robot_base_xyz2); 
%mocap_marker_robot_base_xyz2_f = circshift(mocap_marker_robot_base_xyz2_f, length(mocap_marker_robot_base_xyz2_f)-filter_order/2, 1);
% direct kinematic to marker 
robot_marker_xyz_f = filtfilt(fir_filter, 1, robot_marker_xyz); 
%robot_marker_xyz_f = filter(fir_filter, 1, robot_marker_xyz); 
%robot_marker_xyz_f = circshift(robot_marker_xyz_f, length(robot_marker_xyz_f)-filter_order/2, 1);
% direct kinematic to handle endpoint 
robot_handle_xyz_f = filtfilt(fir_filter, 1, robot_handle_xyz); 
%robot_handle_xyz_f = filter(fir_filter, 1, robot_handle_xyz); 
%robot_handle_xyz_f = circshift(robot_handle_xyz_f, length(robot_handle_xyz_f)-filter_order/2, 1);

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

%% Estimation of time of interest (weight of the robot on the paddle only,
% free movement only, ...)

% calibration set 1
% weight only
idx_last_weight = find(trq_cmd_jnt2 == -0.5, 1, 'first') - 1; % after this point the weight and the cmd will have an influence on measurements
t1_weight = trq_cmd_t(1);
t2_weight = trq_cmd_t(idx_last_weight);
idx_t1 = find(t >= t1_weight, 1, 'first');
idx_t2 = find(t >= t2_weight, 1, 'first');
% free movement
idx_last_static = find(t < t1_weight, 1, 'last');

% % calibration set 2
% % weight only
% idx_last_weight = find(trq_cmd_jnt2 == 2, 1, 'first') - 1; % after this point both the weight and the cmd will have an influence on measurements
% t1_weight = trq_cmd_t(1);
% t2_weight = trq_cmd_t(idx_last_weight);
% idx_t1 = find(t >= t1_weight, 1, 'first');
% idx_t2 = find(t >= t2_weight, 1, 'first');
% % free movement
% idx_last_static = 2e3; % 2sec should be enough for bias estimation

%% Force transformation

m = 0.1096;   %sensor mass (determined by least square method)
l = 0.0103;   %arm lever
[F_r, T_r, ft_bias] = forces_filtering(force_unf_xyz', torque_unf_xyz', thetas', t, idx_last_static, m, l, 'filtering', 'TRUE');

%% impedance estimation on z axis

Fz0 = nanmean(F_r(3,idx_t1:idx_t2));  
z0_mocap = nanmean(mocap_marker_robot_base_xyz_f(idx_t1:idx_t2,3));
z0_mocap2 = nanmean(mocap_marker_robot_base_xyz2_f(idx_t1:idx_t2,3));
z0_rob_m = nanmean(robot_marker_xyz_f(idx_t1:idx_t2,3));
z0_rob_h = nanmean(robot_handle_xyz_f(idx_t1:idx_t2,3));
Fz = F_r(3, idx_t2:end) - Fz0;
Fz_moc = F_r(3, idx_t2:end) - Fz0;
t_rec = t(idx_t2:end);
t_rec_moc = t(idx_t2:end);

% phi = [Vz, z-z0, 1] such as :
% Fz = phi * X (with X = [B; K; epsilon], the mechanical impedance param 
% of the paddle)
phi_mocap = [-mocap_vel(idx_t2:end, 3), (-mocap_marker_robot_base_xyz_f(idx_t2:end, 3) + z0_mocap),  ones(size(mocap_vel(idx_t2:end, 3)))];
phi_mocap2 = [-mocap_vel2(idx_t2:end, 3), (-mocap_marker_robot_base_xyz2_f(idx_t2:end, 3) + z0_mocap2),  ones(size(mocap_vel2(idx_t2:end, 3)))];
phi_rob_m = [-robot_vel_m(idx_t2:end, 3), (-robot_marker_xyz_f(idx_t2:end, 3) + z0_rob_m),  ones(size(robot_vel_m(idx_t2:end, 3)))];
phi_rob_h = [-robot_vel_h(idx_t2:end, 3), (-robot_handle_xyz_f(idx_t2:end, 3) + z0_rob_h),  ones(size(robot_vel_h(idx_t2:end, 3)))];

phi = {phi_mocap, phi_mocap2, phi_rob_m, phi_rob_h};
impedance = zeros(3,4);

for i = 1:length(phi)
    %impedance(:,i) = prod(prod(inv(prod(phi{i}', phi{i}, 'omitnan')), phi{i}', 'omitnan'), Fz', 'omitnan');
    if i <= 2
        impedance(:,i) =(phi{i}'*phi{i})\phi{i}'*Fz_moc';
    else
        impedance(:,i) =(phi{i}'*phi{i})\phi{i}'*Fz';
    end
    
end

% z force reconstruction
Fz_rec = {[], [], [], []};
for i = 1:length(phi)
    Fz_rec{i} = phi{i}*impedance(:,i);
end

titles = ["Motion Capture with R", "Motion Capture with R2", "Robot direct kinematic at marker", "Robot direct kinematic at handle"];

figure()
for i = 1:length(phi)
    subplot(length(phi), 1, i)
    hold on, grid on
    plot(t_rec, Fz, 'r')
    if i <= 2
        plot(t_rec_moc, Fz_rec{i})
    else
        plot(t_rec, Fz_rec{i})
    end
    legend('Meas.', 'Reconstr')
    title(titles(i))
end

%% Simulink RLS

phi_sim_mocap = timeseries(phi{2}, t_rec - t_rec(1));
phi_sim_marker = timeseries(phi{3}, t_rec - t_rec(1));
Fz_sim = timeseries(Fz', t_rec - t_rec(1));

sim('rls_sim.slx')

imp_rls = {[], []};
Fz_rec_rls = {[], [],};
imp_rls{1} = mocap_imp;
imp_rls{2} = marker_imp;
Fz_rec_rls{1} = sum(phi{2}.*imp_rls{1}, 2);
Fz_rec_rls{2} = sum(phi{3}.*imp_rls{2}, 2);
titles_rls = [titles(2); titles(3)];

figure()
for i = 1:length(imp_rls)
    subplot(length(imp_rls)+1, length(imp_rls), i)
    hold on, grid on
    plot(t_rec, Fz, 'r')
    plot(t_rec, Fz_rec{i+1})
    legend('Meas.', 'Reconstr')
    title(titles_rls(i))
end
for i = 1:length(imp_rls)
    subplot(length(imp_rls)+1, length(imp_rls), i+length(imp_rls))
    hold on, grid on
    plot(t_rec, Fz, 'r')
    plot(t_rec, Fz_rec_rls{i})
    legend('Meas.', 'Reconstr')
    title(titles_rls(i))
end   
for i = 1:length(imp_rls)
    subplot(length(imp_rls)+1, length(imp_rls), i+2*length(imp_rls))
    hold on, grid on
    plot(t_rec, imp_rls{i}(:,2), 'b')
    line([t_rec(1), t_rec(end)], [impedance(2, i+1), impedance(2, i+1)])
    legend('Stiffness estimation RLS', 'Stiffness estimation LSQ')
    title(titles_rls(i))
end  

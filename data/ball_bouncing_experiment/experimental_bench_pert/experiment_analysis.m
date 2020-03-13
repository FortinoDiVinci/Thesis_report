clear all
close all

addpath('../../youBot_analysis/Utils');
addpath('../../youBot_analysis/Jacobian');
addpath('../../youBot_analysis/Motion_capture_validation/utils');
addpath('../../force_torque_sensor')

%% kinematic data, ball and paddle trajectories

dt = 1e-3;

folder_names = ["pert_10/", "pert_15bis/", "pert_15ter/", "pert_10/"];

for fld_idx = 1:length(folder_names)

    joint_states = readtable(strcat(folder_names(fld_idx), 'bagfile-_joint_states.csv'));
    mocap = readtable(strcat(folder_names(fld_idx), 'bagfile-_vrpn_client_node_robot_marker_pose.csv'));
    %tf = readtable(strcat(folder_names(fld_idx), 'bagfile-_tf_sensor.csv'));
    ball_pose = readtable(strcat(folder_names(fld_idx), 'bagfile-_ball_pose.csv'));
    force_unf = readtable(strcat(folder_names(fld_idx), 'bagfile-_netft_data.csv'));
    disturbance = readtable(strcat(folder_names(fld_idx), 'bagfile-_arm_1_disturbance_val.csv'));
    
    % time interpolation
    joint_t = (joint_states.x_time - joint_states.x_time(1))*1e-9;
    mocap_t = (mocap.x_time - joint_states.x_time(1))*1e-9;
    force_unf_t = (force_unf.x_time - joint_states.x_time(1))*1e-9;
    ball_pose_t = (ball_pose.x_time - joint_states.x_time(1))*1e-9;
    %tf_t = (tf.x_time - joint_states.x_time(1))*1e-9;
    
    %t_end = min([joint_t(end); mocap_t(end); force_unf_t(end); ball_pose_t(end); tf_t(end)]);
    %t_start = max([joint_t(1); mocap_t(1); force_unf_t(1); ball_pose_t(1); tf_t(1)]);
    t_end = min([joint_t(end); mocap_t(end); force_unf_t(end); ball_pose_t(end)]);
    t_start = max([joint_t(1); mocap_t(1); force_unf_t(1); ball_pose_t(1)]);
    t{fld_idx} = t_start:dt:t_end;
    t{fld_idx} = t{fld_idx}';
    t_free_mov{fld_idx} = (max(joint_t(1), mocap_t(1)):dt:t_start)';
    t_dist{fld_idx} = (disturbance.x_time - joint_states.x_time(1))*1e-9;
    t_dist{fld_idx} = t_dist{fld_idx}(1:end-2);
    
    z_b{fld_idx} = interp1(ball_pose_t, ball_pose.field_pose_position_z, t{fld_idx});
    %[~, un_idx] = unique(tf_t);
    %z_p{fld_idx} = (interp1(tf_t(un_idx), tf.field_transforms0_transform_translation_z(un_idx), t{fld_idx}) - 0.3) * 3;
    
    thetas{fld_idx} = [interp1(joint_t, joint_states.field_position0, t{fld_idx}), ...
                       interp1(joint_t, joint_states.field_position1, t{fld_idx}), ...
                       interp1(joint_t, joint_states.field_position2, t{fld_idx}), ...
                       interp1(joint_t, joint_states.field_position3, t{fld_idx}), ...
                       interp1(joint_t, joint_states.field_position4, t{fld_idx})];
    
    thetas_fm{fld_idx} = [interp1(joint_t, joint_states.field_position0, t_free_mov{fld_idx}), ...
                       interp1(joint_t, joint_states.field_position1, t_free_mov{fld_idx}), ...
                       interp1(joint_t, joint_states.field_position2, t_free_mov{fld_idx}), ...
                       interp1(joint_t, joint_states.field_position3, t_free_mov{fld_idx}), ...
                       interp1(joint_t, joint_states.field_position4, t_free_mov{fld_idx})];
                           
	mocap_marker{fld_idx} = [interp1(mocap_t, mocap.field_pose_position_x, t{fld_idx}), ...
                            interp1(mocap_t, mocap.field_pose_position_y, t{fld_idx}), ...
                            interp1(mocap_t, mocap.field_pose_position_z, t{fld_idx})];
                        
	mocap_marker_fm{fld_idx} = [interp1(mocap_t, mocap.field_pose_position_x, t_free_mov{fld_idx}), ...
                                interp1(mocap_t, mocap.field_pose_position_y, t_free_mov{fld_idx}), ...
                                interp1(mocap_t, mocap.field_pose_position_z, t_free_mov{fld_idx})];
    
    forces_unf{fld_idx} = [interp1(force_unf_t, force_unf.field_wrench_force_x, t{fld_idx}), ...
                           interp1(force_unf_t, force_unf.field_wrench_force_y, t{fld_idx}), ...
                           interp1(force_unf_t, force_unf.field_wrench_force_z, t{fld_idx})]; 
                       
    torques_unf{fld_idx} = [interp1(force_unf_t, force_unf.field_wrench_torque_x, t{fld_idx}), ...
                           interp1(force_unf_t, force_unf.field_wrench_torque_y, t{fld_idx}), ...
                           interp1(force_unf_t, force_unf.field_wrench_torque_z, t{fld_idx})]; 
                            
    for i=1:length(t{fld_idx})
        T = MGD_T0handle(thetas{fld_idx}(i,1), thetas{fld_idx}(i,2), thetas{fld_idx}(i,3), thetas{fld_idx}(i,4), thetas{fld_idx}(i,5));
        z_p{fld_idx}(i, 1) = (T(3,4) - 0.3)*3;
    end
end

%% Optitrack VS Robot Endpoint

for fld_idx = 1:length(folder_names)
    % endpoint computation
    robot_marker{fld_idx} = zeros(length(t{fld_idx}), 3);
    robot_marker_fm{fld_idx} = zeros(length(t_free_mov{fld_idx}), 3);

    for i=1:length(t{fld_idx})
        T = MGD_T0marker(thetas{fld_idx}(i,1), thetas{fld_idx}(i,2), thetas{fld_idx}(i,3), thetas{fld_idx}(i,4), thetas{fld_idx}(i,5));
        robot_marker{fld_idx}(i, :) = T(1:3,4);
    end
    for i=1:length(t_free_mov{fld_idx})
        T_fm = MGD_T0marker(thetas_fm{fld_idx}(i,1), thetas_fm{fld_idx}(i,2), thetas_fm{fld_idx}(i,3), thetas_fm{fld_idx}(i,4), thetas_fm{fld_idx}(i,5)); % free move
        robot_marker_fm{fld_idx}(i, :) = T_fm(1:3,4);
    end
    % transformation recalibration is done using free movement
    [R2, Bfit, ErrorStats] = absor(mocap_marker_fm{fld_idx}', robot_marker_fm{fld_idx}');

    mocap_marker_robot_base_fm{fld_idx} = zeros(size(mocap_marker_fm{fld_idx}));
    for i=1:length(t_free_mov{fld_idx})
        temp_hom = R2.M * [mocap_marker_fm{fld_idx}(i,:), 1]';
        mocap_marker_robot_base_fm{fld_idx}(i,:) = temp_hom(1:3);
    end
    
    figure()
    hold on, grid on
    %plot(t, opti_track_xyz(:,3));
    plot(t_free_mov{fld_idx}, mocap_marker_robot_base_fm{fld_idx}(:,3));
    plot(t_free_mov{fld_idx}, robot_marker_fm{fld_idx}(:,3));
    legend('mocap z', 'marker z')
    title('Motion capture and Direct Kinematics fitting in free movement')    
    
    mocap_marker_robot_base{fld_idx} = zeros(size(mocap_marker{fld_idx}));
    for i=1:length(t{fld_idx})
        temp_hom = R2.M * [mocap_marker{fld_idx}(i,:), 1]';
        mocap_marker_robot_base{fld_idx}(i,:) = temp_hom(1:3);
    end

    figure()
    hold on, grid on
    %plot(t, opti_track_xyz(:,3));
    plot(t{fld_idx}, mocap_marker_robot_base{fld_idx}(:,3));
    plot(t{fld_idx}, robot_marker{fld_idx}(:,3));
    legend('mocap z', 'marker z')
    title('Motion capture VS Direct Kinematics')
end

fld_idx = 3; % successful camera kinematic data calibration

%% ball and paddle 

% ball velocity
vz_b{fld_idx} = zeros(size(z_b{fld_idx}));
for i = 2:length(t{fld_idx}) - 1
    vz_b{fld_idx}(i) = ( z_b{fld_idx}(i + 1) - z_b{fld_idx}(i - 1) ) / (2*dt);
end
vz_b{fld_idx}(1) = vz_b{fld_idx}(2);
vz_b{fld_idx}(end) = vz_b{fld_idx}(end-1);

filter_order = 100;
fir_filter = fir1(filter_order, (30/(1/dt/2)), 'low');
vz_b_fil{fld_idx} = filtfilt(fir_filter, 1, vz_b{fld_idx});

% impact detection
idx_ball_on_ramp = find(z_b{fld_idx} ~= 1, 1, 'first');
idx_ball_off_ramp = find(z_b{fld_idx}(idx_ball_on_ramp:end) < 1, 1, 'first') + idx_ball_on_ramp;

j = 1;
impact = [];
idx_window = 100;
i = idx_ball_off_ramp;
while( i < length(t{fld_idx}) )
    a = detect_impact(vz_b_fil{fld_idx}, i, idx_window);
    if a == 0
        break
    end
    impact(j, 1) = a;
    impact(j, 2) = t{fld_idx}(a);
    j = j + 1;
    i = a + 3*idx_window;
end

% %Velocity and impact detection
% figure()
% hold on, grid on
% plot(t{fld_idx}, vz_b_fil{fld_idx})
% line([impact(:,2), impact(:,2)], [-5, 5], 'Color','black','LineStyle','--');

% Disturbance 
t_10th_impact = impact(10,2);
t_positiv_dist{fld_idx} = t_dist{fld_idx}((find(disturbance.field_data(1:end-2) > 0))');
t_negativ_dist{fld_idx} = t_dist{fld_idx}((find(disturbance.field_data(1:end-2) < 0))');
t_off_dist{fld_idx} = t_dist{fld_idx}(find((disturbance.field_data(1:end-2) == 0))');

figure()
hold on, grid on
plot(t{fld_idx}, z_b{fld_idx}, 'r')
plot(t{fld_idx}, z_p{fld_idx}, 'b')
line([t_positiv_dist{fld_idx}, t_positiv_dist{fld_idx}], [-0.2, 0.2], 'Color','green','LineStyle','--');
line([t_negativ_dist{fld_idx}, t_negativ_dist{fld_idx}], [-0.2, 0.2], 'Color','red','LineStyle','--');
line([t_off_dist{fld_idx}, t_off_dist{fld_idx}], [-0.2, 0.2], 'Color','black','LineStyle','--');
line([t{fld_idx}(idx_ball_on_ramp), t_10th_impact], [0.8, 0.8], 'Color','black','LineStyle','--');
line([t_10th_impact, t{fld_idx}(end)], [1.00, 1.00], 'Color','black','LineStyle','--');
legend('ball', 'paddle')
xlabel('time (s)')
ylabel('height (m)')

%% Forces

m = 0.1096;   %sensor mass (determined by least square method)
l = 0.0103;   %arm lever
[F_r{fld_idx}, T_r{fld_idx}, ft_bias{fld_idx}] = forces_filtering(forces_unf{fld_idx}', torques_unf{fld_idx}', thetas{fld_idx}', t{fld_idx}, 2e3, m, l, 'filtering', 'TRUE');

figure()
subplot(2,1,1)
hold on, grid on
plot(t{fld_idx}, z_p{fld_idx})
line([t_positiv_dist{fld_idx}, t_positiv_dist{fld_idx}], [-0.2, 0.2], 'Color','green','LineStyle','--');
line([t_negativ_dist{fld_idx}, t_negativ_dist{fld_idx}], [-0.2, 0.2], 'Color','red','LineStyle','--');
line([t_off_dist{fld_idx}, t_off_dist{fld_idx}], [-0.2, 0.2], 'Color','black','LineStyle','--');
ylabel('height (m)')
legend('paddle')
subplot(2,1,2)
hold on, grid on
plot(t{fld_idx}, F_r{fld_idx}(3,:))
line([t_positiv_dist{fld_idx}, t_positiv_dist{fld_idx}], [-10, 10], 'Color','green','LineStyle','--');
line([t_negativ_dist{fld_idx}, t_negativ_dist{fld_idx}], [-10, 10], 'Color','red','LineStyle','--');
line([t_off_dist{fld_idx}, t_off_dist{fld_idx}], [-10, 10], 'Color','black','LineStyle','--');
ylabel('force (N)')
xlabel('time (s)')
legend('interaction forces')

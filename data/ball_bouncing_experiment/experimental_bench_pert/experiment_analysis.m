clear all
close all

addpath('../../youBot_analysis/Utils');
addpath('../../youBot_analysis/Jacobian');
addpath('../../youBot_analysis/Motion_capture_validation/utils');
addpath('../../force_torque_sensor')

%%%%%%%%%%%%%%%%%%
%% MACROS & variables
%%%%%%%%%%%%%%%%%%

DISPLAY_MOCAP_FIT                   = 1
DISPLAY_BALL_BOUNCING_IMPACTS       = 0
SINGLE_MOCAP_FITTING                = 1
IS_BALL_BOUNCING                    = 0
USE_DEFAULT_TF_MATRIX               = 0
SAVE_DATA                           = 1 % specify name for the file

%% kinematic data, ball and paddle trajectories

dt = 1e-3;

%folder_names = ["pert_10/", "pert_15bis/", "pert_15ter/", "pert_10/"];
%folder_names = ["impact/"];
%folder_names = "2020_06_17/" + ["impact_2/","impact_debug/","impact_reduced/","impact_reduced_dist/"];
%names = ["antonello_v/", "baptiste_b/", "cristina_m/1/", "cristina_m/2/", "cristina_v/", "joy_f/", "maria_m/1/", "maria_m/2/", "martin_s/", "remi_a/", "sorin_o/", "thomas_c/"];
%names = "calibrated_spring/" + ["1/", "2/", "3/", "4/"];
%names = "calibrated_environnement/calibrated_spring_" + ["1/", "2/", "3/", "4/", "5/", "6/"];
%names = ["vincent_f/"];
names = "data_02-Sep-2020_17h45/" +["no_pert/", "long_pert/", "short_pert/", "spring_no_pert/", "spring_long_pert/", "spring_long_pert2/", "spring_short_pert/"];
folder_names = "preliminary_experimental_data/" + names;
if SAVE_DATA
    saved_data_name = "data_eval_pert";
end

NO_DISTURBANCE = cell(size(folder_names));
NO_DISTURBANCE(:,:) = {0};
NO_TRQ_CMD_DIST = cell(size(folder_names)); % long period torque cmd
NO_TRQ_CMD_DIST (:,:) = {0};
NO_IMPULSE = cell(size(folder_names));
NO_IMPULSE(:,:) = {0};
NO_MOCAP = cell(size(folder_names));
NO_MOCAP(:,:) = {0};
NO_BALL_BOUNC = cell(size(folder_names));
NO_BALL_BOUNC(:,:) = {0};

%kinematic_coeff = [6, 6, 6, 6, 6, 6, 6, 6, 3, 6, 6, 3];


for fld_idx = 1:length(folder_names)

    joint_states = readtable(strcat(folder_names(fld_idx), 'bagfile-_joint_states.csv'));
    try
    mocap = readtable(strcat(folder_names(fld_idx), 'bagfile-_vrpn_client_node_robot_marker_pose.csv'));
    catch
        NO_MOCAP{fld_idx} = 1;
    end
    %tf = readtable(strcat(folder_names(fld_idx), 'bagfile-_tf_sensor.csv'));
    try
        ball_pose = readtable(strcat(folder_names(fld_idx), 'bagfile-_ball_pose.csv'));
    catch
        NO_BALL_BOUNC{fld_idx} = 1;
    end
    force_unf = readtable(strcat(folder_names(fld_idx), 'bagfile-_netft_data.csv'));
    try
        disturbance = readtable(strcat(folder_names(fld_idx), 'bagfile-_arm_1_disturbance_val.csv'));
    catch
        NO_DISTURBANCE{fld_idx} = 1;
    end
    try
        torque_cmd = readtable(strcat(folder_names(fld_idx), 'bagfile-_arm_1_arm_controller_torque_command.csv'));
    catch
        NO_TRQ_CMD_DIST{fld_idx} = 1;
    end
    try
        impulse = readtable(strcat(folder_names(fld_idx), 'bagfile-_impulse.csv'));
    catch
        NO_IMPULSE{fld_idx} = 1;
    end
        
    % time interpolation
    force_unf_t = (force_unf.x_time - joint_states.x_time(1))*1e-9;
    if ~NO_BALL_BOUNC{fld_idx}
        ball_pose_t = (ball_pose.x_time - joint_states.x_time(1))*1e-9;
    end
    %tf_t = (tf.x_time - joint_states.x_time(1))*1e-9;
    joint_t = (joint_states.x_time - joint_states.x_time(1))*1e-9;
    if NO_MOCAP{fld_idx} == 0
        mocap_t = (mocap.x_time - joint_states.x_time(1))*1e-9;
        if NO_BALL_BOUNC{fld_idx}
            t_end = min([joint_t(end); mocap_t(end); force_unf_t(end)]);
            t_start = max([joint_t(1); mocap_t(1); force_unf_t(1)]);
        else
            t_end = min([joint_t(end); mocap_t(end); force_unf_t(end); ball_pose_t(end)]);
            t_start = max([joint_t(1); mocap_t(1); force_unf_t(1); ball_pose_t(1)]);
        end
        t_free_mov{fld_idx} = (max(joint_t(1), mocap_t(1)):dt:t_start)';
    else
        if NO_BALL_BOUNC{fld_idx}
            t_end = min([joint_t(end); force_unf_t(end)]);
            t_start = max([joint_t(1); force_unf_t(1)]);   
        else
            t_end = min([joint_t(end); force_unf_t(end); ball_pose_t(end)]);
            t_start = max([joint_t(1); force_unf_t(1); ball_pose_t(1)]);
        end    
        t_free_mov{fld_idx} = (joint_t(1):dt:t_start)';
    end
    
    t{fld_idx} = t_start:dt:t_end;
    t{fld_idx} = t{fld_idx}';
    
    if ~NO_DISTURBANCE{fld_idx}
        t_dist{fld_idx} = (disturbance.x_time - joint_states.x_time(1))*1e-9;
        dist{fld_idx} = disturbance.field_data;
        %t_dist{fld_idx} = t_dist{fld_idx}(1:end-2);
    end
    if ~NO_TRQ_CMD_DIST{fld_idx}
        t_trq_cmd{fld_idx} = (torque_cmd.x_time - joint_states.x_time(1))*1e-9;
        val_trq_cmd{fld_idx} = torque_cmd.field_torques0_value;
        %t_dist{fld_idx} = t_dist{fld_idx}(1:end-2);
    end
    if ~NO_IMPULSE{fld_idx}
        t_impulse{fld_idx} = (impulse.x_time - joint_states.x_time(1))*1e-9;
        imp{fld_idx} = impulse.field_data;
    end
    
    if ~NO_BALL_BOUNC{fld_idx}
        z_b{fld_idx} = interp1(ball_pose_t, ball_pose.field_pose_position_z, t{fld_idx});
    end
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

    joint_eff{fld_idx} = [interp1(joint_t, joint_states.field_effort0, t{fld_idx}), ...
                       interp1(joint_t, joint_states.field_effort1, t{fld_idx}), ...
                       interp1(joint_t, joint_states.field_effort2, t{fld_idx}), ...
                       interp1(joint_t, joint_states.field_effort3, t{fld_idx}), ...
                       interp1(joint_t, joint_states.field_effort4, t{fld_idx})];
    
    joint_eff_fm{fld_idx} = [interp1(joint_t, joint_states.field_effort0, t_free_mov{fld_idx}), ...
                       interp1(joint_t, joint_states.field_effort1, t_free_mov{fld_idx}), ...
                       interp1(joint_t, joint_states.field_effort2, t_free_mov{fld_idx}), ...
                       interp1(joint_t, joint_states.field_effort3, t_free_mov{fld_idx}), ...
                       interp1(joint_t, joint_states.field_effort4, t_free_mov{fld_idx})];
    
    if ~NO_MOCAP{fld_idx}
        mocap_marker{fld_idx} = [interp1(mocap_t, mocap.field_pose_position_x, t{fld_idx}), ...
                            interp1(mocap_t, mocap.field_pose_position_y, t{fld_idx}), ...
                            interp1(mocap_t, mocap.field_pose_position_z, t{fld_idx})];
                        
        mocap_marker_fm{fld_idx} = [interp1(mocap_t, mocap.field_pose_position_x, t_free_mov{fld_idx}), ...
                                interp1(mocap_t, mocap.field_pose_position_y, t_free_mov{fld_idx}), ...
                                interp1(mocap_t, mocap.field_pose_position_z, t_free_mov{fld_idx})];
    end
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
    %disp('iter: ' + string(fld_idx))
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
    
    if USE_DEFAULT_TF_MATRIX
      transformation_matrix{fld_idx} = [0.9939    0.0214    0.1082   -0.1188
                                       0.1085   -0.0115   -0.9940    0.3065
                                      -0.0200    0.9997   -0.0138    0.0513
                                       0         0         0    1.0000];
    
    elseif SINGLE_MOCAP_FITTING & fld_idx ~= 1
        transformation_matrix{fld_idx} = transformation_matrix{1};
    else
        
        if NO_MOCAP{fld_idx}
            disp(fld_idx)
            continue
        end        
        %disp('mocap: ' + string(fld_idx))
        
        % transformation recalibration is done using free movement
        [R2, Bfit, ErrorStats] = absor(mocap_marker_fm{fld_idx}', robot_marker_fm{fld_idx}');

        transformation_matrix{fld_idx} = R2.M;

        mocap_marker_robot_base_fm{fld_idx} = zeros(size(mocap_marker_fm{fld_idx}));
        for i=1:length(t_free_mov{fld_idx})
            temp_hom = R2.M * [mocap_marker_fm{fld_idx}(i,:), 1]';
            mocap_marker_robot_base_fm{fld_idx}(i,:) = temp_hom(1:3);
        end

        if DISPLAY_MOCAP_FIT 

            figure()
            hold on, grid on
            %plot(t, opti_track_xyz(:,3));
            plot(t_free_mov{fld_idx}, mocap_marker_robot_base_fm{fld_idx}(:,3));
            plot(t_free_mov{fld_idx}, robot_marker_fm{fld_idx}(:,3));
            legend('mocap z', 'marker z')
            title('Motion capture and Direct Kinematics fitting in free movement')   
        end      
    end

    mocap_marker_robot_base{fld_idx} = zeros(size(mocap_marker{fld_idx}));
    for i=1:length(t{fld_idx})
        temp_hom = transformation_matrix{fld_idx} * [mocap_marker{fld_idx}(i,:), 1]';
        mocap_marker_robot_base{fld_idx}(i,:) = temp_hom(1:3);
    end

    if DISPLAY_MOCAP_FIT 

        figure()
        hold on, grid on
        %plot(t, opti_track_xyz(:,3));
        plot(t{fld_idx}, mocap_marker_robot_base{fld_idx}(:,3));
        plot(t{fld_idx}, robot_marker{fld_idx}(:,3));
        legend('mocap z', 'marker z')
        title('Motion capture VS Direct Kinematics: ' + names(fld_idx))
        
    end
end


%fld_idx = 3; % successful camera kinematic data calibration

%% ball and paddle 

if IS_BALL_BOUNCING

    for fld_idx = 1:length(folder_names)

        if NO_BALL_BOUNC{fld_idx}
            continue
        end

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
        idx_ball_off_ramp{fld_idx} = find(z_b{fld_idx}(idx_ball_on_ramp:end) < 1, 1, 'first') + idx_ball_on_ramp;

        j = 1;
        impact = [];
        idx_window = 100;
        i = idx_ball_off_ramp{fld_idx};
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
        t_positiv_dist{fld_idx} = t_dist{fld_idx}((find(dist{fld_idx} > 0))');
        t_negativ_dist{fld_idx} = t_dist{fld_idx}((find(dist{fld_idx} < 0))');
        t_off_dist{fld_idx} = t_dist{fld_idx}(find((dist{fld_idx} == 0))');

        if DISPLAY_BALL_BOUNCING_IMPACTS

            figure()
            hold on, grid on
            plot(t{fld_idx}, z_b{fld_idx}, 'r')
            plot(t{fld_idx}, z_p{fld_idx}, 'b')
            if ~NO_MOCAP{fld_idx}            
                plot(t{fld_idx}, (mocap_marker_robot_base{fld_idx}(:,3)-0.3)*kinematic_coeff(fld_idx), 'c')
            end
            line([t_positiv_dist{fld_idx}, t_positiv_dist{fld_idx}], [-0.2, 0.2], 'Color','green','LineStyle','--');
            line([t_negativ_dist{fld_idx}, t_negativ_dist{fld_idx}], [-0.2, 0.2], 'Color','red','LineStyle','--');
            line([t_off_dist{fld_idx}, t_off_dist{fld_idx}], [-0.2, 0.2], 'Color','black','LineStyle','--');
            line([t{fld_idx}(idx_ball_on_ramp), t_10th_impact], [0.8, 0.8], 'Color','black','LineStyle','--');
            line([t_10th_impact, t{fld_idx}(end)], [1.00, 1.00], 'Color','black','LineStyle','--');
            legend('ball', 'paddle')
            xlabel('time (s)')
            ylabel('height (m)')
            title('Ball bouncing impacts: ' + names(fld_idx))

        end

    end

end

if SAVE_DATA
    if exist(strcat(saved_data_name,".mat"), "file")
        warning('The file ' + saved_data_name + ".mat, already exists.")
        str_in = input('Do you really want to erase it ?','s');
        if str_in ~= "yes" && str_in ~= "YES" && str_in ~= "Yes" && str_in ~= "Y" && str_in ~= "y"
            return
        end
    end
    save(strcat(saved_data_name,".mat"),"dist", "dt", "folder_names", "forces_unf", "joint_eff", ...
        "joint_eff_fm", "mocap_marker", "mocap_marker_fm", "mocap_marker_robot_base", ...
        "mocap_marker_robot_base_fm", "names", "NO_BALL_BOUNC", "NO_DISTURBANCE", "NO_IMPULSE", ...
        "NO_MOCAP", "NO_TRQ_CMD_DIST", "robot_marker", "robot_marker_fm", "t", "t_dist", "thetas", ...
        "thetas_fm", "torques_unf", "transformation_matrix", "z_b", "z_p");
end

return

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

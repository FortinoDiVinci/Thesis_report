clear all

addpath('utils')
addpath('../../data/utils')
addpath('../../data/force_torque_sensor')

%%%%%%%%%%%%%%%%%%
%% MACROS & variables
%%%%%%%%%%%%%%%%%%
NB_JOINTS = 5;
DISPLAY_MOCAP_FIT = 1;
first_exp2_user029 = 0; % to deal with missing topic

%% File selection
files = dir('users/*/*.bag');
is_calibration = zeros(size(files));
is_exp = zeros(size(files));
is_learning_ball_bouncing = zeros(size(files));
is_learning_phri = zeros(size(files));
%bagselect = rosbag('vfo10.bag');
for ii = 1:length(files)
    if contains(files(ii).name, "calibration")
        is_calibration(ii) = 1;
        date_time(ii) = datenum(files(ii).name(13:end-4),'yyyy-mm-dd-HH-MM-SS');
    elseif contains(files(ii).name, "exp_1")
        is_exp(ii) = 1;
        date_time(ii) = datenum(files(ii).name(7:end-4),'yyyy-mm-dd-HH-MM-SS');
    elseif contains(files(ii).name, "exp_2")
        is_exp(ii) = 2;
        date_time(ii) = datenum(files(ii).name(7:end-4),'yyyy-mm-dd-HH-MM-SS');
    elseif contains(files(ii).name, "l_ball")
        is_learning_ball_bouncing(ii) = 1;
        date_time(ii) = datenum(files(ii).name(17:end-4),'yyyy-mm-dd-HH-MM-SS');
    elseif contains(files(ii).name, "l_phri")
        is_learning_phri(ii) = 1;
        date_time(ii) = datenum(files(ii).name(8:end-4),'yyyy-mm-dd-HH-MM-SS');
    end
end

[~, chron_order] = sort(date_time);
files = files(chron_order);
is_calibration = is_calibration(chron_order);
is_exp = is_exp(chron_order);

%% Data reading
for idx = 1:length(files)
    file_path = files(idx).folder + "\" + files(idx).name;
    bagselect = rosbag(file_path);
    %% Topic extraction
    joint_data = readMessages(select(bagselect,'Topic','/joint_states'),...
        'DataFormat','struct');
    ft_sensor_data = readMessages(select(bagselect,'Topic','/netft_data'),...
        'DataFormat','struct');
    motion_capture_data = readMessages(select(bagselect,'Topic',...
        '/vrpn_client_node/robot_marker/pose'),'DataFormat','struct');
    disturbance_data = readMessages(select(bagselect,'Topic', ...
        '/arm_1/disturbance_val'),'DataFormat','struct');
    ball_data = readMessages(select(bagselect,'Topic','/ball_pose'),...
        'DataFormat','struct');
    parameters_data = readMessages(select(bagselect,'Topic',...
        '/ball_simulator/parameter_updates'),'DataFormat','struct');

    %% Time extraction
    t_date{idx} = datetime(bagselect.StartTime,'ConvertFrom','epochtime','Format',...
        'dd-MMM-yyyy HH:mm:ss');
    t_q{idx} = select(bagselect,'Topic','/joint_states').MessageList.Time;
    t_ft{idx} = select(bagselect,'Topic','/netft_data').MessageList.Time;
    t_mc{idx} = select(bagselect,'Topic','/vrpn_client_node/robot_marker/pose').MessageList.Time;
    t_d{idx} = select(bagselect,'Topic','/arm_1/disturbance_val').MessageList.Time;
    t_b{idx} = select(bagselect,'Topic','/ball_pose').MessageList.Time;

    clear bagselect

    %% Topic data extraction
    for ii = 1:NB_JOINTS
        raw_q{idx}(:,ii) = cellfun(@(x) double(x.Position(ii)), joint_data);
    end
    clear joint_data
    try
        raw_force{idx}(:,1) = cellfun(@(x) double(x.Wrench.Force.X), ft_sensor_data);
        raw_force{idx}(:,2) = cellfun(@(x) double(x.Wrench.Force.Y), ft_sensor_data);
        raw_force{idx}(:,3) = cellfun(@(x) double(x.Wrench.Force.Z), ft_sensor_data);
        raw_torque{idx}(:,1) = cellfun(@(x) double(x.Wrench.Torque.X), ft_sensor_data);
        raw_torque{idx}(:,2) = cellfun(@(x) double(x.Wrench.Torque.Y), ft_sensor_data);
        raw_torque{idx}(:,3) = cellfun(@(x) double(x.Wrench.Torque.Z), ft_sensor_data);
    catch
        % no force data
        raw_force{idx} = [];
        raw_torque{idx} = [];
        if ~is_calibration(idx)
            warning("Missing force torque data for user " + string(files(idx).folder(end-2:end)) + ...
                ", in file: " + string(files(idx).name));
        end
    end
    clear ft_sensor_data

    raw_mocap{idx}(:,1) = cellfun(@(x) double(x.Pose.Position.X), motion_capture_data);
    raw_mocap{idx}(:,2) = cellfun(@(x) double(x.Pose.Position.Y), motion_capture_data);
    raw_mocap{idx}(:,3) = cellfun(@(x) double(x.Pose.Position.Z), motion_capture_data);
    clear motion_capture_data 

    dist_val{idx} = cellfun(@(x) double(x.Data), disturbance_data);
    clear disturbance_data

    raw_ball_z{idx} = cellfun(@(x) double(x.Pose.Position.Z), ball_data);
    clear ball_data

    field_name{1} = 'user';
    field_value{1} = files(idx).folder(end-2:end);
    field_name{2} = 'experience';
    if is_exp(idx) == 1
        field_value{2} = "exp_1";
    elseif is_exp(idx) == 2
        field_value{2} = "exp_2";
    elseif is_calibration(idx)
        field_value{2} = "calib";
    elseif is_learning_phri(idx)
        field_value{2} = "l_phri";
    elseif is_learning_ball_bouncing(idx)
        field_value{2} = "l_bb";
    else
        field_value{2} = "unknown";
    end
    try
        for i = 1:length(parameters_data{end}.Doubles)
            field_name{i+2} = parameters_data{end}.Doubles(i).Name;
            field_value{i+2} = parameters_data{end}.Doubles(i).Value(end);
        end
    catch
        warning('Missing /ball_simulator/parameter_updates topic');
        % This user data parameter topic is missing (manually fed)
        if strcmp(field_value{1}, '029')
            field_name{3} = 'restitution';
            field_name{4} = 'ball_mass';
            field_name{5} = 'paddle_mass';
            field_name{6} = 'scale';
            field_name{7} = 'gravity';
            field_name{8} = 'radius';
            field_name{9} = 'target_height';
            field_name{10} = 'paddle_frequency'; % unused
            field_name{11} = 'paddle_amplitude'; % unused
            
            field_value{3} = 0.6;
            field_value{4} = 0.058;
            field_value{5} = 1;
            field_value{6} = 6;
            field_value{7} = 9.807;
            field_value{8} = 0.325;
            field_value{10} = 0.5; % unused
            field_value{11} = 0.3; % unused
            if is_calibration(idx)
                field_value{9} = NaN;
            elseif is_exp(idx) == 1 || is_learning_phri(idx) || is_learning_ball_bouncing(idx)
                field_value{9} = 1.75;
            elseif is_exp(idx) == 2
                if first_exp2_user029 
                    field_value{9} = 1.5;
                else
                    field_value{9} = 2.0;
                end
                first_exp2_user029 = 1;
            end
        end    
    end
    exp_parameters(idx) = cell2struct(field_value, field_name, 2);
    clear parameters_data field_value field_name

end

return

%% Synchronisation
% Time synchronisation
for idx = 1:length(files)
    dt = 1e-3;
    t_start(idx) = max([t_q{idx}(1); t_mc{idx}(1); t_ft{idx}(1); t_b{idx}(1)]);
    t_end(idx) = min([t_q{idx}(end); t_mc{idx}(end); t_ft{idx}(end); t_b{idx}(end)]);
    t{idx} = (t_start(idx):dt:t_end(idx))';

    for ii = 1:NB_JOINTS
        q{idx}(:,ii) = interp1(t_q{idx}, raw_q{idx}(:,ii), t{idx});
    end
    for ii = 1:3
        mocap{idx}(:,ii) = interp1(t_mc{idx}, raw_mocap{idx}(:,ii), t{idx});
    end
    for ii = 1:3
        ft_sensor{idx}(:,ii) = interp1(t_ft{idx}, raw_force{idx}(:,ii), t{idx});
        ft_sensor{idx}(:,ii+3) = interp1(t_ft{idx}, raw_torque{idx}(:,ii), t{idx});
    end
    z_b{idx} = interp1(t_b{idx}, raw_ball_z{idx}, t{idx});
end
clear t_q t_mc t_ft t_b

%% spatial synchronisation
% between motion capture coordinates and robot coordinates
for idx = 1:length(files)
    for i=1:length(t{idx})
        T = MGD_T0marker(q{idx}(i,1), q{idx}(i,2), q{idx}(i,3), q{idx}(i,4), q{idx}(i,5)); % htf matrix
        robot_endpoint{idx}(i, :) = T(1:3,4);
    end 

    % redo calibration every time a new calibration is available
    if is_calibration(ii)
        [R2, Bfit, ErrorStats] = absor(mocap{idx}', robot_endpoint{idx}');
        tf_matrix = R2.M;
    end

    for i=1:length(t)
        temp_t = tf_matrix*[mocap{idx}(i,:), 1]'; % homogenous coordinates
        mocap_robot_endpoint{idx}(i, :) = temp_t(1:3);
    end 

    if DISPLAY_MOCAP_FIT 
        figure
        hold on, grid on
        plot(t{idx}, mocap_robot_endpoint{idx}(:,3))
        plot(t{idx}, robot_endpoint{idx}(:,3))
        legend('mocap z', 'MGD z')
        title('Motion capture VS Direct Kinematics')
    end
end
%% ball bouncing error
for idx = 1:length(files)
    if isempty(z_b{idx})
        idx_ball_off_ramp(idx) = NaN;
        idx_apex{idx} = [NaN];
        bounce_err{idx}.data = [NaN];
        bounce_err{idx}.mean = NaN;
        bounce_err{idx}.std = NaN;
        z_p{idx} = [NaN];
        continue
    end
    [idx_ball_off_ramp(idx), ~, ~] = offRampIdx(z_b{idx}, dt);
    idx_apex{idx} = detect_apexes(z_b{idx}, t{idx}, idx_ball_off_ramp(idx));
    bounce_err{idx}.data = z_b{idx}(idx_apex{idx}) - exp_parameters(idx).target_height;
    bounce_err{idx}.mean = nanmean(bounce_err{idx}.data);
    bounce_err{idx}.std = nanstd(bounce_err{idx}.data);
    be_movmean = movmean(bounce_err{idx}.data, 7);
    
    z_p{idx} = (robot_endpoint{idx}(:,3) + virtual_pos_offset)*exp_parameters(idx).scale;
    
    if DISPLAY_BALL_BOUNCING
        figure
        hold on, grid on
        plot(t{idx}, z_b{idx}, 'r')
        plot(t{idx}, z_p{idx}, 'b')
        plot(t{idx}(idx_apex{idx}), be_movmean + exp_parameters(idx).target_height, 'Linewidth', 1.5)
        line([t{idx}(idx_ball_on_ramp), t{idx}(end)], [exp_parameters(idx).target_height, exp_parameters(idx).target_height], 'Color','black','LineStyle','--');
        legend('mocap z', 'MGD z')
        title('Motion capture VS Direct Kinematics')
    end
    
end
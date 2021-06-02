clear all

addpath('utils')
addpath('../../data/utils')
addpath('../../data/force_torque_sensor')

%%%%%%%%%%%%%%%%%%
%% MACROS & variables
%%%%%%%%%%%%%%%%%%
NB_JOINTS = 5;
DISPLAY_MOCAP_FIT = 1;

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
    if is_exp == 1
        field_value{2} = "exp_1";
    elseif is_exp == 2
        field_value{2} = "exp_2";
    elseif is_calibration
        field_value{2} = "calib";
    elseif is_learning_phri
        field_value{2} = "l_phri";
    elseif is_learning_ball_bouncing
        field_value{2} = "l_bb";
    end
    try
        for i = 1:length(parameters_data{end}.Doubles)
            field_name{i+2} = parameters_data{end}.Doubles(i).Name;
            field_value{i+2} = parameters_data{end}.Doubles(i).Value(end);
        end
    catch
        warning('Missing /ball_simulator/parameter_updates topic');
    end
    exp_parameters{idx} = cell2struct(field_value,field_name,2);
    clear parameters_data

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

for i=1:length(t)
    T = MGD_T0marker(q{idx}(i,1), q{idx}(i,2), q{idx}(i,3), q{idx}(i,4), q{idx}(i,5)); % htf matrix
    robot_endpoint{idx}(i, :) = T(1:3,4);
end 

% redo calibration every time a new calibration is available
if is_calibration(ii)
    [R2, Bfit, ErrorStats] = absor(mocap', robot_endpoint');
    tf_matrix = R2.M;
end

for i=1:length(t)
    temp_t = tf_matrix*[mocap{idx}(i,:), 1]'; % homogenous coordinates
    mocap_robot_endpoint{idx}(i, :) = temp_t(1:3);
end 

if DISPLAY_MOCAP_FIT 
    figure
    hold on, grid on
    plot(t, mocap_robot_endpoint(:,3))
    plot(t, robot_endpoint(:,3))
    legend('mocap z', 'MGD z')
    title('Motion capture VS Direct Kinematics')
end

%% ball bouncing error

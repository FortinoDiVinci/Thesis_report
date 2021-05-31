clear all

addpath('/utils')
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
        date_time(ii) = datenum(files(ii).name(17:end-4),'yyyy-mm-dd-HH-MM-SS');
    elseif contains(files(ii).name, "l_phri")
        date_time(ii) = datenum(files(ii).name(8:end-4),'yyyy-mm-dd-HH-MM-SS');
    end
end

[~, chron_order] = sort(date_time);
files = files(chron_order);
is_calibration = is_calibration(chron_order);
is_exp = is_exp(chron_order);


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
    '/ball_simulator_parameter_updates'),'DataFormat','struct');

%% Time extraction
t_date = datetime(bagselect.StartTime,'ConvertFrom','epochtime','Format',...
    'dd-MMM-yyyy HH:mm:ss');
t_q = select(bagselect,'Topic','/joint_states').MessageList.Time;
t_ft = select(bagselect,'Topic','/netft_data').MessageList.Time;
t_mc = select(bagselect,'Topic','/vrpn_client_node/robot_marker/pose').MessageList.Time;
t_d = select(bagselect,'Topic','/arm_1/disturbance_val').MessageList.Time;
t_b = select(bagselect,'Topic','/ball_pose').MessageList.Time;

clear bagselect

%% Topic data extraction
for ii = 1:NB_JOINTS
    raw_q(:,ii) = cellfun(@(x) double(x.Position(ii)), joint_data);
end
clear joint_data

raw_force(:,1) = cellfun(@(x) double(x.Wrench.Force.X), ft_sensor_data);
raw_force(:,2) = cellfun(@(x) double(x.Wrench.Force.Y), ft_sensor_data);
raw_force(:,3) = cellfun(@(x) double(x.Wrench.Force.Z), ft_sensor_data);
raw_torque(:,1) = cellfun(@(x) double(x.Wrench.Torque.X), ft_sensor_data);
raw_torque(:,2) = cellfun(@(x) double(x.Wrench.Torque.Y), ft_sensor_data);
raw_torque(:,3) = cellfun(@(x) double(x.Wrench.Torque.Z), ft_sensor_data);
clear ft_sensor_data

raw_mocap(:,1) = cellfun(@(x) double(x.Pose.Position.X), motion_capture_data);
raw_mocap(:,2) = cellfun(@(x) double(x.Pose.Position.Y), motion_capture_data);
raw_mocap(:,3) = cellfun(@(x) double(x.Pose.Position.Z), motion_capture_data);
clear motion_capture_data 

dist_val = cellfun(@(x) double(x.Data), disturbance_data);
clear disturbance_data

% raw_ball_z = cellfun(@(x) double(x.Pose.Position.Z), ball_data);
% clear ball_data
% 
% exp_param.alpha = cellfun(@(x) double(x.Pose.Position.Z), parameters_data);
% for i = 1:length(parameters_data{1}.name)
%     field_name(i) = parameters_data{1}.name(i);
%     field_value(i) = parameters_data{1}.value(i);
% end
% clear parameters_data

%% Synchronisation
dt = 1e-3;
t_start = max([t_q(1); t_mc(1); t_ft(1); t_b(1)]);
t_end = min([t_q(end); t_mc(end); t_ft(end); t_b(end)]);
t = (t_start:dt:t_end)';

for ii = 1:NB_JOINTS
    q(:,ii) = interp1(t_q, raw_q(:,ii), t);
end
for ii = 1:3
    mocap(:,ii) = interp1(t_mc, raw_mocap(:,ii), t);
end
for ii = 1:3
    ft_sensor(:,ii) = interp1(t_ft, raw_force(:,ii), t);
    ft_sensor(:,ii+3) = interp1(t_ft, raw_torque(:,ii), t);
end
% z_b = interp1(t_b, raw_ball_z, t);

%% spatial synchronisation
% between motion capture coordinates and robot coordinates

for i=1:length(t)
    T = MGD_T0marker(q(i,1), q(i,2), q(i,3), q(i,4), q(i,5)); % htf matrix
    robot_endpoint(i, :) = T(1:3,4);
end 

if is_calibration
    [R2, Bfit, ErrorStats] = absor(mocap', robot_endpoint');
    tf_matrix = R2.M;
end

for i=1:length(t)
    temp_t = tf_matrix*[mocap(i,:), 1]'; % homogenous coordinates
    mocap_robot_endpoint(i, :) = temp_t(1:3);
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

clear all

%addpath('utils')
addpath('../../utils')
addpath('../../force_torque_sensor')
addpath('../Utils')

%%%%%%%%%%%%%%%%%%
%% MACROS & variables
%%%%%%%%%%%%%%%%%%

NB_JOINTS = 5;
SAVE_DATA = 1;
if SAVE_DATA
    saved_data_name = "exp_june_2021";
end

%% File selection
files = dir('data/*/*.bag');
admittance_gain_conf = zeros(size(files));
empty_data = zeros(size(files));

for idx = 1:length(files)
    date_time(idx) = datenum(files(idx).name(5:end-4),'yyyy-mm-dd-HH-MM-SS');
    if contains(files(idx).folder, "actual_tuning")
        admittance_gain_conf(idx) = 1;
    elseif contains(files(idx).folder, "ki_max")
        admittance_gain_conf(idx) = 2;
    elseif contains(files(idx).folder, "kp_max")   
        admittance_gain_conf(idx) = 3;   
    else 
        admittance_gain_conf(idx) = 4;
    end
end

% corrupted file ?
files(admittance_gain_conf == 4) = [];
admittance_gain_conf(admittance_gain_conf == 4) = [];
empty_data = zeros(size(files));

%% Data acquisition
for idx = 1:length(files)
    file_path_name = files(idx).folder + "\" + files(idx).name;
    bagselect = rosbag(file_path_name);
    
    %% Topic extraction
    try
        joint_data = readMessages(select(bagselect,'Topic','/joint_states'),...
            'DataFormat','struct');
        ft_sensor_data = readMessages(select(bagselect,'Topic','/netft_data'),...
            'DataFormat','struct');
    catch e
        fprintf(1,'An error occured while reading rosbag data:\n%s',e.message);
        empty_data(idx) = 1;
        continue
    end
    %% Time extraction
    t_q{idx} = select(bagselect,'Topic','/joint_states').MessageList.Time;
    t_ft{idx} = select(bagselect,'Topic','/netft_data').MessageList.Time;
    
    clear bagselect
    %% Main data extraction
    for ii = 1:NB_JOINTS
        raw_q{idx}(:,ii) = cellfun(@(x) double(x.Position(ii)), joint_data);
    end
    clear joint_data
    raw_force{idx}(:,1) = cellfun(@(x) double(x.Wrench.Force.X), ft_sensor_data);
    raw_force{idx}(:,2) = cellfun(@(x) double(x.Wrench.Force.Y), ft_sensor_data);
    raw_force{idx}(:,3) = cellfun(@(x) double(x.Wrench.Force.Z), ft_sensor_data);
    raw_torque{idx}(:,1) = cellfun(@(x) double(x.Wrench.Torque.X), ft_sensor_data);
    raw_torque{idx}(:,2) = cellfun(@(x) double(x.Wrench.Torque.Y), ft_sensor_data);
    raw_torque{idx}(:,3) = cellfun(@(x) double(x.Wrench.Torque.Z), ft_sensor_data);
    
    clear ft_sensor_data
    
end

%% TODO: deal with empty data
% if any(empty_data)
%     files(empty_data)
% end

%% Time Synchronisation
for idx = 1:length(files)
    dt = 1e-3;
    
    t_start(idx) = max([t_q{idx}(1); t_ft{idx}(1)]);
    t_end(idx) = min([t_q{idx}(end); t_ft{idx}(end)]); 
    t{idx} = (0:dt:t_end(idx)-t_start(idx))';
    
    for ii = 1:NB_JOINTS
        q{idx}(:,ii) = interp1(t_q{idx}-t_start(idx), raw_q{idx}(:,ii), t{idx});
    end
    for ii = 1:3
        ft_sensor{idx}(:,ii) = interp1(t_ft{idx}-t_start(idx), raw_force{idx}(:,ii), t{idx});
        ft_sensor{idx}(:,ii+3) = interp1(t_ft{idx}-t_start(idx), raw_torque{idx}(:,ii), t{idx});
    end
end

clear t_q t_ft

%% Endpoint coordinates
for idx = 1:length(files)
    for i=1:length(t{idx})
        T = MGD_T0marker(q{idx}(i,1), q{idx}(i,2), q{idx}(i,3), q{idx}(i,4), q{idx}(i,5)); % htf matrix
        robot_endpoint{idx}(i, :) = T(1:3,4);
    end  
    endpoint_force{idx} = forces_filtering(ft_sensor{idx}(:,1:3)', ft_sensor{idx}(:,4:6)', q{idx}', t{idx})'; 
end

for idx = 1:length(files)
    figure
    subplot(2,1,1)
    plot(t{idx}, robot_endpoint{idx}(:, 3));
    subplot(2,1,2)
    plot(t{idx}, endpoint_force{idx}(:, 3));
end
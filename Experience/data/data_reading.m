clear all

addpath('utils')
addpath('../../data/utils')
addpath('../../data/force_torque_sensor')
addpath('../../data/youBot_analysis/Utils')

%%%%%%%%%%%%%%%%%%
%% MACROS & variables
%%%%%%%%%%%%%%%%%%
NB_JOINTS = 5;
DISPLAY_MOCAP_FIT = 1;
DISPLAY_BALL_BOUNCING = 1;
SAVE_DATA = 1;

first_exp2_user029 = 0; % to deal with missing topic
virtual_pos_offset = -0.32;
mocap_delay = 9e-3; % 9ms delay provided by the datasheet
if SAVE_DATA
    saved_data_name = "exp_june_2021_9ms_delay_";
end

%% File selection
allfiles = dir('users/*/*.bag');
%% only process users that have not been done yet
% last_read = load('exp_june_2021_ter.mat', 'exp_parameters');
% user_already_read = unique(vertcat(last_read.exp_parameters.user),'rows');
% file_path = vertcat(files.folder);
% file_path = file_path(:,end-2:end);
% del_idx = zeros(length(file_path),1);
% for i = 1:length(user_already_read)
%     del_idx = del_idx | (string(user_already_read(i,:)) == string(file_path));
% end
% files(del_idx) = []; % to delete data from june...
% allfiles = files(del_idx); % tu use only data from june...
% clear del_idx last_read user_already_read file_path


%%
chunk = [0,59,98,154,205,247,289]; % chosen to start with calibration

is_calibration_raw = zeros(size(allfiles));
is_exp_raw = zeros(size(allfiles));
is_learning_ball_bouncing_raw = zeros(size(allfiles));
is_learning_phri_raw = zeros(size(allfiles));
for ii = 1:length(allfiles)
    if contains(allfiles(ii).name, "calibration")
        is_calibration_raw(ii) = 1;
        date_time(ii) = datenum(allfiles(ii).name(13:end-4),'yyyy-mm-dd-HH-MM-SS');
    elseif contains(allfiles(ii).name, "exp_1")
        is_exp_raw(ii) = 1;
        date_time(ii) = datenum(allfiles(ii).name(7:end-4),'yyyy-mm-dd-HH-MM-SS');
    elseif contains(allfiles(ii).name, "exp_2")
        is_exp_raw(ii) = 2;
        date_time(ii) = datenum(allfiles(ii).name(7:end-4),'yyyy-mm-dd-HH-MM-SS');
    elseif contains(allfiles(ii).name, "l_ball")
        is_learning_ball_bouncing_raw(ii) = 1;
        date_time(ii) = datenum(allfiles(ii).name(17:end-4),'yyyy-mm-dd-HH-MM-SS');
    elseif contains(allfiles(ii).name, "l_phri")
        is_learning_phri_raw(ii) = 1;
        date_time(ii) = datenum(allfiles(ii).name(8:end-4),'yyyy-mm-dd-HH-MM-SS');
    else
        % otherwise wrong naming ? / peculiar case
        date_time(ii) = datenum(allfiles(ii).name(end-22:end-4),'yyyy-mm-dd-HH-MM-SS');
    end
end

[~, chron_order] = sort(date_time);
%files = files(chron_order);
allfiles = allfiles(chron_order);
is_calibration_all = is_calibration_raw(chron_order);
is_exp_all = is_exp_raw(chron_order);
is_learning_ball_bouncing_all = is_learning_ball_bouncing_raw(chron_order);
is_learning_phri_all = is_learning_phri_raw(chron_order);
clear chron_order is_calibration_raw is_exp_raw is_learning_ball_bouncing_raw ...
    is_learning_phri_raw

for chunk_nb = 6:length(chunk)
tic
files = allfiles(chunk(chunk_nb-1)+1:chunk(chunk_nb));
is_calibration = is_calibration_all(chunk(chunk_nb-1)+1:chunk(chunk_nb));
is_exp = is_exp_all(chunk(chunk_nb-1)+1:chunk(chunk_nb));
is_learning_ball_bouncing = is_learning_ball_bouncing_all(chunk(chunk_nb-1)+1:chunk(chunk_nb));
is_learning_phri = is_learning_phri_all(chunk(chunk_nb-1)+1:chunk(chunk_nb));

%% Data reading
for idx = 1:length(files)
    file_path = files(idx).folder + "\" + files(idx).name;
    bagselect = rosbag(file_path);
    %% Topic extraction
%     joint_data = readMessages(select(bagselect,'Topic','/joint_states'),...
%         'DataFormat','struct');
%     ft_sensor_data = readMessages(select(bagselect,'Topic','/netft_data'),...
%         'DataFormat','struct');
%     motion_capture_data = readMessages(select(bagselect,'Topic',...
%         '/vrpn_client_node/robot_marker/pose'),'DataFormat','struct');
%     disturbance_data = readMessages(select(bagselect,'Topic', ...
%         '/arm_1/disturbance_val'),'DataFormat','struct');
%     ball_data = readMessages(select(bagselect,'Topic','/ball_pose'),...
%         'DataFormat','struct');
    parameters_data = readMessages(select(bagselect,'Topic',...
        '/ball_simulator/parameter_updates'),'DataFormat','struct');
% 
%     %% Time extraction
%     t_date{idx} = datetime(bagselect.StartTime,'ConvertFrom','epochtime','Format',...
%         'dd-MMM-yyyy HH:mm:ss');
%     t_q{idx} = select(bagselect,'Topic','/joint_states').MessageList.Time;
%     t_ft{idx} = select(bagselect,'Topic','/netft_data').MessageList.Time;
%     t_mc{idx} = select(bagselect,'Topic','/vrpn_client_node/robot_marker/pose').MessageList.Time;
%     t_d{idx} = select(bagselect,'Topic','/arm_1/disturbance_val').MessageList.Time;
%     t_b{idx} = select(bagselect,'Topic','/ball_pose').MessageList.Time;
%     t_mc{idx} = t_mc{idx} - mocap_delay; % to account for data processing delay of Motive
%     clear bagselect
% 
%     %% Topic data extraction
%     for ii = 1:NB_JOINTS
%         raw_q{idx}(:,ii) = cellfun(@(x) double(x.Position(ii)), joint_data);
%     end
%     clear joint_data
%     try
%         raw_force{idx}(:,1) = cellfun(@(x) double(x.Wrench.Force.X), ft_sensor_data);
%         raw_force{idx}(:,2) = cellfun(@(x) double(x.Wrench.Force.Y), ft_sensor_data);
%         raw_force{idx}(:,3) = cellfun(@(x) double(x.Wrench.Force.Z), ft_sensor_data);
%         raw_torque{idx}(:,1) = cellfun(@(x) double(x.Wrench.Torque.X), ft_sensor_data);
%         raw_torque{idx}(:,2) = cellfun(@(x) double(x.Wrench.Torque.Y), ft_sensor_data);
%         raw_torque{idx}(:,3) = cellfun(@(x) double(x.Wrench.Torque.Z), ft_sensor_data);
%     catch
%         % no force data
%         raw_force{idx} = [];
%         raw_torque{idx} = [];
%         if ~is_calibration(idx)
%             warning("Missing force torque data for user " + string(files(idx).folder(end-2:end)) + ...
%                 ", in file: " + string(files(idx).name));
%         end
%     end
%     clear ft_sensor_data
% 
%     raw_mocap{idx}(:,1) = cellfun(@(x) double(x.Pose.Position.X), motion_capture_data);
%     raw_mocap{idx}(:,2) = cellfun(@(x) double(x.Pose.Position.Y), motion_capture_data);
%     raw_mocap{idx}(:,3) = cellfun(@(x) double(x.Pose.Position.Z), motion_capture_data);
%     clear motion_capture_data 
% 
%     dist_val{idx} = cellfun(@(x) double(x.Data), disturbance_data);
%     clear disturbance_data
% 
%     raw_ball_z{idx} = cellfun(@(x) double(x.Pose.Position.Z), ball_data);
%     clear ball_data

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
        % The ball simulator node was not always launched for calibration
        if strcmp(field_value{1}, '029') || is_calibration(idx) || ...
                is_learning_phri(idx)
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
%save(strcat(saved_data_name + string(chunk_nb-1),".mat"), "exp_parameters", '-append');
%clear exp_parameters 

clear first_exp2_user029 file_path
toc
disp('All data loaded')

tic
%% Synchronisation
% Time synchronisation
for idx = 1:length(files)
    dt = 1e-3;
    if is_calibration(idx)
        t_start(idx) = max([t_q{idx}(1); t_mc{idx}(1)]);
        t_end(idx) = min([t_q{idx}(end); t_mc{idx}(end)]);        
    else
        t_start(idx) = max([t_q{idx}(1); t_mc{idx}(1); t_ft{idx}(1); t_b{idx}(1)]);
        t_end(idx) = min([t_q{idx}(end); t_mc{idx}(end); t_ft{idx}(end); t_b{idx}(end)]);
    end
    t{idx} = (0:dt:t_end(idx)-t_start(idx))';
    t_d{idx} = t_d{idx} - t_start(idx);
    t_mc{idx} = t_mc{idx} - t_start(idx);
    t_q{idx} = t_q{idx} - t_start(idx);
    t_ft{idx} = t_ft{idx} - t_start(idx);
    t_b{idx} = t_b{idx} - t_start(idx);
    
    for ii = 1:NB_JOINTS
        q{idx}(:,ii) = interp1(t_q{idx}, raw_q{idx}(:,ii), t{idx});
    end
    for ii = 1:3
        mocap{idx}(:,ii) = interp1(t_mc{idx}, raw_mocap{idx}(:,ii), t{idx});
    end
    if is_calibration(idx)
        ft_sensor{idx}(:,1:3) = NaN(size(mocap{idx}(:,1:3)));
        ft_sensor{idx}(:,4:6) = NaN(size(mocap{idx}(:,1:3)));
        z_b{idx} =  NaN(size(mocap{idx}(:,1)));
    else
        for ii = 1:3
            ft_sensor{idx}(:,ii) = interp1(t_ft{idx}, raw_force{idx}(:,ii), t{idx});
            ft_sensor{idx}(:,ii+3) = interp1(t_ft{idx}, raw_torque{idx}(:,ii), t{idx});
        end
        z_b{idx} = interp1(t_b{idx}, raw_ball_z{idx}, t{idx});
    end
end
clear t_q t_mc t_ft t_b
clear raw_q raw_mocap raw_force raw_torque raw_ball_z
toc
disp('Data interpolated')
tic
%% spatial synchronisation
% between motion capture coordinates and robot coordinates
for idx = 1:length(files)
%     if idx == 1
%         continue
%     end
%     for i=length(t{idx}):-1:1
%         T = MGD_T0marker(q{idx}(i,1), q{idx}(i,2), q{idx}(i,3), q{idx}(i,4), q{idx}(i,5)); % htf matrix
%         robot_endpoint{idx}(i, :) = T(1:3,4);
%     end 
    for i=length(t{idx}):-1:1     
        Tp = MGD_T0marker_position(q{idx}(i,1), q{idx}(i,2), q{idx}(i,3), q{idx}(i,4)); %
        robot_endpoint{idx}(i, :) = Tp;
    end 
    
    % redo calibration every time a new calibration is available
    if is_calibration(idx)
        [R2, ~, ~] = absor(mocap{idx}', robot_endpoint{idx}');
        tf_matrix = R2.M;
        clear R2
    end
 
    for i=length(t{idx}):-1:1
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
clear tf_matrix temp_t T
toc
disp('Mocap synchronized')
tic
%% ball bouncing error
for idx = length(files):-1:1
    if idx == 1
        continue
    end
    if isempty(z_b{idx}) || is_calibration(idx) || is_learning_phri(idx) %|| ...
%            is_learning_ball_bouncing(idx) 
        idx_ball_off_ramp(idx) = NaN;
        idx_apex{idx} = [NaN];
        bounce_err{idx}.data = [NaN];
        bounce_err{idx}.mean = NaN;
        bounce_err{idx}.std = NaN;
        z_p{idx} = [NaN];
        continue
    end
    [idx_ball_off_ramp(idx), ~, ~] = offRampIdx(z_b{idx}, dt);
    idx_apex{idx} = detectApexes(z_b{idx}, t{idx}, idx_ball_off_ramp(idx));
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
        plot(t{idx}(idx_apex{idx}), be_movmean + exp_parameters(idx).target_height, 'g', 'Linewidth', 1.5)
        plot(t{idx}(idx_apex{idx}), z_b{idx}(idx_apex{idx}), 'yo')
        line([t{idx}(idx_ball_off_ramp(idx)), t{idx}(end)], [exp_parameters(idx).target_height, exp_parameters(idx).target_height], 'Color','black','LineStyle','--');
        legend('ball', 'paddle')
        title("Ball bouncing task, user#" + string(exp_parameters(idx).user))
    end
    
end
toc
disp('Data ball bouncing')
tic
clear idx ii i

% to avoid unvoluntary data erasing
if exist(strcat(saved_data_name,".mat"), "file")
    warning('The file ' + saved_data_name + ".mat, already exists.")
    str_in = input('Do you really want to erase it ?','s');
    if str_in ~= "yes" && str_in ~= "YES" && str_in ~= "Yes" && str_in ~= "Y" && str_in ~= "y"
        return
    end
end

save(strcat(saved_data_name + string(chunk_nb-1),".mat"), '-v7.3');
toc
disp('Data saved')
clear q t bounce_err idx_apex idx_ball_off_ramp z_p mocap_robot_endpoint robot_endpoint ...
    z_b ft_sensor mocap t_d dist_val t_date exp_parameters t_end t_start 
end % chunk nb
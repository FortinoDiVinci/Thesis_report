folder_name = 'data_12-Oct-2021_11h05/contact_less_2021-10-12-10-59-47/';
dt = 1e-3;
NB_JOINTS = 5;

%velocity_cmd = readtable('bagfile-_arm_1_arm_controller_velocity_command.csv');
joint_set_points = readtable(strcat(folder_name, 'bagfile-_arm_1_joint_set_points.csv'));
joint_states = readtable(strcat(folder_name, 'bagfile-_joint_states.csv'));

joint_t = (joint_states.x_time - joint_states.x_time(1))*1e-9;
joint_sp_t = (joint_set_points.x_time - joint_states.x_time(1))*1e-9;

t_start = max([joint_t(1); joint_sp_t(1)]); 
t_end = min([joint_t(end); joint_sp_t(end)]); 
t = (t_start:dt:t_end)';

th = [interp1(joint_t, joint_states.field_position0, t), ...
       interp1(joint_t, joint_states.field_position1, t), ...
       interp1(joint_t, joint_states.field_position2, t), ...
       interp1(joint_t, joint_states.field_position3, t), ...
       interp1(joint_t, joint_states.field_position4, t)];

dth = [interp1(joint_t, joint_states.field_velocity0, t), ...
       interp1(joint_t, joint_states.field_velocity1, t), ...
       interp1(joint_t, joint_states.field_velocity2, t), ...
       interp1(joint_t, joint_states.field_velocity3, t), ...
       interp1(joint_t, joint_states.field_velocity4, t)];
                
dth_sp = [interp1(joint_sp_t, joint_set_points.field_velocity0, t), ...
          interp1(joint_sp_t, joint_set_points.field_velocity1, t), ...
          interp1(joint_sp_t, joint_set_points.field_velocity2, t), ...
          interp1(joint_sp_t, joint_set_points.field_velocity3, t), ...
          interp1(joint_sp_t, joint_set_points.field_velocity4, t)];

% /!\ eff_sp was define so that is is equal to i_sp./ Gear_ratio.* Tq_cnst
eff_sp = [interp1(joint_sp_t, joint_set_points.field_effort0, t), ...
          interp1(joint_sp_t, joint_set_points.field_effort1, t), ...
          interp1(joint_sp_t, joint_set_points.field_effort2, t), ...
          interp1(joint_sp_t, joint_set_points.field_effort3, t), ...
          interp1(joint_sp_t, joint_set_points.field_effort4, t)];

eff = [interp1(joint_t, joint_states.field_effort0, t), ...
       interp1(joint_t, joint_states.field_effort1, t), ...
       interp1(joint_t, joint_states.field_effort2, t), ...
       interp1(joint_t, joint_states.field_effort3, t), ...
       interp1(joint_t, joint_states.field_effort4, t)];
   
%% display

figure
subplot(2,1,1)
plot(t, th(:,4))
subplot(2,1,2)
plot(t, dth_sp(:,4))
hold on
plot(t, dth(:,4))


time = t(t>70.4 & t<80.2);
j4_velocity = dth(t>70.4 & t<80.2,4);
j4_velocity_cmd = dth_sp(t>70.4 & t<80.2,4);
T = table(time,j4_velocity,j4_velocity_cmd);

figure
plot(time, j4_velocity_cmd)
hold on
plot(time, j4_velocity)

writetable(T,'joint_4_velocity_response.csv','Delimiter',',')  
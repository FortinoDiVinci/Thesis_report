folder_name = 'data_12-Oct-2021_11h05/contact_2021-10-12-10-50-52/';
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

dth = [interp1(joint_t, joint_states.field_velocity0, t), ...
       interp1(joint_t, joint_states.field_velocity1, t), ...
       interp1(joint_t, joint_states.field_velocity2, t), ...
       interp1(joint_t, joint_states.field_velocity3, t), ...
       interp1(joint_t, joint_states.field_velocity4, t)];

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
plot(t, dth(:,4))
subplot(2,1,2)
plot(t, eff_sp(:,4))
hold on
plot(t, eff(:,4))

t_min = 137.4;
t_max = 143.8;
idx = t>t_min & t<t_max;

time = t(idx);
j4_torque = eff(idx,4);
j4_torque_cmd = eff_sp(idx, 4);
T = table(time,j4_torque,j4_torque_cmd);

figure
plot(time, j4_torque_cmd)
hold on
plot(time, j4_torque)

writetable(T,'joint_4_torque_response.csv','Delimiter',',')  
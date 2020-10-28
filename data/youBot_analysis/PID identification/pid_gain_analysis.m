clear all

%% identification

folder_name = 'joint_identification_test/';
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
                
dth_sp = [interp1(joint_sp_t, joint_set_points.field_velocity0, t), ...
          interp1(joint_sp_t, joint_set_points.field_velocity1, t), ...
          interp1(joint_sp_t, joint_set_points.field_velocity2, t), ...
          interp1(joint_sp_t, joint_set_points.field_velocity3, t), ...
          interp1(joint_sp_t, joint_set_points.field_velocity4, t)];

eff_sp = [interp1(joint_sp_t, joint_set_points.field_effort0, t), ...
          interp1(joint_sp_t, joint_set_points.field_effort1, t), ...
          interp1(joint_sp_t, joint_set_points.field_effort2, t), ...
          interp1(joint_sp_t, joint_set_points.field_effort3, t), ...
          interp1(joint_sp_t, joint_set_points.field_effort4, t)];

eps_dth = dth_sp - dth;
      
p_gain = [0,2500,1500,2000,0];
i_gain = [0,3000,900,1000,0];
d_gain = [0,0,0,0,0];
torque_constant = [0.0335, 0.0335, 0.0335, 0.051 ,0.049]; %Nm/A
gear_ratio = [1/156, 1/156, 1/100, 1/71 , 1/71];

%%

figure
hold on, grid on
subplot(2,1,1)
plot(t, eps_dth(:,[2,3,4]), '-');
l = legend('$\epsilon \dot{\theta}_2$', '$\epsilon \dot{\theta}_3$', '$\epsilon \dot{\theta}_4$');
set(l,'Interpreter','latex');
title('Joint velocity errors')
subplot(2,1,2)
plot(t, eff_sp(:,[2,3,4]));
legend('\tau_2', '\tau_3', '\tau_4')
title('Joint effort setpoints')

%%

dt_vel = 1e-3; %velocity hardware sampling time

s = tf('s');

Kp = p_gain./256;
Ki = i_gain./65536;

C2 = Kp(2) + Ki(2)/s;
C3 = Kp(3) + Ki(3)/s;
C4 = Kp(4) + Ki(4)/s;

C2_d = c2d(C2, 1e-3, 'tustin');
C3_d = c2d(C3, 1e-3, 'tustin');
C4_d = c2d(C4, 1e-3, 'tustin');

[y2,t2]=lsim(C2_d,eps_dth(:,2));
[y3,t3]=lsim(C3_d,eps_dth(:,3));
[y4,t4]=lsim(C4_d,eps_dth(:,4));

i_cmd = [y2, y3, y4];
eff_e = i_cmd.*torque_constant(2:4)./gear_ratio(2:4);
ts = [t2, t3, t4];

figure
plot(ts,eff_e)
hold on, grid on
plot(t, eff_sp(:,[2,3,4])-mean(eff_sp(:,[2,3,4])));
legend('\tau_{e2}', '\tau_{e2}', '\tau_{e4}', '\tau_2', '\tau_3', '\tau_4')
title('Effort estimated using robot gain divided by counter VS real')

%% identification

idx_start = [0, 6.213e4, 3.469e4, 3130, 0];
idx_end = [0, 7.736e4, 5.318e4, 3.044e4, 0];
orders = [1,2,0];
data = cell(NB_JOINTS,1);
H = cell(NB_JOINTS,1); % transfert function to identify
H_d = cell(NB_JOINTS,1); % discrete transfert function to identify
eff_r = cell(NB_JOINTS,1); % reconstructed effort with identified digital tf

for i = 1:NB_JOINTS
    if idx_start(i) == 0 || idx_end(i) == 0
        continue
    else
        y = eff_sp(idx_start(i):idx_end(i),i);
        u = eps_dth(idx_start(i):idx_end(i),i);
        data{i} = iddata(y-mean(y),u,dt);
        H_d{i} = arx(data{i},orders);
        eff_r{i} = sim(H_d{i},u);
        H{i} = d2c(H_d{i}, 'tustin');
    end
end

figure
for i = 2:4
    subplot(3,1,i-1)
    plot(t(idx_start(i):idx_end(i)),eff_sp(idx_start(i):idx_end(i),i) ...
        - mean(eff_sp(idx_start(i):idx_end(i),i)))
    hold on
    plot(t(idx_start(i):idx_end(i)), eff_r{i})
end


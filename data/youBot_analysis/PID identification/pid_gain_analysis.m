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
      
eps_dth = dth_sp - dth;
      
p_gain = [0,2500,1500,2000,0];
i_gain = [0,3000,900,1000,0];
d_gain = [0,0,0,0,0];
torque_constant = [0.0335, 0.0335, 0.0335, 0.051 ,0.049]; %Nm/A
gear_ratio = [1/156, 1/156, 1/100, 1/71 , 1/71];

%%

% figure
% hold on, grid on
% subplot(2,1,1)
% plot(t, eps_dth(:,[2,3,4]), '-');
% l = legend('$\epsilon \dot{\theta}_2$', '$\epsilon \dot{\theta}_3$', '$\epsilon \dot{\theta}_4$');
% set(l,'Interpreter','latex');
% title('Joint velocity errors')
% subplot(2,1,2)
% plot(t, eff_sp(:,[2,3,4]));
% legend('\tau_2', '\tau_3', '\tau_4')
% title('Joint effort setpoints')

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

% analysis in continuous time brings the exact same result
%[y2c,t2c]=lsim(C2,eps_dth(:,2),t);
%[y3c,t3c]=lsim(C3,eps_dth(:,3),t);
%[y4c,t4c]=lsim(C4,eps_dth(:,4),t);

% 
sum_err = zeros(3,1);
for ii = 1:length(t)
    for joint_i = 1:3
        sum_err(joint_i) = sum_err(joint_i) + eps_dth(ii, joint_i+1);
        y_yB(joint_i, ii) = Kp(joint_i+1)* eps_dth(ii, joint_i+1) + ...
            Ki(joint_i+1) * sum_err(joint_i);
    end
end

i_sp_sim = y_yB';
i_cmd = [y2, y3, y4];
eff_e = i_cmd.*torque_constant(2:4)./gear_ratio(2:4);
i_sp = eff_sp./torque_constant.*gear_ratio;
ts = [t2, t3, t4];

%ic_cmd = [y2c, y3c, y4c];
%eff_ec = ic_cmd.*torque_constant(2:4)./gear_ratio(2:4);

% find start of cmd
for ii= 1:5
    try
        idx_s(ii) = find(diff(dth_sp(:,ii)) > 1e-3, 1, 'first');
        idx_end(ii) = find(diff(dth_sp(:,ii)) > 1e-3, 1, 'last');
    catch
        idx_s(ii) = NaN;
        idx_end(ii) = NaN;
    end
end

default_color = lines(8);
[b,a] = butter(5, 50/(1/(2*dt_vel)), 'low'); %5th order, 50hz filtering

%
fontsize0 = 26; %30
fontsize1 = 20;
paperSizeX = 35; %cm %40
paperSizeY = 28; %cm %30

axis_lim = [65.5, 73,-0.8,1.2;
            35, 46,-Inf,Inf;
            3.6, 14.5,-1,1];

figure(71)
tiledlayout(3,1,'TileSpacing','compact','Padding','compact')
for i = 1:3
    nexttile
    idx = idx_s(i+1):idx_end(i+1);
    idx_avg = 1:idx_s(i+1);
    hold on
    sig = i_sp_sim(idx, i) - mean(i_sp_sim(idx_avg,i));
    plot(t(idx), sig, 'Color', [default_color(1,:), 0.2]);
    p1 = plot(t(idx), filtfilt(b,a,sig), 'Color', default_color(1,:), ...
        'Linewidth', 2);
    sig = i_sp(idx,i+1) - mean(i_sp(idx_avg,i+1));
    plot(t(idx), sig, 'Color', [default_color(2,:), 0.2]);
    p2 = plot(t(idx), filtfilt(b,a,sig), 'Color', default_color(2,:), ...
        'Linewidth', 2);
    legend([p1, p2], {'i_{sim}', 'i_{real}'})
    title("Joint n" + string(i+1))
    ylabel('Current (A)')   
    set(gca,'Fontsize',fontsize0)
    axis(axis_lim(i,:))
end
xlabel('Time (s)')

% set(gcf, 'PaperUnits', 'centimeters', 'PaperSize', [paperSizeX paperSizeY]);
% myfiguresize = [0, 0, paperSizeX, paperSizeY];
% set(gcf,'PaperPositionMode','manual', 'PaperPosition', myfiguresize);
% print -dpdf -f71 -r300 current_simulation_vs_real

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


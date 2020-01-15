clear all

%% identification for joint 5

%velocity_cmd = readtable('bagfile-_arm_1_arm_controller_velocity_command.csv');
joint_set_points = readtable('2020_01_14/bagfile-_arm_1_joint_set_points.csv');
joint_states = readtable('2020_01_14/bagfile-_joint_states.csv');

t = joint_states.x_time;
vq5 = joint_states.field_velocity3;
eq5 = joint_states.field_effort3;
q5 = joint_states.field_position3;
%vq5_cmd = interp1(velocity_cmd.x_time, velocity_cmd.field_velocities0_value, joint_states.x_time);
eq5_sp = interp1(joint_set_points.x_time, joint_set_points.field_effort3, joint_states.x_time);
vq5_sp = interp1(joint_set_points.x_time, joint_set_points.field_velocity3, joint_states.x_time);
epsilon_vq5 = vq5_sp - vq5;

p_gain = 800/256;
torque_constant = 0.049; %Nm/A
gear_ratio = 1/71;

vq5_c = zeros(length(q5),1);
for i = 2:length(q5)-1
    vq5_c(i) = (q5(i+1) - q5(i-1)) / (2*(t(i+1) - t(i-1))*1e-9);
end

epsilon = (vq5_sp - vq5);
eq5_sp_ext = [ones(length(eq5_sp), 1), eq5_sp];
p_data = (eq5_sp/torque_constant*gear_ratio) \ (epsilon);
p_data_ext = (eq5_sp_ext) \ (epsilon);

%%

figure
plot((joint_states.x_time - joint_states.x_time(1))*1e-9, vq5_cmd);
hold on, grid on
plot((joint_states.x_time - joint_states.x_time(1))*1e-9, vq5_sp);
plot((joint_states.x_time - joint_states.x_time(1))*1e-9, vq5);
plot((joint_states.x_time - joint_states.x_time(1))*1e-9, vq5_c);
legend('vel ros setp', 'vel real setp', 'meas vel', 'pos centered deriv')

%%
figure
plot((t-t(1))*1e-9, p_data);
hold on, grid on
plot((t-t(1))*1e-9, eq5_sp/torque_constant);
plot((t-t(1))*1e-9, epsilon);
legend('H(p)', 'I', '\epsilon')

%%
figure
plot((t-t(1))*1e-9, vq5_sp);
hold on, grid on
plot((t-t(1))*1e-9, eq5_sp);
plot((t-t(1))*1e-9, vq5);
plot((t-t(1))*1e-9, eq5);
plot((t-t(1))*1e-9, epsilon_vq5);
legend('dq_c(4)', '\tau_c(4)', 'dq(4)', '\tau(4)', '\epsilon(5)')

s = tf('s');

Kp = 1.269;
Ki = 38.64;
C = (Kp*s + Ki)/s;
C_d = c2d(C, 0.001, 'tustin');
[y,t1]=lsim(C_d,epsilon_vq5);

figure
plot((t-t(1))*1e-9, eq5);
hold on, grid on
plot(t1, y);
plot((t-t(1))*1e-9, epsilon_vq5);
legend('\tau','est','\dot(q)')
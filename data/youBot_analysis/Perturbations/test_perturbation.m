%% 
clear all

path = pwd;
addpath(strcat(path, '/../Jacobian'))

joint_state = readtable('16_01_20/bagfile-_joint_states.csv');
joint_setpoint = readtable('16_01_20/bagfile-_arm_1_joint_set_points.csv');
perturbation = readtable('16_01_20/bagfile-arm_1_disturbance_time.csv');

t = (joint_state.x_time - joint_state.x_time(1))*1e-9;
t_pert = (perturbation.field_stamp - joint_state.x_time(1))*1e-9;
t_pert_rb = (perturbation.x_time - joint_state.x_time(1))*1e-9;

%interp
th2_sp = interp1(joint_setpoint.x_time, joint_setpoint.field_position1, joint_state.x_time);
th3_sp = interp1(joint_setpoint.x_time, joint_setpoint.field_position2, joint_state.x_time);
th4_sp = interp1(joint_setpoint.x_time, joint_setpoint.field_position3, joint_state.x_time);

eff2_sp = interp1(joint_setpoint.x_time, joint_setpoint.field_effort1, joint_state.x_time);
eff3_sp = interp1(joint_setpoint.x_time, joint_setpoint.field_effort2, joint_state.x_time);
eff4_sp = interp1(joint_setpoint.x_time, joint_setpoint.field_effort3, joint_state.x_time);

Fxyz = zeros(length(joint_state.x_time),3);
Fxyz_cons = zeros(length(joint_state.x_time),3);

Pxyz = zeros(length(joint_state.x_time),3);

% cartesian force computation from joint torque
for i=1:length(joint_state.x_time)
    th2 = joint_state.field_position1(i);
    th3 = joint_state.field_position2(i);
    th4 = joint_state.field_position3(i); 
    
    tau2 =  joint_state.field_effort1(i);
    tau3 =  joint_state.field_effort2(i);
    tau4 =  joint_state.field_effort3(i);
    
    Jt = youBot_jacobian_XZRy(th2,th3,th4);
    Fxyz(i,:) = Jt\[tau2;tau3;tau4];
end

for i=1:length(joint_state.x_time)
    Jt = youBot_jacobian_XZRy(th2_sp(i), th3_sp(i), th4_sp(i));
    Fxyz_cons(i,:) = Jt\[eff2_sp(i);eff3_sp(i);eff4_sp(i)];  
end

% cartesian position computation
for i=1:length(joint_state.x_time)
    th1 = joint_state.field_position0(i);
    th2 = joint_state.field_position1(i);
    th3 = joint_state.field_position2(i);
    th4 = joint_state.field_position3(i); 
    th5 = joint_state.field_position4(i);
    H = MGD_T0sensor(th1,th2,th3,th4,th5);
    Pxyz(i,:) = H(1:3,4);
end

%%
figure()
hold on, grid on
plot(t, joint_state.field_position3)
plot(t, joint_state.field_effort3)

figure()
temp = find(t > 35);
idx1 = temp(1);
temp = find(t_pert > 35);
idx2 = temp(1);
t_pert_rb_clip = t_pert_rb(idx2:end);
t_pert_clip = t_pert(idx2:end);

hold on, grid on
plot(t(idx1:end), Fxyz((idx1:end),3))
plot(t(idx1:end), Fxyz_cons((idx1:end),3))

for i=1:length(t_pert_clip)
    plot([t_pert_clip(i) t_pert_clip(i)],[-50 0], 'b--')
    plot([t_pert_rb_clip(i) t_pert_rb_clip(i)],[-50 0], 'k--')
end
legend('meas', 'cmd', 'perturbation')
title('Interaction force')

figure()
hold on, grid on
plot(t(idx1:end), Pxyz((idx1:end),3))

for i=1:length(t_pert_clip)
    plot([t_pert_clip(i) t_pert_clip(i)],[0.2 0.4], 'b--')
    plot([t_pert_rb_clip(i) t_pert_rb_clip(i)],[0.2 0.4], 'k--')
end
legend('meas', 'perturbation')
title('Cartesian position')

%% Impedance estimation

% force and position differential (non pert - pert)

temp = find(t_pert_clip > 84);
pert_imp_idx = temp(1);
pert_imp_t = t_pert_clip(pert_imp_idx);

temp = find(t >= pert_imp_t);
pert_imp_idx_f = temp(1);
pert_imp_f_t = t(pert_imp_idx_f);

f_before_pert = mean(Fxyz(pert_imp_idx_f-1000:pert_imp_idx_f,3));
p_before_pert = mean(Pxyz(pert_imp_idx_f-100:pert_imp_idx_f,3));

temp = find(t >= pert_imp_f_t + 0.250);
pert_imp_idx2_f = temp(1);
pert_imp_f_t2 = t(pert_imp_idx2_f);

f_tilde = Fxyz(pert_imp_idx_f:pert_imp_idx2_f, 3) - f_before_pert;
p_tilde = Pxyz(pert_imp_idx_f:pert_imp_idx2_f, 3) - p_before_pert;

t_pert_imp = t(pert_imp_idx_f:pert_imp_idx2_f);

figure
hold on, grid on
ylabel('Fz (N)')
plot(t_pert_imp, f_tilde)
yyaxis right
ylabel('Z (m)')
plot(t_pert_imp, p_tilde)
title('perturbation')

% velocity & acceleration estimation

for ii=2:length(p_tilde)-1
	v_tilde(ii) = ( p_tilde(ii+1) - p_tilde(ii-1) )/(t_pert_imp(ii+1) - t_pert_imp(ii-1));  
end

for ii=2:length(v_tilde)-1
	a_tilde(ii) = ( v_tilde(ii+1) - v_tilde(ii-1) )/(t_pert_imp(ii+1) - t_pert_imp(ii-1));  
end

figure
hold on, grid on
plot(t_pert_imp, p_tilde)
plot(t_pert_imp(2:end), v_tilde)
plot(t_pert_imp(3:end), a_tilde)
legend('position', 'velocity', 'acceleration')

% impedance estimation

phi = [a_tilde', v_tilde(1:end-1)', p_tilde(1:end-2) ones(length(a_tilde), 1)];
y = f_tilde(1:end-2);
impedance = (phi'*phi)\phi'*y;
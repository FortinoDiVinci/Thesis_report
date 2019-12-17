velocity_cmd = readtable('bagfile-_arm_1_arm_controller_velocity_command.csv');
joint_states = readtable('bagfile-_joint_states.csv');

vq1 = joint_states.field_velocity1;
vq2 = joint_states.field_velocity2;
vq3 = joint_states.field_velocity3;

vq1_cmd = interp1(velocity_cmd.x_time, velocity_cmd.field_velocities0_value, joint_states.x_time);
vq2_cmd = interp1(velocity_cmd.x_time, velocity_cmd.field_velocities1_value, joint_states.x_time);
vq3_cmd = interp1(velocity_cmd.x_time, velocity_cmd.field_velocities2_value, joint_states.x_time);

vq1_err = vq1_cmd - vq1;
vq2_err = vq2_cmd - vq2;
vq3_err = vq3_cmd - vq3;

figure
plot((joint_states.x_time - joint_states.x_time(1))*1e-9, vq1_err);
hold on, grid on
plot((joint_states.x_time - joint_states.x_time(1))*1e-9, vq2_err);
plot((joint_states.x_time - joint_states.x_time(1))*1e-9, vq3_err);

%%

th2 = joint_states.field_position1;
th3 = joint_states.field_position2;
th4 = joint_states.field_position3;
iden = eye(3,3);
sum_err = zeros(length(th2),1);

for i = 1:length(joint_states.field_position1)
    err = iden - Jacobian_tot_youbot_vf_inverse_3joints(th2(i), th3(i), th4(i)) * youBot_jacobian_XZRy(th2(i),th3(i),th4(i));
    sq_err = err.^2;
    sum_err(i) = sum(sq_err(:,:));
end

figure
plot((joint_states.x_time - joint_states.x_time(1))*1e-9, sum_err);

%%

X_err = zeros(3, length(joint_states.field_position1));

for i = 1:length(joint_states.field_position1)
    X_err(:,i) = youBot_jacobian_XZRy(th2(i),th3(i),th4(i)) * [vq1_err(i); vq2_err(i); vq3_err(i)];
end

figure
plot((joint_states.x_time - joint_states.x_time(1))*1e-9, X_err);
legend('x','z','ry')
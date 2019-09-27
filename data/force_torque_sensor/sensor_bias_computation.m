clear all
close all

%%%%%%%%%%%%%%%%%%%
% DATA EXTRACTION %
%%%%%%%%%%%%%%%%%%%

%read_ft_sensor = importdata('2019_06_25_netft_data.txt');
%read_tf = importdata('2019_06_25_tf.txt');
read_ft_sensor = importdata('bias_gravity_data\netft_sensor_data_2019_04_19.txt');
read_tf = importdata('bias_gravity_data\fk_sensor_data_2019_04_19.txt');

QUATERNION = 1;  % if no translation are in the file

stamp_ft_sensor = str2double(read_ft_sensor.textdata(2:end, 3));
stamp_ft_sensor = (stamp_ft_sensor)*1e-9;
force = read_ft_sensor.data(:, 1:3);
torque = read_ft_sensor.data(:, 4:6);

if QUATERNION == 1
    sensor_tf = read_tf.data(:,:);
    stamp_tf =  str2double(read_tf.textdata(2:end,1));
    stamp_tf = (stamp_tf)*1e-9;
    quaternion = [sensor_tf(:, end), sensor_tf(:, end-3:end-1)];
else
    % Only extract the transforms concerning the sensor (time, translation and
    % rotation)
    sensor_tf = read_tf.textdata(strcmp(read_tf.textdata(:,5), 'sensor'),[1,6,7,8,9,10,11,12]);
    stamp_tf =  str2double(sensor_tf(:,1));
    stamp_tf = (stamp_tf)*1e-9;
    quaternion = str2double([sensor_tf(:, end), sensor_tf(:, end-3:end-1)]);
end

g = 9.81;

if stamp_tf(1) > stamp_ft_sensor(1)
    stamp_tf = stamp_tf - stamp_ft_sensor(1);
    stamp_ft_sensor = stamp_ft_sensor - stamp_ft_sensor(1);
else
    stamp_ft_sensor = stamp_ft_sensor - stamp_tf(1); 
    stamp_tf = stamp_tf - stamp_tf(1);
end

if length(stamp_ft_sensor) > length(stamp_tf)
    t = stamp_ft_sensor;
    q_w = interp1(stamp_tf, quaternion(:,1), t);
    q_x = interp1(stamp_tf, quaternion(:,2), t);
    q_y = interp1(stamp_tf, quaternion(:,3), t);
    q_z = interp1(stamp_tf, quaternion(:,4), t);
    quaternion = [q_w, q_x, q_y, q_z];
    f = [force(:, 1), force(:, 2), force(:, 3)]';
    tq = [torque(:, 1), torque(:, 2), torque(:, 3)]';
else
    t = stamp_tf;
    f_x = interp1(stamp_ft_sensor, force(:,1), t);
    f_y = interp1(stamp_ft_sensor, force(:,2), t);
    f_z = interp1(stamp_ft_sensor, force(:,3), t);
    t_x = interp1(stamp_ft_sensor, torque(:,1), t);
    t_y = interp1(stamp_ft_sensor, torque(:,2), t);
    t_z = interp1(stamp_ft_sensor, torque(:,3), t);
    
    f = [f_x, f_y, f_z]';
    tq = [t_x, t_y, t_z]';
end

%%%%%%%%%%%%%%%%
% LEAST SQUARE %
%%%%%%%%%%%%%%%%

% Force computation

y = f(:);

dummy_eye = [eye(3), [0;0;0]];
phi = repmat(dummy_eye, length(t), 1);

weight = [0, 0, -g];
weight_proj = quatrotate(quaternion, weight);
weight_proj1 = weight_proj';
weight_proj = weight_proj1(:);

phi(:,4) = weight_proj;

phi = phi(7:end-6, :);
y = y(7:end-6, :);
t = t(3:end-2, :);
f = f(:, 3:end-2);
tq = tq(:, 3:end-2);
weight_proj1 = weight_proj1(:, 3:end-2);

x = (phi'*phi)\(phi'*y);

m = x(4) % estimation of the mass

% Torque computation

y2 = tq(:);
mass = [0; 0; -m];
z_vect = repmat(mass, 1, length(t));
c = cross(weight_proj1, z_vect);

phi2 = phi;
phi2(:,4) = c(:);

x2 = (phi2'*phi2)\(phi2'*y2);

%%%%%%%%%%%%%%
% ESTIMATION %
%%%%%%%%%%%%%%

% Force

y_opt = phi*x;

f_x_opt = y_opt(1:3:end);
f_y_opt = y_opt(2:3:end);
f_z_opt = y_opt(3:3:end);

Err_x = f(1,:)' - f_x_opt;
Err_y = f(2,:)' - f_y_opt;
Err_z = f(3,:)' - f_z_opt;

C_x = xcorr(Err_x);
C_y = xcorr(Err_y);
C_z = xcorr(Err_z);

% Torque

y2_opt = phi2*x2;

t_x_opt = y2_opt(1:3:end);
t_y_opt = y2_opt(2:3:end);
t_z_opt = y2_opt(3:3:end);

Err_tx = tq(1,:)' - t_x_opt;
Err_ty = tq(2,:)' - t_y_opt;
Err_tz = tq(3,:)' - t_z_opt;

C_x2 = xcorr(Err_tx);
C_y2 = xcorr(Err_ty);
C_z2 = xcorr(Err_tz);

%%%%%%%%%
% PLOTS %
%%%%%%%%%

% Force

figure(1)
hold on
plot(stamp_ft_sensor, force(:,1), 'b');
plot(t, f_x_opt, 'r');
title('Force X, data vs estimation')
legend('Data', 'Est.')
grid on

figure(2)
hold on
plot(stamp_ft_sensor, force(:,2), 'b');
plot(t, f_y_opt, 'r');
title('Force Y, data vs estimation')
legend('Data', 'Est.')
grid on

figure(3)
hold on
plot(stamp_ft_sensor, force(:,3), 'b');
plot(t, f_z_opt, 'r');
title('Force Z, data vs estimation')
legend('Data', 'Est.')
grid on

figure(4)
hold on
plot(C_x);
title('Force X, correlation')
grid on

figure(5)
hold on
plot(C_y);
title('Force Y, correlation')
grid on

figure(6)
hold on
plot(C_z);
title('Force Z, correlation')
grid on

% Torque

figure(7)
hold on
plot(stamp_ft_sensor, torque(:,1), 'b');
plot(t, t_x_opt, 'r');
title('Torque X, data vs estimation')
legend('Data', 'Est.')
grid on

figure(8)
hold on
plot(stamp_ft_sensor, torque(:,2), 'b');
plot(t, t_y_opt, 'r');
title('Torque Y, data vs estimation')
legend('Data', 'Est.')
grid on

figure(9)
hold on
plot(stamp_ft_sensor, torque(:,3), 'b');
plot(t, t_z_opt, 'r');
title('Torque Z, data vs estimation')
legend('Data', 'Est.')
grid on

figure(10)
hold on
plot(C_x2);
title('Torque X, correlation')
grid on

figure(11)
hold on
plot(C_y2);
title('Torque Y, correlation')
grid on

figure(12)
hold on
plot(C_z2);
title('Torque Z, correlation')
grid on
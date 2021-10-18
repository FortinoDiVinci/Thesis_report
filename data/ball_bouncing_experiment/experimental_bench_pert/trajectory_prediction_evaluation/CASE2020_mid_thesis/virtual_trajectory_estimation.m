clear all
close all

addpath('../../Other data')

load('successful_exp_data.mat')
load('successful_exp_data_filtered_forces.mat')

t_positiv_dist{1} = t_dist{1}((find(dist{1}(1:end-2) > 0))');
t_negativ_dist{1} = t_dist{1}((find(dist{1}(1:end-2) < 0))');
t_off_dist{1} = t_dist{1}(find((dist{1}(1:end-2) == 0))');
t_on_dist{1} = [t_positiv_dist{1}; t_negativ_dist{1}];

%%%%%%%%%%
%% Virtual Trajectory estimation
%%%%%%%%%%

p_gain = 3;

for i = 1:length(t_on_dist{1}(:))
    
    idx_pert = find(t{1} >= t_on_dist{1}(i), 1, 'first');

    idx_fit = [(idx_pert - 100):(idx_pert) , (idx_pert + 200):(idx_pert + 300)];
    idx_tot = [(idx_pert - 100):(idx_pert + 300)];
    idx_val = [(idx_pert - 500):(idx_pert - 400) , (idx_pert - 200):(idx_pert - 100)];
    idx_qua = [(idx_pert - 400):(idx_pert - 200)];

    % estimation set
    
    x = t{1}(idx_fit);
    y = z_p{1}(idx_fit)./p_gain;
    P_z_pert(:, i) = z_p{1}(idx_tot)./p_gain;
    t_fit(:, i) = t{1}(idx_pert - 100:idx_pert + 300);
    s(:, i) = spline(x, y, t_fit(:,i));
    pc(:, i) = pchip(x, y, t_fit(:,i));
    ma(:, i) = interp1(x, y, t_fit(:,i),'makima');
    %v5c(:, i) = interp1(x, y, t_fit(:,i),'v5cubic');
    
    F_z_pert(:, i) = (F_r{1}(3, idx_tot))';
    y2 = (F_r{1}(3, idx_fit))';
    s2(:, i) = spline(x, y2, t_fit(:,i));
    pc2(:, i) = pchip(x, y2, t_fit(:,i));
    ma2(:, i) = interp1(x, y2, t_fit(:,i),'makima');
    %v5c2(:, i) = interp1(x, y2, t_fit(:,i),'v5cubic');
    
    % validation set
    
    x_val = t{1}(idx_val);
    y_val = z_p{1}(idx_val)./p_gain;
    t_val(:, i) = t{1}(idx_pert - 500:idx_pert - 100);
    s_val(:, i) = spline(x_val, y_val, t_val(:,i));
    pc_val(:, i) = pchip(x_val, y_val, t_val(:,i));
    ma_val(:, i) = interp1(x_val, y_val, t_val(:,i),'makima');
    
    y2_val = F_r{1}(3, idx_val);
    s2_val(:, i) = spline(x_val, y2_val, t_val(:,i));
    pc2_val(:, i) = pchip(x_val, y2_val, t_val(:,i));
    ma2_val(:, i) = interp1(x_val, y2_val, t_val(:,i),'makima');  
    
    % quality evaluation of the validation set RMSE
    
    s_qua(i) = sqrt(mean((z_p{1}(idx_qua)./p_gain - s_val(101:end-100, i)).^2)); 
    pc_qua(i) = sqrt(mean((z_p{1}(idx_qua)./p_gain - pc_val(101:end-100, i)).^2)); 
    ma_qua(i) = sqrt(mean((z_p{1}(idx_qua)./p_gain - ma_val(101:end-100, i)).^2)); 
    
    s2_qua(i) = sqrt(mean((F_r{1}(3, idx_qua)' - s2_val(101:end-100, i)).^2)); 
    pc2_qua(i) = sqrt(mean((F_r{1}(3, idx_qua)' - pc2_val(101:end-100, i)).^2)); 
    ma2_qua(i) = sqrt(mean((F_r{1}(3, idx_qua)' - ma2_val(101:end-100, i)).^2)); 
    
end


%% methodology validation

figure()
subplot(2,1,1)
hold on, grid on
l(1) = plot(t{1}, z_p{1}./p_gain, 'Color', [0    0.4470    0.7410]);
temp = plot(t_val, s_val, '-.', 'Color', [0.8500    0.3250    0.0980]);
l(2) = temp(1);
temp = plot(t_val, pc_val, '-.', 'Color', [0.9290    0.6940    0.1250]);
l(3) = temp(1);
temp = plot(t_val, ma_val, '-.', 'Color', [0.4940    0.1840    0.5560]);
l(4) = temp(1);
legend(l, 'real', 'spline', 'pchip', 'makima')
title('Position')

clear l;
subplot(2,1,2)
hold on, grid on
l(1) = plot(t{1}, F_r{1}(3, :), 'Color', [0    0.4470    0.7410]);
temp = plot(t_val, s2_val, '-.', 'Color', [0.8500    0.3250    0.0980]);
l(2) = temp(1);
temp = plot(t_val, pc2_val, '-.', 'Color', [0.9290    0.6940    0.1250]);
l(3) = temp(1);
temp = plot(t_val, ma2_val, '-.', 'Color', [0.4940    0.1840    0.5560]);
l(4) = temp(1);
legend(l, 'real', 'spline', 'pchip', 'makima')
title('Force')

% validation quality
figure()
subplot(2,1,1)
hold on, grid on
plot(s_qua)
plot(pc_qua)
plot(ma_qua)
legend('spline', 'pchip', 'makima')
title('Position RMSE of validation set')

subplot(2,1,2)
hold on, grid on
plot(s2_qua)
plot(pc2_qua)
plot(ma2_qua)
legend('spline', 'pchip', 'makima')
title('Force RMSE of validation set')

%% estimation 

figure()
subplot(2,1,1)
hold on, grid on
l(1) = plot(t{1}, z_p{1}./p_gain, 'Color', [0    0.4470    0.7410]);
temp = plot(t_fit, s, '-.', 'Color', [0.8500    0.3250    0.0980]);
l(2) = temp(1);
temp = plot(t_fit, pc, '-.', 'Color', [0.9290    0.6940    0.1250]);
l(3) = temp(1);
temp = plot(t_fit, ma, '-.', 'Color', [0.4940    0.1840    0.5560]);
l(4) = temp(1);
%plot(t_fit, v5c, '-.', 'Color', [0.4660    0.6740    0.1880])
temp = line([t_positiv_dist{1}, t_positiv_dist{1}], [-0.2, 0.2], 'Color','green','LineStyle','--');
l(5) = temp(1);
temp = line([t_negativ_dist{1}, t_negativ_dist{1}], [-0.2, 0.2], 'Color','red','LineStyle','--');
l(6) = temp(1);
legend(l, 'real', 'spline', 'pchip', 'makima', 'pert+', 'pert-')
title('Position')

clear l;
subplot(2,1,2)
hold on, grid on
l(1) = plot(t{1}, F_r{1}(3, :), 'Color', [0    0.4470    0.7410]);
temp = plot(t_fit, s2, '-.', 'Color', [0.8500    0.3250    0.0980]);
l(2) = temp(1);
temp = plot(t_fit, pc2, '-.', 'Color', [0.9290    0.6940    0.1250]);
l(3) = temp(1);
temp = plot(t_fit, ma2, '-.', 'Color', [0.4940    0.1840    0.5560]);
l(4) = temp(1);
%plot(t_fit, v5c2, '-.', 'Color', [0.4660    0.6740    0.1880]);
legend(l, 'real', 'spline', 'pchip', 'makima')
title('Force')

% figure
% plot(t{1}, z_p{1})
% line([t_positiv_dist{1}, t_positiv_dist{1}], [-0.2, 0.2], 'Color','green','LineStyle','--');
% line([t_negativ_dist{1}, t_negativ_dist{1}], [-0.2, 0.2], 'Color','red','LineStyle','--');

%%%%%%%%%%
%% Impedance estimation
%%%%%%%%%%

% Velocity estimation
s_vel = NaN(size(s));
z_p_vel = NaN(size(s_vel));
s_vel_f = NaN(size(s));
z_p_vel_f = NaN(size(s_vel));

for j = 1:length(t_on_dist{1}(:))
    for i = 2:length(t_fit) - 1
        s_vel(i, j) = ( s(i + 1, j) -  s(i - 1, j) ) / (2*dt);
        z_p_vel(i,j) = ( P_z_pert(i + 1, j) -  P_z_pert(i - 1, j) ) / (2*dt);
    end
end
filter_order = 100;
fir_filter = fir1(filter_order, (40*dt), 'low');
s_vel_f(2:end-1, :) = filtfilt(fir_filter, 1, s_vel(2:end-1, :));
z_p_vel_f(2:end-1, :) = filtfilt(fir_filter, 1, z_p_vel(2:end-1, :));

% Acceleration estimation
s_acc = NaN(size(s));
z_p_acc = NaN(size(s_acc));
s_acc_f = NaN(size(s));
z_p_acc_f = NaN(size(s_acc));

for j = 1:length(t_on_dist{1}(:))
    for i = 3:length(t_fit) - 2
        s_acc(i, j) = ( s_vel_f(i + 1, j) -  s_vel_f(i - 1, j) ) / (2*dt);
        z_p_acc(i,j) = ( z_p_vel_f(i + 1, j) -  z_p_vel_f(i - 1, j) ) / (2*dt);
    end
end

s_acc_f(3:end-2, :) = filtfilt(fir_filter, 1, s_vel(3:end-2, :));
z_p_acc_f(3:end-2, :) = filtfilt(fir_filter, 1, z_p_vel(3:end-2, :));

clear l
figure()
subplot(3,1,1)
hold on, grid on
temp = plot(t_fit, P_z_pert, 'Color', [0    0.4470    0.7410]);
l(1) = temp(1);
temp = plot(t_fit, s, '-.', 'Color', [0.8500    0.3250    0.0980]);
l(2) = temp(1);
legend(l, 'real', 'virtual')
title('positions')

subplot(3,1,2)
hold on, grid on
temp = plot(t_fit, z_p_vel_f, 'Color', [0    0.4470    0.7410]);
l(1) = temp(1);
temp = plot(t_fit, s_vel_f, '-.', 'Color', [0.8500    0.3250    0.0980]);
l(2) = temp(1);
legend(l, 'real', 'virtual')
title('velocities')

subplot(3,1,3)
hold on, grid on
temp = plot(t_fit, z_p_acc_f, 'Color', [0    0.4470    0.7410]);
l(1) = temp(1);
temp = plot(t_fit, s_acc_f, '-.', 'Color', [0.8500    0.3250    0.0980]);
l(2) = temp(1);
legend(l, 'real', 'virtual')
title('accelerations') 

% Impedance LS evaluation

impedance = zeros(4, length(t_on_dist{1}));

for j = 1:length(t_on_dist{1})
    phi(:, :, j) = [s(100:end-100,j) - P_z_pert(100:end-100,j), s_vel_f(100:end-100,j) - z_p_vel_f(100:end-100,j), s_acc(100:end-100,j) - z_p_acc(100:end-100,j) eye(size(s(100:end-100,j)))];
    dFz(:,j) = F_z_pert(100:end-100,j) - s2(100:end-100,j);
    
    impedance(:,j) = (phi(:, :, j)'*phi(:, :, j))\phi(:, :, j)'*dFz(:,j);
end
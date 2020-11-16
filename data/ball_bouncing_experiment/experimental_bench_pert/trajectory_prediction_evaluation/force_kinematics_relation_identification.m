clear all
close all

addpath('../../force_torque_sensor')
addpath('../../youBot_analysis/Utils')

load("data_eval_unperturbed_behaviour.mat");

SINE_INPUT = [0,1];
STEP_INPUT = [1,0];

idx_sine1 = [6900, 2.84e4];
idx_sine2 = [3.73e4, 6.91e4];

nb_exp = length(folder_names);

z = cell(size(folder_names));
fz = cell(size(folder_names));

for i = 1:nb_exp

    [f,~,~]= forces_filtering(forces_unf{i}', torques_unf{i}', thetas{i}', t{i});
    fc = 25; % cut off frequency
    [b,a] = butter(2,fc/(1/(2*dt)),'low'); 

    z{i} = filtfilt(b,a,mocap_marker_robot_base{i}(:,3));
    fz{i} = -1*filtfilt(b,a,f(3,:));

end

fc = 0.1; % cut off frequency
[b,a] = butter(2,fc/(1/(2*dt)),'low'); 

z_avg{2} = filtfilt(b,a,mocap_marker_robot_base{2}(:,3));
f_avg{2} = filtfilt(b,a,-fz{2});


u = -fz{2}'; % pour changer l'effort vu par le capteur, en effort du bras ?
y = z{2};
dy = Iu_diffcent(t{2}, y);

idx_peaks = crossing(dy);
idx_peaks = idx_peaks(diff(idx_peaks)>100);

idx_pks_sine1 = idx_peaks(idx_peaks>idx_sine1(1)&idx_peaks<idx_sine1(2));
idx_pks_sine2 = idx_peaks(idx_peaks>idx_sine2(1)&idx_peaks<idx_sine2(2));

% figure
% plot(t{2},y)
% hold on
% plot(t{2}(idx_pks_sine1), y(idx_pks_sine1), '*')
% plot(t{2}(idx_pks_sine2), y(idx_pks_sine2), '*')

freq1 = 1/(2*mean(diff(idx_pks_sine1))*dt);
freq2 = 1/(2*mean(diff(idx_pks_sine2))*dt);

%% moving average

mvg_avg_wnd = 2/(freq1*dt);

mvg_avg_offset = zeros(size(y));

for ii = 1:length(y)
    if ii <= ceil(mvg_avg_wnd/2)
        mvg_avg_offset(ii) = mean(y(1:floor(mvg_avg_wnd)+1));
    elseif ii >= length(y) - ceil(mvg_avg_wnd/2)
        mvg_avg_offset(ii) = mean(y(end-floor(mvg_avg_wnd+1):end));
    else
        mvg_avg_offset(ii) = mean(y(ii-floor(mvg_avg_wnd/2):ii+floor(mvg_avg_wnd/2)));
    end
    if ii == idx_sine2(1)
        disp('new freq')
        mvg_avg_wnd = 2/(freq2*dt);
    end
end

%% system identification 

%sys2id_tr = [y(1:28500/2),u(1:28500/2)];
arm_iddata_stiff_training = iddata(y(1:28500/2),u(1:28500/2),dt);
arm_iddata_stiff_tot = iddata(y(1:28500),u(1:28500),dt);
arm_iddata_loose_training = iddata(y(4e4:(6.9e4-4e4)/2+4e4),u(4e4:(6.9e4-4e4)/2+4e4),dt);
arm_iddata_loose_tot = iddata(y(4e4:6.9e4),u(4e4:6.9e4),dt);

arm_iddata{1} = arm_iddata_stiff_training;
arm_iddata{2} = arm_iddata_loose_training;

orders = [2,2,0];

sys = {};
sys_wo_opt = {};
sys_cont = {};
sys_2 = {};

y_rec = {};
y_rec_wo_opt = {};
y_rec_cont = {};
y_rec2 = {};

for i = 1:1   
    [lambda,r] = arxRegul(arm_iddata{i},orders);
    opt = arxOptions;
    opt.Regularization.Lambda = lambda;
    opt.Regularization.R = r;
    
    sys{i} = arx(arm_iddata{i},orders,opt);
    sys_wo_opt{i} = arx(arm_iddata{i},orders);
    sys_cont{i} = d2c(sys{i},'tustin');
    sys_2{i} = tfest(arm_iddata{i}, 2, 0);
    
    y_rec{i} = sim(sys{i},u);
    y_rec_wo_opt{i} = sim(sys_wo_opt{i},u);
    y_rec_cont{i} = sim(sys_cont{i},u,dt);
    y_rec2{i} = sim(sys_2{i},u);
    
end

figure
for i = 1:2  
    subplot(2,1,i)
    plot(y)
    hold on
    plot(y_rec{i})
    plot(y_rec_wo_opt{i})
end

figure
plot(t{2}, y)
hold on
plot(t{2}, y_rec2{1} + y(1), ':')
plot(t{2}, y_rec2{1} + mean(y(:)), ':')
plot(t{2}, mvg_avg_offset, '--')
legend('y', 'y+y(1)', 'y+mean', 'mvg avg')

%t{2}(4e4)
%time = [0:length(y)-1]'*1e-3;
time_1 = [0:28500-1]'*1e-3;
time_2 = [4e4:6.9e4]'*1e-3;

%% execute identification on training data using matlab apps

y_rec_1 = lsim(tf1,u(1:28500), time_1);
y_rec_2 = lsim(tf2,u(4e4:6.9e4), time_2);
y_rec_3 = lsim(tf1,u(4e4:6.9e4), time_2);
y_rec_1_tot = lsim(tf1,u, t{2});

y_rec_tf_cont = sim(tf_cont, u);

offset = y(1);

num = tf1.Numerator;
den = tf1.Denominator;

figure
subplot(2,1,1)
plot(time_1, y(1:28500))
hold on
plot(time_1, y_rec_1+offset, ':')
legend('meas.', 'reconstr.')
title("Stiff interaction")

subplot(2,1,2)
plot(time_2, y(4e4:6.9e4))
hold on
plot(time_2, y_rec_2+offset, ':')
plot(time_2, y_rec_3+offset, ':')
legend('meas.', 'reconstr.', 'reconstr2')
title("Loose interaction")


figure
plot(t{2},y)
hold on
plot(t{2}, y_rec_tf_cont+offset, ':')
legend('meas.', 'reconstr.')
title("Stiff interaction")

%y_rec_tf2 = lsim(tf2, u, time);

% figure
% plot(time, y)
% hold on
% plot(time, y_rec+0.32)
% 
% figure
% plot(time, y)
% hold on
% plot(time, y_rec_tf2+0.32)

figure
plot(u)
yyaxis right
plot(y)
hold on
plot(y_rec_1_tot+y(1)) 
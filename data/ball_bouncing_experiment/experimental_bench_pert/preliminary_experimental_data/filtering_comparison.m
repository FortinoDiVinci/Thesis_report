% tested for data_vfo_3_phases exp7 and data_without_impacts_2020_11_17
clear all

addpath('../../../force_torque_sensor')
load('data_vfo_3_phases.mat')
exp_nb = 7;
% load('../data_2020_Nov_17/data_without_impacts_2020_11_17.mat')
% exp_nb = 2;

% fz computation
[f,~,~]= forces_filtering(forces_unf{exp_nb}', torques_unf{exp_nb}', ...
        thetas{exp_nb}', t{exp_nb});
fz_temp = f(3,:);

[b2,a2] = butter(10,9/(1/(2*dt)),'low'); 
fz2{exp_nb} = -1*filtfilt(b2,a2,fz_temp);
[b,a] = butter(2,25/(1/(2*dt)),'low'); 
fz{exp_nb} = -1*filtfilt(b,a,fz_temp);
[b0,a0] = butter(10,50/(1/(2*dt)),'low'); % The filtering at 50HZ seems to
% show no difference compare to 25Hz, it should be prefered therefore...
fz0{exp_nb} = -1*filtfilt(b,a,fz_temp);
[b_bw10,a_bw10] = butter(10,8/(1/(2*dt)),'low'); 
fz_bw10{exp_nb} = -1*filtfilt(b_bw10,a_bw10,fz_temp);
[b_bw100,a_bw100] = butter(100,8/(1/(2*dt)),'low'); 

if ~NO_DISTURBANCE{exp_nb}
    % perturbation sync
    dist_timings = t_dist{exp_nb}(1:2:end); % extract only the start of the disturbance
    % extraction of the perturbation indexes
    for pert_idx = 1:length(dist_timings)
        idx_perts(pert_idx) = find(t{exp_nb} >= dist_timings(pert_idx), 1, 'first');
    end
end

figure
hold on
plot(t{exp_nb}, -fz_temp)
plot(t{exp_nb}, fz{exp_nb})
plot(t{exp_nb}, fz2{exp_nb})
if ~NO_DISTURBANCE{exp_nb}
    plot(t{exp_nb}(idx_perts), fz{exp_nb}(idx_perts), 'kp')
    legend('original', '25Hz LP', '10Hz LP', 'pert')
else
    legend('original', '25Hz LP', '10Hz LP')
end
%xlim([52.5,54.5]) % for data_vfo_3_phases.mat exp_nb 7

%[n,Wn] = buttord([8 11]/(1/(2*dt)),[6.5 12.5]/(1/(2*dt)),0.1,20);
ellip_order = 7;
ellip_cfreq = 8.;
[b3,a3] = ellip(ellip_order,1,20,ellip_cfreq/(1/(2*dt)),'low');  %order 7 (max)
fz3{exp_nb} = -1*filtfilt(b3,a3,fz_temp);
% elliptic filter delay analysis for causal filtering
delay = grpdelay(b3,a3);
list_freq = linspace(0,1,512)./(2*dt);
figure('DefaultAxesFontSize',14)
plot(list_freq,delay)
title("Elliptic filter " + string(ellip_order) + "^{th} order, with " + ...
    string(ellip_cfreq) + "Hz cut off frequency")
xlabel('Frequency (Hz)')
ylabel('Delay (nb samples)')
% causal filtering
fz3_ca{exp_nb} = -1*filter(b3,a3,fz_temp);
fz3_ca_dc{exp_nb} = circshift(fz3_ca{exp_nb},-50); % delay compensated
fz3_ca_dc{exp_nb}(end-50:end) = NaN;

[b4,a4] = ellip(7,1,20,9./(1/(2*dt)),'low'); 
fz4{exp_nb} = -1*filtfilt(b4,a4,fz_temp);

figure('DefaultAxesFontSize',14)
plot(t{exp_nb}, -fz_temp)
hold on
plot(t{exp_nb}, fz{exp_nb})
plot(t{exp_nb}, fz2{exp_nb})
plot(t{exp_nb},fz4{exp_nb})
xlabel('Time (s)')
ylabel('Force (N)')
title('Interaction force on z axis')
if ~NO_DISTURBANCE{exp_nb}
    plot(t{exp_nb}(idx_perts), fz{exp_nb}(idx_perts), 'kp', 'Markersize', 15)
    legend('original', '25Hz LP', '10Hz LP', 'eliptic LP 9Hz', 'pert')
else
    legend('original', '25Hz LP', 'eliptic LP 9Hz')
end
xlim([52.5,54.5]) % for data_vfo_3_phases.mat exp_nb 7

figure
hold on
plot(t{exp_nb},fz{exp_nb} - fz3{exp_nb})
plot(t{exp_nb},fz{exp_nb} - fz4{exp_nb})
legend('8Hz', '9Hz')
title('Eliptic 5^t^h order low pass filter errors')

% m10hz=mean(fz{exp_nb} - fz2{exp_nb});
% std10hz=std(fz{exp_nb} - fz2{exp_nb});
% m8hz=mean(fz{exp_nb} - fz3{exp_nb});
% std8hz=std(fz{exp_nb} - fz3{exp_nb});
% m9hz=mean(fz{exp_nb} - fz4{exp_nb});
% std9hz=std(fz{exp_nb} - fz4{exp_nb});

% 08/01/2021 update
figure('DefaultAxesFontSize',14)
plot(t{exp_nb}, -fz_temp)
hold on
plot(t{exp_nb}, fz_bw10{exp_nb})
plot(t{exp_nb},fz3{exp_nb})
plot(t{exp_nb},fz3_ca{exp_nb})
plot(t{exp_nb},fz3_ca_dc{exp_nb})
xlabel('Time (s)')
ylabel('Force (N)')
title('Interaction force filterings (NC = non causal)')
legend('original','10^{th} BW LP - 8Hz NC', '7^{th} EL LP - 8Hz NC',...
    '7^{th} EL LP - 8Hz', '7^{th} EL LP - 8Hz delay comp.')

% error analysis
e_ell_nc = (fz3{exp_nb} + fz_temp); % filtfilt, non causal
e_ell_dc = (fz3_ca_dc{exp_nb} + fz_temp); % delay compensated with causal filter

mean_e_ell_nc = nanmean(e_ell_nc);
std_e_ell_nc = nanstd(e_ell_nc);
mean_e_ell_dc = nanmean(e_ell_dc);
std_e_ell_dc = nanstd(e_ell_dc);

%using design filter functions
Fs = 1/dt;
Fp = 8;
Fc = 8.5;
Fr = .1;
At = 20; % dB

lp_FIR_filt = dsp.LowpassFilter('SampleRate',Fs,'FilterType','FIR',...
    'DesignForMinimumOrder',true,'PassbandFrequency',Fp,...
    'StopbandFrequency',Fc,'PassbandRipple',Fr,'StopbandAttenuation',At);
lp_IIR_filt = clone(lp_FIR_filt);
lp_IIR_filt.FilterType = 'IIR';

fvtool(lp_FIR_filt,'Fs',1/dt);

fz_FIR = lp_FIR_filt(-fz_temp);
fz_IIR = lp_IIR_filt(-fz_temp);

mag = max(-fz_temp) - min(-fz_temp);
mag_FIR = max(fz_FIR) - min(fz_FIR);
mag_IIR = max(fz_IIR) - min(fz_IIR);
fz_FIR2 = mag/mag_FIR*(fz_FIR);
fz_IIR2 = mag/mag_IIR*(fz_IIR);

figure('DefaultAxesFontSize',14)
hold on
plot(t{exp_nb},-fz_temp)
plot(t{exp_nb},fz_FIR2,'--')
plot(t{exp_nb},fz_IIR2, '--')
legend('original', 'FIR filter', 'IIR filter')

%%  12/01/2021 update
df = designfilt('lowpassfir','PassbandFrequency',Fp,...
  'StopbandFrequency',Fc,'PassbandRipple',0.1,...
  'StopbandAttenuation',20,'SampleRate',Fs);

filt_order = filtord(df);
fvtool(df);
info(df)
mean_delay = floor(mean(grpdelay(df)));

% filter test on perturbed trajectories (perturbation rejection)
y_filtfilt = filtfilt(df, -fz_temp);
y_filt = filter(df, -fz_temp);
y_filt = circshift(y_filt,-mean_delay); % delay compensated
y_filt(end-mean_delay:end) = NaN;

figure('DefaultAxesFontSize',14)
hold on
plot(t{exp_nb}, -fz_temp)
plot(t{exp_nb}(idx_perts), -fz_temp(idx_perts), 'pk')
plot(t{exp_nb}, y_filtfilt, '.-')
plot(t{exp_nb}, y_filt)
legend('original', 'pert', 'filtfilt', 'filter')

%filter test on non perturbed trajectories, evaluation of the non
%perturbed trajectory modifications

addpath('../data_2020_Nov_17')
load('data_without_impacts_2020_11_17.mat')

[f,~,~]= forces_filtering(forces_unf{exp_nb}', torques_unf{exp_nb}', ...
        thetas{exp_nb}', t{exp_nb});
fz_temp_np = f(3,:);

y_filtfilt_np = filtfilt(df, -fz_temp_np);
y_filt_np = filter(df, -fz_temp_np);
y_filt_np = circshift(y_filt_np,-mean_delay); % delay compensated
y_filt_np(end-mean_delay:end) = NaN;

figure('DefaultAxesFontSize',14)
hold on
plot(t{exp_nb}, -fz_temp_np)
plot(t{exp_nb}, y_filtfilt_np, '.-')
plot(t{exp_nb}, y_filt_np)
legend('original', 'filtfilt', 'filter')

err_y_filtfilt_np = y_filtfilt_np + fz_temp_np;
err_y_filt_np = y_filt_np + fz_temp_np;

disp("Non perturbed trajectory errors after filtering using a " + ... 
    string(filt_order) + "th order low pass filter with " ...
    + string(Fp) + "Hz passband freq and " + string(Fc) + " Hz cut off freq.")
disp('Filtfilt results:')
disp("Mean err: " + nanmean(err_y_filtfilt_np))
disp("3 std dev err: " + 3*nanstd(err_y_filtfilt_np))
disp('Filter results with delay compensation')
disp("Mean err: " + nanmean(err_y_filt_np))
disp("3 std dev err: " + 3*nanstd(err_y_filt_np))


%% Human arm dynamics
s = tf('s');

fo = 5;     % filter order
rp = 1;     % Peak-to-peak passband ripple
rs = 40;    % Stopband attenuation
wp = 7;     % Passband edge frequency

[b_el,a_el] = ellip(fo,rp,rs,wp*2*pi,'low', 's');  
[b_el_d,a_el_d] = ellip(fo,rp,rs,wp/(1/(2*dt)),'low'); 
Fe_d = filt(b_el_d,a_el_d, dt); % elliptic filter digital
Fe = tf(b_el,a_el); % elliptic filter

figure
hold on
bode(Fe)
bode(Fe_d)
legend('continuous', 'digital')
h = gcr;
setoptions(h,'FreqUnits','Hz')

% typical human arm characteristics ?
M = 1.5;        % 0.8 - 2.5kg
B = 20;         % 8 - 44N.s/m
K = 40:10:700;  % 40 - 700N/m

H_arm = M*s^2 + B*s + K;
H_arm_d = c2d(H_arm, dt, 'tustin');

figure
hold on
bode(Fe.*H_arm(1), 'r')
bode(Fe.*H_arm(floor(length(H_arm)/2)), 'k')
bode(Fe.*H_arm(end), 'g')
bode(H_arm(1), ':r')
bode(H_arm(floor(length(H_arm)/2)), ':k')
bode(H_arm(end), ':g')
h = gcr;
setoptions(h,'FreqUnits','Hz')

% 
% figure
% hold on
% bode(Fe.*H_arm_d(1), 'r')
% bode(Fe.*H_arm_d(end), 'g')
% bode(Fe.*H_arm_d(floor(length(H_arm)./2)), 'k')
% bode(H_arm_d(1), ':r')
% bode(H_arm_d(end), ':g')
% bode(H_arm_d(floor(length(H_arm)./2)), ':k')
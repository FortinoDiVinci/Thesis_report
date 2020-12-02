% fz needs to be computed
% only test for data_vfo_10.mat

%% TODO: This script needs to be cleaned to work as a stand alone prog

[b2,a2] = butter(10,9/(1/(2*dt)),'low'); 
fz2{exp_nb} = -1*filtfilt(b2,a2,fz_temp);

figure
hold on
plot(t{exp_nb}, -fz_temp)
plot(t{exp_nb}, fz{exp_nb})
plot(t{exp_nb}, fz2{exp_nb})
plot(t{exp_nb}(idx_perts), fz{exp_nb}(idx_perts), 'kp')
legend('original', '25Hz LP', '10Hz LP', 'pert')
xlim([52.5,54.5])

%[n,Wn] = buttord([8 11]/(1/(2*dt)),[6.5 12.5]/(1/(2*dt)),0.1,20);
[b3,a3] = ellip(5,1,20,7./(1/(2*dt)),'low');  
freqz(b3,a3)
fz3{exp_nb} = -1*filtfilt(b3,a3,fz_temp);
figure
plot(t{exp_nb}, -fz_temp)
hold on
plot(t{exp_nb}, fz{exp_nb})
plot(t{exp_nb},fz3{exp_nb})


plot(t{exp_nb}(idx_perts), fz{exp_nb}(idx_perts), 'kp')
legend('original', '25Hz LP', 'custom', 'pert')
xlim([52.5,54.5])


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
clear all
load('../preliminary_experimental_data/data_vfo_3_phases.mat')
addpath('../../../force_torque_sensor')
exp_nb = 7;

% fz computation
[f,~,~]= forces_filtering(forces_unf{exp_nb}', torques_unf{exp_nb}', ...
        thetas{exp_nb}', t{exp_nb});
fz = -f(3,:);

if ~NO_DISTURBANCE{exp_nb}
    % perturbation sync
    dist_timings = t_dist{exp_nb}(1:2:end); % extract only the start of the disturbance
    % extraction of the perturbation indexes
    for pert_idx = 1:length(dist_timings)
        idx_perts(pert_idx) = find(t{exp_nb} >= dist_timings(pert_idx), 1, 'first');
    end
end

% filter characteristics
Fs = 1/dt;
Fp = 8;
Fc = 8.5;
Fr = .1;
At = 20; % dB

df = designfilt('lowpassfir','PassbandFrequency',Fp,...
  'StopbandFrequency',Fc,'PassbandRipple',0.1,...
  'StopbandAttenuation',20,'SampleRate',Fs);
y_filtfilt = filtfilt(df, fz);

df2 = designfilt('lowpassfir','PassbandFrequency',10,...
  'StopbandFrequency',10.5,'PassbandRipple',0.1,...
  'StopbandAttenuation',20,'SampleRate',Fs);
y_filtfilt2 = filtfilt(df2, fz);

df3 = designfilt('lowpassfir','PassbandFrequency',12,...
  'StopbandFrequency',12.5,'PassbandRipple',0.1,...
  'StopbandAttenuation',20,'SampleRate',Fs);
y_filtfilt3 = filtfilt(df3, fz);

figure
plot(t{exp_nb}, fz)
hold on
plot(t{exp_nb}, y_filtfilt)
plot(t{exp_nb}(idx_perts), fz(idx_perts), '*')

idx1=find(t{exp_nb}>=23, 1, 'first');
idx2=find(t{exp_nb}>=24.2, 1, 'first');
idx3=find(t{exp_nb}>=dist_timings(find(dist_timings>=23,1,'first')), 1, 'first');

time = t{exp_nb}(idx1:idx2);
force = fz(idx1:idx2)';
filtered = y_filtfilt(idx1:idx2)';
perturbation = NaN(size(time));
perturbation(idx3-idx1) = fz(idx3);

figure
plot(time,force)
hold on
plot(time,filtered)
plot(time, perturbation, '*')

table_chunk = table(time, force, filtered, perturbation);
% write(table_chunk,'force_signal_perturbed_filtered_chunk.csv','Delimiter',',');

[numz,denz] = tf(df);
Hf = tf(numz,denz,1e-3);
% figure
% bode(Hf,{1*2*pi,20*2*pi})

f_w = logspace(0,2,1e4);
f_w = f_w(find(f_w >= 5, 1, 'first'):find(f_w >= 25, 1, 'first'));
[mag, ~] = bode(Hf,2*pi*f_w);
mag_db = squeeze(20.*log10(abs(mag)))';

figure
semilogx(f_w, mag_db);
freq = f_w';
magnitude = mag_db';
table_Hf = table(freq, magnitude);
% write(table_Hf,'filter_bode_magnitude.csv','Delimiter',',');
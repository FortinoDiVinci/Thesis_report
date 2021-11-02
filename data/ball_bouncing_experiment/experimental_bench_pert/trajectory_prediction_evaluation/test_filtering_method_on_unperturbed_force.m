clear all

necessary_variables = {'forces_unf','torques_unf','thetas','dt','t',...
    'idx_ball_off_ramp'};
load("../data_2020_Nov_17/data_without_impacts_2020_11_17.mat", necessary_variables{:});

SIGNAL_IDX = 2;

[f_int,~,~]= forces_filtering(forces_unf{SIGNAL_IDX}', torques_unf{SIGNAL_IDX}', ...
            thetas{SIGNAL_IDX}', t{SIGNAL_IDX}');
nsig = -1.*f_int(3,:);

idx1=find(t{SIGNAL_IDX}>=146,1,'first');
idx2=find(t{SIGNAL_IDX}<149.68,1,'last');

time_chunk = t{SIGNAL_IDX}(idx1:idx2);
force_chunk = nsig(idx1:idx2)';

Fs = 1/dt;
Fp = 8;
Fc = 8.5;
Fr = .1;
At = 20; % dB

df = designfilt('lowpassfir','PassbandFrequency',Fp,...
  'StopbandFrequency',Fc,'PassbandRipple',0.1,...
  'StopbandAttenuation',20,'SampleRate',Fs);
y_filtfilt = filtfilt(df, nsig);

filtered_chunk = y_filtfilt(idx1:idx2)';


table_chunk = table(time_chunk, force_chunk, filtered_chunk);
%write(table_chunk,'force_signal_filtered_chunk.csv','Delimiter',',');

[b,a] = butter(2,50/(1/(2*dt)),'low'); 
fsig = filtfilt(b,a,nsig);
[b,a] = butter(2,25/(1/(2*dt)),'low'); 
fsig2 = filtfilt(b,a,nsig);

filt_50hz_chunk = fsig(idx1:idx2)';
filt_25hz_chunk = fsig2(idx1:idx2)';
filt_50hz = fsig;

figure
plot(time_chunk, force_chunk);
hold on
plot(time_chunk, filtered_chunk);
plot(time_chunk, filt_50hz_chunk);
plot(time_chunk, filt_25hz_chunk);
legend('meas.', 'filt', '50Hz', '25Hz')

rms(filtered_chunk - filt_50hz_chunk)
max(filtered_chunk - filt_50hz_chunk)


figure
plot(t{SIGNAL_IDX}, nsig)
hold on
plot(t{SIGNAL_IDX}, filt_50hz)
plot(t{SIGNAL_IDX}, y_filtfilt)

rms(y_filtfilt - filt_50hz)
rms(y_filtfilt - nsig)
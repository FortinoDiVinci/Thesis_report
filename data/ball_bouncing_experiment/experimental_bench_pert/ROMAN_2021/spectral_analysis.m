% Force spectral analysis
clear all
close all

addpath('../../../youBot_analysis/Utils');
addpath('../../../force_torque_sensor');
addpath('../data_2020_Nov_17')

%load('data_without_impacts_2020_11_17.mat')
%load('data_with_impacts_2020_11_17.mat')
%load('data_vfo_10.mat')

file_name = ["data_without_impacts_2020_11_17.mat", "data_with_impacts_2020_11_17.mat"];

for FILE_IDX = 1:2
    
    load(file_name(FILE_IDX))
    
    fs = 1e3; % sampling frequency
    force_fft = [];
    position_fft = [];

    fz = [];
    z = [];
    
    for SIGNAL_IDX = 2:length(folder_names) % the first exp. is just kinematics

        [f,~,~] = forces_filtering(forces_unf{SIGNAL_IDX}', torques_unf{SIGNAL_IDX}', ...
            thetas{SIGNAL_IDX}', t{SIGNAL_IDX}');

        fz = [fz, -1.*f(3,15000:end-2000)];  % first 15 sec and last 2 sec are often non cyclic   
        z = [z, mocap_marker_robot_base{SIGNAL_IDX}(15000:end-2000,3)'];
        
    end
    y = fft(fz);    
    n = length(fz);                   % number of samples
    force_fft.f = (0:n-1)*(fs/n);     % frequency range
    force_fft.power = abs(y).^2/n;    % power of the DFT
    
    y2 = fft(z);                      % number of samples
    position_fft.f = (0:n-1)*(fs/n);  % frequency range
    position_fft.power = abs(y2).^2/n;% power of the DFT
    
    
    [~,tmp] = fileparts(file_name(FILE_IDX));
    spec_analysis(FILE_IDX).name = tmp;
    spec_analysis(FILE_IDX).force = force_fft;
    spec_analysis(FILE_IDX).position = position_fft;
       
end

return

clearvars -except spec_analysis

% plot with logarithmic scale
figure('DefaultAxesFontSize',16)
subplot(2,1,1)
semilogy(spec_analysis(1).force.f, spec_analysis(1).force.power,...
	spec_analysis(2).force.f, spec_analysis(2).force.power)
xlim([0,25])
ylabel('Power')
xlabel('Frequency')
title('Force')
legend('without impact','with impact')
subplot(2,1,2)
semilogy(spec_analysis(1).position.f, spec_analysis(1).position.power,...
	spec_analysis(2).position.f, spec_analysis(2).position.power)
xlim([0,25])
ylabel('Power')
xlabel('Frequency')
title('Position')
legend('without impact','with impact')

% filtering data for readability

dfs = spec_analysis(1).position.f(end)/length(spec_analysis(1).position.f);
[b,a] = butter(10,50/(1/(2*dfs)),'low'); 

for i = 1:2
spec_analysis(i).force.power_filt = filtfilt(b,a, spec_analysis(i).force.power);
spec_analysis(i).position.power_filt = filtfilt(b,a, spec_analysis(i).position.power);
end

figure('DefaultAxesFontSize',12)
subplot(2,1,1)
semilogy(spec_analysis(1).force.f, spec_analysis(1).force.power_filt,...
	spec_analysis(2).force.f, spec_analysis(2).force.power_filt)
xlim([0,25])
ylabel('Power')
xlabel('Frequency (Hz)')
title('Force')
legend('without impact','with impact')
subplot(2,1,2)
semilogy(spec_analysis(1).position.f, spec_analysis(1).position.power_filt,...
	spec_analysis(2).position.f, spec_analysis(2).position.power_filt)
xlim([0,25])
ylabel('Power')
xlabel('Frequency (Hz)')
title('Position')
legend('without impact','with impact')
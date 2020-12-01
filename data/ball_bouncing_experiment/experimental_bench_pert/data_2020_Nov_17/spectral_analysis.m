% Force spectral analysis
clear all
close all

addpath('../../../youBot_analysis/Utils');
addpath('../../../force_torque_sensor');

load('data_without_impacts_2020_11_17.mat')
%load('data_with_impacts_2020_11_17.mat')
%load('data_vfo_10.mat')

file_name = ["data_without_impacts_2020_11_17.mat", "data_with_impacts_2020_11_17.mat"];

for FILE_IDX = 1:2
    
    load(file_name(FILE_IDX))
    
    fs = 1e3; % sampling frequency
    force_fft = [];
    position_fft = [];
    min_len = inf;
    max_len = 0;

    for SIGNAL_IDX = 2:length(folder_names) % the first exp. is just kinematics

        [f,~,~] = forces_filtering(forces_unf{SIGNAL_IDX}', torques_unf{SIGNAL_IDX}', ...
            thetas{SIGNAL_IDX}', t{SIGNAL_IDX}');

        fz = -1.*f(3,5000:end-2000);  % first 5 sec and last 2 sec are often non cyclic  
        y = fft(fz);
        n = length(fz);                                 % number of samples
        force_fft(SIGNAL_IDX-1).f = (0:n-1)*(fs/n);     % frequency range
        force_fft(SIGNAL_IDX-1).power = abs(y).^2/n; % power of the DFT

        z = mocap_marker_robot_base{SIGNAL_IDX}(5000:end-2000,3);

        y = fft(z); 
        position_fft(SIGNAL_IDX-1).f = (0:n-1)*(fs/n);     % frequency range
        position_fft(SIGNAL_IDX-1).power = abs(y).^2/n; % power of the DFT
        if n < min_len
            min_len = n;
        end
        if n > max_len
            max_len = n;
            max_idx = SIGNAL_IDX;
        end
    end

    f_ref =  position_fft(max_idx).f;

    for it = 1:length(force_fft)
        force_fft(it).power_interp = interp1(force_fft(it).f, force_fft(it).power, f_ref)';
        position_fft(it).power_interp = interp1(position_fft(it).f, position_fft(it).power, f_ref)';
    end
    [~,tmp] = fileparts(file_name(FILE_IDX));
    spec_analysis(FILE_IDX).name = tmp;
    spec_analysis(FILE_IDX).force = force_fft;
    spec_analysis(FILE_IDX).position = position_fft;
    spec_analysis(FILE_IDX).f_interp = f_ref;
    
    %spec_analysis(FILE_IDX).force.mean_pwr = mean([force_fft.power_interp],2);
    %spec_analysis(FILE_IDX).position.mean_pwr = mean([position_fft.power_interp],2);
       
end


return

clearvars -except spec_analysis

figure
subplot(2,1,1)
hold on
for i = 1:length(spec_analysis)
    plot(spec_analysis(i).f_interp, mean([spec_analysis(i).force.power_interp],2))
end
xlim([0.1,10])
ylabel('Power')
xlabel('Frequency')
title('Force')
legend('Without impact','with impact')
subplot(2,1,2)
hold on
for i = 1:length(spec_analysis)
    plot(spec_analysis(i).f_interp, mean([spec_analysis(i).position.power_interp],2))
end
xlim([0.1,10])
ylabel('Power')
xlabel('Frequency')
title('Position')
legend('without impact','with impact')

figure
hold on
for i = 1:length(spec_analysis(1).position)
    plot(spec_analysis(1).position(i).f, spec_analysis(1).position(i).power)
end
xlim([0.1,10])
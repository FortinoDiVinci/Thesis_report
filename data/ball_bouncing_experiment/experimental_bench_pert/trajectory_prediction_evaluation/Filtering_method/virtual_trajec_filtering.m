clear all

addpath('../../../../force_torque_sensor')
addpath('../../utils')
addpath('../../../../utils')
load('../../data_2021_April/exp_1.mat')
exp_nb = 1;

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

if ~NO_BALL_BOUNC{exp_nb}
    idx_ball_i = impacts_extraction2(t{exp_nb}, z_b{exp_nb});
end

idx_all_pert = [idx_perts, idx_ball_i];

fz_n = fz;
[b,a] = butter(2,50/(1/(2*dt)),'low'); %2nd order 50Hz low pass Butterworth
fz = filtfilt(b,a,fz);

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

% perturbation
len_pert = 80;%65;
idx_select_pert = NaN(len_pert);
fz_wo_pert = fz;
%fz_virt = y_filtfilt;
for ii_pert = 1:length(idx_all_pert)
    idx_select_pert = idx_all_pert(ii_pert) + (0:len_pert-1);    
    % 
    t_pert = t{exp_nb}(idx_select_pert);
    % extraction of the perturbation to separate it from the rest of the
    % signal
    fz_pert = fz(idx_select_pert);
    fz_pert_rep = repmat(fz_pert,1,500);    
    pert_conti_comp = filtfilt(df, fz_pert_rep);
    interp_line = interp1([t_pert(1),t_pert(end)], ...
        [fz_pert(1),fz_pert(end)], t_pert, 'linear');
    % the continuous component of the perturbation is extracted at the
    % center of the filtered repetition of the perturbation, it is the
    % stable part (the boundaries can have undesired oscillations)
    contin_comp = pert_conti_comp(250*len_pert:250*len_pert+len_pert-1) - ...
        interp_line';
    fz_wo_pert(idx_select_pert) = fz_wo_pert(idx_select_pert) - contin_comp;
    %
    % virtual trajectories population for each perturbation chunk
%     if ii_pert == 1
%         idx_mid_pert = floor((idx_perts(ii_pert)+idx_perts(ii_pert+1))/2);
%         fz_virt(1:idx_mid_pert) = fz_wo_pert_filt(1:idx_mid_pert);
%     elseif ii_pert == length(idx_perts)
%         fz_virt(idx_mid_pert+1:end) = fz_wo_pert_filt(idx_mid_pert+1:end);
%     else
%         prev_idx = idx_mid_pert;
%         idx_mid_pert = floor((idx_perts(ii_pert)+idx_perts(ii_pert+1))/2);
%         fz_virt(prev_idx+1:idx_mid_pert) = fz_wo_pert_filt(prev_idx+1:idx_mid_pert);
%     end  
    
end 

% when the continuous component of the perturbation is suppressed from
% the original signal at the perturbation timing, it still needs to be
% filtered to remove discontinuties
fz_virt = filtfilt(df, fz_wo_pert);

%% tp error
% error at the time of perturbation introduction
e_filt = rms(fz(idx_perts-1) - y_filtfilt(idx_perts-1));
e_algo = rms(fz(idx_perts-1) - fz_virt(idx_perts-1));

%% display exemples

figure('DefaultAxesFontSize',14)
hold on
plot(t{exp_nb}, fz_n)
plot(t{exp_nb}, fz)
plot(t{exp_nb}, y_filtfilt)
plot(t{exp_nb}, fz_virt)
plot(t{exp_nb}(idx_perts), fz(idx_perts), 'p', 'Markersize', 15)
plot(t{exp_nb}(idx_ball_i), fz(idx_ball_i), '^', 'Markersize', 15)
legend('original', '50Hz', 'filtered', 'algorithm', 'pert.')

% extraction of some cycles
t_min = [42.6, 72.6, 222.6, 252.5];
t_max = [43.6, 73.6, 223.8, 254.5];

for i = 1:length(t_min)
    idx1=find(t{exp_nb}>t_min(i), 1, 'first');
    idx2=find(t{exp_nb}>t_max(i), 1, 'first');
    idx_imp1 = idx_ball_i(find(t{exp_nb}(idx_ball_i) > t_min(i), 2, 'first'));
    idx_per1 = idx_perts(find(t{exp_nb}(idx_perts) >t_min(i), 1, 'first'));

    time = t{exp_nb}(idx1:idx2);
    force = fz_n(idx1:idx2)';
    BW50hz = fz(idx1:idx2)';
    filtered = y_filtfilt(idx1:idx2)';  
    algorithm = fz_virt(idx1:idx2)';
    impact = NaN(size(t{exp_nb}));
    impact(idx_imp1) = fz_n(idx_imp1);
    impact = impact(idx1:idx2);
    pert = NaN(size(t{exp_nb}));
    pert(idx_per1) = fz_n(idx_per1);
    pert = pert(idx1:idx2);

    figure
    plot(time, force)
    hold on
    plot(time, BW50hz)
    plot(time, filtered)
    plot(time, algorithm)
    plot(time, pert, 'k*')
    plot(time, impact, 'k+')

    table1 = table(time, force, BW50hz, filtered, algorithm, pert, impact);
    write(table1,"filtering_force_algorithm_"+string(i)+".csv",'Delimiter',',');
end


clear all

addpath('../../../force_torque_sensor')
load('data_vfo_3_phases.mat')
exp_nb = 7;
% load('../data_2020_Nov_17/data_without_impacts_2020_11_17.mat')
% exp_nb = 2;

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

% perturbation
len_pert = 65;
idx_select_pert = NaN(len_pert);
fz_virt = y_filtfilt;
for ii_pert = 1:length(idx_perts)
    idx_select_pert = idx_perts(ii_pert) + (0:len_pert-1);    
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
    fz_wo_pert = fz;
    fz_wo_pert(idx_select_pert) = fz_wo_pert(idx_select_pert) - contin_comp;
    % when the continuous component of the perturbation is suppressed from
    % the original signal at the perturbation timing, it still needs to be
    % filtered to remove discontinuties
    fz_wo_pert_filt = filtfilt(df, fz_wo_pert);
    % virtual trajectories population for each perturbation chunk
    if ii_pert == 1
        idx_mid_pert = floor((idx_perts(ii_pert)+idx_perts(ii_pert+1))/2);
        fz_virt(1:idx_mid_pert) = fz_wo_pert_filt(1:idx_mid_pert);
    elseif ii_pert == length(idx_perts)
        fz_virt(idx_mid_pert+1:end) = fz_wo_pert_filt(idx_mid_pert+1:end);
    else
        prev_idx = idx_mid_pert;
        idx_mid_pert = floor((idx_perts(ii_pert)+idx_perts(ii_pert+1))/2);
        fz_virt(prev_idx+1:idx_mid_pert) = fz_wo_pert_filt(prev_idx+1:idx_mid_pert);
    end  
    
end 

figure('DefaultAxesFontSize',14)
hold on
plot(t{exp_nb}, fz)
plot(t{exp_nb}, y_filtfilt)
plot(t{exp_nb}, fz_virt)
legend('original', 'filtered', 'algorithm')

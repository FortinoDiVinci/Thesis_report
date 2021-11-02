%% force virtual trajectory evaluation

clear all

addpath('../../utils')
addpath('../../../../force_torque_sensor')
addpath('../../../../utils')
addpath('../../../../youBot_analysis/Utils')

%% PARAMETERS
% MACRO
FILE_NAME = '../../data_2020_Nov_17/data_without_impacts_2020_11_17.mat';
% PARAMS
% data processing
low_pass_cutoff_freq = 50; % Input signal are lp filt. before computation
filter_order = 2;
% impedance and trajectory windows 
% (the following param consider a sampling frequency of 1kHz)
wndw_virt_traj   = 200; % 200ms (position)
wndw_virt_f_traj = 100; % 65ms  (force) 
wndw_imp_eval    = 200; % 200ms       
window = max(wndw_imp_eval, wndw_virt_traj);
%
nb_param = 3; % K B M

%% DATA LOADING
var_req = {'t', 'thetas', 'forces_unf', 'torques_unf', 'z_b', 'dt', ...
    'idx_ball_off_ramp', 'mocap_marker_robot_base'};
load(FILE_NAME, var_req{:});

if ~exist('dt', 'var')
    dt = 1e-3;
end

% delete calibration experiments
no_forces = cellfun(@isempty, forces_unf);
if any(no_forces)
    t(no_forces) = [];
    thetas(no_forces) = [];
    mocap_marker_robot_base(no_forces) = [];
    forces_unf(no_forces) = [];
    torques_unf(no_forces) = [];
    z_b(no_forces) = [];
    idx_ball_off_ramp(no_forces) = [];
end

idx_start_arr = cell2mat(idx_ball_off_ramp);
if idx_start_arr(9) == 2 
    % an error occured in data_without_impacts_2020_11_17 9th session
    % it is manually fed
    idx_start_arr(9) = 16700;
end
clear idx_ball_off_ramp
tot_nb_exp = length(t);

%% DATA PRE-PROCESSING

fz = {};
[b,a] = butter(filter_order,low_pass_cutoff_freq/(1/(2*dt)),'low'); 

% force and position pre-processing
for exp_nb = tot_nb_exp:-1:1    
    f_tmp = forces_filtering(forces_unf{exp_nb}', torques_unf{exp_nb}', ...
        thetas{exp_nb}', t{exp_nb});     
    fz{exp_nb} = -filtfilt(b,a,f_tmp(3,:))'; % f(e->r) = -f(r->e) = -fsens   
    z{exp_nb} = filtfilt(b,a,mocap_marker_robot_base{exp_nb}(:,3));
end

clear forces_unf torques_unf thetas f_tmp mocap_marker_robot_base

% fake perturbation feeding
idx_perts = {};
pert_space = 1.3;
for exp_nb = tot_nb_exp:-1:1  
    % last 10 seconds are ignored (often irrelevant, non-rhythmic, etc.)
    t_start = t{exp_nb}(idx_start_arr(exp_nb));
    t_end = t{exp_nb}(end) - 10;   
    idx_perts{exp_nb} = floor(1.3*(t_start/1.3:t_end/1.3)./dt);
    dist_val{exp_nb} = ones(size(idx_perts{exp_nb}));
end

idx_start = zeros(tot_nb_exp,1);
% cycle separation for the extraction of the instant of the fake
% perturbations
df = designfilt('lowpassfir', 'PassbandFrequency', 1.2,...
            'StopbandFrequency', 2.5, 'StopbandAttenuation', 20,...
            'PassbandRipple', 0.1, 'SampleRate', 1/dt);

for exp_nb = tot_nb_exp:-1:1 
    fz_filt{exp_nb} = filtfilt(df,fz{exp_nb});
    dfz_filt{exp_nb} = Iu_diffcent(fz_filt{exp_nb}, t{exp_nb});
    z_filt{exp_nb} = filtfilt(df,z{exp_nb});
    dz_filt{exp_nb} = Iu_diffcent(z_filt{exp_nb}, t{exp_nb});
    idx_pks{exp_nb} = crossing(dfz_filt{exp_nb});
    idx_pks_p{exp_nb} = crossing(dz_filt{exp_nb});
    idx_u_pks{exp_nb} = idx_pks{exp_nb}(fz_filt{exp_nb}(idx_pks{exp_nb}) > 0);
    idx_l_pks{exp_nb} = idx_pks{exp_nb}(fz_filt{exp_nb}(idx_pks{exp_nb}) < 0);
    %v_b = Iu_diffcent(z_b{exp_nb}, t{exp_nb}); % ball velocity
    %idx_start(exp_nb) = find(abs(v_b) > 0, 1, 'first'); % first ball mvt
    idx_start(exp_nb) = idx_start_arr(exp_nb);
end

% sorting
for exp_nb = tot_nb_exp:-1:1 
    i = 0;
    phase{exp_nb} = [];
    for pert_i = idx_perts{exp_nb}
        i = i + 1;
        prev_upk = find(t{exp_nb}(pert_i) > t{exp_nb}(idx_u_pks{exp_nb}), 1, 'last');
        cyc_duration = t{exp_nb}(idx_u_pks{exp_nb}(prev_upk+1)) - ...
            t{exp_nb}(idx_u_pks{exp_nb}(prev_upk));
        p_pert_in_cyc = (t{exp_nb}(pert_i) - t{exp_nb}(idx_u_pks{exp_nb}(prev_upk)))/...
            cyc_duration;
        if p_pert_in_cyc < 0.10 || p_pert_in_cyc > 0.9
            phase{exp_nb} = [phase{exp_nb}; "upper pk"];
        elseif p_pert_in_cyc > 0.15 && p_pert_in_cyc < 0.45
            phase{exp_nb} = [phase{exp_nb}; "decreas"];
        elseif p_pert_in_cyc > 0.50 && p_pert_in_cyc < 0.70
            phase{exp_nb} = [phase{exp_nb}; "lower pk"];
        elseif p_pert_in_cyc > 0.75 && p_pert_in_cyc < 0.85
            phase{exp_nb} = [phase{exp_nb}; "increas"];
        else
            phase{exp_nb} = [phase{exp_nb}; "ambiguous"];
        end
    end 
end

figure('DefaultAxesFontSize',14)
bar([...
sum(cell2mat(cellfun(@(x) sum(strcmp(x, "upper pk")), phase, 'UniformOutput', false))),...
sum(cell2mat(cellfun(@(x) sum(strcmp(x, "decreas")), phase, 'UniformOutput', false))),...
sum(cell2mat(cellfun(@(x) sum(strcmp(x, "lower pk")), phase, 'UniformOutput', false))),...
sum(cell2mat(cellfun(@(x) sum(strcmp(x, "increas")), phase, 'UniformOutput', false))),...
sum(cell2mat(cellfun(@(x) sum(strcmp(x, "ambiguous")), phase, 'UniformOutput', false)))])
phase_names = ["upper pk", "decreas", "lower pk", "increas", "ambiguous"];
set(gca,'xticklabel',phase_names)
title('Moments of the cycle the perturbation occured')
ylabel('Number of perturbations')

def_col(1,:) = [0.4940, 0.1840, 0.5560];
def_col(2,:) = [0.4660, 0.6740, 0.1880];
def_col(3,:) = [0.3010, 0.7450, 0.9330];
def_col(4,:) = [0.6350, 0.0780, 0.1840];
def_col(5,:) = [0.25, 0.25, 0.25];   

figure
for i = 1:length(fz_filt)
    subplot(5,3,i)
    hold on
    plot(t{i}-t{i}(1), fz{i})
    plot(t{i}-t{i}(1), fz_filt{i})
    plot(t{i}(idx_u_pks{i})-t{i}(1), fz_filt{i}(idx_u_pks{i}), 'rp')
    plot(t{i}(idx_l_pks{i})-t{i}(1), fz_filt{i}(idx_l_pks{i}), 'gp')
    plot(t{i}(idx_start(i))-t{i}(1), fz_filt{i}(idx_start(i)), 'mp', 'Markersize', 12)
    plot(t{i}(idx_perts{i})-t{i}(1), fz_filt{i}(idx_perts{i}), 'x')
    for cyc_phase = 1:length(phase_names) -1
        idx_cyc = idx_perts{i}(strcmp(phase{i}, phase_names(cyc_phase)));
        plot(t{i}(idx_cyc)-t{i}(1), fz_filt{i}(idx_cyc), 'o', 'Color', ...
            def_col(cyc_phase,:), 'MarkerFaceColor', def_col(cyc_phase,:))
    end
end

% extraction of the delta of force
methods_names = ["filter+", "4sineNlcOptM", "4sineOptM", "3sineNlcOptM", ...
    "3sineOptM", "2sineNlcOptM", "2sineOptM", "5sineNlcOptM", "5sineOptM"];

for exp_nb = tot_nb_exp:-1:1
    delta_fz{exp_nb,1} = DIFF_TRAJECT(window, wndw_virt_f_traj, fz{exp_nb},...
        t{exp_nb}, idx_perts{exp_nb}, dist_val{exp_nb}, 0, "filter+");
    for i = 2:length(methods_names)
        delta_fz{exp_nb,i} = copyObj(delta_fz{exp_nb,1});
        delta_fz{exp_nb,i}.header = methods_names(i);
    end
    % Virtual trajectory methods
    delta_fz{exp_nb,1}.computeDiffTraject('VirtTrajMethod', 'filterPlus'); 
    delta_fz{exp_nb,2}.computeDiffTraject('VirtTrajMethod', 'sineOptM',...
        'OptNbSine', 4, 'OptlinearComp', 0);
    delta_fz{exp_nb,3}.computeDiffTraject('VirtTrajMethod', 'sineOptM',...
        'OptNbSine', 4, 'OptlinearComp', 1);
    delta_fz{exp_nb,4}.computeDiffTraject('VirtTrajMethod', 'sineOptM',...
        'OptNbSine', 3, 'OptlinearComp', 0);
    delta_fz{exp_nb,5}.computeDiffTraject('VirtTrajMethod', 'sineOptM',...
        'OptNbSine', 3, 'OptlinearComp', 1); 
    delta_fz{exp_nb,6}.computeDiffTraject('VirtTrajMethod', 'sineOptM',...
        'OptNbSine', 2, 'OptlinearComp', 0);
    delta_fz{exp_nb,7}.computeDiffTraject('VirtTrajMethod', 'sineOptM',...
        'OptNbSine', 2, 'OptlinearComp', 1);
    delta_fz{exp_nb,8}.computeDiffTraject('VirtTrajMethod', 'sineOptM',...
        'OptNbSine', 5, 'OptlinearComp', 0);
    delta_fz{exp_nb,9}.computeDiffTraject('VirtTrajMethod', 'sineOptM',...
        'OptNbSine', 5, 'OptlinearComp', 1);
end

save('test_sine_opt_7.mat', 'delta_fz', 'phase', 'z', 'z_b', '-v7.3');

%%

LOAD_DATA = 1;

if LOAD_DATA
    load('test_sine_opt_7.mat');
    filter_errors = [];
    filter_errors_100ms = [];
    sine3_errors = [];
    sine3_errors_100ms = [];
    
    df = designfilt('lowpassfir','PassbandFrequency',8,...
      'StopbandFrequency',8.5,'PassbandRipple',0.1,...
      'StopbandAttenuation',20,'SampleRate',1e3);
    y_filtfilt = filtfilt(df, delta_fz{exp_nb,1}.complete_traject);

    
    colors = lines(4);
    
    figure
    hold on
    p = plot(delta_fz{exp_nb,1}.time, delta_fz{exp_nb,1}.complete_traject);
    pf = plot(delta_fz{exp_nb,1}.t_traject, delta_fz{exp_nb,1}.virt_traject, 'Color', colors(2,:));
    ps = plot(delta_fz{exp_nb,4}.t_traject, delta_fz{exp_nb,4}.virt_traject, 'Color', colors(3,:));
    pff = plot(delta_fz{exp_nb,1}.time, y_filtfilt, ':', 'Color', colors(4,:));
    legend([p, pf(1), ps(1), pff], ["meas.", "F+", "Sine", "FF"])
    
    for exp_nb = 1:size(delta_fz,1)
        filter_errors = [filter_errors, delta_fz{exp_nb,1}.diff_traject];
        filter_errors_100ms = [filter_errors_100ms, delta_fz{exp_nb,1}.diff_traject(1:101,:)];
        sine3_errors = [sine3_errors, delta_fz{exp_nb,4}.diff_traject];
        sine3_errors_100ms = [sine3_errors_100ms, delta_fz{exp_nb,4}.diff_traject(1:101,:)];
    end
    rms(filter_errors(:))
    rms(filter_errors_100ms(:))
    rms(sine3_errors(:))
    rms(sine3_errors_100ms(:))
    
    
    
end
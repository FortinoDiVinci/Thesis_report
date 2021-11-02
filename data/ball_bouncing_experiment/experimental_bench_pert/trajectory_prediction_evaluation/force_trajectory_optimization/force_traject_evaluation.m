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
var_req = {'t', 'thetas', 'forces_unf', 'z_b'};
load(FILE_NAME, var_req{:});

if ~exist('dt', 'var')
    dt = 1e-3;
end

tot_nb_exp = length(t);

%% DATA PRE-PROCESSING

fz = {};
[b,a] = butter(filter_order,low_pass_cutoff_freq/(1/(2*dt)),'low'); 

% force and position pre-processing
for exp_nb = tot_nb_exp:-1:2    
    torques_unf = forces_unf{exp_nb}; % the torque is irrelevant here
    f_tmp = forces_filtering(forces_unf{exp_nb}', torques_unf', ...
        thetas{exp_nb}', t{exp_nb});     
    fz{exp_nb} = -filtfilt(b,a,f_tmp(3,:))'; % f(r->e) = -f(e->r) = -fsens    
end

% fake perturbation feeding
idx_perts = {};
pert_space = 1.3;
for exp_nb = tot_nb_exp:-1:2  
    % first and last 10 seconds are ignored (often irrelevant)
    t_start = t{exp_nb}(1) + 10;
    t_end = t{exp_nb}(end) - 10;   
    idx_perts{exp_nb} = floor(1.3*(t_start/1.3:1:t_end/1.3)./dt);
    dist_val{exp_nb} = ones(size(idx_perts{exp_nb}));
    %idx_perts{exp_nb} = idx_perts{exp_nb}(idx_perts{exp_nb} < length(fz{exp_nb}));
end

idx_start = zeros(tot_nb_exp,1);
% cycle separation for the extraction of the instant of the fake
% perturbations
df = designfilt('lowpassfir', 'PassbandFrequency', 1.2,...
            'StopbandFrequency', 2.5, 'StopbandAttenuation', 20,...
            'PassbandRipple', 0.1, 'SampleRate', 1/dt);

for exp_nb = tot_nb_exp:-1:2 
    fz_filt{exp_nb} = filtfilt(df,fz{exp_nb});
    dfz_filt{exp_nb} = Iu_diffcent(fz_filt{exp_nb}, t{exp_nb});
    idx_pks{exp_nb} = crossing(dfz_filt{exp_nb});
    idx_u_pks{exp_nb} = idx_pks{exp_nb}(fz_filt{exp_nb}(idx_pks{exp_nb}) > 0);
    idx_l_pks{exp_nb} = idx_pks{exp_nb}(fz_filt{exp_nb}(idx_pks{exp_nb}) < 0);
    v_b = Iu_diffcent(z_b{exp_nb}, t{exp_nb}); % ball velocity
    idx_start(exp_nb) = find(abs(v_b) > 0, 1, 'first'); % first ball mvt
end

% sorting
for exp_nb = tot_nb_exp:-1:2 
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
for i = 2:length(fz_filt)
    subplot(5,3,i-1)
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
methods_names = ["filter+", "sineOpt+", "5sineOpt+", "5sineNlcOpt+", ...
    "3sineOpt+", "3sineNlcOpt+", "sineOptM", "3sineNlcOptM", "3sineOptM", ...
    "2sineNlcOptM", "2sineOptM"];

for exp_nb = tot_nb_exp:-1:2
    delta_fz{exp_nb,1} = DIFF_TRAJECT(window, wndw_virt_f_traj, fz{exp_nb},...
        t{exp_nb}, idx_perts{exp_nb}, dist_val{exp_nb}, 0, "filter+");
    for i = 2:length(methods_names)
        delta_fz{exp_nb,i} = copyObj(delta_fz{exp_nb,1});
        delta_fz{exp_nb,i}.header = methods_names(i);
    end
    % Virtual trajectory methods
    delta_fz{exp_nb,1}.computeDiffTraject('VirtTrajMethod', 'filterPlus'); 
    delta_fz{exp_nb,2}.computeDiffTraject('VirtTrajMethod', 'sineOpt+',...
        'OptSolverName', 'lsqnonlin');
    delta_fz{exp_nb,3}.computeDiffTraject('VirtTrajMethod', 'sineOpt+',...
        'OptSolverName', 'lsqnonlin', 'OptNbSine', 5);
    delta_fz{exp_nb,4}.computeDiffTraject('VirtTrajMethod', 'sineOpt+',...
        'OptSolverName', 'lsqnonlin', 'OptNbSine', 5, 'OptlinearComp', 0);
    delta_fz{exp_nb,5}.computeDiffTraject('VirtTrajMethod', 'sineOpt+',...
        'OptSolverName', 'lsqnonlin', 'OptNbSine', 3);
    delta_fz{exp_nb,6}.computeDiffTraject('VirtTrajMethod', 'sineOpt+',...
        'OptSolverName', 'lsqnonlin', 'OptNbSine', 3, 'OptlinearComp', 0);
    delta_fz{exp_nb,7}.computeDiffTraject('VirtTrajMethod', 'sineOptM',...
        'OptSolverName', 'lsqnonlin');
    delta_fz{exp_nb,8}.computeDiffTraject('VirtTrajMethod', 'sineOptM',...
        'OptSolverName', 'lsqnonlin', 'OptNbSine', 3, 'OptlinearComp', 0);
    delta_fz{exp_nb,9}.computeDiffTraject('VirtTrajMethod', 'sineOptM',...
        'OptSolverName', 'lsqnonlin', 'OptNbSine', 3);
    delta_fz{exp_nb,10}.computeDiffTraject('VirtTrajMethod', 'sineOptM',...
        'OptSolverName', 'lsqnonlin', 'OptNbSine', 2, 'OptlinearComp', 0);
    delta_fz{exp_nb,11}.computeDiffTraject('VirtTrajMethod', 'sineOptM',...
        'OptSolverName', 'lsqnonlin', 'OptNbSine', 2);
end

% save('test_sine_opt_6.mat', 'delta_fz', 'phase', 'z', 'z_b', '-v7.3');

return

delta_all_err = {};
delta_all_param = {};
for i = 1:length(delta_all)
    delta_all_err{i} = [delta_all(i,:).diff_traject];
    delta_all_param{i} = [delta_all(i,:).opt_param];
end

% error histogram
figure('DefaultAxesFontSize',14)
for i = 2:2:length(delta_all)
    subplot(3,2,i/2)
    hold on
    histogram(delta_all_err{1}, 'BinWidth', 0.1)
    histogram(delta_all_err{i}, 'BinWidth', 0.1)
    histogram(delta_all_err{i+1}, 'BinWidth', 0.1)
    title("Virtual force error histogram")
    legend(names(1), names(i), names(i+1))
end
% error histogram avg and std
for i = 1:length(delta_all)
    avg_ = nanmean(delta_all_err{i}(:));
    std_ = nanstd(delta_all_err{i}(:));
    fprintf('Method %s with an average mean profile error of %.3f and an average std of %.3f\n', names(i), avg_, std_);
end

% temporal representation of the error (mean and std)
figure('DefaultAxesFontSize',14)
for i = 2:2:length(delta_all)
    subplot(3,2,i/2)
    hold on
    plotStdSurface(nanmean(delta_all_err{1},2), nanstd(delta_all_err{1},0,2), (0:199), [0, 0.4470, 0.7410])
    plotStdSurface(nanmean(delta_all_err{i},2), nanstd(delta_all_err{i},0,2), (0:199), [0.8500, 0.3250, 0.0980])
    plotStdSurface(nanmean(delta_all_err{i+1},2), nanstd(delta_all_err{i+1},0,2), (0:199), [0.9290, 0.6940, 0.1250])
    plot((0:199), nanmean(delta_all_err{1},2), 'Linewidth', 1.5, 'Color', [0, 0.4470, 0.7410]) 
    plot((0:199), nanmean(delta_all_err{i},2), 'Linewidth', 1.5, 'Color', [0.8500, 0.3250, 0.0980]) 
    plot((0:199), nanmean(delta_all_err{i+1},2), 'Linewidth', 1.5, 'Color', [0.9290, 0.6940, 0.1250]) 
    title("Virtual force error mean and std for " + string(size(delta_all_err{1},2)) + " estimations")
    legend(names(1), names(i), names(i+1))
end

colors(1,:) = [0, 0.4470, 0.7410];
colors(2,:) = [0.8500, 0.3250, 0.0980];
colors(3,:) = [0.9290, 0.6940, 0.1250];
colors(4,:) = [0.4940, 0.1840, 0.5560];
colors(5,:) = [0.4660, 0.6740, 0.1880];
colors(6,:) = [0.3010, 0.7450, 0.9330];
colors(7,:) = [0.6350, 0.0780, 0.1840];
colors(8,:) = [0.25, 0.25, 0.25];          	
colors(9,:) = [0.1, 0.5, 0.1];   
colors(10,:) = [1, 0.7137, 0.7569];

% parameters distribution
figure('DefaultAxesFontSize',14)
for i = 1:3
    for j = 2:length(delta_all)
        subplot(3,5,(i-1)*5+floor(j/2))
        hold on
        plot(delta_all(j,2).opt_param(i,:), 'x', 'Color', colors(j-1,:))
        plot(delta_all(j,2).opt_param(3+i,:), 'o', 'Color', colors(j-1,:))
        if length(delta_all(j,2).opt_param(:,1)) > 3*3+2
            plot(delta_all(j,2).opt_param(9+i,:), '+', 'Color', colors(j-1,:))
        end
        if length(delta_all(j,2).opt_param(:,1)) > 2*3+2
            plot(delta_all(j,2).opt_param(6+i,:), '*', 'Color', colors(j-1,:))
        end
        if j == 2
            switch i
                case 1
                    ylabel("A_i (N)");
                case 2
                    ylabel("f_i (Hz)");
                case 3
                    ylabel("\phi_i (rad.s^{-1})");
            end
        end
        if (i == 1) && mod(j,2) == 0
            title(sprintf("%s{%f %f %f}%s VS %s{%f %f %f}%s", '\color[rgb]', colors(j-1,:), names(j),'\color[rgb]', colors(j,:), names(j+1)))
        end
    end
end

return

% evaluation
filtPls_err = [];
sineOpt_err = [];
sineOptP_err = [];
sineOptM_err = [];
sineOptP_nonlin_err = [];
sineOptM_nonlin_err = [];
for exp_nb = tot_nb_exp:-1:2
    filtPls_err = [filtPls_err, delta_fz_filt(exp_nb).virt_traject - ...
        delta_fz_filt(exp_nb).traject];
    sineOpt_err = [sineOpt_err, delta_fz_sine(exp_nb).virt_traject - ...
        delta_fz_sine(exp_nb).traject];
    sineOptP_err = [sineOptP_err, delta_fz_sineP(exp_nb).virt_traject - ...
        delta_fz_sineP(exp_nb).traject];
    sineOptM_err = [sineOptM_err, delta_fz_sineM(exp_nb).virt_traject - ...
        delta_fz_sineM(exp_nb).traject];
    sineOptP_nonlin_err = [sineOptP_nonlin_err, delta_fz_sineP_nonlin(exp_nb).virt_traject - ...
        delta_fz_sineP_nonlin(exp_nb).traject];
    sineOptM_nonlin_err = [sineOptM_nonlin_err, delta_fz_sineM_nonlin(exp_nb).virt_traject - ...
        delta_fz_sineM_nonlin(exp_nb).traject];
end

return

figure('DefaultAxesFontSize',14)
subplot(2,2,1)
hold on
histogram(filtPls_err, 'BinWidth', 0.1)
histogram(sineOpt_err, 'BinWidth', 0.1)
histogram(sineOptP_err, 'BinWidth', 0.1)
legend('Filter +', 'Sine opt.', 'Sine opt. +')
xlabel('Force (N)')
title("Force estimation errors, for " + string(size(sineOpt_err,2)) + " estimations")
subplot(2,2,2)
hold on
histogram(filtPls_err, 'BinWidth', 0.1)
histogram(sineOptP_err, 'BinWidth', 0.1, 'FaceColor', [0.9290, 0.6940, 0.1250])
histogram(sineOptM_err, 'BinWidth', 0.1, 'FaceColor', [0.4940, 0.1840, 0.5560])
legend('Filter +', 'Sine opt. +', 'Sine opt. M')
xlabel('Force (N)')
title("Force estimation errors, for " + string(size(sineOpt_err,2)) + " estimations")
subplot(2,2,3)
hold on
plotStdSurface(nanmean(filtPls_err,2), nanstd(filtPls_err,0,2), (0:203), [0, 0.4470, 0.7410])
plotStdSurface(nanmean(sineOpt_err,2), nanstd(sineOpt_err,0,2), (0:203), [0.8500, 0.3250, 0.0980])
plotStdSurface(nanmean(sineOptP_err,2), nanstd(sineOptP_err,0,2), (0:203), [0.9290, 0.6940, 0.1250])
plot((0:203), nanmean(filtPls_err,2), 'Linewidth', 1.5, 'Color', [0, 0.4470, 0.7410])
plot((0:203), nanmean(sineOpt_err,2), 'Linewidth', 1.5, 'Color', [0.8500, 0.3250, 0.0980])
plot((0:203), nanmean(sineOptP_err,2), 'Linewidth', 1.5, 'Color', [0.9290, 0.6940, 0.1250])
title("Mean error profile with standard deviation")
xlabel('Time (s)')
ylabel('Force (N)')
xlim([0,203])
subplot(2,2,4)
hold on
plotStdSurface(nanmean(filtPls_err,2), nanstd(filtPls_err,0,2), (0:203), [0, 0.4470, 0.7410])
plotStdSurface(nanmean(sineOptP_err,2), nanstd(sineOptP_err,0,2), (0:203), [0.9290, 0.6940, 0.1250])
plotStdSurface(nanmean(sineOptM_err,2), nanstd(sineOptM_err,0,2), (0:203), [0.4940, 0.1840, 0.5560])
plot((0:203), nanmean(filtPls_err,2), 'Linewidth', 1.5, 'Color', [0, 0.4470, 0.7410])
plot((0:203), nanmean(sineOptP_err,2), 'Linewidth', 1.5, 'Color', [0.9290, 0.6940, 0.1250])
plot((0:203), nanmean(sineOptM_err,2), 'Linewidth', 1.5, 'Color', [0.4940, 0.1840, 0.5560])
title("Mean error profile with standard deviation")
xlabel('Time (s)')
ylabel('Force (N)')
xlim([0,203])

%% Nonlinear solver method comparisons
figure('DefaultAxesFontSize',14)
subplot(2,2,1)
hold on
histogram(sineOptP_err, 'BinWidth', 0.1, 'FaceColor', [0.9290, 0.6940, 0.1250])
histogram(sineOptP_nonlin_err, 'BinWidth', 0.1, 'FaceColor', [0.75, 0.75, 0])
legend('lsqcurvefit', 'lsqnonlin')
xlabel('Force (N)')
title("Force estimation errors, for " + string(size(sineOpt_err,2)) + " estimations (sine +)")
subplot(2,2,2)
hold on
histogram(sineOptM_err, 'BinWidth', 0.1, 'FaceColor', [0.4940, 0.1840, 0.5560])
histogram(sineOptM_nonlin_err, 'BinWidth', 0.1, 'FaceColor', [0.6350, 0.0780, 0.1840])
legend('lsqcurvefit', 'lsqnonlin')
xlabel('Force (N)')
title("Force estimation errors, for " + string(size(sineOpt_err,2)) + " estimations (sine M)")
subplot(2,2,3)
hold on
plotStdSurface(nanmean(sineOptP_err,2), nanstd(sineOptP_err,0,2), (0:203), [0.9290, 0.6940, 0.1250])
plotStdSurface(nanmean(sineOptP_err,2), nanstd(sineOptP_nonlin_err,0,2), (0:203), [0.75, 0.75, 0])
plot((0:203), nanmean(sineOptP_err,2), 'Linewidth', 1.5, 'Color', [0.9290, 0.6940, 0.1250])
plot((0:203), nanmean(sineOptP_nonlin_err,2), 'Linewidth', 1.5, 'Color', [0.75, 0.75, 0])
title("Mean error profile with standard deviation")
xlabel('Time (s)')
ylabel('Force (N)')
xlim([0,203])
subplot(2,2,4)
hold on
plotStdSurface(nanmean(sineOptM_err,2), nanstd(sineOptM_err,0,2), (0:203), [0.4940, 0.1840, 0.5560])
plotStdSurface(nanmean(sineOptM_err,2), nanstd(sineOptM_nonlin_err,0,2), (0:203), [0.6350, 0.0780, 0.1840])
plot((0:203), nanmean(sineOptM_err,2), 'Linewidth', 1.5, 'Color', [0.4940, 0.1840, 0.5560])
plot((0:203), nanmean(sineOptM_nonlin_err,2), 'Linewidth', 1.5, 'Color', [0.6350, 0.0780, 0.1840])
title("Mean error profile with standard deviation")
xlabel('Time (s)')
ylabel('Force (N)')
xlim([0,203])

% nanmean(filtPls_err,2)
% nanstd(filtPls_err,0,2)
% nanmean(sineOptP_err,2)
% nanstd(sineOptP_err,0,2)
% nanmean(sineOpt_err,2)
% nanstd(sineOpt_err,0,2)
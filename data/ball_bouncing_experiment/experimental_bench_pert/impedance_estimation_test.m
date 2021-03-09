clear all

addpath('utils')
addpath('../../force_torque_sensor')
addpath('../../utils')

%% PARAMETERS
% MACRO
FILE_NAME = 'preliminary_experimental_data/data_vfo_3_phases.mat';
% PARAMS
% data processing
low_pass_cutoff_freq = 50; % Input signal are lp filt. before computation
filter_order = 2;
% impedance and trajectory windows 
% (the following param consider a sampling frequency of 1kHz)
wndw_virt_traj   = 200; % 200ms (position)
wndw_virt_f_traj = 065; % 65ms  (force) 
wndw_imp_eval    = 200; % 200ms       
p_delay          = 12; % 0ms
f_delay          = 0;
window = max(wndw_imp_eval, wndw_virt_traj);
%
nb_param = 3; % K B M

%% DATA LOADING

load(FILE_NAME);
var_list = who;
necessary_var = ["t","thetas", "forces_unf", "mocap_marker_robot_base", ...
    "t_dist", "dist"];

for i_var = necessary_var
    if ~any(strcmp(i_var,var_list))
        error("The data in " + FILE_NAME + " does not have all the " + ...
            "required variables. " + string(i_var) + " is missing.")
    end
end

if ~exist('dt', 'var')
    dt = 1e-3;
end

tot_nb_exp = length(t);

%% DATA PRE-PROCESSING

z = {};
fz = {};

[b,a] = butter(filter_order,low_pass_cutoff_freq/(1/(2*dt)),'low'); 

% force and position pre-processing
for exp_nb = tot_nb_exp:-1:1
    
    torques_unf = forces_unf{exp_nb}; % the torque is irrelevant here
    f_tmp = forces_filtering(forces_unf{exp_nb}', torques_unf', ...
        thetas{exp_nb}', t{exp_nb}); 
    
    z{exp_nb} = filtfilt(b,a,mocap_marker_robot_base{exp_nb}(:,3));
    fz{exp_nb} = -filtfilt(b,a,f_tmp(3,:))'; % f(r->e) = -f(e->r) = -fsens
    
end

% disturbance timing extractions
for exp_nb = tot_nb_exp:-1:1
    % get only the rising edges of perturbations
    dist_timings = t_dist{exp_nb}(1:2:end);
    dist_val{exp_nb} = dist{exp_nb}(1:2:end);
    for pert_idx = 1:length(dist_timings)
        idx_perts{exp_nb}(pert_idx) = find(t{exp_nb} >= ...
            dist_timings(pert_idx), 1, 'first');
    end
    % if the experiment was interrupted during the last perturbation
    if (idx_perts{exp_nb}(end) + window + max(p_delay,f_delay))  > length(t{exp_nb})
        % the last perturbation will no be use for impedance estimation
        idx_perts{exp_nb} = idx_perts{exp_nb}(1:end-1);
        dist_val{exp_nb} = dist_val{exp_nb}(1:end-1);
    end
end

idx_25 = idx_perts{exp_nb}(round(10.*dist_val{exp_nb}) == 101 | (round(10.*dist_val{exp_nb}) == -99 ));
idx_50 = idx_perts{exp_nb}(round(10.*dist_val{exp_nb}) == 102 | (round(10.*dist_val{exp_nb}) == -98 ));
idx_75 = idx_perts{exp_nb}(round(10.*dist_val{exp_nb}) == 103 | (round(10.*dist_val{exp_nb}) == -97 ));

figure('DefaultAxesFontSize',14)
hold on
plot(t{exp_nb}, z{exp_nb})
plot(t{exp_nb}(idx_25), z{exp_nb}(idx_25), 'xr')
plot(t{exp_nb}(idx_50), z{exp_nb}(idx_50), 'xg')
plot(t{exp_nb}(idx_75), z{exp_nb}(idx_75), 'xb')

%% DATA PROCESSING 

min_delay = 5;

for p_delay = 15:-1:min_delay
if p_delay > 8 && p_delay < 13
    continue
end
% delta_z = {};
% delta_fz = {};
% impedance = {};
% impedance_arx = {};

methods_names = ["filter+", "sineOpt+", "5sineOpt+", "5sineNlcOpt+", ...
    "3sineOpt+", "3sineNlcOpt+", "sineOptM", "3sineNlcOptM", "3sineOptM", ...
    "2sineNlcOptM", "2sineOptM"];

% extraction of the delta of position and force
for exp_nb = tot_nb_exp:-1:1
    % delta z and its derivatives
    delta_z{exp_nb} = DIFF_TRAJECT(window, wndw_virt_traj, z{exp_nb}, ...
        t{exp_nb}, idx_perts{exp_nb}, dist_val{exp_nb}, p_delay);
    delta_z{exp_nb}.computeDiffTraject('VirtTrajMethod', 'spline');
    delta_z{exp_nb}.computeDerivatives();
    % delta fz
    delta_fz{exp_nb,1} = DIFF_TRAJECT(window, wndw_virt_f_traj, fz{exp_nb},...
        t{exp_nb}, idx_perts{exp_nb}, dist_val{exp_nb}, f_delay, "filter+");
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

% impedance computation
for exp_nb = tot_nb_exp:-1:1   
    impedance{exp_nb,1} = IMPEDANCE_DATA(nb_param, length(idx_perts{exp_nb}),...
        wndw_imp_eval);
    impedance{exp_nb,1}.init_phi(delta_z{exp_nb}.diff_traject(1:wndw_imp_eval,:), ...
        delta_z{exp_nb}.d_diff_traject(1:wndw_imp_eval,:), ... % speed
        delta_z{exp_nb}.dd_diff_traject(1:wndw_imp_eval,:)); % acceleration
    for i = 2:length(methods_names)
        impedance{exp_nb,i} = copyObj(impedance{exp_nb,1});
        impedance{exp_nb,i}.init_y(delta_fz{exp_nb,i}.diff_traject(1:wndw_imp_eval,:));
    end
    impedance{exp_nb,1}.init_y(delta_fz{exp_nb,1}.diff_traject(1:wndw_imp_eval,:));   
    impedance{exp_nb,1}.lsq('NulInitialCond'); % least square optimization evaluation
    for i = 1:length(methods_names)
        impedance{exp_nb,i}.arx('NulInitialCond');
        impedance{exp_nb,i}.causalSim(dt,'NulInitialCond'); 
    end
end

data(p_delay-min_delay+1).delta_fz = delta_fz;
data(p_delay-min_delay+1).delta_z = delta_z;
data(p_delay-min_delay+1).impedance = impedance;

end

% save('imp_test_06.mat','data'); % raise error (corrupt file?)
save('imp_test_07.mat','data','-v7.3');
 
return

%% DATA POST-PROCESSING (statistical analysis)
c = clock;

stiff = [];
damp = [];
mass = [];
rho = [];

stiff_a = [];
damp_a = [];
mass_a = [];
rho_a = [];

nrmse_f = [];
nrmse_f_a = [];
nrmse_p = [];
nrmse_p_a = [];

stiff_aS = [];
damp_aS = [];
mass_aS = [];
rho_aS = [];      
stiff_aSP = [];
damp_aSP = [];
mass_aSP = [];
rho_aSP = [];     
stiff_aSM = [];
damp_aSM = [];
mass_aSM = [];
rho_aSM = [];

nrmse_p_aS = [];
nrmse_p_aSP = [];
nrmse_p_aSM = [];

opt_param_SP = [];
opt_param_SM = [];
    
% pert cycle
idx_25 = [];
idx_50 = [];
idx_75 = [];

for exp_nb = 1:tot_nb_exp  
    stiff = [stiff, impedance{exp_nb}.xi(1,:)];
    damp = [damp, impedance{exp_nb}.xi(2,:)];
    mass = [mass, impedance{exp_nb}.xi(3,:)];
    rho = [rho, impedance{exp_nb}.xi(4,:)];   
    stiff_a = [stiff_a, impedance_arx{exp_nb}.xi(1,:)];
    damp_a = [damp_a, impedance_arx{exp_nb}.xi(2,:)];
    mass_a = [mass_a, impedance_arx{exp_nb}.xi(3,:)];
    rho_a = [rho_a, impedance_arx{exp_nb}.xi(4,:)];  
    % rec errors
    nrmse_f = [nrmse_f, impedance{exp_nb}.nrmse];
    nrmse_f_a = [nrmse_f_a, impedance_arx{exp_nb}.nrmse];
    nrmse_p = [nrmse_p, impedance{exp_nb}.nrmse_pos];
    nrmse_p_a = [nrmse_p_a, impedance_arx{exp_nb}.nrmse_pos];
    % other trajectory estimation methods
    stiff_aS = [stiff_aS, impedance_arxS{exp_nb}.xi(1,:)];
    damp_aS = [damp_aS, impedance_arxS{exp_nb}.xi(2,:)];
    mass_aS = [mass_aS, impedance_arxS{exp_nb}.xi(3,:)];
    rho_aS = [rho_aS, impedance_arxS{exp_nb}.xi(4,:)];      
    stiff_aSP = [stiff_aSP, impedance_arxSP{exp_nb}.xi(1,:)];
    damp_aSP = [damp_aSP, impedance_arxSP{exp_nb}.xi(2,:)];
    mass_aSP = [mass_aSP, impedance_arxSP{exp_nb}.xi(3,:)];
    rho_aSP = [rho_aSP, impedance_arxSP{exp_nb}.xi(4,:)];  
    stiff_aSM = [stiff_aSM, impedance_arxSM{exp_nb}.xi(1,:)];
    damp_aSM = [damp_aSM, impedance_arxSM{exp_nb}.xi(2,:)];
    mass_aSM = [mass_aSM, impedance_arxSM{exp_nb}.xi(3,:)];
    rho_aSM = [rho_aSM, impedance_arxSM{exp_nb}.xi(4,:)]; 
    % rec errors
    nrmse_p_aS = [nrmse_p_aS, impedance_arxS{exp_nb}.nrmse_pos];
    nrmse_p_aSP = [nrmse_p_aSP, impedance_arxSP{exp_nb}.nrmse_pos];
    nrmse_p_aSM = [nrmse_p_aSM, impedance_arxSM{exp_nb}.nrmse_pos];
    %
    opt_param_SP = [opt_param_SP, delta_fzSP{exp_nb}.opt_param];
    opt_param_SM = [opt_param_SM, delta_fzSM{exp_nb}.opt_param];
    % for perturbation classification
    idx_25 = [idx_25; (round(10.*delta_z{exp_nb}.pert_val) == 101 | round(10.*delta_z{exp_nb}.pert_val) == -99)];
    idx_50 = [idx_50; (round(10.*delta_z{exp_nb}.pert_val) == 102 | round(10.*delta_z{exp_nb}.pert_val) == -98)];
    idx_75 = [idx_75; (round(10.*delta_z{exp_nb}.pert_val) == 103 | round(10.*delta_z{exp_nb}.pert_val) == -97)];
end

% data{p_delay+1}.delay = p_delay;
% data{p_delay+1}.stiff = stiff;
% data{p_delay+1}.damp = damp;
% data{p_delay+1}.mass = mass;
% data{p_delay+1}.rho = rho;
% data{p_delay+1}.stiff_a = stiff_a;
% data{p_delay+1}.damp_a = damp_a;
% data{p_delay+1}.mass_a = mass_a;
% data{p_delay+1}.rho_a = rho_a;
% data{p_delay+1}.nrmse_f = nrmse_f;
% data{p_delay+1}.nrmse_p = nrmse_p;
% data{p_delay+1}.nrmse_f_a = nrmse_f_a;
% data{p_delay+1}.nrmse_p_a = nrmse_p_a;

data(p_delay+1).delay = p_delay;
data(p_delay+1).cycle25 = idx_25;
data(p_delay+1).cycle50 = idx_50;
data(p_delay+1).cycle75 = idx_75;
data(p_delay+1).lsq.stiff = stiff;
data(p_delay+1).lsq.damp = damp;
data(p_delay+1).lsq.mass = mass;
data(p_delay+1).lsq.rho = rho;
data(p_delay+1).lsq.nrmse_f = nrmse_f;
data(p_delay+1).lsq.nrmse_p = nrmse_p;

data(p_delay+1).arx.stiff = stiff_a;
data(p_delay+1).arx.damp = damp_a;
data(p_delay+1).arx.mass = mass_a;
data(p_delay+1).arx.rho = rho_a;
data(p_delay+1).arx.nrmse_f = nrmse_f_a;
data(p_delay+1).arx.nrmse_p = nrmse_p_a;

data(p_delay+1).arxS.stiff = stiff_aS;
data(p_delay+1).arxS.damp = damp_aS;
data(p_delay+1).arxS.mass = mass_aS;
data(p_delay+1).arxS.rho = rho_aS;
data(p_delay+1).arxS.nrmse_p = nrmse_p_aS;

data(p_delay+1).arxSP.stiff = stiff_aSP;
data(p_delay+1).arxSP.damp = damp_aSP;
data(p_delay+1).arxSP.mass = mass_aSP;
data(p_delay+1).arxSP.rho = rho_aSP;
data(p_delay+1).arxSP.nrmse_p = nrmse_p_aSP;
data(p_delay+1).arxSP.opt_param_s = opt_param_SP;

data(p_delay+1).arxSM.stiff = stiff_aSM;
data(p_delay+1).arxSM.damp = damp_aSM;
data(p_delay+1).arxSM.mass = mass_aSM;
data(p_delay+1).arxSM.rho = rho_aSM;
data(p_delay+1).arxSM.nrmse_p = nrmse_p_aSM;
data(p_delay+1).arxSM.opt_param_s = opt_param_SM;

%end

%save("imp_data_delay_sine_opt.mat",'data')

return

for i = 1:41
    mean_nrmse_p_a(i) = nanmean(rmoutliers(data{i}.nrmse_p_a));
    mean_nrmse_f(i) = nanmean(rmoutliers(data{i}.nrmse_f));
end

figure('DefaultAxesFontSize',14)
hold on
plot((0:40), 100.*mean_nrmse_p_a)
plot((0:40), 100.*mean_nrmse_f)
ylim([0,100])
legend('ARX', 'LSQ')
title('mean NRMSE against delay')
xlabel('Delay (ms)')
ylabel('%')

return

%% Display to compare LSQ and ARX methods
exp_nb = 1;
% trajectory estimation
figure
ax(1) = subplot(2,1,1);
hold on
plot(delta_z{exp_nb}.time, delta_z{exp_nb}.complete_traject)
plot(delta_z{exp_nb}.t_traject, delta_z{exp_nb}.virt_traject, 'Color', [0.8500, 0.3250, 0.0980])
plot(delta_z{exp_nb}.t_traject(3:end-2,:), impedance{exp_nb}.rec_pos + ...
    delta_z{exp_nb}.virt_traject(3:end-2,:), '--', 'Color', [0.4660, 0.6740, 0.1880])
plot(delta_z{exp_nb}.t_traject(3:end-2,:), impedance_arx{exp_nb}.rec_pos + ...
    delta_z{exp_nb}.virt_traject(3:end-2,:), '--', 'Color', [0.6350, 0.0780, 0.1840])
ylim([-5,5]);
title('Position')
legend('meas.', 'virtual')
xlabel('Time (s)')
ylabel('Distance (m)')
ax(2) = subplot(2,1,2);
hold on
plot(delta_fz{exp_nb}.time, delta_fz{exp_nb}.complete_traject)
plot(delta_fz{exp_nb}.t_traject, delta_fz{exp_nb}.virt_traject, 'Color', [0.8500, 0.3250, 0.0980])
plot(delta_fz{exp_nb}.t_traject(3:end-2,:), impedance{exp_nb}.rec_y + ...
    delta_fz{exp_nb}.virt_traject(3:end-2,:), '--', 'Color', [0.4660, 0.6740, 0.1880])
plot(delta_fz{exp_nb}.t_traject(3:end-2,:), impedance_arx{exp_nb}.rec_y + ...
    delta_fz{exp_nb}.virt_traject(3:end-2,:), '--', 'Color', [0.6350, 0.0780, 0.1840])
title('Force')
legend('meas.', 'virtual')
xlabel('Time (s)')
ylabel('Force (N)')
linkaxes(ax,'x')

% parameters identification
figure('DefaultAxesFontSize',14)
subplot(3,1,1)
hold on
plot(stiff)
plot(stiff_a)
legend('LSQ', 'ARX')
subplot(3,1,2)
hold on
plot(damp)
plot(damp_a)
legend('LSQ', 'ARX')
subplot(3,1,3)
hold on
plot(mass)
plot(mass_a)
legend('LSQ', 'ARX')

disp('Stiffness')
[nanmean(rmoutliers(stiff)), nanmean(rmoutliers(stiff_a))]
[nanstd(rmoutliers(stiff)), nanstd(rmoutliers(stiff_a))]
disp('Damping')
[nanmean(rmoutliers(damp)), nanmean(rmoutliers(damp_a))]
[nanstd(rmoutliers(damp)), nanstd(rmoutliers(damp_a))]
disp('Mass')
[nanmean(rmoutliers(mass)), nanmean(rmoutliers(mass_a))]
[nanstd(rmoutliers(mass)), nanstd(rmoutliers(mass_a))]
disp('NRMSE Force')
[nanmean(rmoutliers(nrmse_f)), nanmean(rmoutliers(nrmse_f_a))]
[nanstd(rmoutliers(nrmse_f)), nanstd(rmoutliers(nrmse_f_a))]
disp('NRMSE Position')
[nanmean(rmoutliers(nrmse_p)), nanmean(rmoutliers(nrmse_p_a))]
[nanstd(rmoutliers(nrmse_p)), nanstd(rmoutliers(nrmse_p_a))]

all_imp_arx = [impedance_arx{:}];
all_imp_lsq = [impedance{:}];

% 
figure
subplot(2,1,1)
hold on
p1 = plot([all_imp_lsq.rmse]);
p2 = plot([all_imp_arx.rmse]);
ylim([0,50]);
title('RMSE reconstruction errors')
legend([p1,p2],{'LSQ', 'ARX'})
subplot(2,1,2)
hold on
p1 = plot([all_imp_lsq.r_2]);
yyaxis right
p2 = plot([all_imp_arx.r_2]);
title('R^2 scores')
legend([p1,p2],{'LSQ', 'ARX'})
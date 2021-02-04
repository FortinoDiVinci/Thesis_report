% equivalent of the live script impedance_identification
% this script was created for quicker execution

clear all
%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Problem definition
syms Ms Bs Ks;
% impedance state space repr.
A = [  0 ,   1 ; -Ks/Ms, -Bs/Ms];
B = [0; 1/Ms];
C = [1, 0];
% discretization
dt = 1e-3; % 1ms
Ad = expm(A*dt);
Bd = A\(Ad - eye(2))*B;
Cd = C;
% discrete transfert function
syms zL;
Hd = simplify(Cd*(zL*eye(2) - Ad)^(-1)*Bd);
%syms sig1
%Hd = subs(Hd,(Bs^2 - 4*Ks*Ms)^(1/2), sig1); % for readability
[n,d] = numden(Hd);
n = collect(n,zL); % rearange expr.
d = collect(d,zL);
b_coef = flip(coeffs(n,zL)); % high order first
a_coef = flip(coeffs(d,zL)); % high order first
disp("Numerator order: " + string(length(b_coef)-1))
disp("Denominator order: " + string(length(a_coef)-1))
a1 = a_coef(2)/a_coef(1); % z coef
a0 = a_coef(3)/a_coef(1); % 1 coef
b0 = b_coef(2)/b_coef(1); % num
Kh = b_coef(1)/a_coef(1); % gain
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Identification of the coefficients
na = length(a_coef)-1; % order of Ak
nb = length(b_coef)-1; % order of Bk 
nk = 0; % delay in Bk
%% data loading
load 'data_2020_Nov_17/data_without_impacts_2020_11_17.mat' 't' ...
    'thetas' 'forces_unf' % real data
addpath('utils') 
addpath('../../force_torque_sensor') % for force pre-processing
addpath('../../utils') %
% force signal processing
exp_nb = randi([1 15],1,1); % chose a random experimental dataset
t = t{exp_nb} - t{exp_nb}(1); % t0 = 0s
f_tmp = forces_filtering(forces_unf{exp_nb}', forces_unf{exp_nb}', ...
    thetas{exp_nb}', t); % from sensor base to robot base
[b,a] = butter(2,50/(1/(2*dt)),'low'); % BW 2nd order low pass filter (cutoff freq. 50 Hz)
f_tmp = -f_tmp(3,:)'; % fz conversion from f(e->r) to f(r->e), robot force on the environment
f = filtfilt(b,a,f_tmp); % zero phase digital filtering
clear forces_unf thetas f_tmp % clear unecessary data
%% Simulating data with real input
input_force.signals.values = f;
input_force.time = t;
% impedance parameters that will be identified
Mv = 0.2;
Bv = 10;
Kv = 200;
% perturbation introduced in simulation
ext_signal = 1;
pert_mag = 10; % setpoint as defined in the real experiment
pert_space = ceil(2.8/dt); % samples 2.8 sec between perturbations
pert_duration = 0.030/dt;  % samples, 30 ms
% perturbation filter
xi = sqrt(2)/2;
w0 = 40*pi*1;
K_f = 3e3;
t_max = t(end);
out = sim('Impedance_env_simulation/KBM_sim',t_max);
% Collecting simulated data
fz = out.force.data;
z = out.position.data;
t = out.force.Time;
fz0 = out.virt_force.data; % simulated virtual force
z0 = out.virt_position.data; % simulated virtual position
% get the rising edges indexes of the perturbations
pert_idx = find(diff(out.perturbations.data) > 0)';
pert_val = pert_mag.*ones(size(pert_idx));
%% Impedance identification algorithm
% PARAMETERS
wndw_virt_traj   = ceil(0.200/dt); % 200ms (position)
wndw_virt_f_traj = ceil(0.065/dt); % 65ms  (force) 
wndw_imp_eval    = ceil(0.200/dt); % 200ms       
idx_delay            = ceil(0.000/dt); % 0ms
window = max(wndw_imp_eval, wndw_virt_traj);
nb_param = 3; % K B M
% DATA PRE-PROCESSING
delta_z = DIFF_TRAJECT(window, wndw_virt_traj, z, t, pert_idx, pert_val, idx_delay);
delta_fz = DIFF_TRAJECT(window, wndw_virt_f_traj, fz, t, pert_idx, pert_val, idx_delay);
delta_z.computeDiffTraject('VirtTrajMethod', 'spline');
delta_fz.computeDiffTraject('VirtTrajMethod', 'filterPlus'); % this step might take few seconds
delta_z.computeDerivatives();
% Actual simulated virtual trajectories 
% The trajectories should be the same repeated, if the perturbation is 
% perfectly extracted
delta_z_sim = copyObj(delta_z);
delta_fz_sim = copyObj(delta_fz);
delay_wdw = delta_z.delay;
estim_wdw = delta_z.estim_window;
for i = 1:delta_z_sim.nb_traject
    pert_interval = delta_z_sim.pert_ind(i)+(-2:estim_wdw+1)+delay_wdw;
    delta_z_sim.virt_traject(:,i) = z0(pert_interval);
    delta_fz_sim.virt_traject(:,i) = fz0(pert_interval);
end
delta_z_sim.computeDiffTraject('VirtTrajMethod', 'manual');
delta_fz_sim.computeDiffTraject('VirtTrajMethod', 'manual');
%% Data formating for ARX identification
for i = delta_z.nb_traject:-1:1 
    data{i} = iddata(delta_z.diff_traject(:,i), ...
        delta_fz.diff_traject(:,i),dt);
    data{i}.TimeUnit = 's';
    data{i}.InputUnit = 'N';
    data{i}.OutputUnit = 'm'; 
end
data_sim = iddata(delta_z_sim.diff_traject(:,1),...
    delta_fz_sim.diff_traject(:,1),dt); % ideal data
data_sim.TimeUnit = 's';
data_sim.InputUnit = 'N';
data_sim.OutputUnit = 'm';
% ARX identification
for i = delta_z.nb_traject:-1:1 
    arx_id{i} = arx(data{i},[na nb+1 nk],'IntegrateNoise',true); 
    arx_id{i}.Name = 'Arx ID';
end
arx_id_sim = arx(data_sim,[na nb+1 nk]);
arx_id_sim.Name = 'Arx ID with ideal data';
% display ARX fit
for i = delta_z.nb_traject:-1:1 
    [~,arx_id_fit(i),~] = compare(data{i},arx_id{i});
end
[~,arx_id_sim_fit,~] = compare(data_sim,arx_id_sim);
figure('DefaultAxesFontSize',14)
hold on
plot(arx_id_fit)
plot([1, delta_z.nb_traject], [arx_id_sim_fit, arx_id_sim_fit])
title('Fit %')
legend('arx', 'arx with ideal data')
% K D M identification
assume(Ks, 'real');
assume(Bs, 'real');
assume(Ms, 'real');
%assume(sig1, 'real');
for i = 1:length(arx_id)
    % declare the equations system
    eq.A(1) = a1 - arx_id{i}.A(2)/arx_id{i}.A(1) == 0;
    eq.A(2) = a0 - arx_id{i}.A(3)/arx_id{i}.A(1) == 0;
    eq.B(1) = Kh*(b0 + 1) - (arx_id{i}.B(1) + arx_id{i}.B(2))/arx_id{i}.A(1) == 0;
    %eq.A(1) = subs(eq.A(1), sig1, (Bs^2 - 4*Ks*Ms)^(1/2));
    %eq.A(2) = subs(eq.A(2), sig1, (Bs^2 - 4*Ks*Ms)^(1/2));
    %eq.B(1) = subs(eq.B(1), sig1, (Bs^2 - 4*Ks*Ms)^(1/2));
    imp_param_id(i) = solve({eq.A(1),eq.A(2),eq.B(1)},[Ks;Bs;Ms]);
end
% ideal scenario with known virtual trajectories
eq.A(1) = a1 - arx_id_sim.A(2)/arx_id_sim.A(1) == 0;
eq.A(2) = a0 - arx_id_sim.A(3)/arx_id_sim.A(1) == 0;
eq.B(1) = Kh*(b0 + 1) - (arx_id_sim.B(1) + arx_id_sim.B(2))/arx_id_sim.A(1) == 0;
%eq.A(1) = subs(eq.A(1), sig1, (Bs^2 - 4*Ks*Ms)^(1/2));
%eq.A(2) = subs(eq.A(2), sig1, (Bs^2 - 4*Ks*Ms)^(1/2));
%eq.B(1) = subs(eq.B(1), sig1, (Bs^2 - 4*Ks*Ms)^(1/2));
imp_param_id_sim = vpasolve([eq.A(1),eq.A(2),eq.B(1)],[Ks;Bs;Ms]);
% reverse solving: knowing the real parameters, what should the ARX
% coefficient be ?
a1_r = double(subs(a1, [Ks;Bs;Ms], [Kv;Bv;Mv]));
a0_r = double(subs(a0, [Ks;Bs;Ms], [Kv;Bv;Mv]));
kh_r = double(subs(Kh, [Ks;Bs;Ms], [Kv;Bv;Mv]));
b0_r = double(subs(b0, [Ks;Bs;Ms], [Kv;Bv;Mv]));
khb0_r = double(subs(Kh*(b0 + 1), [Ks;Bs;Ms], [Kv;Bv;Mv]));
%% Least square methodology 
nb_id = min(delta_z.nb_traject, delta_fz.nb_traject);
impedance = IMPEDANCE_DATA(3, nb_id,wndw_imp_eval);
impedance.init_phi(delta_z.diff_traject(1:wndw_imp_eval,:), ...
    delta_z.d_diff_traject(1:wndw_imp_eval,:), ... % speed
    delta_z.dd_diff_traject(1:wndw_imp_eval,:)); % acceleration
impedance.init_y(delta_fz.diff_traject(1:wndw_imp_eval,:));
impedance.lsq(); % least square optimization evaluation
% ideal
impedance_sim = IMPEDANCE_DATA(3, nb_id,wndw_imp_eval);
impedance_sim.init_phi(delta_z_sim.diff_traject(1:wndw_imp_eval,:), ...
    delta_z_sim.d_diff_traject(1:wndw_imp_eval,:), ... % speed
    delta_z_sim.dd_diff_traject(1:wndw_imp_eval,:)); % acceleration
impedance_sim.init_y(delta_fz_sim.diff_traject(1:wndw_imp_eval,:));
impedance_sim.lsq(); % least square optimization evaluation
% The arx method was implemented in the IMPEDANCE_DATA class
impedance_arx = copyObj(impedance);
impedance_arx.arx();
%
for ii = impedance_arx.nb_id:-1:1
    % coeff relative err
    a1_arx = impedance_arx.arx_id{ii}.A(2)/impedance_arx.arx_id{ii}.A(1);
    a0_arx = impedance_arx.arx_id{ii}.A(3)/impedance_arx.arx_id{ii}.A(1);
    b_arx = (impedance_arx.arx_id{ii}.B(1) + impedance_arx.arx_id{ii}.B(2))/...
        impedance_arx.arx_id{ii}.A(1);
    kh_arx = impedance_arx.arx_id{ii}.B(1)/impedance_arx.arx_id{ii}.A(1);
    b0_arx = impedance_arx.arx_id{ii}.B(2)/impedance_arx.arx_id{ii}.B(1);
    a1_rel_e(ii) = abs(a1_r - a1_arx)/a1_r;
    a0_rel_e(ii) = abs(a0_r - a0_arx)/a0_r;
    b_rel_e(ii) = abs(khb0_r - b_arx)/khb0_r;
    b0_rel_e(ii) = abs(b0_r - b0_arx)/b0_r;
    kh_rel_e(ii) = abs(kh_r - kh_arx)/kh_r;
    % virtual trajectory root mean square err
    z0_rmse(ii) = sqrt(mean((delta_z_sim.virt_traject(:,ii) - ...
        delta_z.virt_traject(:,ii)).^2));
    fz0_rmse(ii) = sqrt(mean((delta_fz_sim.virt_traject(:,ii) - ...
        delta_fz.virt_traject(:,ii)).^2));
    % K,B,M relative err
    K_rel_e(ii) = abs(Kv - impedance_arx.xi(1,ii))/Kv;
    B_rel_e(ii) = abs(Bv - impedance_arx.xi(2,ii))/Bv;
    M_rel_e(ii) = abs(Mv - impedance_arx.xi(3,ii))/Mv;
end
% Display
figure('DefaultAxesFontSize',14)
subplot(3,1,1)
title('ARX Identification')
hold on
plot(a1_rel_e.*100, 'DisplayName', 'a_1 RE')
plot(a0_rel_e.*100, 'DisplayName', 'a_0 RE')
%plot(b0_rel_e.*100, 'DisplayName', 'b_0 RE')
%plot(kh_rel_e.*100, 'DisplayName', 'Kh RE')
plot(b_rel_e.*100, 'DisplayName', 'Kh(1+b_0) RE')
ylabel('% E')
yyaxis right
plot((1-impedance_arx.r_2).*100, 'DisplayName', '1-R^2')
ylabel('%')
legend show
subplot(3,1,2)
title('Virtual trajectories errors')
hold on
plot(z0_rmse.*100, '.-', 'DisplayName', 'z_0 RMSE')
ylabel('cm')
yyaxis right
plot(fz0_rmse, '.-', 'DisplayName', 'fz_0 RMSE')
ylabel('N')
legend show
subplot(3,1,3)
title('Impedance parameters errors')
hold on
plot(impedance_arx.rel_std(1, :), 'DisplayName', 'K std RE')
plot(impedance_arx.rel_std(2, :), 'DisplayName', 'B std RE')
plot(impedance_arx.rel_std(3, :), 'DisplayName', 'M std RE')
yyaxis right
plot(K_rel_e.*100, 'DisplayName', 'K RE')
plot(B_rel_e.*100, 'DisplayName', 'B RE')
plot(M_rel_e.*100, 'DisplayName', 'M RE')
ylabel('%')
legend show
%
figure('DefaultAxesFontSize',14)
subplot(3,1,1)
title('Stiffness')
hold on
plot([imp_param_id(:).Ks])
plot([1 length(arx_id)], [imp_param_id_sim.Ks, imp_param_id_sim.Ks])
plot(impedance_arx.xi(1,:), '--')
plot(impedance.xi(1,:))
plot([1,nb_id], [impedance_sim.xi(1), impedance_sim.xi(1)])
xlabel('Identification nb')
ylabel('N/m')
legend('ARX','ideal ARX','new ARX','LSQ','ideal LSQ')
subplot(3,1,2)
title('Damping')
hold on
plot([imp_param_id(:).Bs])
plot([1 length(arx_id)], [imp_param_id_sim.Bs, imp_param_id_sim.Bs])
plot(impedance_arx.xi(2,:), '--')
plot(impedance.xi(2,:))
plot([1,nb_id], [impedance_sim.xi(2), impedance_sim.xi(2)])
xlabel('Identification nb')
ylabel('N.s/m')
legend('ARX','ideal ARX','new ARX','LSQ','ideal LSQ')
subplot(3,1,3)
title('Mass')
hold on
plot([imp_param_id(:).Ms])
plot([1 length(arx_id)], [imp_param_id_sim.Ms, imp_param_id_sim.Ms])
plot(impedance_arx.xi(3,:), '--')
plot(impedance.xi(3,:))
plot([1,nb_id], [impedance_sim.xi(3), impedance_sim.xi(3)])
xlabel('Identification nb')
ylabel('kg')
legend('ARX','ideal ARX','new ARX','LSQ','ideal LSQ')
%% Comparison between the 2 methods
disp("In the ideal scenario where the virtual trajectories are known, "...
    +"the ARX method gives for each param a relative error of: ")
disp("K: " + sprintf('%.3f',double(abs(imp_param_id_sim.Ks-Kv)/Kv)*100) + "%")
disp("B: " + sprintf('%.3f',double(abs(imp_param_id_sim.Bs-Bv)/Bv)*100) + "%")
disp("M: " + sprintf('%.3f',double(abs(imp_param_id_sim.Ms-Mv)/Mv)*100) + "%")
disp("And for the LSQ method: ")
disp("K: " + sprintf('%.3f',abs(impedance_sim.xi(1)-Kv)/Kv*100) + "%")
disp("B: " + sprintf('%.3f',abs(impedance_sim.xi(2)-Bv)/Bv*100) + "%")
disp("M: " + sprintf('%.3f',abs(impedance_sim.xi(3)-Mv)/Mv*100) + "%")
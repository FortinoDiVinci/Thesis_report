clear all

addpath('../')
addpath('../utils/')
addpath('../../../youBot_analysis/Utils')

load('imp_test_05.mat')

%% impedance

% original online phase sorting (intended phase spread)
for exp_nb = 1:length(data(1).delta_z)
    idx_25{:,exp_nb} = (round(10.*data(1).delta_z{exp_nb}.pert_val) == 101 | ...
        round(10.*data(1).delta_z{exp_nb}.pert_val) == -99);
    idx_50{:,exp_nb} = (round(10.*data(1).delta_z{exp_nb}.pert_val) == 102 | ...
        round(10.*data(1).delta_z{exp_nb}.pert_val) == -98);
    idx_75{:,exp_nb} = (round(10.*data(1).delta_z{exp_nb}.pert_val) == 103 | ...
        round(10.*data(1).delta_z{exp_nb}.pert_val) == -97);
end

%% classic phase diagram

figure('DefaultAxesFontSize',14)
for ii = 1:4
    subplot(2,4,ii)
    hold on
    time = data(1).delta_z{ii}.time;
    z = data(1).delta_z{ii}.complete_traject;
    dz = Iu_diffcent(z, time);
    idx_pert = data(1).delta_z{ii}.pert_ind;
    plot(z, dz);
    plot(z(idx_pert(idx_25{:,ii})), dz(idx_pert(idx_25{:,ii})), 'o', ...
        'Color', [0.8500, 0.3250, 0.0980], 'MarkerFaceColor', [0.8500, 0.3250, 0.0980]);
    plot(z(idx_pert(idx_50{:,ii})), dz(idx_pert(idx_50{:,ii})), 'o', ...
        'Color', [0.9290, 0.6940, 0.1250], 'MarkerFaceColor', [0.9290, 0.6940, 0.1250]);
    plot(z(idx_pert(idx_75{:,ii})), dz(idx_pert(idx_75{:,ii})), 'o', ...
        'Color', [0.4940, 0.1840, 0.5560], 'MarkerFaceColor', [0.4940, 0.1840, 0.5560]);
end
for ii = 5:7
    subplot(2,3,ii-1)
    hold on
    time = data(1).delta_z{ii}.time;
    z = data(1).delta_z{ii}.complete_traject;
    dz = Iu_diffcent(z, time);
    idx_pert = data(1).delta_z{ii}.pert_ind;
    plot(z, dz);
    plot(z(idx_pert(idx_25{:,ii})), dz(idx_pert(idx_25{:,ii})), 'o', ...
        'Color', [0.8500, 0.3250, 0.0980], 'MarkerFaceColor', [0.8500, 0.3250, 0.0980]);
    plot(z(idx_pert(idx_50{:,ii})), dz(idx_pert(idx_50{:,ii})), 'o', ...
        'Color', [0.9290, 0.6940, 0.1250], 'MarkerFaceColor', [0.9290, 0.6940, 0.1250]);
    plot(z(idx_pert(idx_75{:,ii})), dz(idx_pert(idx_75{:,ii})), 'o', ...
        'Color', [0.4940, 0.1840, 0.5560], 'MarkerFaceColor', [0.4940, 0.1840, 0.5560]);
end

%% force phase diagram

figure('DefaultAxesFontSize',14)
for ii = 1:4
    subplot(2,4,ii)
    hold on
    time = data(1).delta_fz{ii}.time;
    fz = data(1).delta_fz{ii}.complete_traject;
    dfz = Iu_diffcent(fz, time);
    idx_pert = data(1).delta_fz{ii}.pert_ind;
    plot(fz, dfz);
    plot(fz(idx_pert(idx_25{:,ii})), dfz(idx_pert(idx_25{:,ii})), 'o', ...
        'Color', [0.8500, 0.3250, 0.0980], 'MarkerFaceColor', [0.8500, 0.3250, 0.0980]);
    plot(fz(idx_pert(idx_50{:,ii})), dfz(idx_pert(idx_50{:,ii})), 'o', ...
        'Color', [0.9290, 0.6940, 0.1250], 'MarkerFaceColor', [0.9290, 0.6940, 0.1250]);
    plot(fz(idx_pert(idx_75{:,ii})), dfz(idx_pert(idx_75{:,ii})), 'o', ...
        'Color', [0.4940, 0.1840, 0.5560], 'MarkerFaceColor', [0.4940, 0.1840, 0.5560]);
end
for ii = 5:7
    subplot(2,3,ii-1)
    hold on
    time = data(1).delta_fz{ii}.time;
    fz = data(1).delta_fz{ii}.complete_traject;
    dfz = Iu_diffcent(fz, time);
    idx_pert = data(1).delta_fz{ii}.pert_ind;
    plot(fz, dfz);
    plot(fz(idx_pert(idx_25{:,ii})), dfz(idx_pert(idx_25{:,ii})), 'o', ...
        'Color', [0.8500, 0.3250, 0.0980], 'MarkerFaceColor', [0.8500, 0.3250, 0.0980]);
    plot(fz(idx_pert(idx_50{:,ii})), dfz(idx_pert(idx_50{:,ii})), 'o', ...
        'Color', [0.9290, 0.6940, 0.1250], 'MarkerFaceColor', [0.9290, 0.6940, 0.1250]);
    plot(fz(idx_pert(idx_75{:,ii})), dfz(idx_pert(idx_75{:,ii})), 'o', ...
        'Color', [0.4940, 0.1840, 0.5560], 'MarkerFaceColor', [0.4940, 0.1840, 0.5560]);
end

%%

figure('DefaultAxesFontSize',14)
subplot(1,2,1)
hold on
for ii = 1:7
    time = data(1).delta_z{ii}.time;
    pz = data(1).delta_z{ii}.complete_traject;
    dpz = Iu_diffcent(pz, time);
    idx_pert = data(1).delta_z{ii}.pert_ind;
    plot(pz, dpz, 'Color', [0, 0.4470, 0.7410,0.4]);   
end
for ii = 1:7
    time = data(1).delta_z{ii}.time;
    z = data(1).delta_z{ii}.complete_traject;
    dz = Iu_diffcent(z, time);
    idx_pert = data(1).delta_z{ii}.pert_ind;
    plot(z(idx_pert(idx_25{:,ii})), dz(idx_pert(idx_25{:,ii})), 'o', ...
        'Color', [0.8500, 0.3250, 0.0980], 'MarkerFaceColor', [0.8500, 0.3250, 0.0980]);
    plot(z(idx_pert(idx_50{:,ii})), dz(idx_pert(idx_50{:,ii})), 'o', ...
        'Color', [0.9290, 0.6940, 0.1250], 'MarkerFaceColor', [0.9290, 0.6940, 0.1250]);
    plot(z(idx_pert(idx_75{:,ii})), dz(idx_pert(idx_75{:,ii})), 'o', ...
        'Color', [0.4940, 0.1840, 0.5560], 'MarkerFaceColor', [0.4940, 0.1840, 0.5560]);
end
title('Position')
subplot(1,2,2)
hold on
for ii = 1:7
    time = data(1).delta_fz{ii}.time;
    fz = data(1).delta_fz{ii}.complete_traject;
    dfz = Iu_diffcent(fz, time);
    idx_pert = data(1).delta_fz{ii}.pert_ind;
    plot(fz, dfz, 'Color', [0, 0.4470, 0.7410,0.4]);   
end
for ii = 1:7
    time = data(1).delta_fz{ii}.time;
    fz = data(1).delta_fz{ii}.complete_traject;
    dfz = Iu_diffcent(fz, time);
    idx_pert = data(1).delta_fz{ii}.pert_ind;
    plot(fz(idx_pert(idx_25{:,ii})), dfz(idx_pert(idx_25{:,ii})), 'o', ...
        'Color', [0.8500, 0.3250, 0.0980], 'MarkerFaceColor', [0.8500, 0.3250, 0.0980]);
    plot(fz(idx_pert(idx_50{:,ii})), dfz(idx_pert(idx_50{:,ii})), 'o', ...
        'Color', [0.9290, 0.6940, 0.1250], 'MarkerFaceColor', [0.9290, 0.6940, 0.1250]);
    plot(fz(idx_pert(idx_75{:,ii})), dfz(idx_pert(idx_75{:,ii})), 'o', ...
        'Color', [0.4940, 0.1840, 0.5560], 'MarkerFaceColor', [0.4940, 0.1840, 0.5560]);
end
title('Force')

%% Force traject validation

clear all

addpath('../')
addpath('../utils/')
addpath('../../../youBot_analysis/Utils')

load('../trajectory_prediction_evaluation/force_trajectory_optimization/test_sine_opt_5.mat')
load('../data_2020_Nov_17/data_without_impacts_2020_11_17.mat', 'mocap_marker_robot_base')

dt = 1e-3;
[b,a] = butter(2,50/(1/(2*dt)),'low'); 
for exp_nb = 2:size(delta_fz,1)
    z{exp_nb} = filtfilt(b,a,mocap_marker_robot_base{exp_nb}(:,3));
end

for exp_nb = 2:size(delta_fz,1)
    idx_25{:,exp_nb} = (strcmp(phase{exp_nb}, "upper pk"));
    idx_50{:,exp_nb} = (strcmp(phase{exp_nb}, "decreas"));
    idx_75{:,exp_nb} = (strcmp(phase{exp_nb}, "lower pk"));
end

figure('DefaultAxesFontSize',14)
subplot(1,2,1)
hold on
for ii = 2:size(delta_fz,2)
    time = delta_fz{ii}.time;
    dz = Iu_diffcent(z{ii}, time);
    plot(z{ii}, dz, 'Color', [0, 0.4470, 0.7410,0.4]);
end
for ii = 2:size(delta_fz,2)
    time = delta_fz{ii}.time;
    dz = Iu_diffcent(z{ii}, time);
    idx_pert = delta_fz{ii}.pert_ind;
    scatter(z{ii}(idx_pert(idx_25{:,ii})), dz(idx_pert(idx_25{:,ii})), ...
        'filled', 'MarkerFaceAlpha', 0.75, 'MarkerFaceColor', [0.8500, 0.3250, 0.0980]);
    scatter(z{ii}(idx_pert(idx_50{:,ii})), dz(idx_pert(idx_50{:,ii})), ...
        'filled', 'MarkerFaceAlpha', 0.75, 'MarkerFaceColor', [0.9290, 0.6940, 0.1250]);
    scatter(z{ii}(idx_pert(idx_75{:,ii})), dz(idx_pert(idx_75{:,ii})), ...
        'filled', 'MarkerFaceAlpha', 0.75, 'MarkerFaceColor', [0.4940, 0.1840, 0.5560]);
end
title('Position phase')
xlabel('Position')
ylabel('Velocity')
subplot(1,2,2)
hold on
for ii = 2:size(delta_fz,2)
    time = delta_fz{ii}.time;
    fz = delta_fz{ii}.complete_traject;
    dfz = Iu_diffcent(fz, time);
    plot(fz, dfz, 'Color', [0, 0.4470, 0.7410,0.4]);
end
for ii = 2:size(delta_fz,2)
    time = delta_fz{ii}.time;
    fz = delta_fz{ii}.complete_traject;
    dfz = Iu_diffcent(fz, time);
    idx_pert = delta_fz{ii}.pert_ind;
    scatter(fz(idx_pert(idx_25{:,ii})), dfz(idx_pert(idx_25{:,ii})), ...
        'filled', 'MarkerFaceAlpha', 0.75, 'MarkerFaceColor', [0.8500, 0.3250, 0.0980]);
    scatter(fz(idx_pert(idx_50{:,ii})), dfz(idx_pert(idx_50{:,ii})), ...
        'filled', 'MarkerFaceAlpha', 0.75, 'MarkerFaceColor', [0.9290, 0.6940, 0.1250]);
    scatter(fz(idx_pert(idx_75{:,ii})), dfz(idx_pert(idx_75{:,ii})), ...
        'filled', 'MarkerFaceAlpha', 0.75, 'MarkerFaceColor', [0.4940, 0.1840, 0.5560]);
end
title('Force phase')
xlabel('Force')
ylabel('dF/dt')

%% kmean sorting

method_i = 11;
tmp = [delta_fz{2:end,11}];
X(:,1) = vertcat(z{2:end});
X(:,2) = Iu_diffcent(X(:,1), vertcat(tmp.time));
X(:,3) = vertcat(tmp.complete_traject);
X(:,4) = Iu_diffcent(X(:,3), vertcat(tmp.time));

Xz = [];
Xdz = [];
Xfz = [];
Xdfz = [];
for ii = 2:size(delta_fz,1)
    idx_pert = delta_fz{ii,method_i}.pert_ind;
    dz = Iu_diffcent(z{ii}, delta_fz{ii,method_i}.time);
    dfz = Iu_diffcent(delta_fz{ii,method_i}.complete_traject, delta_fz{ii,method_i}.time);
    Xz = [Xz; z{ii}(idx_pert)];
    Xdz = [Xdz; dz(idx_pert)];
    Xfz = [Xfz; delta_fz{ii,method_i}.complete_traject(idx_pert)];
    Xdfz = [Xdfz; dfz(idx_pert)]; 
end

Xp = [Xz, Xdz, Xfz, Xdfz];
[idx,C] = kmeans(Xp,3);

figure('DefaultAxesFontSize',14)
subplot(1,2,1)
hold on
plot(X(:,1), X(:,2), 'Color', [0, 0.4470, 0.7410,0.4])
scatter(Xz(idx==1), Xdz(idx==1), 'filled', 'MarkerFaceAlpha', 0.75, ...
    'MarkerFaceColor', [0.8500, 0.3250, 0.0980]);
scatter(Xz(idx==2), Xdz(idx==2), 'filled', 'MarkerFaceAlpha', 0.75, ...
    'MarkerFaceColor',  [0.9290, 0.6940, 0.1250]);
scatter(Xz(idx==3), Xdz(idx==3), 'filled', 'MarkerFaceAlpha', 0.75, ...
    'MarkerFaceColor',  [0.4940, 0.1840, 0.5560]);
subplot(1,2,2)
hold on
plot(X(:,3), X(:,4), 'Color', [0, 0.4470, 0.7410,0.4])
scatter(Xfz(idx==1), Xdfz(idx==1), 'filled', 'MarkerFaceAlpha', 0.75, ...
    'MarkerFaceColor', [0.8500, 0.3250, 0.0980]);
scatter(Xfz(idx==2), Xdfz(idx==2), 'filled', 'MarkerFaceAlpha', 0.75, ...
    'MarkerFaceColor',  [0.9290, 0.6940, 0.1250]);
scatter(Xfz(idx==3), Xdfz(idx==3), 'filled', 'MarkerFaceAlpha', 0.75, ...
    'MarkerFaceColor',  [0.4940, 0.1840, 0.5560]);

%%

% get the 20 closest neighboor for every perturbation (in term of position,
% velocity, force and force derivative)
nb_neigh = 20;
idx_neigh = knnsearch(Xp(:,3:4), Xp(:,3:4), 'K', nb_neigh+1);
idx_neigh(:,1) = []; % delete the point itself as closest neighbour

% original perturbation sorting
phase_old = vertcat(phase{2:end});
phase_new = phase_old;
phase_name = ["upper pk", "decreas", "lower pk"];

% correct phase sorting according to the 20 closest neighbours
for ii = 1:length(phase_old)
    idx_i_neigh = idx_neigh(ii,:);
    i_neigh_phase = phase_old(idx_i_neigh);
    cnt(1) = sum(strcmp(i_neigh_phase, phase_name(1)));
    cnt(2) = sum(strcmp(i_neigh_phase, phase_name(2)));
    cnt(3) = sum(strcmp(i_neigh_phase, phase_name(3)));
    [~, m_i] = max(cnt);
    if ~strcmp(phase_name(m_i), phase_old(ii)) && cnt(m_i) >= round(nb_neigh*0.6)
        % change the phase if at least 15 neighboors have the same phase
        phase_new(ii) = phase_name(m_i);
    end
end

fprintf('%f points were corrected', sum(~strcmp(phase_old, phase_new)))

colors(1, :) = [0.8500, 0.3250, 0.0980];
colors(2, :) = [0.9290, 0.6940, 0.1250];
colors(3, :) = [0.4940, 0.1840, 0.5560];

figure('DefaultAxesFontSize',14)
subplot(2,2,1)
hold on
plot(X(:,1), X(:,2), 'Color', [0, 0.4470, 0.7410,0.4])
for i = 1:3
scatter(Xz(strcmp(phase_old, phase_name(i))), Xdz(strcmp(phase_old, phase_name(i))),...
    'filled', 'MarkerFaceAlpha', 0.75,  'MarkerFaceColor', colors(i, :));
end
title('Position old phase')
subplot(2,2,2)
hold on
plot(X(:,1), X(:,2), 'Color', [0, 0.4470, 0.7410,0.4])
for i = 1:3
scatter(Xz(strcmp(phase_new, phase_name(i))), Xdz(strcmp(phase_new, phase_name(i))),...
    'filled', 'MarkerFaceAlpha', 0.75,  'MarkerFaceColor', colors(i, :));
end
title('Position new phase')
subplot(2,2,3)
hold on
plot(X(:,3), X(:,4), 'Color', [0, 0.4470, 0.7410,0.4])
for i = 1:3
scatter(Xfz(strcmp(phase_old, phase_name(i))), Xdfz(strcmp(phase_old, phase_name(i))),...
    'filled', 'MarkerFaceAlpha', 0.75,  'MarkerFaceColor', colors(i, :));
end
title('Force old phase')
subplot(2,2,4)
hold on
plot(X(:,3), X(:,4), 'Color', [0, 0.4470, 0.7410,0.4])
for i = 1:3
scatter(Xfz(strcmp(phase_new, phase_name(i))), Xdfz(strcmp(phase_new, phase_name(i))),...
    'filled', 'MarkerFaceAlpha', 0.75,  'MarkerFaceColor', colors(i, :));
end
title('Force new phase')
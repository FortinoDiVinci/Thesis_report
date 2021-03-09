clear all

addpath('utils/')
addpath('../../youBot_analysis/Utils')

load('imp_test_05.mat')

min_delay = 5; % 
% min_delay = 9; % imp_test_03.mat

nb_max_pert = 0;
for exp_nb = 1:length(data(1).delta_z)
    nb_max_pert = max(nb_max_pert, length(data(1).delta_z{exp_nb}.pert_val));
end

% idx_25 = NaN(nb_max_pert, length(data(1).delta_z));
% idx_50 = NaN(size(idx_25));
% idx_75 = NaN(size(idx_25));

% original online phase sorting (intended phase spread)
for exp_nb = 1:length(data(1).delta_z)
    idx_25{:,exp_nb} = (round(10.*data(1).delta_z{exp_nb}.pert_val) == 101 | ...
        round(10.*data(1).delta_z{exp_nb}.pert_val) == -99);
    idx_50{:,exp_nb} = (round(10.*data(1).delta_z{exp_nb}.pert_val) == 102 | ...
        round(10.*data(1).delta_z{exp_nb}.pert_val) == -98);
    idx_75{:,exp_nb} = (round(10.*data(1).delta_z{exp_nb}.pert_val) == 103 | ...
        round(10.*data(1).delta_z{exp_nb}.pert_val) == -97);
end

%% Phase Cycle verification
% cycle separation for the extraction of the instant of the fake
% perturbations
dt = 1e-3;
df = designfilt('lowpassfir', 'PassbandFrequency', 1.2,...
            'StopbandFrequency', 2.5, 'StopbandAttenuation', 20,...
            'PassbandRipple', 0.1, 'SampleRate', 1/dt);
% recompute the cycle phase of each perturbation
for exp_nb = 1:length(data(1).delta_z)
    fz_filt{exp_nb} = filtfilt(df,data(1).delta_fz{exp_nb}.complete_traject);
    dfz_filt{exp_nb} = Iu_diffcent(fz_filt{exp_nb}, data(1).delta_fz{exp_nb}.time);
    idx_pks{exp_nb} = crossing(dfz_filt{exp_nb});
    idx_u_pks{exp_nb} = idx_pks{exp_nb}(fz_filt{exp_nb}(idx_pks{exp_nb}) > 0);
    idx_l_pks{exp_nb} = idx_pks{exp_nb}(fz_filt{exp_nb}(idx_pks{exp_nb}) < 0);
end

% sorting
for exp_nb = length(data(1).delta_z):-1:1 
    i = 0;
    phase{exp_nb} = [];
    for pert_i = data(1).delta_fz{exp_nb}.pert_ind
        i = i + 1;
        time = data(1).delta_fz{exp_nb}.time;
        prev_upk = find(time(pert_i) > time(idx_u_pks{exp_nb}), 1, 'last');
        cyc_duration = time(idx_u_pks{exp_nb}(prev_upk+1)) - ...
            time(idx_u_pks{exp_nb}(prev_upk));
        p_pert_in_cyc = (time(pert_i) - time(idx_u_pks{exp_nb}(prev_upk)))/...
            cyc_duration;
        if p_pert_in_cyc < 0.9 || p_pert_in_cyc > 0.9
            phase{exp_nb} = [phase{exp_nb}; "upper pk"];
        elseif p_pert_in_cyc > 0.11 && p_pert_in_cyc < 0.49
            phase{exp_nb} = [phase{exp_nb}; "decreas"];
        elseif p_pert_in_cyc > 0.51 && p_pert_in_cyc < 0.70
            phase{exp_nb} = [phase{exp_nb}; "lower pk"];
        elseif p_pert_in_cyc > 0.75 && p_pert_in_cyc < 0.85
            phase{exp_nb} = [phase{exp_nb}; "increas"];
        else
            phase{exp_nb} = [phase{exp_nb}; "ambiguous"];
        end
    end 
end

figure('DefaultAxesFontSize',14)
subplot(2,1,1)
bar([...
sum(cell2mat(cellfun(@(x) sum(strcmp(x, "upper pk")), phase, 'UniformOutput', false))),...
sum(cell2mat(cellfun(@(x) sum(strcmp(x, "decreas")), phase, 'UniformOutput', false))),...
sum(cell2mat(cellfun(@(x) sum(strcmp(x, "lower pk")), phase, 'UniformOutput', false))),...
sum(cell2mat(cellfun(@(x) sum(strcmp(x, "increas")), phase, 'UniformOutput', false))),...
sum(cell2mat(cellfun(@(x) sum(strcmp(x, "ambiguous")), phase, 'UniformOutput', false)))])
phase_names = ["upper pk", "decreas", "lower pk", "increas", "ambiguous"];
set(gca,'xticklabel',phase_names)
title('Moments of the cycle the perturbation occured (recomputed)')
ylabel('Number of perturbations')
subplot(2,1,2)
bar([...
sum(cell2mat(cellfun(@(x) sum(x), idx_25, 'UniformOutput', false))),...
sum(cell2mat(cellfun(@(x) sum(x), idx_50, 'UniformOutput', false))),...
sum(cell2mat(cellfun(@(x) sum(x), idx_75, 'UniformOutput', false))),...
0,...
0])
phase_names = ["upper pk", "decreas", "lower pk", "increas", "ambiguous"];
set(gca,'xticklabel',phase_names)
title('Moments of the cycle the perturbation occured (online intention)')
ylabel('Number of perturbations')

exp_nb = 2;
figure
subplot(2,1,1)
hold on
plot(data(1).delta_fz{exp_nb}.time, data(1).delta_fz{exp_nb}.complete_traject)
plot(data(1).delta_fz{exp_nb}.time(data(1).delta_fz{exp_nb}.pert_ind(strcmp(phase{exp_nb}, "upper pk"))),...
    data(1).delta_fz{exp_nb}.complete_traject(data(1).delta_fz{exp_nb}.pert_ind(strcmp(phase{exp_nb}, "upper pk"))),...
    'rp')
plot(data(1).delta_fz{exp_nb}.time(data(1).delta_fz{exp_nb}.pert_ind(strcmp(phase{exp_nb}, "decreas"))),...
    data(1).delta_fz{exp_nb}.complete_traject(data(1).delta_fz{exp_nb}.pert_ind(strcmp(phase{exp_nb}, "decreas"))),...
    'gp')
plot(data(1).delta_fz{exp_nb}.time(data(1).delta_fz{exp_nb}.pert_ind(strcmp(phase{exp_nb}, "lower pk"))),...
    data(1).delta_fz{exp_nb}.complete_traject(data(1).delta_fz{exp_nb}.pert_ind(strcmp(phase{exp_nb}, "lower pk"))),...
    'mp')
subplot(2,1,2)
hold on
plot(data(1).delta_fz{exp_nb}.time, data(1).delta_fz{exp_nb}.complete_traject)
plot(data(1).delta_fz{exp_nb}.time(data(1).delta_fz{exp_nb}.pert_ind(idx_25{:,exp_nb})),...
    data(1).delta_fz{exp_nb}.complete_traject(data(1).delta_fz{exp_nb}.pert_ind(idx_25{:,exp_nb})),...
    'rp')
plot(data(1).delta_fz{exp_nb}.time(data(1).delta_fz{exp_nb}.pert_ind(idx_50{:,exp_nb})),...
    data(1).delta_fz{exp_nb}.complete_traject(data(1).delta_fz{exp_nb}.pert_ind(idx_50{:,exp_nb})),...
    'gp')
plot(data(1).delta_fz{exp_nb}.time(data(1).delta_fz{exp_nb}.pert_ind(idx_75{:,exp_nb})),...
    data(1).delta_fz{exp_nb}.complete_traject(data(1).delta_fz{exp_nb}.pert_ind(idx_75{:,exp_nb})),...
    'mp')

%%
ORIGINAL_PHASE_SORTING = 0; % if 0 the phase sorting computed just before
% is used

for delay_i = 1:length(data)
    for method_i = 1:size(data(delay_i).impedance,2)
        tmp_imp = data(delay_i).impedance(:,method_i);
        tmp25_nrmse = [];
        tmp50_nrmse = [];
        tmp75_nrmse = [];
        for exp_nb = 1:length(data(1).delta_z)
            if ORIGINAL_PHASE_SORTING
            tmp25_nrmse = [tmp25_nrmse, tmp_imp{exp_nb}.nrmse_pos(idx_25{:,exp_nb})];
            tmp50_nrmse = [tmp50_nrmse, tmp_imp{exp_nb}.nrmse_pos(idx_50{:,exp_nb})];
            tmp75_nrmse = [tmp75_nrmse, tmp_imp{exp_nb}.nrmse_pos(idx_75{:,exp_nb})];
            else
            tmp25_nrmse = [tmp25_nrmse, tmp_imp{exp_nb}.nrmse_pos(strcmp(phase{exp_nb}, "upper pk"))];
            tmp50_nrmse = [tmp50_nrmse, tmp_imp{exp_nb}.nrmse_pos(strcmp(phase{exp_nb}, "decreas"))];
            tmp75_nrmse = [tmp75_nrmse, tmp_imp{exp_nb}.nrmse_pos(strcmp(phase{exp_nb}, "lower pk"))];
            end
        end
%         end
        nrmse_data{delay_i, 1, method_i} = tmp25_nrmse;
        nrmse_data{delay_i, 2, method_i} = tmp50_nrmse;
        nrmse_data{delay_i, 3, method_i} = tmp75_nrmse;
    end
end

colors(1,:) = [0, 0.4470, 0.7410];
colors(2,:) = [0.8500, 0.3250, 0.0980];
colors(3,:) = [0.9290, 0.6940, 0.1250];
colors(4,:) = [0.4940, 0.1840, 0.5560];
colors(5,:) = [0.4660, 0.6740, 0.1880];
colors(6,:) = [0.3010, 0.7450, 0.9330];
colors(7,:) = [0.6350, 0.0780, 0.1840];
colors(8,:) = [0, 0.25, 0.25];          	
colors(9,:) = [0.1, 0.5, 0.1];   
colors(10,:) = [1, 0.7137, 0.7569];
colors(11,:) = [0.545,0.271,0.075];

marker_pert = ['o', 'x', 'p'];
color_pert = [0,0,0; 0.45,0.45,0.45; 0.85,0.85,0.85];
name_pert = ["supérieure", "descendante", "inférieure"];

figure('DefaultAxesFontSize',14)
for method_i = 1:size(data(1).impedance,2)
    fprintf('Method %s\n', data(1).delta_fz{1,method_i}.header)
    subplot(3,4,method_i)
    hold on
    for delay_i = 1:length(data)
        fprintf('Delay %0.f ms\n', delay_i+min_delay)
        for pert_i = 1:3
            plot((delay_i+min_delay)*ones(size(rmoutliers(nrmse_data{delay_i, pert_i, method_i}))), ...
                rmoutliers(nrmse_data{delay_i, pert_i, method_i}), marker_pert(pert_i), ...
                'Color', colors(method_i,:));
        end
        for pert_i = 1:3
            avg_i = nanmean(rmoutliers(nrmse_data{delay_i, pert_i, method_i}));
            std_i = nanstd(rmoutliers(nrmse_data{delay_i, pert_i, method_i}));
            plot(delay_i+min_delay, avg_i, marker_pert(pert_i), 'Color', ...
            color_pert(pert_i,:), 'Markersize', 12);
            plot(delay_i+min_delay, avg_i+std_i,'_', 'Color', color_pert(pert_i,:),...
                'Markersize', 12);
            plot(delay_i+min_delay, avg_i-std_i,'_', 'Color', color_pert(pert_i,:),...
                'Markersize', 12);
            fprintf('Phase %s du cycle, fit moyen %.2f %% (%.2f)\n', name_pert(pert_i), 100.*avg_i, 100.*std_i)
        end
    end
    title(data(1).delta_fz{1,method_i}.header)
    
end

%% anova analysis
% variations according to the phase 
delay_ = 10 - min_delay + 1;
method_ = 5; %
err_del_meth = [];
phase_grp = [];
imp_data = data(delay_).impedance(:,method_);
for exp_nb = 1:length(imp_data)
    err_del_meth = [err_del_meth, sum(imp_data{exp_nb}.rec_pos_err.^2)];
    tmp_grp = strcmp(phase{exp_nb}, "upper pk");
    tmp_grp = tmp_grp + 2.*strcmp(phase{exp_nb}, "decreas");
    tmp_grp = tmp_grp + 3.*strcmp(phase{exp_nb}, "lower pk");
    phase_grp = [phase_grp, tmp_grp'];
end
p = anova1(err_del_meth,phase_grp);
% variations according to the methods
delay_ = 10 - min_delay + 1;
phase_name = "upper pk";
nb = sum(cell2mat(cellfun(@(x) sum(strcmp(x, phase_name)), phase, 'UniformOutput', false)));
err_imp = [];%NaN(200, nb*size(data(1).impedance,2));
methd_grp = NaN(1, nb*size(data(1).impedance,2));
for method_i = 1:size(data(1).impedance,2)
    imp_data = data(delay_).impedance(:,method_i);
    for exp_nb = 1:length(imp_data)
        grp_ph_idx = strcmp(phase{exp_nb}, phase_name);
        err_imp = [err_imp, sum(imp_data{exp_nb}.rec_pos_err(:,grp_ph_idx).^2)];
    end
    methd_grp(nb*(method_i-1)+1:nb*(method_i)) = method_i;
end
p2 = anova1(err_imp(:,end-2*nb+1:end),methd_grp(end-2*nb+1:end));
%p2 = anova1(err_imp(:,:),methd_grp);

%% 3 ways anova using r2 fit
% group creation (delay, method, cycle phase)
grp_ph_idx = [];
dummy = [data(1).impedance{:,1}];
nb = sum([dummy.nb_id]);

for exp_nb = 1:length(dummy)
    tmp = zeros(dummy(exp_nb).nb_id,1);
    phase_name = ["upper pk", "decreas", "lower pk"];
    for p_n = 1:3
        tmp = tmp + p_n.*strcmp(phase{exp_nb}, phase_name(p_n));
    end
    grp_ph_idx = [grp_ph_idx, tmp'];
end
grp_ph_idx = repmat(grp_ph_idx, 1, size(data(1).impedance,2)*length(data));
tmp = [""];
for method_i = 1:size(data(1).impedance,2)
    tmp((method_i-1)*nb+1:method_i*nb) = data(1).delta_fz{1,method_i}.header;
end
grp_mth_idx = repmat(tmp,1,length(data));
grp_del_idx = zeros(size(grp_ph_idx));
for delay_i = 1:length(data)
    grp_del_idx((delay_i-1)*length(tmp)+1:delay_i*length(tmp)) = delay_i + min_delay - 1;
end
%feeding the r2 values for position reconstruction
for delay_i = 1:length(data)
    for method_i = 1:size(data(1).impedance,2)
        imp_data = [data(delay_i).impedance{:,method_i}];
        r2 = sum(([imp_data(:).rec_pos_err]).^2)./sum((mean([imp_data(:).rec_pos])...
            - [imp_data(:).rec_y]).^2);
        r2_vals((delay_i-1)*size(data(1).impedance,2)*nb+((method_i-1)*nb+1:...
            (method_i)*nb)) = r2;
    end
end
p3 = anovan(r2_vals,{grp_ph_idx,grp_mth_idx,grp_del_idx}, 'varnames',{'phase','method', 'delay'});

%% write data to excel
% write data to table (excel)
for delay_i = 1:length(data)
    for method_i = 1:size(data(1).impedance,2)
        for pert_i = 1:3
            avg_i = nanmean(rmoutliers(nrmse_data{delay_i, pert_i, method_i}));
            std_i = nanstd(rmoutliers(nrmse_data{delay_i, pert_i, method_i}));
            avg_delay_data(3*(method_i-1)+pert_i, delay_i) = avg_i;
            std_delay_data(3*(method_i-1)+pert_i, delay_i) = std_i;
        end
    end
end

for method_i = 1:size(data(1).impedance,2)
    for pert_i = 1:3
        avg_i = nanmean(rmoutliers([nrmse_data{:, pert_i, method_i}]));
        std_i = nanstd(rmoutliers([nrmse_data{:, pert_i, method_i}]));
        avg_delay_data_totd(3*(method_i-1)+pert_i) = avg_i;
        std_delay_data_totd(3*(method_i-1)+pert_i) = std_i;
    end
end

for delay_i = 1:length(data)
    for pert_i = 1:3
        avg_i = nanmean(rmoutliers([nrmse_data{delay_i, pert_i, :}]));
        std_i = nanstd(rmoutliers([nrmse_data{delay_i, pert_i, :}]));
        avg_delay_data_totm(pert_i, delay_i) = avg_i;
        std_delay_data_totm(pert_i, delay_i) = std_i;
    end
end

for pert_i = 1:3
    avg_i = nanmean(rmoutliers([nrmse_data{:, pert_i, :}]));
    std_i = nanstd(rmoutliers([nrmse_data{:, pert_i, :}]));
    avg_delay_data_tot(pert_i) = avg_i;
    std_delay_data_tot(pert_i) = std_i;
end

table_d = table(avg_delay_data_tot);
table_d2 = table(std_delay_data_tot);
writetable(table_d,'avg_val.xlsx')
writetable(table_d2,'std_val.xlsx')
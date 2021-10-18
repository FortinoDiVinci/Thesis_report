%% Script to test the implementation of Burdet et al. algorithm (2000)
clear all
close all

addpath('../../../../utils');
addpath('../../utils');
addpath('../../utils/cycles');

%%%%%%%%%%
% MACRO

% GLOBAL LOOP (iterations of the algorithme
NUMBER_OF_GLOBAL_ITERATIONS = 10; % nb pseudo random signals generated
NUMBER_OF_CONFIGURATIONS = 3;
NB_TIME_DISTORTIONS = [1, 5, 9]; % should be of size NUMBER_OF_CONFIGURATIONS
TIME_DISTORTION_GROWTH = [0, 0.03, 0.03]; % provided as percentage

% INPUT SIGNAL
TIME_VARIANT_MAGN = 1;
TIME_VARIANT_PHASE = 1;
USE_REAL_SIGNAL = 1;
if USE_REAL_SIGNAL
    NUMBER_OF_GLOBAL_ITERATIONS = 1;
end

% DISPLAY
DISP_ALL_CANDIDATE_AND_BEST_MATCH_FOR_CYCLES = 0; % depending on the number
% of candidate it might lead Matlab to crash if set to 1

for SIGNAL_IDX = 1:NUMBER_OF_GLOBAL_ITERATIONS

if ~USE_REAL_SIGNAL
    %%%%%%%%%%%%%%%%%%%%%%
    %% SIGNAL GENERATION

    % "ideal" signal
    dt = 1e-3;      % time sampling
    f = 0.8;        % sine frequency
    alp = 10;       % frequency multiplier for second sine
    phi = pi/2;     % phase delay of second sine
    a1 = 1;         % first sine magnitude
    a2 = 0;%a1/5;   % 2nd sine magnitude
    t_max = 80 - dt;
    t = 0:dt:t_max; % time vector
    t_under_samp = 0:10*dt:t_max; % time

    if TIME_VARIANT_MAGN
        a1t = step(dsp.ColoredNoise('InverseFrequencyPower',2,'SamplesPerFrame',length(t)/10));
        a1t = 1 + interp1(t_under_samp, a1t/(max(a1t)-min(a1t)), t);
        % deal with last nan
        nan_idx = find(isnan(a1t));
        for nan_i = 1:length(nan_idx)
            a1t(nan_idx(nan_i)) = a1t(nan_idx(nan_i)-1);
        end
        fc = 0.5; % cut off frequency
        [b,a] = butter(4,fc/(1/(2*dt)),'low'); 
        a1t = filtfilt(b,a,a1t);
    else
        a1t = a1;
    end

    if TIME_VARIANT_PHASE
        phit = step(dsp.ColoredNoise('InverseFrequencyPower',2,'SamplesPerFrame',length(t)/10));
        phit = phi.*interp1(t_under_samp, phit/(max(phit)-min(phit)), t);
        nan_idx = find(isnan(phit));
        for nan_i = 1:length(nan_idx)
            phit(nan_idx(nan_i)) = phit(nan_idx(nan_i)-1);
        end
        fc = 0.25; % cut off frequency
        [b,a] = butter(4,fc/(1/(2*dt)),'low'); 
        phit = filtfilt(b,a,phit);
    else
        phit = phi;
    end

    sig = a1t.*sin(2*pi*f.*t + phit) + a2*sin(alp*pi*f.*t + phi);

    figure 
    plot(t,sig)

    % low freq noise + gaussian noise
    lf = f/20;
    la = a1/4;
    %lsig = la*sin(2*pi*lf.*t);
    lsig = 0;
    nsig = awgn(sig + lsig, 35);

    hold on
    plot(t, nsig)

    % 50Hz filtering
    fc = 50; % cut off frequency
    [b,a] = butter(2,fc/(1/(2*dt)),'low'); 
    fsig = filtfilt(b,a,nsig);

    plot(t, fsig)
    legend('original', 'noisy', 'filtered')
    title("Input signal n" + num2str(SIGNAL_IDX))
else
    file_name = '../../data_2020_Nov_17/data_without_impacts_2020_11_17.mat';
    load(file_name);
    
    %do data processing for the position and force signal
end
%%%%%%%%%%%%%%%%%%%%%
%% CYCLE SPLITTING

%idx = 1+(0:1/dt/f:t_max/dt);
dsig = Iu_diffcent(t',sig');
idx_pks = crossing(dsig);
idx_pks = idx_pks(1:2:end); % selection of only half the peaks

plot(t(idx_pks), fsig(idx_pks), 'k^')
legend('original', 'noisy', 'filtered', 'cycle')

% freq variations
freq_var = 1./(diff(idx_pks)*dt);
figure
plot(freq_var)
title("Cycles frequencies variations n" + num2str(SIGNAL_IDX) + " _base freq " + num2str(f))

cycles = splitCycles([], fsig, t, idx_pks);

%%%%%%%%%%%%%%%%%%%%%
%% Prediction algorithm

% normalization
[ncycles, min_idx,~,~,~] = cycleNormalization(cycles,0); %

% Display cyclic data
% figure
% subplot(2,1,1)
% hold on
% for i = 1:length(cycles)
%     p1 = plot(cycles(i).force, 'Color', [0 0.4470 0.7410]);
%     p1.Color(4) = 0.75;
% end
% title('Cyclic data')
% subplot(2,1,2)
% hold on
% for i = 1:length(cycles)
%     p1 = plot(ncycles(i).force, 'Color', [0 0.4470 0.7410]);
%     p1.Color(4) = 0.75;
% end
% title('Normalized cyclic data')

RAND_GEN = rand([1 length(ncycles)]);

for CONFIG_IDX = 1:NUMBER_OF_CONFIGURATIONS
% ALGORITHM PARAMETERS
k = 10; % mean will be computed on the 10 previous cycles
m_nb = 15; % nb of magnitudes (should be odd to include x1 mag)
ts_nb = 11; % nb of time shifts (should be oddto include 0 delay)
td_nb = NB_TIME_DISTORTIONS(CONFIG_IDX); % nb of time distortions (should be odd to include x1 distortion)
coef_std = 6; % max magnitude is determined by the std times this coeff
max_time_delay = 40; % in terms of samples
max_time_distortion = TIME_DISTORTION_GROWTH(CONFIG_IDX); % max percentage of growth/compression
pred_eval_idx = 200; % the algorithm will be evaluated on this nb of samples

prev_cycles = ncycles(1:k);
time_shift = linspace(-max_time_delay,max_time_delay,ts_nb);
time_dist = 1+linspace(-max_time_distortion,max_time_distortion,td_nb);
%cand = NaN(m_nb*ts_nb, length(ncycles(1).force));
cand = NaN(m_nb*ts_nb*td_nb, min_idx); 

% cycles iterations
for i = k+1:length(ncycles)-1
    
    % cycle formating, cycles length are reduced to the minimum cycle length
    cell_f = {prev_cycles.force};
    for c_idx = 1:length(cell_f)
        cell_data = cell_f{c_idx};
        cell_f{c_idx} = cell_data(1:min_idx);
    end
    % mean computation
    % /!\ In the original algorithm the mean is computed independantly for 
    % the positive and negative acceleration part, but here, multiple cases
    % happens and the average is therefore not computed independantly
    forces_list = cell2mat(cell_f');
    avg_force = mean(forces_list,1);
    std_force = max(std(forces_list));
    magn_list = 1 + linspace(-coef_std*std_force,coef_std*std_force,m_nb);
    
    % candidates population
    for ii = 1:td_nb
        cand_dist = generateTimeDistortion(avg_force, (1:min_idx), time_dist(ii));
        for jj = 1:m_nb
            cand_magn = cand_dist.*magn_list(jj);
            for kk = 1:ts_nb
                if time_shift(kk) > 0
                    cand_shift = cat(2,cand_magn(1+time_shift(kk):end), ...
                        NaN(1,time_shift(kk))); % complete the missing data with NaN
                elseif time_shift(kk) < 0
                    cand_shift = cat(2,NaN(1,-time_shift(kk)), ...
                        cand_magn(1:end+time_shift(kk)));
                else
                    cand_shift = cand_magn(1+time_shift(kk):end);
                end
                cand(ts_nb*(m_nb*(ii-1)+jj-1)+kk,:) = cand_shift;
            end
        end
    end
    
    % disp candidates, mean signal, and 10 previous signals
%     if i == k+1 %% only disp the first one
%         figure
%         p = plot(cand');
%         hold on
%         plot(forces_list', 'k', 'LineWidth', 1.1);
%         plot(avg_force, '--k', 'LineWidth', 3)
%         for i_p = length(p)
%             p(i_p).Color(4) = 0.3;
%         end
%     end
    
    % best match
    % For this prototype, 300 ms + a rand number between 0 and 300 are used
    % for the unperturbed trajectory comparing
    rand_val = floor(RAND_GEN(i)*300); % 0.3/dt
    d_cost = zeros(size(cand,1),1);
    alt_cost = zeros(size(cand,1),1);
    alpha = 0.94;
    learn_idx = (-99:0) + 0.3/dt + rand_val; % last 100ms indexes
    for c = 1:size(cand,1)
        for idx_t = 1:0.3/dt + rand_val
            d_cost(c) = nansum([(ncycles(i).force(idx_t) - cand(c,idx_t))^2, alpha*d_cost(c)]);
        end
        alt_cost(c) = rms(nansum([ncycles(i).force(learn_idx); -cand(c,learn_idx)],1));
    end
    [~, d_min_idx] = min(d_cost);
    b_cand = cand(d_min_idx,:);
    o_cand = cand(setdiff(1:end,d_min_idx),:);
    [rms_err_choice_alt(i), d_min_idx] = min(alt_cost);
    alt_b_cand = cand(d_min_idx,:);
    rms_err_choice(i) = rms(nansum([ncycles(i).force(learn_idx); -b_cand(learn_idx)],1));
    
    % prev_cycles FIFO 
    % a conditonnal evaluation of the new std should say if the new cycle
    % is added or not.
    prev_cycles = [prev_cycles(2:end), ncycles(i)];
    
    % the prediction begins at time 300ms + rand_val and is evaluated on 
    % 300ms
    start_idx = rand_val + 0.3/dt;
    
    rms_err(i) = rms( ncycles(i).force(start_idx:start_idx+pred_eval_idx) - b_cand(start_idx:start_idx+pred_eval_idx) );
    rms_err_alt(i) = rms( ncycles(i).force(start_idx:start_idx+pred_eval_idx) - alt_b_cand(start_idx:start_idx+pred_eval_idx) );
    
    error1(SIGNAL_IDX, CONFIG_IDX) = STATISTIC_DATA(ncycles(i).force(start_idx-99:start_idx+pred_eval_idx) -...
        b_cand(start_idx-99:start_idx+pred_eval_idx), 'Burdet candidate error');  
    
    error2(SIGNAL_IDX, CONFIG_IDX) = STATISTIC_DATA(ncycles(i).force(start_idx-99:start_idx+pred_eval_idx) -...
        alt_b_cand(start_idx-99:start_idx+pred_eval_idx), 'minRMSE candidate error'); 
    
    if DISP_ALL_CANDIDATE_AND_BEST_MATCH_FOR_CYCLES
    figure
    p1 = plot(o_cand((1:1:end),:)', 'Color', [0.3010 0.7450 0.9330]);
    hold on
    plot(ncycles(i).force, 'Color', [0 0.4470 0.7410])
    plot(b_cand, 'k')
    p2 = plot(alt_b_cand, 'k');
    plot(1:start_idx, b_cand(1:start_idx), 'Color', [0.4660 0.6740 0.1880])
    plot(start_idx:start_idx+pred_eval_idx, ...
        b_cand(start_idx:start_idx+pred_eval_idx), 'Color', [0.8500 0.3250 0.0980])
    for p_idx = 1:length(p1)
        p1(p_idx).Color(4) = 0.3;
    end
    p2.Color(4) = 0.7;
    
    plot(1:start_idx, alt_b_cand(1:start_idx), '--','Color', [0.4660 0.6740 0.1880])
    plot(start_idx:start_idx+pred_eval_idx, ...
        alt_b_cand(start_idx:start_idx+pred_eval_idx), '--', 'Color', [0.8500 0.3250 0.0980])
    
    title(num2str(i) + ") RMS Error c: " + ...
        num2str(rms_err(i)) + ", min RMSE: " + num2str(rms_err_alt(i)))
    end % DISP_ALL_CANDIDATE_AND_BEST_MATCH_FOR_CYCLES
end

%% Display analytics

figure
p1 = plot(rms_err, 'Color', [0 0.4470 0.7410]);
hold on
p2 = plot(rms_err_alt, 'Color', [0.8500 0.3250 0.0980]);
plot(rms_err_choice, 'k')
p3 = plot(rms_err_choice, '--', 'Color', [0 0.4470 0.7410]);
plot(rms_err_choice_alt, 'k')
p4 = plot(rms_err_choice_alt, '--', 'Color', [0.8500 0.3250 0.0980]);
legend([p1, p2, p3, p4], {'Recurcive choice (Burdet)', 'min RMSE over last 100ms', 'Validation (Burdet)', 'Validation (min RMSE)'})
title("Best candidates RMSE over all cycles config n" + num2str(CONFIG_IDX))

% (k+1:end) to avoid considering the k first cycle that are used for mean
% computation
mean_rmse_meth_burdet(SIGNAL_IDX, CONFIG_IDX) = mean(rms_err(k+1:end));
std_rmse_meth_burdet(SIGNAL_IDX, CONFIG_IDX) = std(rms_err(k+1:end));
mean_rmse_meth_minRmse(SIGNAL_IDX, CONFIG_IDX) = mean(rms_err_alt(k+1:end));
std_rmse_meth_minRmse(SIGNAL_IDX, CONFIG_IDX) = std(rms_err_alt(k+1:end));

disp("Iteration (" + num2str(SIGNAL_IDX) + "), config (" + num2str(CONFIG_IDX) + ")")
disp("Burdet et al. (2000) like method mean RMSE: " +...
    num2str(mean_rmse_meth_burdet(SIGNAL_IDX, CONFIG_IDX)) + ...
    " (+/- " + num2str(std_rmse_meth_burdet(SIGNAL_IDX, CONFIG_IDX)) + ")")
disp("Min RMSE method mean RMSE: " + ...
    num2str(mean_rmse_meth_minRmse(SIGNAL_IDX, CONFIG_IDX)) + ...
    " (+/- " + num2str(std_rmse_meth_minRmse(SIGNAL_IDX, CONFIG_IDX)) + ")")
drawnow
saveAllFig('misc_savefig', SIGNAL_IDX, 1); % save all current figures and then closes them
% clear everything but macros, analytics and necessary data about input signal
clearvars -except ncycles min_idx fsig dt NUMBER_OF_GLOBAL_ITERATIONS ...
    NUMBER_OF_CONFIGURATIONS NB_TIME_DISTORTIONS TIME_VARIANT_MAGN ...
    TIME_VARIANT_PHASE SIGNAL_IDX CONFIG_IDX RAND_GEN TIME_DISTORTION_GROWTH ...
    mean_rmse_meth_burdet std_rmse_meth_burdet mean_rmse_meth_minRmse ...
    std_rmse_meth_minRmse DISP_ALL_CANDIDATE_AND_BEST_MATCH_FOR_CYCLES ...
    error1 error2
end % for CONFIG_IDX = 1:NUMBER_OF_CONFIGURATIONS
% clear everything but macros and analytics
clearvars -except NUMBER_OF_GLOBAL_ITERATIONS NUMBER_OF_CONFIGURATIONS ...
    NB_TIME_DISTORTIONS TIME_VARIANT_MAGN TIME_VARIANT_PHASE SIGNAL_IDX ...
    CONFIG_IDX TIME_DISTORTION_GROWTH mean_rmse_meth_burdet std_rmse_meth_burdet ...
    mean_rmse_meth_minRmse std_rmse_meth_minRmse DISP_ALL_CANDIDATE_AND_BEST_MATCH_FOR_CYCLES ...
    error1 error2
end % for SIGNAL_IDX = 1:NUMBER_OF_GLOBAL_ITERATIONS

figure
hold on 
legend_names = [];
for conf_idx = 1:NUMBER_OF_CONFIGURATIONS
    errorbar(mean_rmse_meth_burdet(:,conf_idx), std_rmse_meth_burdet(:,conf_idx))
    errorbar(mean_rmse_meth_minRmse(:,conf_idx), std_rmse_meth_minRmse(:,conf_idx))
    new_legend_names = "RMSE cnfig (" + num2str(conf_idx) +  [") Burdet", ") minR"];
    legend_names = [legend_names, new_legend_names];
end
legend(legend_names)

for conf_idx = 1:NUMBER_OF_CONFIGURATIONS
    disp("Burdet conf n°" + num2str(conf_idx)+ ": mean " + num2str(mean(mean_rmse_meth_burdet(:,conf_idx))));
    disp("MinRMSE conf n°" + num2str(conf_idx)+ ": mean " + num2str(mean(mean_rmse_meth_minRmse(:,conf_idx)))); 
end

for conf_idx = 1:NUMBER_OF_CONFIGURATIONS
    disp("Burdet conf n°" + num2str(conf_idx)+ ": mean std " + num2str(mean(std_rmse_meth_burdet(:,conf_idx))));
    disp("MinRMSE conf n°" + num2str(conf_idx)+ ": mean std " + num2str(mean(std_rmse_meth_minRmse(:,conf_idx)))); 
end

save("test_prediction_trajectoire_3.mat")
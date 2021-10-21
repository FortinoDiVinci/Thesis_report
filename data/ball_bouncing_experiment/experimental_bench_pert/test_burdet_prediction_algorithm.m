%% Script to test the implementation of Burdet et al. algorithm (2000)
clear all
close all

% /!\ For memory purpose, the macro CLEAR_TEMPORARY_DATA flushes all the 
% variables that are not set as exceptions, for more specific details
% look at the end of both signal and configuration loop

addpath('../../youBot_analysis/Utils');
addpath('utils');
addpath('utils/cycles');
addpath('Input_signal_examples/');
addpath('trajectory_prediction_evaluation/Burdet2000');
base_save_path = "trajectory_prediction_evaluation/Burdet2000/";

%%%%%%%%%%
% MACRO

% GLOBAL LOOP (iterations of the algorithme
NB_OF_GLOBAL_ITERATIONS = 1; % nb pseudo random signals generated
NB_OF_CONFIGURATIONS = 1; % For the time distorsion
NB_TIME_DISTORTIONS = [1];%[1, 5, 9]; % should be of size NB_OF_CONFIGURATIONS
TIME_DISTORTION_GROWTH = [0];%[0, 0.03, 0.03]; % provided as percentage
CLEAR_TEMPORARY_DATA = 1; % Not setting this var to 1 can lead to memory crashes
SAVE_FIGURES = 0;
LOW_PASS_FREQ = 20; % Hz (50 Hz)

% INPUT SIGNAL
EXTERNAL_INPUT_SIGNAL = 1;
EXTERNAL_IS_FORCE = 1; % set to 0 to use position
if EXTERNAL_INPUT_SIGNAL
    addpath('../../force_torque_sensor');
    %load("preliminary_experimental_data/data_vfo_10.mat"); 
    necessary_variables = {'forces_unf','torques_unf','thetas', 'dt',...
        'mocap_marker_robot_base','t','idx_ball_off_ramp','dist', 't_dist', ...
        'SINGLE_MOCAP_FITTING'};
    load("data_2020_Nov_17/data_without_impacts_2020_11_17.mat", necessary_variables{:}); 
    t_in = t;
    clear t
    %idx_start = [1.5e4];
    %idx_end = [6.625e5];  
    if iscell(forces_unf)
        if SINGLE_MOCAP_FITTING % first data cell should be deleted
            forces_unf(1) = [];
            if exist('torques_unf') == 1 % test if it is a var in workspace
                torques_unf(1) = [];
            else
                torques_unf = forces_unf;
            end
            thetas(1) = [];
            mocap_marker_robot_base(1) = [];
            idx_ball_off_ramp(1)= [];
            t_in(1) = [];
        else
            % TODO
        end
    end
    idx_start_arr = cell2mat(idx_ball_off_ramp);
    idx_end_arr = cellfun(@length,forces_unf);
    if NB_OF_GLOBAL_ITERATIONS > length(forces_unf)
        NB_OF_GLOBAL_ITERATIONS = length(forces_unf);
    end
    
else
    TIME_VARIANT_MAGN = 1;
    TIME_VARIANT_PHASE = 1;
end

% DISPLAY
DISP_ALL_CANDIDATES_AND_BEST_MATCH_FOR_CYCLES = 1; % depending on the number
% of candidate it might lead Matlab to crash if set to 1
% a temporrary condition has been added

% Other methods to compare with
DO_SPLINE_INTERP = 1;

if DO_SPLINE_INTERP
    DISP_SPLINE_ALL_CANDIDATES = 1;
    NB_OF_METHODS = 5;
    methods_names = ["Burdet candidate for config", "minRMSE candidate for config", ...
        "Mean Spline candidate", "Best Spline candidate", "Worst Spline candidate"];
else
    NB_OF_METHODS = 2;
    methods_names = ["Burdet candidate for config", "minRMSE candidate for config"];
end

% preallocation
errors(NB_OF_GLOBAL_ITERATIONS, NB_OF_CONFIGURATIONS, NB_OF_METHODS) = STATISTIC_MDATA();
for i = 1:NB_OF_METHODS
    for j = 1:NB_OF_GLOBAL_ITERATIONS
        for k = 1:NB_OF_CONFIGURATIONS
            if i > 2
                errors(j,k,i).setName(methods_names(i));
            else
                errors(j,k,i).setName(methods_names(i) + num2str(k));
            end
        end
    end
end

for SIGNAL_IDX = 1:NB_OF_GLOBAL_ITERATIONS
%%%%%%%%%%%%%%%%%%%%%%
%% SIGNAL GENERATION

if EXTERNAL_INPUT_SIGNAL
    t = t_in{SIGNAL_IDX}';
    if EXTERNAL_IS_FORCE
        [f_int,~,~]= forces_filtering(forces_unf{SIGNAL_IDX}', torques_unf{SIGNAL_IDX}', ...
            thetas{SIGNAL_IDX}', t');
        nsig = -1.*f_int(3,:);
        tmp = mocap_marker_robot_base{SIGNAL_IDX};
        nsig_pos = tmp(:,3)'; % z position
        clear tmp
    else % External signal is position
        tmp = mocap_marker_robot_base{SIGNAL_IDX};
        nsig = tmp(:,3)'; % z position
        clear tmp
    end
    % filtering
    fc = LOW_PASS_FREQ; % cut off frequency
    [b,a] = butter(2,fc/(1/(2*dt)),'low'); 
    fsig = filtfilt(b,a,nsig);
    fc = 2; % cut off frequency
    if EXTERNAL_IS_FORCE
        [b,a] = butter(2,fc/(1/(2*dt)),'low'); 
        sig = filtfilt(b,a,nsig_pos);
    else
        [b,a] = butter(2,fc/(1/(2*dt)),'low'); 
        sig = filtfilt(b,a,nsig);
    end
    
    [b,a] = butter(2,10/(1/(2*dt)),'low'); 
    msig = filtfilt(b,a,nsig);
    
    % to avoid the irrelevant data at both the beginning and end of the
    % signal
    idx_start = idx_start_arr(SIGNAL_IDX);
    idx_end = idx_end_arr(SIGNAL_IDX);
    nsig = nsig(idx_start:idx_end);
    fsig = fsig(idx_start:idx_end);
    sig = sig(idx_start:idx_end);
    msig = msig(idx_start:idx_end);
    t = t(idx_start:idx_end);
    dt = (t(end) - t(1)) / length(t);
    %dt = 1e-3;
    
    figure
    hold on
    plot(t, nsig)
    plot(t, fsig)
    plot(t, msig)
    plot(t, sig)
    legend('original', "filtered at " + num2str(LOW_PASS_FREQ,'%.0f') + "Hz", 'filtered at 10Hz', 'filtered at 2Hz')
    title("External input signal n" + num2str(SIGNAL_IDX))
    
else
    % "ideal" signal
    dt = 1e-3;      % time sampling
    f = 0.8;        % sine frequency
    alp = 10;       % frequency multiplier for second sine % modify in generateRhythmicSignal
    phi = pi/2;     % phase delay of second sine % modify in generateRhythmicSignal
    a1 = 1;         % first sine magnitude
    a2 = 0;%a1/5;   % 2nd sine magnitude
    t_max = 80 - dt;
    t = 0:dt:t_max; % time vector
    t_under_samp = 0:10*dt:t_max; % time
    
%     % low freq noise + gaussian noise
%     lf = f/20;
%     la = a1/4;
%     %lsig = la*sin(2*pi*lf.*t);
%     lsig = 0;
%     nsig = awgn(sig + lsig, 35);

    [nsig, sig] = generateRhythmicSignal(dt, t_max, f, 'TimeVariantMagnitude', true, ...
        'TimeVariantPhase', true, 'GaussianNoise', true, 'FirstSineMagnitude',a1, ...
        'SecondSineMagnitude',a2);

    % 50Hz filtering
    fc = LOW_PASS_FREQ; % cut off frequency
    [b,a] = butter(2,fc/(1/(2*dt)),'low'); 
    fsig = filtfilt(b,a,nsig);
    
    figure
    plot(t, sig)
    hold on
    plot(t, nsig)
    plot(t, fsig)
    legend('original', 'noisy', 'filtered')
    title("Input signal n" + num2str(SIGNAL_IDX))
end

%%%%%%%%%%%%%%%%%%%%%
%% CYCLE SPLITTING

%idx = 1+(0:1/dt/f:t_max/dt);
dsig = Iu_diffcent(t',sig');
idx_pks = crossing(dsig);
mag_var = abs(diff(nsig(idx_pks)));
mag_var = mag_var(1:2:end)';
idx_pks = idx_pks(1:2:end); % selection of only half the peaks

plot(t(idx_pks), fsig(idx_pks), 'k^')
if EXTERNAL_INPUT_SIGNAL
    legend('original', 'filtered at 50Hz', 'filtered at 10Hz', 'filtered at 2Hz', 'cycle');
else
    legend('original', 'noisy', 'filtered', 'cycle');
end
%
% legappend('cycle') % could not fix it...

% freq variations
freq_var = (1./(diff(idx_pks)*dt))';
figure
subplot(2,1,1)
plot(freq_var)
title("Cycles frequencies variations n" + num2str(SIGNAL_IDX))% + " _base freq " + num2str(f))
subplot(2,1,2)
plot(mag_var)
title("Cycles magnitudes variations n" + num2str(SIGNAL_IDX))% + " _base freq " + num2str(f))

% complete_signal_properties = table(mag_var,freq_var);
% write(complete_signal_properties,'force_signal_properties.csv','Delimiter',',');
% 
% time = downsample(t((31069:81069)),10)';
% force = downsample(nsig((31069:81069)),10)';  
% complete_signal = table(time,force);
% write(complete_signal,'force_signal.csv','Delimiter',',');

if EXTERNAL_INPUT_SIGNAL
    
    if exist('dist') == 1 
        dist_val = dist{SIGNAL_IDX};
        dist_idx = zeros(size(t_dist{SIGNAL_IDX}));
        del_list = [];
        for i = 1:length(t_dist{SIGNAL_IDX})
             tmp = find(t >= t_dist{SIGNAL_IDX}(i), 1, 'first');
             if isempty(tmp)
                del_list = [del_list, i];
             else
                 dist_idx(i) = tmp;
             end
            if dist_val(i) == 0 % to avoid considering the end of a perturbation
                % as no perturbation...
                dist_val(i) = sign(dist_val(i-1));
            end
        end
        % delete data that is outside the time limits
        dist_idx(del_list) =[];
        dist_val(del_list) =[];
    else
        dist_idx = [];
        dist_val = [];
    end
    
    complete_cycles = splitCycles([], fsig, t, idx_pks, dist_idx, dist_val);
    j = 1;
    for i = 1:length(complete_cycles)
        if complete_cycles(i).isPerturbed
            continue;
        else
            cycles(j) = complete_cycles(i).copy();
            j = j + 1;
        end
    end
else
    cycles = splitCycles([], fsig, t, idx_pks);
end

%%%%%%%%%%%%%%%%%%%%%
%% Prediction algorithm

% normalization
[ncycles, min_idx,~,~,~] = cycleNormalization(cycles,0); %

%Display cyclic data
figure
subplot(2,1,1)
hold on
for i = 1:length(cycles)
    p1 = plot(cycles(i).force, 'Color', [0 0.4470 0.7410]);
    p1.Color(4) = 0.75;
end
title('Cyclic data')
subplot(2,1,2)
hold on
for i = 1:length(cycles)
    p1 = plot(ncycles(i).force, 'Color', [0 0.4470 0.7410]);
    p1.Color(4) = 0.75;
end
title('Normalized cyclic data')

RAND_GEN = rand([1 length(ncycles)]);

for CONFIG_IDX = 1:NB_OF_CONFIGURATIONS
% ALGORITHM PARAMETERS
k = 10; % mean will be computed on the 10 previous cycles
m_nb = 15; % nb of magnitudes (should be odd to include x1 mag)
ts_nb = 11; % nb of time shifts (should be oddto include 0 delay)
td_nb = NB_TIME_DISTORTIONS(CONFIG_IDX); % nb of time distortions (should be odd to include x1 distortion)
coef_std = 1.5; % max magnitude is determined by the std times this coeff
max_time_delay = 60;%40; % in terms of samples
max_time_distortion = TIME_DISTORTION_GROWTH(CONFIG_IDX); % max percentage of growth/compression
pred_eval_idx = 200-1; % the algorithm will be evaluated on this nb of samples + 1
offset_idx_pred = 300; %

prev_cycles = cycles(1:k); % ncycles
time_shift = linspace(-max_time_delay,max_time_delay,ts_nb);
time_dist = 1+linspace(-max_time_distortion,max_time_distortion,td_nb);
%cand = NaN(m_nb*ts_nb, length(ncycles(1).force));
cand = NaN(m_nb*ts_nb*td_nb, min_idx); 

% cycles iterations
for i = k+1:length(cycles)-1 % ncycles
    
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
    
%     % candidates population
%     for ii = 1:td_nb
%         cand_dist = generateTimeDistortion(avg_force, (1:min_idx), time_dist(ii));
%         for jj = 1:m_nb
%             cand_magn = cand_dist.*magn_list(jj);
%             for kk = 1:ts_nb
%                 if time_shift(kk) > 0
%                     cand_shift = cat(2,cand_magn(1+time_shift(kk):end), ...
%                         NaN(1,time_shift(kk))); % complete the missing data with NaN
%                 elseif time_shift(kk) < 0
%                     cand_shift = cat(2,NaN(1,-time_shift(kk)), ...
%                         cand_magn(1:end+time_shift(kk)));
%                 else
%                     cand_shift = cand_magn(1+time_shift(kk):end);
%                 end
%                 cand(ts_nb*(m_nb*(ii-1)+jj-1)+kk,:) = cand_shift;
%             end
%         end
%     end
    
    cand = generateCandidatesBurdet2000(forces_list, magn_list, time_dist, time_shift);

    % best match
    % For this prototype, 300 ms + a rand number between 0 and 300 are used
    % for the unperturbed trajectory comparison
    rand_val = floor(RAND_GEN(i)*offset_idx_pred); % offset_idx_pred
    d_cost = zeros(size(cand,1),1);
    alt_cost = zeros(size(cand,1),1);
    alpha = 0.94;
    learn_idx = (-99:0) + offset_idx_pred + rand_val; % last 100ms indexes
    for c = 1:size(cand,1)
        for idx_t = 1:offset_idx_pred + rand_val
            d_cost(c) = nansum([(cycles(i).force(idx_t) - cand(c,idx_t))^2, alpha*d_cost(c)]); % ncycles
        end
        alt_cost(c) = rms(nansum([cycles(i).force(learn_idx); -cand(c,learn_idx)],1)); % ncycles
    end
    [~, d_min_idx] = min(d_cost);
    b_cand = cand(d_min_idx,:);
    o_cand = cand(setdiff(1:end,d_min_idx),:);
    [rms_err_choice(i,2), d_min_idx] = min(alt_cost);
    alt_b_cand = cand(d_min_idx,:);
    rms_err_choice(i,1) = rms(nansum([cycles(i).force(learn_idx); -b_cand(learn_idx)],1)); % ncycles
    
    % prev_cycles FIFO 
    % TODO: a conditonnal evaluation of the new std should say if the new cycle
    % is added or not, according to Burdet et al. (2000)
    prev_cycles = [prev_cycles(2:end), cycles(i)]; % ncycles
    pred_indexes = learn_idx(end)+(0:pred_eval_idx); % the indices for prediction
    % To compare with spline method FORTINEAU et al.(2020)
    if DO_SPLINE_INTERP && CONFIG_IDX == 1 % splines have only 1 config
        % params
        nb_landings = 41;
        max_landing_dist = 20;
        lndng = linspace(-max_landing_dist,max_landing_dist,nb_landings);

        spline_cand = NaN(nb_landings,length(pred_indexes));
        for i_pt = 1:nb_landings
            
            % cubic splines need 4 points, for position and derivative
            interp_indexes = [learn_idx(end-1:end), [0,1] + ...
                learn_idx(end) + pred_eval_idx + lndng(i_pt)];
            estimation_indexes = learn_idx(end) + (0:pred_eval_idx+lndng(i_pt));
            
            % The following method is not recommanded, therefore function 
            % spline is used
            %spline_pp = interp1(ncycles(i).time(interp_indexes), ...
            %    ncycles(i).force(interp_indexes), 'cubic', 'pp');
            spline_pp = spline(cycles(i).time(interp_indexes), cycles(i).force(interp_indexes)); % ncycles
            
            % the spleen will only be evaluated for pred_eval_idx points 
            % after the begining of the "prediction", therefore, according
            % to the landing point, indexes needs to either be deleted or
            % added using the same spline equation
            spline_cand(i_pt, :) = ppval(spline_pp,cycles(i).time(pred_indexes)); % ncycles
            spline_cost(i_pt) = rms(nansum([cycles(i).force(pred_indexes); -spline_cand(i_pt, :)],1)); % ncycles
        end 
        [~, spline_min_idx] = min(spline_cost);
        [~, spline_max_idx] = max(spline_cost);
        w_spline_cand = spline_cand(spline_max_idx, :); % worst spline
        b_spline_cand = spline_cand(spline_min_idx, :); % best spline
        mean_spline_cand = mean(spline_cand, 1); % average of the splines

        rms_err(i,3) = rms( cycles(i).force(pred_indexes) - mean_spline_cand ); % ncycles
        rms_err(i,4) = rms( cycles(i).force(pred_indexes) - b_spline_cand ); % ncycles
        rms_err(i,5) = rms( cycles(i).force(pred_indexes) - w_spline_cand ); % ncycles
        
        errors(SIGNAL_IDX, CONFIG_IDX, 3).appendData(cycles(i).force(pred_indexes) -...
                mean_spline_cand); % ncycles
        errors(SIGNAL_IDX, CONFIG_IDX, 4).appendData(cycles(i).force(pred_indexes) -...
             b_spline_cand); % ncycles
        errors(SIGNAL_IDX, CONFIG_IDX, 5).appendData(cycles(i).force(pred_indexes) -...
             w_spline_cand); % ncycles
        
        if DISP_SPLINE_ALL_CANDIDATES && i == 15%k+1
            figure
            p1 = plot(pred_indexes, spline_cand', 'Color', [0.3010 0.7450 0.9330]);
            for p_idx = 1:length(p1)
                p1(p_idx).Color(4) = 0.3;
            end
            hold on
            p0 = plot(cycles(i).force, 'Color', [0 0.4470 0.7410]); % ncycles
            p2 = plot(pred_indexes, mean_spline_cand, 'k');
            p3 = plot(pred_indexes, b_spline_cand, 'Color', [0.4660, 0.6740, 0.1880]);
            p4 = plot(pred_indexes, w_spline_cand, 'Color', [0.6350, 0.0780, 0.1840]);
            legend([p0 p1(1) p2 p3 p4], {"Actual signal", " Spline Candidates", ...
            "Mean spline", "Best spline", "Worst spline"});
        end
    end
    
    
    
    % the prediction begins at time 300ms + rand_val and is evaluated on 
    % 300ms : pred_indexes
    % validation is on the 100 samples before the prediction
    val_indexes = pred_indexes(1) + (-99:-1);
    rms_err(i, 1) = rms( cycles(i).force(pred_indexes) - b_cand(pred_indexes) ); % ncycles
    rms_err(i, 2) = rms( cycles(i).force(pred_indexes) - alt_b_cand(pred_indexes) ); % ncycles

    errors(SIGNAL_IDX, CONFIG_IDX, 1).appendData(cycles(i).force([val_indexes,...
        pred_indexes]) - b_cand([val_indexes, pred_indexes])); % ncycles
    errors(SIGNAL_IDX, CONFIG_IDX, 2).appendData(cycles(i).force([val_indexes,...
        pred_indexes]) - alt_b_cand([val_indexes, pred_indexes])); % ncycles
    
%     errors(SIGNAL_IDX, CONFIG_IDX, 1) = STATISTIC_DATA(ncycles(i).force([val_indexes, pred_indexes]) -...
%         b_cand([val_indexes, pred_indexes]), "Burdet candidate for config" + num2str(CONFIG_IDX));  
%     
%     errors(SIGNAL_IDX, CONFIG_IDX, 2) = STATISTIC_DATA(ncycles(i).force([val_indexes, pred_indexes]) -...
%         alt_b_cand([val_indexes, pred_indexes]), "minRMSE candidate for config" + num2str(CONFIG_IDX)); 
    
    if DISP_ALL_CANDIDATES_AND_BEST_MATCH_FOR_CYCLES
    figure
    p1 = plot(o_cand(:,:)', 'Color', [0.3010 0.7450 0.9330]);
    hold on
    p0 = plot(cycles(i).force, 'Color', [0 0.4470 0.7410]); % ncycles
    plot(b_cand, 'k')   
    p2 = plot(alt_b_cand, 'k');
    p3 = plot(1:pred_indexes(1), b_cand(1:pred_indexes(1)), 'Color', [0.4660 0.6740 0.1880]);
    plot(pred_indexes, b_cand(pred_indexes), 'Color', [0.8500 0.3250 0.0980])
    for p_idx = 1:length(p1)
        p1(p_idx).Color(4) = 0.3;
    end
    p2.Color(4) = 0.7;
    
    p4 = plot(1:pred_indexes(1), alt_b_cand(1:pred_indexes(1)), '--','Color', [0.4660 0.6740 0.1880]);
    plot(pred_indexes, alt_b_cand(pred_indexes), '--', 'Color', [0.8500 0.3250 0.0980])
    if DO_SPLINE_INTERP  && CONFIG_IDX == 1
        p5 = plot(pred_indexes, mean_spline_cand, ':k');
        p6 = plot(pred_indexes, b_spline_cand, ':', 'Color', [0.4940, 0.1840, 0.5560]);
        p7 = plot(pred_indexes, w_spline_cand, ':k', 'Color', [0.6350, 0.0780, 0.1840]);
        legend([p0 p1(1) p3 p4 p5 p6 p7], {"Actual signal", "Candidates", ...
            "BC Burdet", "BC min RMSE", "Mean spline", "Best spline", "Worst spline"});
    else
        legend([p0 p1(1) p3 p4], {"Actual signal", "Candidates", "BC Burdet", "BC min RMSE"});
    end
    %title(num2str(SIGNAL_IDX) + "_" + num2str(CONFIG_IDX) + "- RMS Error c " + ...
    %    num2str(1000*rms_err(i,1),4) + ", min RMSE " + num2str(1000*rms_err(i,2),4) + "in perthousand")
    title("Example of candidates and choices for a cycle of signal " + ...
        num2str(SIGNAL_IDX) + " and config " + num2str(CONFIG_IDX));
    end % DISP_ALL_CANDIDATES_AND_BEST_MATCH_FOR_CYCLES
end

%% Display analytics

figure
p1 = plot(rms_err(:,1), 'Color', [0 0.4470 0.7410]);
hold on
p2 = plot(rms_err(:,2), 'Color', [0.8500 0.3250 0.0980]);
plot(rms_err_choice(:,1), 'k')
p3 = plot(rms_err_choice(:,1), '--', 'Color', [0 0.4470 0.7410]);
plot(rms_err_choice(:,2), 'k')
p4 = plot(rms_err_choice(:,2), '--', 'Color', [0.8500 0.3250 0.0980]);
legend([p1, p2, p3, p4], {'Recurcive choice (Burdet)', 'min RMSE over last 100ms', 'Validation (Burdet)', 'Validation (min RMSE)'})
title("Best candidates RMSE over all cycles config n" + num2str(CONFIG_IDX))

% (k+1:end) to avoid considering the k first cycle that are used for mean
% computation
%mean_rmse_meth_burdet(SIGNAL_IDX, CONFIG_IDX) = mean(rms_err(k+1:end,1));
%std_rmse_meth_burdet(SIGNAL_IDX, CONFIG_IDX) = std(rms_err(k+1:end,1));
%mean_rmse_meth_minRmse(SIGNAL_IDX, CONFIG_IDX) = mean(rms_err(k+1:end,2));
%std_rmse_meth_minRmse(SIGNAL_IDX, CONFIG_IDX) = std(rms_err(k+1:end,2));

disp("Iteration (" + num2str(SIGNAL_IDX) + "), config (" + num2str(CONFIG_IDX) + ")")
disp("Burdet et al. (2000) like method mean RMSE: " +...
    num2str(mean(rms_err(k+1:end,1))) + ...
    " (+/- " + num2str(std(rms_err(k+1:end,1))) + ")")
disp("Min RMSE method mean RMSE: " + ...
    num2str(mean(rms_err(k+1:end,2))) + ...
    " (+/- " + num2str(std(rms_err(k+1:end,2))) + ")")
if DO_SPLINE_INTERP && CONFIG_IDX == 1 % TODO Correct that..
    disp("Avg Spline method mean RMSE: " + ...
        num2str(mean(rms_err(k+1:end,3))) + ...
        " (+/- " + num2str(std(rms_err(k+1:end,3))) + ")")
    disp("Best Spline method mean RMSE: " + ...
        num2str(mean(rms_err(k+1:end,4))) + ...
        " (+/- " + num2str(std(rms_err(k+1:end,4))) + ")")
    disp("Worst Spline method mean RMSE: " + ...
        num2str(mean(rms_err(k+1:end,5))) + ...
        " (+/- " + num2str(std(rms_err(k+1:end,5))) + ")")
end
if SAVE_FIGURES
    drawnow
    saveAllFig(base_save_path + "misc_savefig",...
        SIGNAL_IDX, 1); % save all current figures and then closes them
end
% clear everything but macros, analytics and necessary data about input signal
if CLEAR_TEMPORARY_DATA
    clearvars -except ncycles min_idx fsig dt NB_OF_GLOBAL_ITERATIONS ...
    NB_OF_CONFIGURATIONS NB_TIME_DISTORTIONS TIME_VARIANT_MAGN NB_OF_METHODS...
    TIME_VARIANT_PHASE SIGNAL_IDX CONFIG_IDX RAND_GEN TIME_DISTORTION_GROWTH ...
    mean_rmse_meth_burdet std_rmse_meth_burdet mean_rmse_meth_minRmse ...
    std_rmse_meth_minRmse DISP_ALL_CANDIDATES_AND_BEST_MATCH_FOR_CYCLES ...
    errors base_save_path SAVE_FIGURES CLEAR_TEMPORARY_DATA DO_SPLINE_INTERP ...
    DISP_SPLINE_ALL_CANDIDATES EXTERNAL_INPUT_SIGNAL RAND_GEN_LIST
end
end % for CONFIG_IDX = 1:NB_OF_CONFIGURATIONS
RAND_GEN_LIST{SIGNAL_IDX} = RAND_GEN;
% clear everything but macros and analytics
if CLEAR_TEMPORARY_DATA
    clearvars -except NB_OF_GLOBAL_ITERATIONS NB_OF_CONFIGURATIONS NB_OF_METHODS...
    NB_TIME_DISTORTIONS TIME_VARIANT_MAGN TIME_VARIANT_PHASE SIGNAL_IDX ...
    CONFIG_IDX TIME_DISTORTION_GROWTH mean_rmse_meth_burdet std_rmse_meth_burdet ...
    mean_rmse_meth_minRmse std_rmse_meth_minRmse DISP_ALL_CANDIDATES_AND_BEST_MATCH_FOR_CYCLES ...
    errors base_save_path SAVE_FIGURES CLEAR_TEMPORARY_DATA DO_SPLINE_INTERP ...
    DISP_SPLINE_ALL_CANDIDATES EXTERNAL_INPUT_SIGNAL RAND_GEN_LIST
end
end % for SIGNAL_IDX = 1:NB_OF_GLOBAL_ITERATIONS

% figure
% hold on 
% legend_names = [];
% for conf_idx = 1:NB_OF_CONFIGURATIONS
%     errorbar(mean_rmse_meth_burdet(:,conf_idx), std_rmse_meth_burdet(:,conf_idx))
%     errorbar(mean_rmse_meth_minRmse(:,conf_idx), std_rmse_meth_minRmse(:,conf_idx))
%     new_legend_names = "RMSE cnfig (" + num2str(conf_idx) +  [") Burdet", ") minR"];
%     legend_names = [legend_names, new_legend_names];
% end
% legend(legend_names)
% 
% for conf_idx = 1:NB_OF_CONFIGURATIONS
%     disp("Burdet conf n°" + num2str(conf_idx)+ ": mean " + num2str(mean(mean_rmse_meth_burdet(:,conf_idx))));
%     disp("MinRMSE conf n°" + num2str(conf_idx)+ ": mean " + num2str(mean(mean_rmse_meth_minRmse(:,conf_idx)))); 
% end
% 
% for conf_idx = 1:NB_OF_CONFIGURATIONS
%     disp("Burdet conf n°" + num2str(conf_idx)+ ": mean std " + num2str(mean(std_rmse_meth_burdet(:,conf_idx))));
%     disp("MinRMSE conf n°" + num2str(conf_idx)+ ": mean std " + num2str(mean(std_rmse_meth_minRmse(:,conf_idx)))); 
% end

% figure
% for cnfg_i = 1:length(NB_OF_CONFIGURATIONS)
%     
% end

%save("test_prediction_trajectoire_real_force_1.mat")
save("test_prediction_trajectoire_real_position_1.mat")
%% Script to evaluate spline fitting accuracy

clear all
close all

addpath('../utils');
addpath('../utils/cycles');
addpath('../../../utils');
addpath('../../../force_torque_sensor');

%%%%%%%%%%
% MACRO

ANAL_FORCE = 0; % set to 0 to use position
DISP_SPLINES = 1; 
LOW_PASS_FREQ = 50;
MIN_WINDOW = 50;
MAX_WINDOW = 500;
WINDOW_STEP = 1;
LANDING_LIST = (MIN_WINDOW:WINDOW_STEP:MAX_WINDOW);
EVAL_IDX = 200-1; % splines will be evaluated on this nb of samples + 1
OFFSET_IDX = 300; %  
NB_TEST = 14;

%%%%%%%%%%
% DATA

necessary_variables = {'forces_unf', 'torques_unf', 'thetas', 'dt', 't',...
        'mocap_marker_robot_base', 'idx_ball_off_ramp', 'SINGLE_MOCAP_FITTING'};
load("../data_2020_Nov_17/data_without_impacts_2020_11_17.mat", necessary_variables{:}); 



if SINGLE_MOCAP_FITTING % first data cell should be deleted
    forces_unf(1) = [];
    torques_unf(1) = [];
    thetas(1) = [];
    mocap_marker_robot_base(1) = [];
    idx_ball_off_ramp(1)= [];
    t(1) = [];
end

if NB_TEST > length(t)
    NB_TEST = length(t);
end

idx_start_arr = cell2mat(idx_ball_off_ramp);
idx_end_arr = cellfun(@length,t);
rmse_global_list = [];
nb_splines = floor(MAX_WINDOW - MIN_WINDOW)/WINDOW_STEP+1;
%load('Burdet2000/data/test_prediction_trajectoire_real_position_20Hz.mat', 'RAND_GEN_LIST');
load('random_gen_list.mat')

for EXP_NB = 9:NB_TEST
    
    [f_int,~,~]= forces_filtering(forces_unf{EXP_NB}', torques_unf{EXP_NB}', ...
        thetas{EXP_NB}', t');
    nsig = -1.*f_int(3,:);
    tmp = mocap_marker_robot_base{EXP_NB};
    nsig_pos = tmp(:,3)'; % z position
    clear tmp
        
    % filtering
    fc = LOW_PASS_FREQ; % cut off frequency
    [b,a] = butter(2,fc/(1/(2*dt)),'low'); 
    fsig = filtfilt(b,a,nsig);
    fsig_pos = filtfilt(b,a,nsig_pos);
    fc = 2; % cut off frequency
    [b,a] = butter(2,fc/(1/(2*dt)),'low'); 
    sig = filtfilt(b,a,nsig_pos);
    
    % to avoid the irrelevant data at both the beginning and end of the
    % signal
    idx_start = idx_start_arr(EXP_NB);
    idx_end = idx_end_arr(EXP_NB);
    nsig = nsig(idx_start:idx_end); % noisy force signal
    fsig = fsig(idx_start:idx_end); % filtered force signal
    nsig_pos = nsig_pos(idx_start:idx_end); % noisy position signal
    fsig_pos = fsig_pos(idx_start:idx_end); % filtered position signal
    sig = sig(idx_start:idx_end); % 2Hz low pass filtered position signal
    t_i = t{EXP_NB}(idx_start:idx_end)';
    % dt = (t_i(end) - t_i(1)) / length(t_i);
    
    dsig = Iu_diffcent(t_i',sig');
    idx_pks = crossing(dsig);
    idx_pks = idx_pks(1:2:end); % selection of only half the peaks
    
    cycles = splitCycles(fsig_pos, fsig, t_i, idx_pks);
    
    try
        RAND_GEN = RAND_GEN_LIST{EXP_NB};
    catch % if RAND_GEN_LIST is not yet defined, populates it
        RAND_GEN = rand([1 length(cycles)]);
        RAND_GEN_LIST{EXP_NB} = RAND_GEN;
    end
  
    rmse_list = NaN(nb_splines, length(cycles)-1);
    %fe_list = NaN(nb_splines, length(cycles)-1);
    
    % cycles iterations
    for i = 1:length(cycles)-1 % cycles
        % 300 ms + a rand number between 0 and 300 are used
        % for both the splines & evaluation window starting point
        rand_val = floor(RAND_GEN(i)*OFFSET_IDX);
        idx_p = OFFSET_IDX + rand_val;
        interp_idx = 1:idx_p+MAX_WINDOW;
        eval_idx = (0:EVAL_IDX)+idx_p+1;
        spline_cand = NaN(nb_splines, length(interp_idx));
        spline_cost = NaN(nb_splines, 1); % RMS error on eval_idx
        %spline_f_err = NaN(nb_splines, 1); % final error
        
        if idx_p + MAX_WINDOW + 1 >= length(cycles(i).position)
            % some landing points are on the next cycle
            data_pos = [cycles(i).position, cycles(i+1).position(2:end)];
            data_for = [cycles(i).force, cycles(i+1).force(2:end)];
            data_t = [cycles(i).time, cycles(i+1).time(2:end)];
            if idx_p + MAX_WINDOW + 1 >= length(data_t)
                % this cycle is pathological, it should not be considered
                spline_rmse(sp_nb) = NaN;
                continue
            end
        else
            data_pos = cycles(i).position;
            data_for = cycles(i).force;
            data_t = cycles(i).time;
        end
        
        for sp_nb = 1:nb_splines
            % cubic splines need 4 points, for signal and derivatives
            window = MIN_WINDOW + (WINDOW_STEP*sp_nb-1);
            knots_indexes = [idx_p-1,idx_p,idx_p+window,idx_p+window+1];
            
            % splines polynomial equation
            spline_pp(sp_nb) = spline(data_t(knots_indexes), data_pos(knots_indexes));
            % splines populations on the interval considered
            spline_cand(sp_nb, :) = ppval(spline_pp(sp_nb), data_t(interp_idx));
            spline_rmse(sp_nb) = rms(nansum([data_pos(eval_idx); -spline_cand(sp_nb, eval_idx)],1));
            %spline_f_err(sp_nb) = data_pos(eval_idx(end)) - spline_cand(sp_nb, eval_idx(end));
        end
        
        [~, spline_min_rmse_idx] = min(spline_rmse);
        %[~, spline_min_fe_idx] = min(abs(spline_f_err));
        
        if DISP_SPLINES
            figure(10)
            hold off
            p0 = plot(data_pos, 'Color', [0 0.4470 0.7410], 'linewidth', 2);
            hold on
            p1 = plot(spline_cand', 'Color', [0.3010 0.7450 0.9330]);
            for p_idx = 1:length(p1)
                p1(p_idx).Color(4) = 0.3;
            end
            p_rmse = plot(spline_cand(spline_min_rmse_idx, :), '--', 'Color', [0.4660, 0.6740, 0.1880]);
            %p_fe = plot(spline_cand(spline_min_fe_idx, :), ':', 'Color', [0, 0.5, 0]);   
            legend([p0 p1(1) p_rmse], ["actual","splines","best RMSE"])
            title('Splines fitting for cycle n°' + string(i) + ...
                ', best RMSE window:' + string(LANDING_LIST(spline_min_rmse_idx)) ...
                + 'ms')
        end
        
        rmse_list(:,i) = spline_rmse;
        %fe_list(:,i) = spline_f_err;        
    end
    
    rmse_global_list = [rmse_global_list, rmse_list];
    
end

mean_rmse = mean(rmse_global_list,2);
std_rmse = std(rmse_global_list,0,2);
perc_rmse = prctile(rmse_global_list,[10 25 50 75 90],2);
%mean_fe = mean(fe_list,2);

figure
%plotStdSurface(mean_rmse, std_rmse, LANDING_LIST, lines(1), 1)
hold on
%plot(LANDING_LIST, mean_rmse, 'Color', lines(1), 'linewidth', 2)
plot(LANDING_LIST, perc_rmse(:,1), ':', 'Color', lines(1))
plot(LANDING_LIST, perc_rmse(:,5), ':', 'Color', lines(1))
plot(LANDING_LIST, perc_rmse(:,2), '--', 'Color', lines(1))
plot(LANDING_LIST, perc_rmse(:,4), '--', 'Color', lines(1))
plot(LANDING_LIST, perc_rmse(:,3), 'k')
title('RMSE error')

time_window = LANDING_LIST';
rmse_table = table(time_window,perc_rmse.*1e3);
write(rmse_table,'spline_rmse_evaluation_tot.csv','Delimiter',',');


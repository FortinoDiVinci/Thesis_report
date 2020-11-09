%% Script to test the implementation of Burdet et al. algorithm (2000)
clear all
close all

%%%%%%%%%%
% MACRO

% INPUT SIGNAL
TIME_VARIANT_MAGN = 1;
TIME_VARIANT_PHASE = 1;

%%%%%%%%%%%%%%%%%%%%%%
%% SIGNAL GENERATION

% "ideal" signal
dt = 1e-3;      % time sampling
f = 0.8;        % sine frequency
alp = 10;        % frequency multiplier for second sine
phi = pi/2;   % phase delay of second sine
a1 = 1;         % first sine magnitude
a2 = 0;%a1/5;      % 2nd sine magnitude
t_max = 100 - dt;
t = 0:dt:t_max; % time
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
title('Input signal')

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
title("Cycles frequencies variations, base freq: " + num2str(f))

cycles = splitCycles([], fsig, t, idx_pks);

figure
subplot(2,1,1)
hold on
for i = 1:length(cycles)
    p1 = plot(cycles(i).force, 'Color', [0 0.4470 0.7410]);
    p1.Color(4) = 0.75;
end
title('Cyclic data')

%%%%%%%%%%%%%%%%%%%%%
%% Prediction algorithm

% normalization
ncycles = cycleNormalization(cycles); % should already be the case...

subplot(2,1,2)
hold on
for i = 1:length(cycles)
    p1 = plot(ncycles(i).force, 'Color', [0 0.4470 0.7410]);
    p1.Color(4) = 0.75;
end
title('Normalized cyclic data')

RAND_GEN = rand([1 length(ncycles)]);
% mean computation
% /!\ In the original algorithm the mean is computed independantly for 
% the positive and negative acceleration part, but here, multiple cases
% happens and the average is therefore not computed independantly

k = 10; % mean will be computed on the 10 previous cycles
m_nb = 11; % nb of magnitudes
ts_nb = 11; % nb of time shifts
coef_std = 3; % max magnitude is determined by the std times this coeff

prev_cycles = ncycles(1:k);
%magn_list = 1 + [0:0.01:0.10];
time_shift = 0 + [-40:8:40];
cand = NaN(m_nb*ts_nb, length(ncycles(1).force));
for i = k+1:length(ncycles)-1
    
    % mean computation
    forces_list = cell2mat({prev_cycles.force}');
    avg_force = mean(forces_list,1);
    std_force = max(std(forces_list));
    magn_list = 1 + linspace(-5*std_force,5*std_force,m_nb);
    
    % candidates population
    for ii = 1:ts_nb
        if time_shift(ii) > 0
            cand_shift = cat(2,avg_force(1+time_shift(ii):end), ...
                NaN(1,time_shift(ii))); % complete the missing data with NaN
            %cat(2,ncycles(i).force(1+time_shift(ii):end), ...
            %    ncycles(i+1).force(2:time_shift(ii)+1));
        elseif time_shift(ii) < 0
            cand_shift = cat(2,NaN(1,-time_shift(ii)), ...
                avg_force(1:end+time_shift(ii)));
        else
            cand_shift = avg_force(1+time_shift(ii):end);
            %ncycles(i).force(1:end); 
        end
        for jj = 1:m_nb
            cand(m_nb*(ii-1)+jj,:) = cand_shift.*magn_list(jj);
        end
    end
    
    if i == k+1 %% only disp the first one
        figure
        p = plot(cand');
        hold on
        plot(forces_list', 'k', 'LineWidth', 1.1);
        plot(avg_force, '--k', 'LineWidth', 3)
        for i_p = length(p)
            p(i_p).Color(4) = 0.3;
        end
    end
    
    % best match
    % For this prototype, 300 ms + a rand number between 0 and 300 are used
    % for the unperturbed trajectory comparing
    rand_val = floor(RAND_GEN(i)*0.3/dt);
    d_cost = zeros(size(cand,1),1);
    alt_cost = zeros(size(cand,1),1);
    alpha = 0.94;
    for c = 1:size(cand,1)
        for idx_t = 1:0.3/dt + rand_val
            d_cost(c) = nansum([(ncycles(i).force(idx_t) - cand(c,idx_t))^2, alpha*d_cost(c)]);
        end
        pred_idx = 0.3/dt + rand_val;
        learn_idx = (-99:0)+pred_idx;
        alt_cost(c) = rms(nansum([ncycles(i).force(learn_idx); -cand(c,learn_idx)],1));
    end
    [~, min_idx] = min(d_cost);
    b_cand = cand(min_idx,:);
    o_cand = cand(setdiff(1:end,min_idx),:);
    [~, min_idx] = min(alt_cost);
    alt_b_cand = cand(min_idx,:);
    
    % prev_cycles FIFO 
    % a conditonnal evaluation of the new std should say if the new cycle
    % is added or not.
    prev_cycles = [prev_cycles(2:end), ncycles(i)];
    
    % a 300ms estimated interval is being evaluated 
    start_idx = rand_val + 0.3/dt;
    
    rms_err(i) = rms( ncycles(i).force(start_idx:start_idx+300) - b_cand(start_idx:start_idx+300) );
    rms_err_alt(i) = rms( ncycles(i).force(start_idx:start_idx+300) - alt_b_cand(start_idx:start_idx+300) );
    
    figure
    p1 = plot(o_cand', 'Color', [0.3010 0.7450 0.9330]);
    hold on
    plot(ncycles(i).force, 'Color', [0 0.4470 0.7410])
    plot(b_cand, 'k')
    p2 = plot(alt_b_cand, 'k');
    plot(1:start_idx, b_cand(1:start_idx), 'Color', [0.4660 0.6740 0.1880])
    plot(start_idx:start_idx+300, b_cand(start_idx:start_idx+300), 'Color', [0.8500 0.3250 0.0980])
    for p_idx = 1:length(p1)
        p1(p_idx).Color(4) = 0.3;
    end
    p2.Color(4) = 0.7;
    
    plot(1:start_idx, alt_b_cand(1:start_idx), '--','Color', [0.4660 0.6740 0.1880])
    plot(start_idx:start_idx+300, alt_b_cand(start_idx:start_idx+300), '--', 'Color', [0.8500 0.3250 0.0980])
    
    title("RMS Error c: " + num2str(rms_err(i)) + ", min RMSE: " + num2str(rms_err_alt(i)))
    
end

figure
plot(rms_err)
hold on
plot(rms_err_alt)
legend('Recurcive choice (Burdet)', 'min RMSE over last 100ms')
title('RMSE of the best candidate over all cycles')
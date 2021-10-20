% plot data from a recorded fitting
clear all

addpath('../../utils')
addpath('../../utils/cycles')

load('data/burdet_algo_20Hz_failure2.mat', 'b_cand', 'cand', 'i', ...,
    'alt_b_cand', 'cycles', 'pred_indexes', 'learn_idx', 'nsig', 't');

figure
hold on
p0 = plot(cycles(i).force, 'Color', [0 0.4470 0.7410], 'Linewidth', 2); % ncycles
p1 = plot(cand(:,:)', 'Color', [0.3010 0.7450 0.9330]);
for p_idx = 1:length(p1)
        p1(p_idx).Color(4) = 0.3;
end
plot(alt_b_cand, 'k');
plot(b_cand, 'k--');
p3 = plot(learn_idx, b_cand(learn_idx), 'Color', [0.4660 0.6740 0.1880], ...
    'Linewidth', 2);
plot(pred_indexes, b_cand(pred_indexes), 'Color', [0.8500 0.3250 0.0980], ...
    'Linewidth', 2)
p4 = plot(learn_idx, alt_b_cand(learn_idx), '--','Color', [0.4660 0.6740 0.1880], ...
    'Linewidth', 2);
plot(pred_indexes, alt_b_cand(pred_indexes), '--', 'Color', [0.8500 0.3250 0.0980], ...
    'Linewidth', 2)
legend([p0 p1(1) p3 p4], ["Actual signal", "Candidates", "BC Burdet", "BC min RMSE"]);

candidates = cand';
burdet_candidate = b_cand';
alternative_candidate = alt_b_cand';
actual_trajectory = cycles(i).force(1:length(burdet_candidate))';
idx_p = NaN(size(burdet_candidate));
idx_p(1,1) = pred_indexes(1);

offset = min([min(min(candidates)); actual_trajectory]);
candidates = (candidates - offset)*100; % offset and conversion to cm
burdet_candidate = (burdet_candidate - offset)*100;
alternative_candidate = (alternative_candidate - offset)*100;
actual_trajectory = (actual_trajectory - offset)*100;

cand_table = table(candidates);
best_cand_table = table(actual_trajectory, burdet_candidate, alternative_candidate, idx_p);
candidates = downsample(candidates,10);
idx = [1:10:length(burdet_candidate)]';

figure
hold on
p0 = plot(actual_trajectory, 'Color', [0 0.4470 0.7410], 'Linewidth', 2); % ncycles
p1 = plot(idx, candidates(:,:), 'Color', [0.3010 0.7450 0.9330]);
for p_idx = 1:length(p1)
        p1(p_idx).Color(4) = 0.3;
end
plot(alternative_candidate, 'k');
plot(burdet_candidate, 'k--');
p3 = plot(learn_idx, burdet_candidate(learn_idx), 'Color', [0.4660 0.6740 0.1880], ...
    'Linewidth', 2);
plot(pred_indexes, burdet_candidate(pred_indexes), 'Color', [0.8500 0.3250 0.0980], ...
    'Linewidth', 2)
p4 = plot(learn_idx, alternative_candidate(learn_idx), '--','Color', [0.4660 0.6740 0.1880], ...
    'Linewidth', 2);
plot(pred_indexes, alternative_candidate(pred_indexes), '--', 'Color', [0.8500 0.3250 0.0980], ...
    'Linewidth', 2)
legend([p0 p1(1) p3 p4], ["Actual signal", "Candidates", "BC Burdet", "BC min RMSE"]);

cand_table_subsamp = table(idx,candidates);

write(cand_table,'candidates.csv','Delimiter',',');
write(cand_table_subsamp,'candidates_subsamp.csv','Delimiter',',');
write(best_cand_table,'best_candidates.csv','Delimiter',',');

position = downsample(nsig,10);
time = downsample(t,10);

figure
plot(time, position)

complete_signal = table(time,position);
write(complete_signal,'position_signal.csv','Delimiter',',');
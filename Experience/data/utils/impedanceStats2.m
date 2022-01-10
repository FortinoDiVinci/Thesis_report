function impedanceStats2(cycles, exp_list)

users = [];
folder = "impedance_statistics/";
nb_trials = 5;

for user_i = 1:length(cycles)    
    users = [users, [cycles{user_i}(1).user]];
    %
    ph_u{user_i} = [cycles{user_i}.type_dist];
    ph_ratio_u{user_i} = [cycles{user_i}.ratio_dist];
    K_u{user_i} = [cycles{user_i}.K];
    B_u{user_i} = [cycles{user_i}.B];
    M_u{user_i} = [cycles{user_i}.M];
    R2_u{user_i} = [cycles{user_i}.R2];
    for ni = 1:nb_trials
        tmp = [];
        idx_trial = [cycles{user_i}.exp_it_cmb] == ni;
        idx_pert = [cycles{user_i}.is_dist];
        idx = idx_trial & idx_pert; % only perturbed cycles
        %b_err_it{user_i, ni} = [cycles{user_i}(idx).target_error];
        th_it{user_i, ni} = [cycles{user_i}(idx).target_height];
        ph_it{user_i, ni} = [cycles{user_i}(idx).type_dist];
        ph_ratio_it{user_i, ni} = [cycles{user_i}(idx).ratio_dist];
        K_it{user_i, ni} = [cycles{user_i}(idx).K];
        B_it{user_i, ni} = [cycles{user_i}(idx).B];
        M_it{user_i, ni} = [cycles{user_i}(idx).M];
        R2_it{user_i, ni} = [cycles{user_i}(idx).R2];
%         sum_time = 0; % to add the time of failed experiments
%         f_id_exp = find(idx == 1, 1, 'first');
%         for i=1:length(idx)
%             if idx_trial(i) == 0
%                 continue
%             end
%             if ~isempty(tmp) 
%                 if cycles{user_i}(i).t(1) + sum_time < tmp(i-f_id_exp) % fused experiment
%                     sum_time = sum_time + cycles{user_i}(i-1).t(end) - cycles{user_i}(i).t(1) + 1e-3;
%                 end
%             end
%             tmp = [tmp, cycles{user_i}(i).t(1) + sum_time];
%         end
%         time_it{user_i, ni} = tmp;
    end
end

th_it = cellfun(@unique, th_it);
%%%%%%
% EXP 2: HEIGHT
%%%%%%
% K
K_th_it = [reshape(K_it(th_it == 1.5), [31,1]), reshape(K_it(th_it == 2), [31,1])];
for i = 1:size(K_th_it, 2)
    q = cell2mat(cellfun(@(x) quantile(x,3), K_th_it(:,i), 'UniformOutput', 0));
    K_th_med(:,i) = q(:,2);
    K_th_notch(:,i) = 1.57*(q(:,3)-q(:,1))/sqrt(31);
end
tab = table(users', K_th_med, K_th_notch);
% write(tab,folder+"K_per_target_height.csv",'Delimiter',',');
% B
B_th_it = [reshape(B_it(th_it == 1.5), [31,1]), reshape(B_it(th_it == 2), [31,1])];
for i = 1:size(B_th_it, 2)
    q = cell2mat(cellfun(@(x) quantile(x,3), B_th_it(:,i), 'UniformOutput', 0));
    B_th_med(:,i) = q(:,2);
    B_th_notch(:,i) = 1.57*(q(:,3)-q(:,1))/sqrt(31);
end
tab = table(users', B_th_med, B_th_notch);
% write(tab,folder+"B_per_target_height.csv",'Delimiter',',');
% M
M_th_it = [reshape(M_it(th_it == 1.5), [31,1]), reshape(M_it(th_it == 2), [31,1])];
for i = 1:size(M_th_it, 2)
    q = cell2mat(cellfun(@(x) quantile(x,3), M_th_it(:,i), 'UniformOutput', 0));
    M_th_med(:,i) = q(:,2);
    M_th_notch(:,i) = 1.57*(q(:,3)-q(:,1))/sqrt(31);
end
tab = table(users', M_th_med, M_th_notch);
% write(tab,folder+"M_per_target_height.csv",'Delimiter',',');
% R2
R2_th_it = [reshape(R2_it(th_it == 1.5), [31,1]), reshape(R2_it(th_it == 2), [31,1])];
for i = 1:size(R2_th_it, 2)
    q = cell2mat(cellfun(@(x) quantile(x,3), R2_th_it(:,i), 'UniformOutput', 0));
    R2_th_med(:,i) = q(:,2);
    R2_th_notch(:,i) = 1.57*(q(:,3)-q(:,1))/sqrt(31);
end
tab = table(users', R2_th_med, R2_th_notch);
% write(tab,folder+"R2_per_target_height.csv",'Delimiter',',');
%%%
% same work but with R2 sorting (>0.5)
for user_i = 1:size(K_th_it, 1)
    for i = 1:2
        K_th_it_r2{user_i, i} = K_th_it{user_i, i}(R2_th_it{user_i, i} > 0.5);
        B_th_it_r2{user_i, i} = B_th_it{user_i, i}(R2_th_it{user_i, i} > 0.5);
        M_th_it_r2{user_i, i} = M_th_it{user_i, i}(R2_th_it{user_i, i} > 0.5);
        R2_th_it_r2{user_i, i} = R2_th_it{user_i, i}(R2_th_it{user_i, i} > 0.5);
    end
end
%
for i = 1:size(K_th_it_r2, 2)
    % K
    q = cell2mat(cellfun(@(x) quantile(x,3), K_th_it_r2(:,i), 'UniformOutput', 0));
    K_th_r2_med(:,i) = q(:,2);
    K_th_r2_notch(:,i) = 1.57*(q(:,3)-q(:,1))/sqrt(31);
    % K
    q = cell2mat(cellfun(@(x) quantile(x,3), B_th_it_r2(:,i), 'UniformOutput', 0));
    B_th_r2_med(:,i) = q(:,2);
    B_th_r2_notch(:,i) = 1.57*(q(:,3)-q(:,1))/sqrt(31);
    % K
    q = cell2mat(cellfun(@(x) quantile(x,3), M_th_it_r2(:,i), 'UniformOutput', 0));
    M_th_r2_med(:,i) = q(:,2);
    M_th_r2_notch(:,i) = 1.57*(q(:,3)-q(:,1))/sqrt(31);
    % K
    q = cell2mat(cellfun(@(x) quantile(x,3), R2_th_it_r2(:,i), 'UniformOutput', 0));
    R2_th_r2_med(:,i) = q(:,2);
    R2_th_r2_notch(:,i) = 1.57*(q(:,3)-q(:,1))/sqrt(31);
end
tab = table(users', K_th_r2_med, K_th_r2_notch);
write(tab,folder+"K_per_target_height_r2.csv",'Delimiter',',');

%%%%%%
% EXP 1: PHASE
%%%%%%
ph1_unsorted = mean(ph_ratio_u{1}(:, ph_u{1} == 1));
ph2_unsorted = mean(ph_ratio_u{1}(:, ph_u{1} == 2));
ph3_unsorted = mean(ph_ratio_u{1}(:, ph_u{1} == 3));
[~,srt_ph] = sort([ph1_unsorted,ph2_unsorted,ph3_unsorted]);
% 
K_th2_it = [reshape(K_it(th_it == 1.75), [31,3])];
B_th2_it = [reshape(B_it(th_it == 1.75), [31,3])];
M_th2_it = [reshape(M_it(th_it == 1.75), [31,3])];
R2_th2_it = [reshape(R2_it(th_it == 1.75), [31,3])];
ph_th2_it = [reshape(ph_it(th_it == 1.75), [31,3])];
for user_i = 1:size(K_th2_it, 1)
    K_th2{user_i,1} = [K_th2_it{user_i, :}];
    B_th2{user_i,1} = [B_th2_it{user_i, :}];
    M_th2{user_i,1} = [M_th2_it{user_i, :}];
    R2_th2{user_i,1} = [R2_th2_it{user_i, :}];
    ph_th2{user_i,1} = [ph_th2_it{user_i, :}];
end
% K
for user_i = 1:length(ph_u) 
    q_ph1(user_i,:) = quantile(K_th2{user_i}(ph_th2{user_i} == srt_ph(1)),3);
    q_ph2(user_i,:) = quantile(K_th2{user_i}(ph_th2{user_i} == srt_ph(2)),3);
    q_ph3(user_i,:) = quantile(K_th2{user_i}(ph_th2{user_i} == srt_ph(3)),3);
    q_ph1_r2(user_i,:) = quantile(K_th2{user_i}(ph_th2{user_i} == srt_ph(1) & R2_th2{user_i} > 0.5),3);
    q_ph2_r2(user_i,:) = quantile(K_th2{user_i}(ph_th2{user_i} == srt_ph(2) & R2_th2{user_i} > 0.5),3);
    q_ph3_r2(user_i,:) = quantile(K_th2{user_i}(ph_th2{user_i} == srt_ph(3) & R2_th2{user_i} > 0.5),3);  
end
K_ph_med = [q_ph1(:,2), q_ph2(:,2), q_ph1(:,3)];
K_ph_notch = [q_ph1(:,3)-q_ph1(:,1),q_ph2(:,3)-q_ph2(:,1),q_ph3(:,3)-q_ph3(:,1)].*(1.57/sqrt(31));
K_ph_r2_med = [q_ph1_r2(:,2), q_ph2_r2(:,2), q_ph1_r2(:,3)];
K_ph_r2_notch = [q_ph1_r2(:,3)-q_ph1_r2(:,1),q_ph2_r2(:,3)-q_ph2_r2(:,1),q_ph3_r2(:,3)-q_ph3_r2(:,1)].*(1.57/sqrt(31));
tab = table(users', K_ph_med, K_ph_notch);
% write(tab,folder+"K_per_phase.csv",'Delimiter',',');
tab = table(users', K_ph_r2_med, K_ph_r2_notch);
write(tab,folder+"K_per_phase_r2.csv",'Delimiter',',');
% B
for user_i = 1:length(ph_u) 
    q_ph1(user_i,:) = quantile(B_th2{user_i}(ph_th2{user_i} == srt_ph(1)),3);
    q_ph2(user_i,:) = quantile(B_th2{user_i}(ph_th2{user_i} == srt_ph(2)),3);
    q_ph3(user_i,:) = quantile(B_th2{user_i}(ph_th2{user_i} == srt_ph(3)),3);
end
B_ph_med = [q_ph1(:,2), q_ph2(:,2), q_ph1(:,3)];
B_ph_notch = [q_ph1(:,3)-q_ph1(:,1),q_ph2(:,3)-q_ph2(:,1),q_ph3(:,3)-q_ph3(:,1)].*(1.57/sqrt(31));
tab = table(users', B_ph_med, B_ph_notch);
% write(tab,folder+"B_per_phase.csv",'Delimiter',',');
% M
for user_i = 1:length(ph_u) 
    q_ph1(user_i,:) = quantile(M_th2{user_i}(ph_th2{user_i} == srt_ph(1)),3);
    q_ph2(user_i,:) = quantile(M_th2{user_i}(ph_th2{user_i} == srt_ph(2)),3);
    q_ph3(user_i,:) = quantile(M_th2{user_i}(ph_th2{user_i} == srt_ph(3)),3);
end
M_ph_med = [q_ph1(:,2), q_ph2(:,2), q_ph1(:,3)];
M_ph_notch = [q_ph1(:,3)-q_ph1(:,1),q_ph2(:,3)-q_ph2(:,1),q_ph3(:,3)-q_ph3(:,1)].*(1.57/sqrt(31));
tab = table(users', M_ph_med, M_ph_notch);
% write(tab,folder+"M_per_phase.csv",'Delimiter',',');
% R2
for user_i = 1:length(ph_u) 
    q_ph1(user_i,:) = quantile(R2_th2{user_i}(ph_th2{user_i} == srt_ph(1)),3);
    q_ph2(user_i,:) = quantile(R2_th2{user_i}(ph_th2{user_i} == srt_ph(2)),3);
    q_ph3(user_i,:) = quantile(R2_th2{user_i}(ph_th2{user_i} == srt_ph(3)),3);
end
R2_ph_med = [q_ph1(:,2), q_ph2(:,2), q_ph1(:,3)];
R2_ph_notch = [q_ph1(:,3)-q_ph1(:,1),q_ph2(:,3)-q_ph2(:,1),q_ph3(:,3)-q_ph3(:,1)].*(1.57/sqrt(31));
tab = table(users', R2_ph_med, R2_ph_notch);
% write(tab,folder+"R2_per_phase.csv",'Delimiter',',');

end
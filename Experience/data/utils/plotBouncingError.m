function plotBouncingError(cycles, nb_exp)
%
colors = hsv(length(cycles));
users = [];
folder = "ball_bouncing_statistics/";

for user_i = 1:length(cycles)    
    nb_exp_for_user_i = max([cycles{user_i}.exp_it]);    
    % bouncing error
    b_err{user_i} = [cycles{user_i}.target_error];
    % post perturbation ball bouncing error
%     b_err_post_dist{user_i} = [cycles{user_i}(circshift([cycles{user_i}(1:end-1).is_dist],1)).target_error];
%     b_err_npost_dist{user_i} = [cycles{user_i}(~circshift([cycles{user_i}(1:end-1).is_dist],1)).target_error];
    % (1:end-1) in case last data was perturbed
    b_err_post_dist{user_i} = [cycles{user_i}([cycles{user_i}.is_post_dist]).target_error];
    b_err_npost_dist{user_i} = [cycles{user_i}(~[cycles{user_i}.is_post_dist]).target_error];
    % frequency
    duration{user_i} = [cycles{user_i}.duration];
    idx_new_exp{user_i} = find(diff([cycles{user_i}.exp_it]));
    quartiles(:, user_i) = quantile(b_err{user_i},[0.25,0.5,0.75]);
    users = [users, [cycles{user_i}.user]];
    %
    real_th{user_i} = [cycles{user_i}.actual_target_height];
    [idx_th{user_i}, C_th(:,user_i)] = kmeans(real_th{user_i}', 3); 
    [C_th_sorted(:,user_i), idx_sorted] = sort(C_th(:,user_i));
    b_err_th1{user_i} = b_err{user_i}(idx_th{user_i} == idx_sorted(1));
    b_err_th2{user_i} = b_err{user_i}(idx_th{user_i} == idx_sorted(2));
    b_err_th3{user_i} = b_err{user_i}(idx_th{user_i} == idx_sorted(3));
    for ni = 1:nb_exp
        tmp = [];
        idx = ([cycles{user_i}.exp_it_cmb] == ni);
        b_err_it{user_i, ni} = [cycles{user_i}(idx).target_error];
        th_it{user_i, ni} = [cycles{user_i}(idx).target_height];
        real_th_it{user_i, ni} = [cycles{user_i}(idx).actual_target_height];
        ispre_it{user_i, ni} = [cycles{user_i}(idx).is_dist];
        ispost_it{user_i, ni} = [cycles{user_i}(idx).is_post_dist];
        ispost2_it{user_i, ni} = circshift(ispost_it{user_i, ni},1);  
        ispost2_it{user_i, ni}(1:2) = false;
        ispost3_it{user_i, ni} = circshift(ispost2_it{user_i, ni},1); 
        ispost3_it{user_i, ni}(1:3) = false; 
        be_pre_it{user_i, ni} = b_err_it{user_i, ni}(ispre_it{user_i, ni});
        be_post_it{user_i, ni} = b_err_it{user_i, ni}(ispost_it{user_i, ni});
        be_post2_it{user_i, ni} = b_err_it{user_i, ni}(ispost2_it{user_i, ni});
        be_post3_it{user_i, ni} = b_err_it{user_i, ni}(ispost3_it{user_i, ni});
        sum_time = 0; % to add the time of failed experiments
        f_id_exp = find(idx == 1, 1, 'first');
        for i=1:length(idx)
            if idx(i) == 0
                continue
            end
            if ~isempty(tmp) 
                if cycles{user_i}(i).t(1) + sum_time < tmp(i-f_id_exp) % fused experiment
                    sum_time = sum_time + cycles{user_i}(i-1).t(end) - cycles{user_i}(i).t(1) + 1e-3;
                end
            end
            tmp = [tmp, cycles{user_i}(i).t(1) + sum_time];
        end
        time_it{user_i, ni} = tmp;
    end
end

%%%%%%%%
% Are there significant mean differences between the 3 target heights in
% the bouncing error ?
%%%%%%%%
% For each users:
% figure
% tiledlayout('flow', 'TileSpacing', 'compact', 'Padding', 'compact');
% for user_i = 1:length(cycles)
%     nexttile
%     hold on
%     %C_th_sorted = sort(C_th(:,user_i));
%     res_th(user_i) = anova1([b_err_th1{user_i}, b_err_th2{user_i}, b_err_th3{user_i}],...
%         [C_th_sorted(1,user_i)*ones(size(b_err_th1{user_i})), ...
%          C_th_sorted(2,user_i)*ones(size(b_err_th2{user_i})), ...
%          C_th_sorted(3,user_i)*ones(size(b_err_th3{user_i}))], 'off');
%     boxplot([b_err_th1{user_i}, b_err_th2{user_i}, b_err_th3{user_i}],...
%         [C_th_sorted(1,user_i)*ones(size(b_err_th1{user_i})), ...
%          C_th_sorted(2,user_i)*ones(size(b_err_th2{user_i})), ...
%          C_th_sorted(3,user_i)*ones(size(b_err_th3{user_i}))],'Notch','on');
%      title("User #" + cycles{user_i}(1).user + ", pval: " + ...
%          string(res_th(user_i)*1e2) + "%")
% end

% For all users regrouped:
real_th_all = cell2mat(real_th);
b_err_all = cell2mat(b_err);
[idx_th_all, C_th_all] = kmeans(real_th_all', 3); 
[C_th_all_sorted, idx_th_all_sorted] = sort(C_th_all);
th1_err_all = real_th_all(idx_th_all == idx_th_all_sorted(1));
th2_err_all = real_th_all(idx_th_all == idx_th_all_sorted(2));
th3_err_all = real_th_all(idx_th_all == idx_th_all_sorted(3));

% Are the induced different target heights significantly different
res = anova1([th1_err_all, th2_err_all, th3_err_all], ...
    [C_th_all(idx_th_all_sorted(1))*ones(size(th1_err_all)), C_th_all(idx_th_all_sorted(2))*ones(size(th2_err_all)), ...
     C_th_all(idx_th_all_sorted(3))*ones(size(th3_err_all))], 'off')
figure
boxplot([th1_err_all, th2_err_all, th3_err_all], ...
    [C_th_all(idx_th_all_sorted(1))*ones(size(th1_err_all)), C_th_all(idx_th_all_sorted(2))*ones(size(th2_err_all)), ...
     C_th_all(idx_th_all_sorted(3))*ones(size(th3_err_all))], 'Notch', 'on')
title("Are the real target heights significantly different ?") % yes

b_err_th1_all = b_err_all(idx_th_all == idx_th_all_sorted(1));
b_err_th2_all = b_err_all(idx_th_all == idx_th_all_sorted(2));
b_err_th3_all = b_err_all(idx_th_all == idx_th_all_sorted(3));

% Are the ball bouncing error induced by those different height
% significantly different ?
figure
boxplot([b_err_th1_all, b_err_th2_all, b_err_th3_all], ...
    [C_th_all(idx_th_all_sorted(1))*ones(size(th1_err_all)), ...
     C_th_all(idx_th_all_sorted(2))*ones(size(th2_err_all)), ...
     C_th_all(idx_th_all_sorted(3))*ones(size(th3_err_all))],'Notch','on');
title("Are the bouncing error significantly different according to the target heights?") 
[~,~,stats] = anova1([b_err_th1_all, b_err_th2_all, b_err_th3_all], ...
    [C_th_all(idx_th_all_sorted(1))*ones(size(th1_err_all)), ...
     C_th_all(idx_th_all_sorted(2))*ones(size(th2_err_all)), ...
     C_th_all(idx_th_all_sorted(3))*ones(size(th3_err_all))]);
title("Are the bouncing error significantly different according to target height?") 
multcompare(stats,'CType','bonferroni'); %
% absolute errors
figure
boxplot(abs([b_err_th1_all, b_err_th2_all, b_err_th3_all]), ...
    [C_th_all(idx_th_all_sorted(1))*ones(size(th1_err_all)), ...
     C_th_all(idx_th_all_sorted(2))*ones(size(th2_err_all)), ...
     C_th_all(idx_th_all_sorted(3))*ones(size(th3_err_all))],'Notch','on');
title("Are the absolute bouncing errors significantly different according to the target heights?") 
[~,~,stats] = anova1(abs([b_err_th1_all, b_err_th2_all, b_err_th3_all]), ...
    [C_th_all(idx_th_all_sorted(1))*ones(size(th1_err_all)), ...
     C_th_all(idx_th_all_sorted(2))*ones(size(th2_err_all)), ...
     C_th_all(idx_th_all_sorted(3))*ones(size(th3_err_all))]);
%title("Are the absolute bouncing error significantly different according to target height?") 
multcompare(stats,'CType','bonferroni'); %

%%%%%%%%
% Are there significant bouncing error differences between post perturbed
%%%%%%%%
% cycles and others
[res,~,stats] = anova1([cell2mat(b_err_post_dist), cell2mat(b_err_npost_dist)], ...
    [repmat("post dist.", 1, length(cell2mat(b_err_post_dist))), ...
     repmat("others", 1, length(cell2mat(b_err_npost_dist)))]);
title("Are the bouncing error significantly different when the previous cycle was perturbed?") 
% absolute errors
[res,~,stats] = anova1(abs([cell2mat(b_err_post_dist), cell2mat(b_err_npost_dist)]), ...
    [repmat("post dist.", 1, length(cell2mat(b_err_post_dist))), ...
     repmat("others", 1, length(cell2mat(b_err_npost_dist)))]);
title("Are the bouncing error significantly different when the previous cycle was perturbed?") 

%%%%%%%%
% Duration VS height (correlation expected)
%%%%%%%%
% durations = cell2mat(duration)';
% target_heights = cell2mat(real_th)';
% 
% [idx_th, c_th] = kmeans(target_heights, 3);
% [c_th, idx_sorted] = sort(c_th);
% mean(durations(idx_th==idx_sorted(1))) % 1.0217
% mean(durations(idx_th==idx_sorted(2))) % 0.9302
% mean(durations(idx_th==idx_sorted(3))) % 0.8570

figure
plot(cell2mat(duration),cell2mat(b_err), '*')
% 92 - 2707 - 42277 - 43086[duration_s, order] = sort(cell2mat(duration));
% b_err_s = b_err_all(order);
% subsamp_d = downsample(duration_s(2707:42277), 50);
% subsamp_d2 = downsample(duration_s(92:2707), 10);
% subsamp_d3 = downsample(duration_s(42277:43086), 5);
% subsamp_d = [duration_s(1:91), subsamp_d2, subsamp_d, subsamp_d3, duration_s(43087:end)];
% subsamp_b_e = downsample(b_err_s(2707:42277), 50);
% subsamp_b_e2 = downsample(b_err_s(92:2707), 10);
% subsamp_b_e3 = downsample(b_err_s(42277:43086), 5);
% subsamp_b_e = [b_err_s(1:91),subsamp_b_e2, subsamp_b_e,subsamp_b_e3, b_err_s(43087:end)];
% figure
% plot(subsamp_d,subsamp_b_e, '*')
% d = subsamp_d';
% eps = subsamp_b_e';
% tab_be = table(d, eps);
% write(tab_be,'target_height_vs_duration.csv','Delimiter',',');

% figure
% plot(1./cell2mat(duration),cell2mat(real_th), '*')
% hold on
% for i=1:3
%     plot(mean(durations(idx_th==i)), c_th(i), 'rs')
% end

%%%%%%%
% Errors for all users
%%%%%%%
figure
tiledlayout('flow', 'TileSpacing', 'compact', 'Padding', 'compact');
for user_i = 1:length(cycles)
    nexttile
    hold on
    plot(b_err{user_i}, 'color', [colors(user_i,:), 0.05])
    ma = movmean(b_err{user_i}, 20); % moving average
    plot(ma, 'color', colors(user_i,:))
    plot(idx_new_exp{user_i}, ma(idx_new_exp{user_i}), 'k^', ...
        'MarkerFaceColor', colors(user_i,:), 'markersize', 10) 
    ylim([-0.4, 0.4])
    title("User #" + cycles{user_i}(1).user)
end

%%%%%%
% Users' expertise / precision
%%%%%%
[res,~,stats] = anova1(abs(cell2mat(b_err)), users);%, 'off');
multcompare(stats,'CType','bonferroni'); 
figure
boxplot(abs(cell2mat(b_err)), users, 'PlotStyle','compact');
title("User's ball bouncing errors")

% K Means partionning
%if res < 0.05
% repeatability score
repeat_score = quartiles(3, :) - quartiles(1, :); % repeatability
repeat_score_red = [repeat_score(1:17), repeat_score(19:end)]; % 18 is an outlier
[idx_rep_tmp, C_rep] = kmeans(repeat_score_red', 3, 'Display', 'final');
idx_rep = [idx_rep_tmp(1:17); 0; idx_rep_tmp(18:end)];
[~,idx_max_err] = max(C_rep);
idx_rep(18) = idx_max_err;
[C_rep_sorted, idx_rep_sorted] = sort(C_rep);   
% precision score
users_err_median = cellfun(@(x) median(sqrt(x.^2)), b_err); % 18 is an outlier
users_err_median_red = [users_err_median(1:17), users_err_median(19:end)];
[idx_acc_tmp, C_acc] = kmeans(users_err_median_red', 3, 'Display', 'final');  
idx_acc = [idx_acc_tmp(1:17); 0; idx_acc_tmp(18:end)];
[~,idx_max_err] = max(C_acc);
idx_acc(18) = idx_max_err;
[C_acc_sorted, idx_acc_sorted] = sort(C_acc);       
%end

color_list = lines(4);
idx_users = 1:length(cycles);
%repeatability
figure
plot(idx_users,repeat_score, '*', 'Color', color_list(1,:))
hold on
plot(idx_users(idx_rep == 1),repeat_score(idx_rep == 1), 'o', 'Color', color_list(2,:))
yline(C_rep(1), '--', 'Color', color_list(2,:))
plot(idx_users(idx_rep == 2),repeat_score(idx_rep == 2), 'o', 'Color', color_list(3,:))
yline(C_rep(2), '--', 'Color', color_list(3,:))
plot(idx_users(idx_rep == 3),repeat_score(idx_rep == 3), 'o', 'Color', color_list(4,:))
yline(C_rep(3), '--', 'Color', color_list(4,:))
xticks(idx_users)
xticklabels(convertStringsToChars(unique(users)));
xtickangle(60)
title("User's repeatability scores, with 3 kmeans for groups")
%precision
figure
plot(idx_users,users_err_median, '*', 'Color', color_list(1,:))
hold on
plot(idx_users(idx_acc == 1),users_err_median(idx_acc == 1), 'o', 'Color', color_list(2,:))
yline(C_acc(1), '--', 'Color', color_list(2,:))
plot(idx_users(idx_acc == 2),users_err_median(idx_acc == 2), 'o', 'Color', color_list(3,:))
yline(C_acc(2), '--', 'Color', color_list(3,:))
plot(idx_users(idx_acc == 3),users_err_median(idx_acc == 3), 'o', 'Color', color_list(4,:))
yline(C_acc(3), '--', 'Color', color_list(4,:))
xticks(idx_users)
xticklabels(convertStringsToChars(unique(users)));
xtickangle(60)
title("User's precision scores, with 3 kmeans for groups")

for i = 1:3
    err_repeat{i} = cell2mat(b_err([idx_rep == i]));
    err_precis{i} = cell2mat(b_err([idx_acc == i]));
end
for i = 1:3
    disp("Repeatability group: " + string(C_rep(i)) + ", has " + string(sum(idx_rep == i)) +...
        " participant(s)");
end
for i = 1:3
    disp("Precision group: " + string(C_acc(i)) + ", has " + string(sum(idx_acc == i)) +...
        " participant(s)");
end
%%%%%%
% Expertise by repeatability: are ball bouncing errors significantly different
figure
boxplot([err_repeat{1}, err_repeat{2}, err_repeat{3}], ...
    [C_rep(1)*ones(size(err_repeat{1})), C_rep(2)*ones(size(err_repeat{2})), ...
    C_rep(3)*ones(size(err_repeat{3}))],'Notch','on');
title("Are the bouncing error significantly different according to the 3 repeatability groups?") 
[~,~,stats] = anova1([err_repeat{1}, err_repeat{2}, err_repeat{3}], ...
    [C_rep(1)*ones(size(err_repeat{1})), C_rep(2)*ones(size(err_repeat{2})), ...
    C_rep(3)*ones(size(err_repeat{3}))]);
multcompare(stats,'CType','bonferroni');
% absolute errors
figure
boxplot(abs(cell2mat(err_repeat)), ...
    [C_rep(1)*ones(size(err_repeat{1})), C_rep(2)*ones(size(err_repeat{2})), ...
    C_rep(3)*ones(size(err_repeat{3}))],'Notch','on');
title("Are the absolute bouncing error significantly different according to the 3 repeatability groups?") 
[~,~,stats] = anova1(abs(cell2mat(err_repeat)), ...
    [C_rep(1)*ones(size(err_repeat{1})), C_rep(2)*ones(size(err_repeat{2})), ...
    C_rep(3)*ones(size(err_repeat{3}))]);
multcompare(stats,'CType','bonferroni');
% Expertise by precision: are ball bouncing errors significantly different
figure
boxplot([err_precis{1}, err_precis{2}, err_precis{3}], ...
    [C_acc(1)*ones(size(err_precis{1})), C_acc(2)*ones(size(err_precis{2})), ...
    C_acc(3)*ones(size(err_precis{3}))],'Notch','on');
title("Are the bouncing error significantly different according to the 3 precision groups?") 
[~,~,stats] = anova1([err_precis{1}, err_precis{2}, err_precis{3}], ...
    [C_acc(1)*ones(size(err_precis{1})), C_acc(2)*ones(size(err_precis{2})), ...
    C_acc(3)*ones(size(err_precis{3}))]);
multcompare(stats,'CType','bonferroni');

% order = out2(@() sort(C_rep));
% idx_acc_ord = order(1)*(idx_rep == 1) + order(2)*(idx_rep == 2) ...
%      + order(3)*(idx_rep == 3);

%%%%%%
% Influence of the experiment's order ...
%%%%%%
b_err_per_exp = cell(1,5);
b_err_per_exp_post_dist = cell(1,5);
b_err_per_exp_npost_dist = cell(1,5);
for user_i = 1:length(cycles) 
    tmp_err = [cycles{user_i}.target_error];
    for exp_it_nb = nb_exp:-1:1
        b_err_per_exp{exp_it_nb} = [b_err_per_exp{exp_it_nb}, ...
            tmp_err([cycles{user_i}.exp_it_cmb] == exp_it_nb)];
        b_err_per_exp_post_dist{exp_it_nb} = [b_err_per_exp_post_dist{exp_it_nb}, ...
            tmp_err(([cycles{user_i}.exp_it_cmb] == exp_it_nb) & ([cycles{user_i}.is_post_dist]))];
        b_err_per_exp_npost_dist{exp_it_nb} = [b_err_per_exp_npost_dist{exp_it_nb}, ...
            tmp_err(([cycles{user_i}.exp_it_cmb] == exp_it_nb) & (~[cycles{user_i}.is_post_dist]))];
    end
end
clear tmp_err
figure
boxplot(cell2mat(b_err_per_exp), ...
    [ones(size(b_err_per_exp{1})), 2*ones(size(b_err_per_exp{2})), ...
    3*ones(size(b_err_per_exp{3})), 4*ones(size(b_err_per_exp{4})), ...
    5*ones(size(b_err_per_exp{5}))],'Notch','on');
title("Are the bouncing error significantly different according to the experiment order?") 
[~,~,stats] = anova1(cell2mat(b_err_per_exp), [ones(size(b_err_per_exp{1})), ...
    2*ones(size(b_err_per_exp{2})), 3*ones(size(b_err_per_exp{3})), ...
    4*ones(size(b_err_per_exp{4})), 5*ones(size(b_err_per_exp{5}))]);
multcompare(stats,'CType','bonferroni');
% absolute errors
figure
boxplot(abs(cell2mat(b_err_per_exp)), ...
    [ones(size(b_err_per_exp{1})), 2*ones(size(b_err_per_exp{2})), ...
    3*ones(size(b_err_per_exp{3})), 4*ones(size(b_err_per_exp{4})), ...
    5*ones(size(b_err_per_exp{5}))],'Notch','on');
title("Are the absolute bouncing error significantly different according to the experiment order?") 
[~,~,stats] = anova1(abs(cell2mat(b_err_per_exp)), [ones(size(b_err_per_exp{1})), ...
    2*ones(size(b_err_per_exp{2})), 3*ones(size(b_err_per_exp{3})), ...
    4*ones(size(b_err_per_exp{4})), 5*ones(size(b_err_per_exp{5}))]);
multcompare(stats,'CType','bonferroni');
%%%%%%%
% perturbation influence per experiments
%%%%%%%
figure
boxplot(abs([cell2mat(b_err_per_exp_post_dist), cell2mat(b_err_per_exp_npost_dist)]), ...
    [ones(size(b_err_per_exp_post_dist{1})), 2*ones(size(b_err_per_exp_post_dist{2})), ...
    3*ones(size(b_err_per_exp_post_dist{3})), 4*ones(size(b_err_per_exp_post_dist{4})), ...
    5*ones(size(b_err_per_exp_post_dist{5})), repmat("a",1,length(b_err_per_exp_npost_dist{1})),...
    repmat("b",1,length(b_err_per_exp_npost_dist{2})),repmat("c",1,length(b_err_per_exp_npost_dist{3})),...
    repmat("d",1,length(b_err_per_exp_npost_dist{4})),repmat("e",1,length(b_err_per_exp_npost_dist{5}))], 'Notch','on');
title("Are the bouncing error significantly different according to the experiment order?") 
[~,~,stats] = anova1([cell2mat(b_err_per_exp_post_dist), cell2mat(b_err_per_exp_npost_dist)], ...
    [ones(size(b_err_per_exp_post_dist{1})), 2*ones(size(b_err_per_exp_post_dist{2})), ...
    3*ones(size(b_err_per_exp_post_dist{3})), 4*ones(size(b_err_per_exp_post_dist{4})), ...
    5*ones(size(b_err_per_exp_post_dist{5})), repmat("a",1,length(b_err_per_exp_npost_dist{1})),...
    repmat("b",1,length(b_err_per_exp_npost_dist{2})),repmat("c",1,length(b_err_per_exp_npost_dist{3})),...
    repmat("d",1,length(b_err_per_exp_npost_dist{4})),repmat("e",1,length(b_err_per_exp_npost_dist{5}))]);
multcompare(stats,'CType','bonferroni');

%%%%%%%
% expertise influence with experiment order
%%%%%%%
exp_order = [];
usr_expertise = [];
target_h = [];
expert_list = ["adv.", "inter.", "novi."];
%idx_th_all;
for user_i = 1:length(cycles) 
    exp_order_tmp = [cycles{user_i}.exp_it_cmb];
    exp_order = [exp_order, exp_order_tmp];
    name_exp = expert_list(idx_rep_sorted(idx_rep(user_i)));
    exp_expertise_tmp = repmat(name_exp, 1, length(exp_order_tmp));
    usr_expertise = [usr_expertise, exp_expertise_tmp];
    target_height_tmp = [cycles{user_i}.target_height];
    target_h = [target_h, target_height_tmp];
end

figure
[res_n,~,stats] = anovan(abs(b_err_all.*1e2),{exp_order target_h usr_expertise},'model','interaction','varnames',{'n', 'th','exp'});
res = multcompare(stats,'Dimension',[2 3]);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Same work with means per conditions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% split user according to the experiment they started with
try
    th_it = cellfun(@unique, th_it);
catch
    error('Experiments itération wrongly splitted');
end
first_h2 = th_it(:,1) == 1.75; % exp 1
first_h1 = th_it(:,1) == 1.5; % exp 2 h1
first_h3 = th_it(:,1) == 2; % exp 2 h1
third_h1 = th_it(:,3) == 1.5; % exp 2 is second, and h1 is first of 2nd session
third_h3 = th_it(:,3) == 2;
%%%%%%
% EXP 1
%%%%%%
for user_i = 1:size(b_err_it, 1) % b_err_it{user_i, ni}
    if first_h2(user_i) == 0
        vec = [3,4,5]; % started with exp 2
    else
        vec = [1,2,5]; % started with exp 1
    end
    % time dependency
    k = 0;
    for ni = vec 
        k = k + 1;
        t0 = time_it{user_i, ni};
        t0 = t0(1);
        times = t0 + (60:60:240);
        idx_timing = [];
        idx_timing(1) = 1;
        missing_time = 0;
        for t_i = times
            try
                idx_timing(end+1) = find([time_it{user_i, ni}] > t_i, 1, 'first');
            catch
                warning("User nb " + string(user_i) + " had an experiment under 5 min")
                missing_time = missing_time + 1;
            end
        end
        idx_timing(end+1) = length([time_it{user_i, ni}]);
        b_err_tmp = b_err_it{user_i, ni};
        tmp_mean = [];
        tmp_std = [];
        for i = 1:length(idx_timing) - 1
            tmp_mean = [tmp_mean, mean(b_err_tmp(idx_timing(i):idx_timing(i+1)))];
            tmp_std = [tmp_std, std(b_err_tmp(idx_timing(i):idx_timing(i+1)))];
        end
        if missing_time ~= 0
            tmp_mean(end+1:end+missing_time) = NaN;
            tmp_std(end+1:end+missing_time) = NaN;
        end            
        b_err_mean_learning{user_i, ni} = tmp_mean;
        b_err_std_learning{user_i, ni} = tmp_std;
        b_err_mean_trials{user_i, ni} = mean(b_err_tmp);
        b_err_std_trials{user_i, ni} = std(b_err_tmp);
    end
end
k = 1;
for i = [1,2,5]
    exp1_first_be_mean{k} = vertcat(b_err_mean_learning{first_h2, i}); % trials are splited into minutes
    exp1_first_be_std{k} = vertcat(b_err_std_learning{first_h2, i});
    exp1_first_trials_be_mean(:,k) = vertcat(b_err_mean_trials{first_h2, i}); % trials are whole
    exp1_first_trials_be_std(:,k) = vertcat(b_err_std_trials{first_h2, i});
    k = k+1;
end
k = 1;
for i = [3,4,5]
    exp1_last_be_mean{k} = vertcat(b_err_mean_learning{~first_h2, i});
    exp1_last_be_std{k} = vertcat(b_err_std_learning{~first_h2, i});
    exp1_last_trials_be_mean(:,k) = vertcat(b_err_mean_trials{~first_h2, i});
    exp1_last_trials_be_std(:,k) = vertcat(b_err_std_trials{~first_h2, i});
    k = k+1;
end
% 
exp1_first_be_mean{1}(7,4) = nanmean(exp1_first_be_mean{1}(:,4)); % missing data
exp1_first_be_mean{1}(7,5) = nanmean(exp1_first_be_mean{1}(:,5)); % missing data
exp1_first_be_std{1}(7,4) = std(exp1_first_be_mean{1}(:,4), 'omitnan'); % missing data
exp1_first_be_std{1}(7,5) = std(exp1_first_be_mean{1}(:,5), 'omitnan'); % missing data
%
users = cellfun(@(x) unique([x.user]), cycles);
users_exp1_first = users(first_h2)';
users_exp1_last = users(~first_h2)';
%
% tab_1 = table(users_exp1_first, exp1_first_be_mean{1}, exp1_first_be_mean{2}, exp1_first_be_mean{3},'VariableNames',["users", "trial_1", "trial_2", "trial_5"]); 
% write(tab_1,folder+"exp1_first_mean_be.csv",'Delimiter',',');
% tab_1 = table(users_exp1_first, exp1_first_be_std{1}, exp1_first_be_std{2}, exp1_first_be_std{3},'VariableNames',["users", "trial_1", "trial_2", "trial_5"]); 
% write(tab_1,folder+"exp1_first_std_be.csv",'Delimiter',',');
% tab_2 = table(users_exp1_last, exp1_last_be_mean{1}, exp1_last_be_mean{2}, exp1_last_be_mean{3},'VariableNames',["users", "trial_3", "trial_4", "trial_5"]); 
% write(tab_2,folder+"exp1_last_mean_be.csv",'Delimiter',',');
% tab_2 = table(users_exp1_last, exp1_last_be_std{1}, exp1_last_be_std{2}, exp1_last_be_std{3},'VariableNames',["users", "trial_3", "trial_4", "trial_5"]); 
% write(tab_2,folder+"exp1_last_std_be.csv",'Delimiter',',');
% tab_1_trials = table(users_exp1_first, exp1_first_trials_be_mean(:,1), exp1_first_trials_be_mean(:,2), exp1_first_trials_be_mean(:,3),'VariableNames',["users", "trial_1", "trial_2", "trial_5"]);
% write(tab_1_trials,folder+"exp1_first_trials_mean_be.csv",'Delimiter',',');
% tab_1_trials = table(users_exp1_first, exp1_first_trials_be_std(:,1), exp1_first_trials_be_std(:,2), exp1_first_trials_be_std(:,3),'VariableNames',["users", "trial_1", "trial_2", "trial_5"]);
% write(tab_1_trials,folder+"exp1_first_trials_std_be.csv",'Delimiter',',');
% tab_2_trials = table(users_exp1_last, exp1_last_trials_be_mean(:,1), exp1_last_trials_be_mean(:,2), exp1_last_trials_be_mean(:,3),'VariableNames',["users", "trial_3", "trial_4", "trial_5"]);
% write(tab_2_trials,folder+"exp1_last_trials_mean_be.csv",'Delimiter',',');
% tab_2_trials = table(users_exp1_last, exp1_last_trials_be_std(:,1), exp1_last_trials_be_std(:,2), exp1_last_trials_be_std(:,3),'VariableNames',["users", "trial_3", "trial_4", "trial_5"]);
% write(tab_2_trials,folder+"exp1_last_trials_std_be.csv",'Delimiter',',');

%%%%%%
% EXP 2
%%%%%%

%real_th_mean = cellfun(@mean, real_th_it);
real_th_it_h2 = reshape(real_th_it(th_it == 1.75), [31,3]);
real_th_it_h1 = reshape(real_th_it(th_it == 1.5), [31,1]);
real_th_it_h3 = reshape(real_th_it(th_it == 2), [31,1]);

for user_i = 1:size(real_th_mean, 1)
    real_th_it_h2_reg{user_i,1} = [real_th_it_h2{user_i, :}];
end
real_th1_mean = cellfun(@mean, real_th_it_h1);
real_th2_mean = cellfun(@mean, real_th_it_h2_reg);
real_th3_mean = cellfun(@mean, real_th_it_h3);
real_th_mean = [real_th1_mean, real_th2_mean, real_th3_mean];
% tab = table(users', real_th1_mean, real_th2_mean, real_th3_mean);
% write(tab,folder+"real_target_height.csv",'Delimiter',',');

%bouncing errors
be_it_h2 = reshape(b_err_it(th_it == 1.75), [31,3]);
be_it_h2 = be_it_h2(:,1); % only first column is kept
be_it_h1 = reshape(b_err_it(th_it == 1.5), [31,1]);
be_it_h3 = reshape(b_err_it(th_it == 2), [31,1]);
be_th1_mean = cellfun(@mean, be_it_h1);
be_th2_mean = cellfun(@mean, be_it_h2);
be_th3_mean = cellfun(@mean, be_it_h3);
be_th1_std = cellfun(@std, be_it_h1);
be_th2_std = cellfun(@std, be_it_h2);
be_th3_std = cellfun(@std, be_it_h3);
exp_1_first = th_it(:,1) == 1.75;

tab = table(users', exp_1_first, be_th1_mean, be_th2_mean, be_th3_mean, be_th1_std, be_th2_std, be_th3_std);
write(tab,folder+"be_target_height_mean_std.csv",'Delimiter',',');

std([be_th1_mean; be_th2_mean; be_th3_mean])

% % for boxplot
% real_th_mean_s = sort(real_th_mean);
% h = [1.5;1.75;2];
% n = length(real_th_mean);
% q = quantile(real_th_mean,3);
% iqe = q(3,:) - q(1,:);
% avg = mean(real_th_mean)';
% med = q(2,:)';
% lq = q(1,:)';
% uq = q(1,:)';
% un = (q(2,:) + 1.57*(q(3,:)-q(1,:))/sqrt(n))';
% ln = (q(2,:) - 1.57*(q(3,:)-q(1,:))/sqrt(n))';
% for i = 1:3
%     uw(i,1) = real_th_mean_s(find(q(3,i) + 1.5*(q(3,i)-q(1,i)) >= real_th_mean_s(:,i), 1, 'last'),i);
%     lw(i,1) = real_th_mean_s(find(q(3,i) - 1.5*(q(3,i)-q(1,i)) <= real_th_mean_s(:,i), 1, 'first'),i);
% end
% tab = table(avg, med, un, ln, uq, lq, uw, lw, h);
% write(tab,folder+"real_target_height_boxplot.csv",'Delimiter',',');

% % starting second experiment with h1
% % exp 2 first
% h1_first_be_mean = cellfun(@mean, b_err_it(first_h1, 1));
% h3_sec_be_mean = cellfun(@mean, b_err_it(first_h1, 2));
% nb_h1_first_exp2_first = size(h1_first_be_mean, 1);
% % exp 2 second
% tmp = cellfun(@mean, b_err_it(third_h1, 3));
% tmp2 = cellfun(@mean, b_err_it(third_h1, 4));
% nb_h1_first_exp2_second = size(tmp, 1);
% h1_first_be_mean = [h1_first_be_mean; tmp];
% h3_sec_be_mean = [h3_sec_be_mean; tmp2]; 
% % starting second experiment with h1
% % exp 2 first
% h1_sec_be_mean = cellfun(@mean, b_err_it(first_h3, 2));
% h3_first_be_mean = cellfun(@mean, b_err_it(first_h3, 1));
% nb_h3_first_exp2_first = size(h1_sec_be_mean, 1);
% % exp 2 second
% tmp = cellfun(@mean, b_err_it(third_h3, 3));
% tmp2 = cellfun(@mean, b_err_it(third_h3, 4));
% nb_h3_first_exp2_second = size(tmp, 1);
% h1_sec_be_mean = [h1_sec_be_mean; tmp2];
% h3_first_be_mean = [h3_first_be_mean; tmp]; 
% target height mixed...
all_mean_be_err = cellfun(@mean, b_err_it);
all_std_be_err = cellfun(@std, b_err_it);
% tab_all = table(users', all_mean_be_err);
% write(tab_all,folder+"all_mean_be.csv",'Delimiter',',');
% tab_all = table(users', all_std_be_err);
% write(tab_all,folder+"all_std_be.csv",'Delimiter',',');

%%%%%%
% Perturbations
%%%%%%

post_dist_mean_be_err = cellfun(@mean, b_err_post_dist)';
post_dist_std_be_err = cellfun(@std, b_err_post_dist)';
npost_dist_mean_be_err = cellfun(@mean, b_err_npost_dist)';
npost_dist_std_be_err = cellfun(@std, b_err_npost_dist)';
tab_pert = table(users', post_dist_mean_be_err, npost_dist_mean_be_err, post_dist_std_be_err, npost_dist_std_be_err);
%write(tab_pert,folder+"perturbations_be.csv",'Delimiter',',');

for user_i = 1:length(cycles)    
    b_err_post_dist{user_i} = [cycles{user_i}([cycles{user_i}.is_post_dist]).target_error];
    b_err_pre_dist{user_i} = [cycles{user_i}().target_error];
    is_post_post_dist = [0,0];
    for i = 3:length(cycles{user_i})
        if cycles{user_i}(i-1).is_post_dist == 1
            is_post_post_dist(1,i) = 1;
        else
            is_post_post_dist(1,i) = 0;
        end
    end
    is_post_post_post_dist = [0,0,0];
    for i = 4:length(cycles{user_i})
        if is_post_post_dist(i-1) == 1
            is_post_post_post_dist(1,i) = 1;
        else
            is_post_post_post_dist(1,i) = 0;
        end
    end
    b_err_post_post_dist{user_i} = [cycles{user_i}(logical(is_post_post_dist)).target_error];
    b_err_post_post_post_dist{user_i} = [cycles{user_i}(logical(is_post_post_post_dist)).target_error];
    clear is_post_post_dist
end
% mean
pre_dist_mean_be_err = cellfun(@mean, b_err_pre_dist)';
post_dist_mean_be_err = cellfun(@mean, b_err_post_dist)';
post2_dist_mean_be_err = cellfun(@mean, b_err_post_post_dist)';
post3_dist_mean_be_err = cellfun(@mean, b_err_post_post_post_dist)';
% std
pre_dist_std_be_err = cellfun(@std, b_err_pre_dist)';
post_dist_std_be_err = cellfun(@std, b_err_post_dist)';
post2_dist_std_be_err = cellfun(@std, b_err_post_post_dist)';
post3_dist_std_be_err = cellfun(@std, b_err_post_post_post_dist)';

tab = table(users', pre_dist_mean_be_err, post_dist_mean_be_err, post2_dist_mean_be_err, post3_dist_mean_be_err, ...
    pre_dist_std_be_err, post_dist_std_be_err, post2_dist_std_be_err, post3_dist_std_be_err);
% write(tab,folder+"perturbations_all_be.csv",'Delimiter',',');

% perturbations according to experiment
be_pre_it_exp1 = reshape(be_pre_it(th_it == 1.75), [31,3]);
be_pre_it_exp2 = [be_pre_it(th_it == 1.5), be_pre_it(th_it == 2)];
be_post_it_exp1 = reshape(be_post_it(th_it == 1.75), [31,3]);
be_post_it_exp2 = [be_post_it(th_it == 1.5), be_post_it(th_it == 2)];
be_post2_it_exp1 = reshape(be_post2_it(th_it == 1.75), [31,3]);
be_post2_it_exp2 = [be_post2_it(th_it == 1.5), be_post2_it(th_it == 2)];
be_post3_it_exp1 = reshape(be_post3_it(th_it == 1.75), [31,3]);
be_post3_it_exp2 = [be_post3_it(th_it == 1.5), be_post3_it(th_it == 2)];
% merging data for the same experiment (2 first trials for exp 1)
for i = 1:size(be_pre_it_exp1, 1)
    be_pre_it_exp1{i,1} = [be_pre_it_exp1{i,1}, be_pre_it_exp1{i,2}];
    be_pre_it_exp2{i,1} = [be_pre_it_exp2{i,1}, be_pre_it_exp2{i,2}];
    be_post_it_exp1{i,1} = [be_post_it_exp1{i,1}, be_post_it_exp1{i,2}];
    be_post_it_exp2{i,1} = [be_post_it_exp2{i,1}, be_post_it_exp2{i,2}];
    be_post2_it_exp1{i,1} = [be_post2_it_exp1{i,1}, be_post2_it_exp1{i,2}];
    be_post2_it_exp2{i,1} = [be_post2_it_exp2{i,1}, be_post2_it_exp2{i,2}];
    be_post3_it_exp1{i,1} = [be_post3_it_exp1{i,1}, be_post3_it_exp1{i,2}];
    be_post3_it_exp2{i,1} = [be_post3_it_exp2{i,1}, be_post3_it_exp2{i,2}];
end
be_dist_it_exp1_mean = [cellfun(@mean, be_pre_it_exp1(:,1)), ...
                        cellfun(@mean, be_post_it_exp1(:,1)), ...
                        cellfun(@mean, be_post2_it_exp1(:,1)), ...
                        cellfun(@mean, be_post3_it_exp1(:,1))];
be_dist_it_exp1_std = [cellfun(@std, be_pre_it_exp1(:,1)), ...
                        cellfun(@std, be_post_it_exp1(:,1)), ...
                        cellfun(@std, be_post2_it_exp1(:,1)), ...
                        cellfun(@std, be_post3_it_exp1(:,1))];
be_dist_it_exp2_mean = [cellfun(@mean, be_pre_it_exp2(:,1)), ...
                       cellfun(@mean, be_post_it_exp2(:,1)), ...
                       cellfun(@mean, be_post2_it_exp2(:,1)), ...
                       cellfun(@mean, be_post3_it_exp2(:,1))];
be_dist_it_exp2_std = [cellfun(@std, be_pre_it_exp2(:,1)), ...
                       cellfun(@std, be_post_it_exp2(:,1)), ...
                       cellfun(@std, be_post2_it_exp2(:,1)), ...
                       cellfun(@std, be_post3_it_exp2(:,1))];  
%
be_dist_it_all_mean = [be_dist_it_exp1_mean; be_dist_it_exp2_mean];
be_dist_it_all_std = [be_dist_it_exp1_std; be_dist_it_exp2_std];
u = [users';users'];
exp = [repmat("exp_1", 31, 1); repmat("exp_2", 31, 1)];

tab = table(u, exp, be_dist_it_all_mean, be_dist_it_all_std);
write(tab,folder+"perturbations_all_be_per_exp.csv",'Delimiter',',');                  
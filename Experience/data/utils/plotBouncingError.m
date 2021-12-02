function plotBouncingError(cycles, nb_exp)
%
colors = hsv(length(cycles));
users = [];

for user_i = 1:length(cycles)    
    nb_exp_for_user_i = max([cycles{user_i}.exp_it]);    
    % bouncing error
    b_err{user_i} = [cycles{user_i}.target_error];
    % post perturbation ball bouncing error
    b_err_post_dist{user_i} = [cycles{user_i}(circshift([cycles{user_i}(1:end-1).is_dist],1)).target_error];
    b_err_npost_dist{user_i} = [cycles{user_i}(~circshift([cycles{user_i}(1:end-1).is_dist],1)).target_error];
    % (1:end-1) in case last data was perturbed
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
end

%%%%%%%%
% Are there significant mean differences between the 3 target heights in
% the bouncing error ?
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

%%% Are there significant bouncing error differences between post perturbed
% cycles and others
[~,~,stats] = anova1([cell2mat(b_err_post_dist), cell2mat(b_err_npost_dist)], ...
    [repmat("post dist.", 1, length(cell2mat(b_err_post_dist))), ...
     repmat("others", 1, length(cell2mat(b_err_npost_dist)))]);
title("Are the bouncing error significantly different when the previous cycle was perturbed?") 
% absolute errors
[~,~,stats] = anova1(abs([cell2mat(b_err_post_dist), cell2mat(b_err_npost_dist)]), ...
    [repmat("post dist.", 1, length(cell2mat(b_err_post_dist))), ...
     repmat("others", 1, length(cell2mat(b_err_npost_dist)))]);
title("Are the bouncing error significantly different when the previous cycle was perturbed?") 

%%% Duration VS height (correlation expected)
% figure
% plot(cell2mat(duration),cell2mat(b_err), '*')

%%%%%
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

% Users' precision
[res,~,stats] = anova1(abs(cell2mat(b_err)), users);%, 'off');
multcompare(stats,'CType','bonferroni'); 
figure
boxplot(abs(cell2mat(b_err)), users, 'PlotStyle','compact');
title("User's ball bouncing errors")

% significant differences between users accuracy
if res < 0.05
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
end

color_list = lines(4);
idx_users = 1:length(cycles);
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
title("User's repeatability scores, with 3 kmeans for groups")

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
b_err_per_exp = cell(1,5);
for user_i = 1:length(cycles) 
    tmp_err = [cycles{user_i}.target_error];
    for exp_it_nb = nb_exp:-1:1
        b_err_per_exp{exp_it_nb} = [b_err_per_exp{exp_it_nb}, ...
            tmp_err([cycles{user_i}.exp_it_cmb] == exp_it_nb)];
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


% exp_duration_all = {};
% for user_i = 1:length(cycles)  
%     cycles_i = cycles{user_i};
%     nb_exp_for_user_i = max([cycles{user_i}.exp_it]); 
%     % bouncing error per experiment
%     idx_exp = find(diff([cycles{user_i}.exp_it]));
%     idx_exp = [0,idx_exp];
%     idx_exp_fused = [];
%     idx_fuse = [];
%     exp_fuse_id = 1;
%     exp_duration = [];
%     fuse_duration = [];
%     incomplete_exp = 0;
%     for exp_it = 1:length(idx_exp)-1
%         exp_duration(exp_it) = cycles_i(idx_exp(exp_it+1)).t(end)-cycles_i(idx_exp(exp_it)+1).t(1);
%         if exp_duration(exp_it) < 290
%             % cycle is less than 4min50sec then it was completed with
%             % another one            
%             idx_fuse = [idx_fuse, exp_it];
%             fuse_duration = [fuse_duration, exp_duration(exp_it)];
%             
%             if incomplete_exp
%                 if sum(fuse_duration) > (290 + 30*(length(fuse_duration) -1))
%                     idx_exp_fused = [idx_exp_fused, exp_fuse_id*ones(1, length(fuse_duration))];
%                     fuse_duration = [];
%                     incomplete_exp = 0;
%                     exp_fuse_id = exp_fuse_id + 1;
%                 end
%             else
%                 incomplete_exp = 1;
%             end
%         end        
%     end
%     %last duration
%     exp_duration(exp_it+1) = cycles_i(end).t(end)-cycles_i(idx_exp(exp_it)+1).t(1);
%     % last experience was not complete
%     if incomplete_exp
%         idx_exp = [idx_exp, exp_fuse_id*ones(1, length(fuse_duration))];
%     end
%     
%     exp_duration_all{user_i} = exp_duration;
%     
% %     if ~isempty(idx_fuse)
% %         for i = 1:exp_fuse_id-1
% %             idx_fuse(idx_exp == i)
% %         end
% %     end
%     
%     %b_err{user_i} = [cycles{user_i}.target_error];
%     
% end % for user_i = 1:length(cycles)  
    
end

%%%

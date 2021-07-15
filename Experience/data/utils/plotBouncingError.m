function plotBouncingError(cycles)
%
colors = hsv(length(cycles));
users = [];

for user_i = 1:length(cycles)
    
    nb_exp_for_user_i = max([cycles{user_i}.exp_it]);    
    for exp_i = 1:nb_exp_for_user_i
        
    end
    % bouncing error
    b_err{user_i} = [cycles{user_i}.target_error];
    idx_new_exp{user_i} = find(diff([cycles{user_i}.exp_it]));
    quartiles(:, user_i)= quantile(b_err{user_i},[0.25,0.5,0.75]);
    users = [users, [cycles{user_i}.user]];
end

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
    ylim([-0.2, 0.2])
    title("User #" + cycles{user_i}(1).user)
end

% Users' precision
res = anova1(cell2mat(b_err), users);
% significant differences between users accuracy
if res < 0.05
    repeat_score = quartiles(3, :) - quartiles(1, :); % repeatability
    [idx_acc, C_acc] = kmeans(repeat_score', 3); 
end
    order = out2(@() sort(C_acc));
    idx_acc_ord = order(1)*(idx_acc == 1) + order(2)*(idx_acc == 2) ...
         + order(3)*(idx_acc == 3);
    
end


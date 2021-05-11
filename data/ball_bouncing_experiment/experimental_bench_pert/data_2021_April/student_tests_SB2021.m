% student test for submission to Société de Biomécanique 2021

r1 = mean([cycles_class{1}([cycles_class{1}.type_dist] == 1).ratio_dist]);
r2 = mean([cycles_class{1}([cycles_class{1}.type_dist] == 2).ratio_dist]);
r3 = mean([cycles_class{1}([cycles_class{1}.type_dist] == 3).ratio_dist]);
[~,idxs] = sort([r1,r2,r3]);
[~,idxs_s] = sort(idxs);

% infra user student tests
for i = 1:3
    for j = 1:3
        user_c{i,j} = data_sorted.stiffness{i, idxs_s(j)}; % c
    end
end

cellfun(@median, user_c)

for i = 1:3
    [hyp_c(i,1),prob_c(i,1),ci,stats] = ttest2(user_c{i,1},user_c{i,2});
    [hyp_c(i,2),prob_c(i,2),ci,stats] = ttest2(user_c{i,1},user_c{i,3});
    [hyp_c(i,3),prob_c(i,3),ci,stats] = ttest2(user_c{i,2},user_c{i,3});
end

for i = 1:3
    [hyp_u(i,1),prob_u(i,1),ci,stats] = ttest2(user_c{1,i},user_c{2,i});
    [hyp_u(i,2),prob_u(i,2),ci,stats] = ttest2(user_c{1,i},user_c{3,i});
    [hyp_u(i,3),prob_u(i,3),ci,stats] = ttest2(user_c{2,i},user_c{3,i});
end


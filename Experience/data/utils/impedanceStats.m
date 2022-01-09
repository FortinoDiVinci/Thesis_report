function impedanceStats(cycles, exp_list)
cycles_dist = cell(1,length(cycles));

for user_i = 1:length(cycles)        
    cycles_dist{user_i} = cycles{user_i}([cycles{user_i}.is_dist]);
end
clear cycles

[exp_list_sorted,idx_exp_s] = sort(exp_list);
path = "impedance_results_2/";

%%%%%%%%
% Sort identification
%%%%%%%%
users_vect = [];
e_vect = [];
h_vect = [];
ph_vect = [];
ph_vect_ratio = [];
n_vect = [];
K_vect = [];
B_vect = [];
M_vect = [];
R2_vect = [];
out_vect = [];
for user_i = 1:length(cycles_dist) 
    users_vect = [users_vect, cycles_dist{user_i}.user];
    e_vect = [e_vect, repmat(exp_list(user_i),1,length(cycles_dist{user_i}))];
    h_vect = [h_vect, [cycles_dist{user_i}.target_height]];
    ph_vect = [ph_vect, [cycles_dist{user_i}.type_dist]];
    ph_vect_ratio = [ph_vect_ratio, [cycles_dist{user_i}.ratio_dist]];
    n_vect = [n_vect, [cycles_dist{user_i}.exp_it_cmb]];
    K_vect = [K_vect, [cycles_dist{user_i}.K]];
    B_vect = [B_vect, [cycles_dist{user_i}.B]];
    M_vect = [M_vect, [cycles_dist{user_i}.M]];
    R2_vect = [R2_vect, [cycles_dist{user_i}.R2]];
    %[~,idx_out] = rmoutliers([cycles_dist{user_i}.K], 'ThresholdFactor', 5);
    tmp_outl = [cycles_dist{user_i}.is_outl];
    out_vect = [out_vect, tmp_outl];
    disp("User#"+ cycles_dist{user_i}(1).user + " has " + string(1e2*sum(tmp_outl)/length(tmp_outl)) + "% outliers")
end

ph1 = mean(ph_vect_ratio(ph_vect == 1));
ph2 = mean(ph_vect_ratio(ph_vect == 2));
ph3 = mean(ph_vect_ratio(ph_vect == 3));

[~,srt_ph] = sort([ph1,ph2,ph3]);
th_list = [1.5,1.75,2.0];

idx_ph(1,:) = ph_vect == srt_ph(1); % 25%
idx_ph(2,:) = ph_vect == srt_ph(2); % 50%
idx_ph(3,:) = ph_vect == srt_ph(3); % 75%

idx_h(1,:) = h_vect == 1.5;
idx_h(2,:) = h_vect == 1.75;
idx_h(3,:) = h_vect == 2;

idx_e(1,:) = e_vect == 1; % novice
idx_e(2,:) = e_vect == 2; % intermediate
idx_e(3,:) = e_vect == 3; % advanced

for i = 1:5
    idx_n(i,:) = n_vect == i;
end

list_users = unique(users_vect);
list_users = list_users(idx_exp_s);
for ii= 1:length(cycles_dist)
    idx_u(ii,:) = strcmp(users_vect, list_users(ii)); % sorted by expertise
end
idx_r2 = (R2_vect > 0.5);
impedance_vect = [K_vect; B_vect; M_vect; R2_vect];
impedance_vect_R2 = [K_vect(idx_r2); B_vect(idx_r2); M_vect(idx_r2); R2_vect(idx_r2)];
param_names = ["K", "B", "M", "R2"];

for us = 1:length(list_users) % users
    users_data(us).name = list_users(us);
    %% COND1 - height variation
    for t_h = 1:3 % target height
        idx = idx_ph(1,:) & idx_h(t_h,:) & idx_u(us,:) & idx_r2;
        n = sum(idx);
        users_data(us).n_h_ph1(t_h) = n;
        % K
        q = quantile(impedance_vect(1, idx),3);
        iqe = q(3) - q(1);
        users_data(us).K_h_ph1(t_h) = q(2);
        users_data(us).K_h_ph1_notch(t_h) = 1.57*iqe/sqrt(n);
        % B
        q = quantile(impedance_vect(2, idx),3);
        iqe = q(3) - q(1);
        users_data(us).B_h_ph1(t_h) = q(2);
        users_data(us).B_h_ph1_notch(t_h) = 1.57*iqe/sqrt(n);
        % M
        q = quantile(impedance_vect(3, idx),3);
        iqe = q(3) - q(1);
        users_data(us).M_h_ph1(t_h) = q(2);
        users_data(us).M_h_ph1_notch(t_h) = 1.57*iqe/sqrt(n);
        % R^2
        q = quantile(impedance_vect(4, idx),3);
        iqe = q(3) - q(1);
        users_data(us).R2_h_ph1(t_h) = q(2);
        users_data(us).R2_h_ph1_notch(t_h) = 1.57*iqe/sqrt(n);
    end
    %% COND 1 - temporal variation (exp order)
    for exp_n = 1:5 % target height
        idx = idx_ph(1,:) & idx_n(exp_n,:) & idx_u(us,:) & idx_r2;
        n = sum(idx);
        users_data(us).n_exp_ph1(exp_n) = n;
        % K
        q = quantile(impedance_vect(1, idx),3);
        iqe = q(3) - q(1);
        users_data(us).K_exp_ph1(exp_n) = q(2);
        users_data(us).K_exp_ph1_notch(exp_n) = 1.57*iqe/sqrt(n);
        % B
        q = quantile(impedance_vect(2, idx),3);
        iqe = q(3) - q(1);
        users_data(us).B_exp_ph1(exp_n) = q(2);
        users_data(us).B_exp_ph1_notch(exp_n) = 1.57*iqe/sqrt(n);
        % M
        q = quantile(impedance_vect(3, idx),3);
        iqe = q(3) - q(1);
        users_data(us).M_exp_ph1(exp_n) = q(2);
        users_data(us).M_exp_ph1_notch(exp_n) = 1.57*iqe/sqrt(n);
        % R^2
        q = quantile(impedance_vect(4, idx),3);
        iqe = q(3) - q(1);
        users_data(us).R2_exp_ph1(exp_n) = q(2);
        users_data(us).R2_exp_ph1_notch(exp_n) = 1.57*iqe/sqrt(n);
    end
    %% COND 2 
    for ph = 1:3 % target height
        idx = idx_ph(ph,:) & idx_u(us,:) & idx_r2;
        n = sum(idx);
        users_data(us).n_ph(ph) = n;
        % K
        q = quantile(impedance_vect(1, idx),3);
        iqe = q(3) - q(1);
        users_data(us).K_ph(ph) = q(2);
        users_data(us).K_ph_notch(ph) = 1.57*iqe/sqrt(n);
        % B
        q = quantile(impedance_vect(2, idx),3);
        iqe = q(3) - q(1);
        users_data(us).B_ph(ph) = q(2);
        users_data(us).B_ph_notch(ph) = 1.57*iqe/sqrt(n);
        % M
        q = quantile(impedance_vect(3, idx),3);
        iqe = q(3) - q(1);
        users_data(us).M_ph(ph) = q(2);
        users_data(us).M_ph_notch(ph) = 1.57*iqe/sqrt(n);
        % R^2
        q = quantile(impedance_vect(4, idx),3);
        iqe = q(3) - q(1);
        users_data(us).R2_ph(ph) = q(2);
        users_data(us).R2_ph_notch(ph) = 1.57*iqe/sqrt(n);
    end
end
%% COND 1 
K_h = vertcat(users_data.K_h_ph1);
B_h = vertcat(users_data.B_h_ph1);
M_h = vertcat(users_data.M_h_ph1);
R2_h = vertcat(users_data.R2_h_ph1);
grp_h = [repmat("h1",1,31), repmat("h2",1,31), repmat("h3",1,31)];
imp_vect_h_ph1 = [K_h, B_h, M_h, R2_h];

K_h_notch = vertcat(users_data.K_h_ph1_notch);
B_h_notch = vertcat(users_data.B_h_ph1_notch);
M_h_notch = vertcat(users_data.M_h_ph1_notch);
R2_h_notch = vertcat(users_data.R2_h_ph1_notch);
for i = 1:size(K_h,1)
    for j = 1:size(K_h,2)
        K_h_text(i,j) = "$" + sprintf("%.2f",K_h(i,j)) + " \pm " + sprintf("%.2f",K_h_notch(i,j)) + "$";
        B_h_text(i,j) = "$" + sprintf("%.2f",B_h(i,j)) + " \pm " + sprintf("%.2f",B_h_notch(i,j)) + "$";
        M_h_text(i,j) = "$" + sprintf("%.0f",1000*M_h(i,j)) + " \pm " + sprintf("%.0f",1000*M_h_notch(i,j)) + "$";
    end
end

%matrix2latex(K_h_text, "K_ph1_hi_r2.tex", 'columnLabels', ["$h_1^*$","$h_2^*$","$h_3^*$"], 'rowLabels', list_users, 'alignment', 'c');
%matrix2latex(B_h_text, "B_ph1_hi_r2.tex", 'columnLabels', ["$h_1^*$","$h_2^*$","$h_3^*$"], 'rowLabels', list_users, 'alignment', 'c');
%matrix2latex(M_h_text, "M_ph1_hi_r2.tex", 'columnLabels', ["$h_1^*$","$h_2^*$","$h_3^*$"], 'rowLabels', list_users, 'alignment', 'c');

%matrix2latex(K_h', "K_ph1_hi_r2.tex", 'rowLabels', ["$\phi_1$","$\phi_2$","$\phi_3$"], 'columnLabels', list_users, 'alignment', 'c', 'format', '%.1f');
%matrix2latex(K_h, "K_ph1_hi_r2.tex", 'columnLabels', ["$\phi_1$","$\phi_2$","$\phi_3$"], 'rowLabels', list_users, 'alignment', 'c', 'format', '%.1f');

figure
for i = 1:3
    subplot(3,3,(i-1)*3+1)
    histogram(K_h(:,i),'BinWidth',25)
    title("K h_" + string(i))
    subplot(3,3,(i-1)*3+2)
    histogram(B_h(:,i),'BinWidth',1)
    title("B h_" + string(i))
    subplot(3,3,(i-1)*3+3)
    histogram(M_h(:,i),'BinWidth',0.05)
    title("M h_" + string(i))
end
pK = anova1(K_h(:)', grp_h, 'off');
pB = anova1(B_h(:)', grp_h, 'off');
pM = anova1(M_h(:)', grp_h, 'off');

figure
subplot(1,3,1)
boxplot(K_h(:)', grp_h,'Notch','on')
title("K estimation against target height")
subplot(1,3,2)
boxplot(B_h(:)', grp_h,'Notch','on')
title("B estimation against target height")
subplot(1,3,3)
boxplot(M_h(:)', grp_h,'Notch','on')
title("M estimation against target height")

% save data
h = [1.5;1.75;2.0];
for id = 1:size(impedance_vect_R2,1)
    idx_param = 3*(id-1)+(1:3);
    data_id = imp_vect_h_ph1(:,idx_param);
    data_id_s = sort(data_id);
    q = quantile(data_id,3);
    avg = mean(data_id)';
    med = q(2,:)';
    un = (q(2,:) + 1.57*(q(3,:)-q(1,:))/sqrt(31))';
    ln = (q(2,:) - 1.57*(q(3,:)-q(1,:))/sqrt(31))';
    uq = (q(3,:))';
    lq = (q(1,:))';
    for i = 1:3
        uw(i,1) = data_id_s(find(q(3,i) + 1.5*(q(3,i)-q(1,i)) >= data_id_s(:,i), 1, 'last'),i);
        lw(i,1) = data_id_s(find(q(3,i) - 1.5*(q(3,i)-q(1,i)) <= data_id_s(:,i), 1, 'first'),i);
    end
    tab = table(avg, med, un, ln, uq, lq, uw, lw, h);
    %write(tab,path+"height_variation_all_users_r2" + param_names(id) + ".csv",'Delimiter',',');
    clear avg med un ln uq lq uw lw ph
end

% -> no effect of the height

% %% For non parametric analysis
% kruskalwallis(K_h(:)', [repmat("h1",1,31), repmat("h2",1,31), repmat("h3",1,31)]);
% kruskalwallis(B_h(:)', [repmat("h1",1,31), repmat("h2",1,31), repmat("h3",1,31)]);
% kruskalwallis(M_h(:)', [repmat("h1",1,31), repmat("h2",1,31), repmat("h3",1,31)]);

%% COND 1: temporal effect
K_exp = vertcat(users_data.K_exp_ph1);
B_exp = vertcat(users_data.B_exp_ph1);
M_exp = vertcat(users_data.M_exp_ph1);
R2_exp = vertcat(users_data.R2_exp_ph1);
grp_exp = [ones(1,31), repmat(2,1,31), repmat(3,1,31), repmat(4,1,31), repmat(5,1,31)];
imp_vect_exp_ph1 = [K_exp, B_exp, M_exp, R2_exp];

pK = anova1(K_exp(:)', grp_exp, 'off');
pB = anova1(B_exp(:)', grp_exp, 'off');
pM = anova1(M_exp(:)', grp_exp, 'off');

figure
subplot(1,3,1)
boxplot(K_exp(:)', grp_exp,'Notch','on')
title("K estimation against exp order")
subplot(1,3,2)
boxplot(B_exp(:)', grp_exp,'Notch','on')
title("B estimation against exp order")
subplot(1,3,3)
boxplot(M_exp(:)', grp_exp,'Notch','on')
title("M estimation against exp order")

% save data
exp_nb = [1;2;3;4;5];
for id = 1:size(impedance_vect_R2,1)
    idx_param = 5*(id-1)+(1:5);
    data_id = imp_vect_exp_ph1(:,idx_param);
    data_id_s = sort(data_id);
    q = quantile(data_id,3);
    avg = mean(data_id)';
    med = q(2,:)';
    un = (q(2,:) + 1.57*(q(3,:)-q(1,:))/sqrt(31))';
    ln = (q(2,:) - 1.57*(q(3,:)-q(1,:))/sqrt(31))';
    uq = (q(3,:))';
    lq = (q(1,:))';
    for i = 1:5
        uw(i,1) = data_id_s(find(q(3,i) + 1.5*(q(3,i)-q(1,i)) >= data_id_s(:,i), 1, 'last'),i);
        lw(i,1) = data_id_s(find(q(3,i) - 1.5*(q(3,i)-q(1,i)) <= data_id_s(:,i), 1, 'first'),i);
    end
    tab = table(avg, med, un, ln, uq, lq, uw, lw, exp_nb);
    write(tab,path+"time_variation_all_users_r2" + param_names(id) + ".csv",'Delimiter',',');
    clear avg med un ln uq lq uw lw ph
end

%% COND 2 
K_ph = vertcat(users_data.K_ph);
B_ph = vertcat(users_data.B_ph);
M_ph = vertcat(users_data.M_ph);
R2_ph = vertcat(users_data.R2_ph);
grp_ph = [repmat("ph1",1,31), repmat("ph2",1,31), repmat("ph3",1,31)];
imp_vect_ph = [K_ph, B_ph, M_ph, R2_ph];

figure
for i = 1:3
    subplot(3,3,(i-1)*3+1)
    histogram(K_ph(:,i),'BinWidth',25)
    title("K \phi_" + string(i))
    subplot(3,3,(i-1)*3+2)
    histogram(B_ph(:,i),'BinWidth',1)
    title("B \phi_" + string(i))
    subplot(3,3,(i-1)*3+3)
    histogram(M_ph(:,i),'BinWidth',0.05)
    title("M \phi_" + string(i))
end

K_ph_notch = vertcat(users_data.K_ph_notch);
B_ph_notch = vertcat(users_data.B_ph_notch);
M_ph_notch = vertcat(users_data.M_ph_notch);
R2_ph_notch = vertcat(users_data.R2_ph_notch);
for i = 1:size(K_h,1)
    for j = 1:size(K_h,2)
        K_ph_text(i,j) = "$" + sprintf("%.2f",K_ph(i,j)) + " \pm " + sprintf("%.2f",K_ph_notch(i,j)) + "$";
        B_ph_text(i,j) = "$" + sprintf("%.2f",B_ph(i,j)) + " \pm " + sprintf("%.2f",B_ph_notch(i,j)) + "$";
        M_ph_text(i,j) = "$" + sprintf("%.0f",1000*M_ph(i,j)) + " \pm " + sprintf("%.0f",1000*M_ph_notch(i,j)) + "$";
    end
end

% matrix2latex(K_ph_text, "K_ph_r2.tex", 'columnLabels', ["$\phi_1$","$\phi_2$","$\phi_3$"], 'rowLabels', list_users, 'alignment', 'c');
% matrix2latex(B_ph_text, "B_ph_r2.tex", 'columnLabels', ["$\phi_1$","$\phi_2$","$\phi_3$"], 'rowLabels', list_users, 'alignment', 'c');
% matrix2latex(M_ph_text, "M_ph_r2.tex", 'columnLabels', ["$\phi_1$","$\phi_2$","$\phi_3$"], 'rowLabels', list_users, 'alignment', 'c');



pK_ph = anova1(K_ph(:)', grp_ph, 'off');
pB_ph = anova1(B_ph(:)', grp_ph, 'off');
pM_ph = anova1(M_ph(:)', grp_ph, 'off');
%% For non parametric analysis
pK_ph_kw = kruskalwallis(K_ph(:)', grp_ph, 'off');
pB_ph_kw = kruskalwallis(B_ph(:)', grp_ph, 'off');
pM_ph_kw = kruskalwallis(M_ph(:)', grp_ph, 'off');

figure
subplot(1,3,1)
boxplot(K_ph(:)', grp_ph, 'Notch','on')
title("K estimation against phase")
subplot(1,3,2)
boxplot(B_ph(:)', grp_ph, 'Notch','on')
title("B estimation against phase")
subplot(1,3,3)
boxplot(M_ph(:)', grp_ph, 'Notch','on')
title("M estimation against phase")

% save data
phi = [0.25;0.50;0.75];
for id = 1:size(impedance_vect_R2,1)
    idx_param = 3*(id-1)+(1:3);
    data_id = imp_vect_ph(:,idx_param);
    data_id_s = sort(data_id);
    q = quantile(data_id,3);
    avg = mean(data_id)';
    med = q(2,:)';
    un = (q(2,:) + 1.57*(q(3,:)-q(1,:))/sqrt(31))';
    ln = (q(2,:) - 1.57*(q(3,:)-q(1,:))/sqrt(31))';
    uq = (q(3,:))';
    lq = (q(1,:))';
    for i = 1:3
        uw(i,1) = data_id_s(find(q(3,i) + 1.5*(q(3,i)-q(1,i)) >= data_id_s(:,i), 1, 'last'),i);
        lw(i,1) = data_id_s(find(q(3,i) - 1.5*(q(3,i)-q(1,i)) <= data_id_s(:,i), 1, 'first'),i);
    end
    tab = table(avg, med, un, ln, uq, lq, uw, lw, phi);
    %write(tab,path+"phase_variation_all_users_r2_" + param_names(id) + ".csv",'Delimiter',',');
    clear avg med un ln uq lq uw lw ph
end

%% COND2 Multi factor

grp_exp = repmat(exp_list_sorted, 1, 3);

figure
[~,~,stats_K] = anovan(K_ph(:)', {grp_ph grp_exp},'model','interaction',...
    'varnames',{'phi','exp'});
results = multcompare(stats_K,'CType','bonferroni','Dimension',[1,2]);
%
figure
[~,~,stats_B] = anovan(B_ph(:)', {grp_ph grp_exp},'model','interaction',...
    'varnames',{'phi','exp'});
results = multcompare(stats_B,'CType','bonferroni','Dimension',[1,2]);
%
figure
[~,~,stats_M] = anovan(M_ph(:)', {grp_ph grp_exp},'model','interaction',...
    'varnames',{'phi','exp'});
results = multcompare(stats_M,'CType','bonferroni','Dimension',[1,2]);


end
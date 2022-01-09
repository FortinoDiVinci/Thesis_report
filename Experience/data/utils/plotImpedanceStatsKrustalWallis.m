function plotImpedanceStatsKrustalWallis(cycles, exp_list)

cycles_dist = cell(1,length(cycles));

for user_i = 1:length(cycles)        
    cycles_dist{user_i} = cycles{user_i}([cycles{user_i}.is_dist]);
end
clear cycles

[~,idx_exp_s] = sort(exp_list);


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

list_users = unique(users_vect);
list_users = list_users(idx_exp_s);
for ii= 1:length(cycles_dist)
    idx_u(ii,:) = strcmp(users_vect, list_users(ii)); % sorted by expertise
end
idx_r2 = (R2_vect > 0.5);
impedance_vect = [K_vect; B_vect; M_vect; R2_vect];
impedance_vect_R2 = [K_vect(idx_r2); B_vect(idx_r2); M_vect(idx_r2); R2_vect(idx_r2)];
param_names = ["K", "B", "M", "R2"];

%%%%%%%%%%%%%%
% COND1: same phase (25%), height variation 
%%%%%%%%%%%%%%
idx_name = ["h1,e1","h1,e2","h1,e3","h2,e1","h2,e2","h2,e3","h3,e1","h3,e2","h3,e3"];
k = 1;
for i = 1:3
	for j = 1:3
        idx(k,:) = idx_ph(1,:) & idx_h(i,:) & idx_e(j,:) & idx_r2;
        if ~exist('val_idx', 'var')
            val_idx = [""];
        end
        val_idx(idx(k,:)) = idx_name(k);
        k = k + 1;
    end
end
val_idx_ph1 = rmmissing(val_idx);
[p,tbl,stats] = kruskalwallis(K_vect(idx_ph(1,:)&idx_r2),val_idx_ph1);
multcompare(stats);

%%%%%%%%%%%%%%
% COND2: same phase (25%), height variation 
%%%%%%%%%%%%%%
idx_name = ["h1,e1","h1,e2","h1,e3","h2,e1","h2,e2","h2,e3","h3,e1","h3,e2","h3,e3"];
k = 1;
for i = 1:3
	for j = 1:3
        idx(k,:) = idx_ph(1,:) & idx_h(i,:) & idx_e(j,:) & idx_r2;
        if ~exist('val_idx', 'var')
            val_idx = [""];
        end
        val_idx(idx(k,:)) = idx_name(k);
        k = k + 1;
    end
end
val_idx_ph1 = rmmissing(val_idx);
[p,tbl,stats] = kruskalwallis(K_vect(idx_ph(1,:)&idx_r2),val_idx_ph1);
multcompare(stats);



%%%%%%%%%%%%%%
% COND1: same phase (25%), height variation 
%%%%%%%%%%%%%%
% % all users togethers
% for id = 1:size(impedance_vect,1)
%     for i = 1:3
%         idx(i,:) = idx_ph(1,:) & idx_h(i,:); % ph 25% only
%         q = quantile(impedance_vect(id, idx(i,:)),3);
%         imp_data_sort = sort(impedance_vect(id, idx(i,:)));
%         iqe = q(3) - q(1);
%         n(i,1) = sum(idx(i,:));
%         med(i,1) = q(2);
%         un(i,1) = imp_data_sort(find(q(2) + 1.57*iqe/sqrt(n(i,1)) >= imp_data_sort, 1, 'last'));
%         ln(i,1) = imp_data_sort(find(q(2) - 1.57*iqe/sqrt(n(i,1)) <= imp_data_sort, 1, 'first'));
%         uq(i,1) = q(3);
%         lq(i,1) = q(1);
%         uw(i,1) = imp_data_sort(find(q(3) + 1.5*iqe >= imp_data_sort, 1, 'last'));        
%         lw(i,1) = imp_data_sort(find(q(3) - 1.5*iqe <= imp_data_sort, 1, 'first'));  %q(1) - 1.5*iqe;
%         h(i,1) = mean(h_vect(idx_h(i,:)));
%     end
%     tabi = table(n, med, un, ln, uq, lq, uw, lw, h);
%     write(tabi,path+"height_variation_all_users_" + param_names(id) + ".csv",'Delimiter',',');
%     clear n med un ln uq lq uw lw h
% end
% 
% % users per expertise
% k = 1;
% for id = 1:size(impedance_vect,1)
%     for i = 1:3
%         for j = 1:3
%             idx(k,:) = idx_ph(1,:) & idx_h(i,:) & idx_e(j,:); % ph 25% only
%             q = quantile(impedance_vect(id, idx(k,:)),3);
%             iqe = q(3) - q(1);
%             imp_data_sort = sort(impedance_vect(id, idx(k,:)));
%             n(k,1) = sum(idx(k,:));
%             med(k,1) = q(2);
%             un(k,1) = imp_data_sort(find(q(2) + 1.57*iqe/sqrt(n(i,1)) >= imp_data_sort, 1, 'last'));
%             ln(k,1) = imp_data_sort(find(q(2) - 1.57*iqe/sqrt(n(i,1)) <= imp_data_sort, 1, 'first'));
%             uq(k,1) = q(3);
%             lq(k,1) = q(1);
%             uw(k,1) = imp_data_sort(find(q(3) + 1.5*iqe >= imp_data_sort, 1, 'last'));        
%             lw(k,1) = imp_data_sort(find(q(3) - 1.5*iqe <= imp_data_sort, 1, 'first'));
%             h(k,1) = mean(h_vect(idx_h(i,:)));
%             e(k,1) = mean(e_vect(idx_e(j,:)));
%             k = k + 1;
%         end
%     end
%     tabi = table(n, med, un, ln, uq, lq, uw, lw, h, e);
%     write(tabi,path+"height_variation_expert_" + param_names(id) + ".csv",'Delimiter',',');
%     clear n med un ln uq lq uw lw h e
%     k = 1;
% end
% % h1e1, h1e2, h1e3, h2e1, ...
% 
% % individuals
% k = 1;
% for id = 1:size(impedance_vect,1)
%     for j = 1:length(list_users)        
%         for i = 1:3
%             idx(k,:) = idx_ph(1,:) & idx_h(i,:) & idx_u(j,:); % ph 25% only
%             q = quantile(impedance_vect(id, idx(k,:)),3);
%             iqe = q(3) - q(1);
%             imp_data_sort = sort(impedance_vect(id, idx(k,:)));
%             n(k,1) = sum(idx(k,:));
%             med(k,1) = q(2);
%             un(k,1) = imp_data_sort(find(q(2) + 1.57*iqe/sqrt(n(i,1)) >= imp_data_sort, 1, 'last'));
%             ln(k,1) = imp_data_sort(find(q(2) - 1.57*iqe/sqrt(n(i,1)) <= imp_data_sort, 1, 'first'));
%             uq(k,1) = q(3);
%             lq(k,1) = q(1);
%             uw(k,1) = imp_data_sort(find(q(3) + 1.5*iqe >= imp_data_sort, 1, 'last'));        
%             lw(k,1) = imp_data_sort(find(q(3) - 1.5*iqe <= imp_data_sort, 1, 'first'));
%             u(k,1) = list_users(j);
%             h(k,1) = mean(h_vect(idx_h(i,:)));
%             k = k + 1;
%         end
%     end
%     tabi = table(n, med, un, ln, uq, lq, uw, lw, u, h);
%     write(tabi,path+"height_variation_users_" + param_names(id) + ".csv",'Delimiter',',');
%     clear n med un ln uq lq uw lw u h
%     k = 1;
% end
% % h1u1, h2u2, h3u3, h1u2, ...
% %%%%%%%%%%%%%%
% % COND2: same height (1.75), phase variation 
% %%%%%%%%%%%%%%
% % all users togethers
% for id = 1:size(impedance_vect,1)
%     for i = 1:3
%         idx(i,:) = idx_h(2,:) & idx_ph(i,:); % h 1.75 only
%         q = quantile(impedance_vect(id, idx(i,:)),3);
%         iqe = q(3) - q(1);
%         imp_data_sort = sort(impedance_vect(id, idx(i,:)));
%         n(i,1) = sum(idx(i,:));
%         med(i,1) = q(2);
%         un(i,1) = imp_data_sort(find(q(2) + 1.57*iqe/sqrt(n(i,1)) >= imp_data_sort, 1, 'last'));
%         ln(i,1) = imp_data_sort(find(q(2) - 1.57*iqe/sqrt(n(i,1)) <= imp_data_sort, 1, 'first'));
%         uq(i,1) = q(3);
%         lq(i,1) = q(1);
%         uw(i,1) = imp_data_sort(find(q(3) + 1.5*iqe >= imp_data_sort, 1, 'last'));        
%         lw(i,1) = imp_data_sort(find(q(3) - 1.5*iqe <= imp_data_sort, 1, 'first'));
%         ph(i,1) = mean(ph_vect_ratio(idx_ph(i,:)));
%     end
%     tabi = table(n, med, un, ln, uq, lq, uw, lw, ph);
%     write(tabi,path+"phase_variation_all_users_" + param_names(id) + ".csv",'Delimiter',',');
%     clear n med un ln uq lq uw lw ph
% end
% 
% % users per expertise
% k = 1;
% for id = 1:size(impedance_vect,1)
%     for i = 1:3
%         for j = 1:3
%             idx(k,:) = idx_h(2,:) & idx_ph(i,:) & idx_e(j,:); % h 1.75 only
%             q = quantile(impedance_vect(id, idx(k,:)),3);
%             iqe = q(3) - q(1);
%             imp_data_sort = sort(impedance_vect(id, idx(k,:)));
%             n(k,1) = sum(idx(k,:));
%             med(k,1) = q(2);
%             un(k,1) = imp_data_sort(find(q(2) + 1.57*iqe/sqrt(n(i,1)) >= imp_data_sort, 1, 'last'));
%             ln(k,1) = imp_data_sort(find(q(2) - 1.57*iqe/sqrt(n(i,1)) <= imp_data_sort, 1, 'first'));
%             uq(k,1) = q(3);
%             lq(k,1) = q(1);
%             uw(k,1) = imp_data_sort(find(q(3) + 1.5*iqe >= imp_data_sort, 1, 'last'));        
%             lw(k,1) = imp_data_sort(find(q(3) - 1.5*iqe <= imp_data_sort, 1, 'first'));
%             ph(k,1) = mean(ph_vect_ratio(idx_ph(i,:)));
%             e(k,1) = mean(e_vect(idx_e(j,:)));
%             k = k + 1;
%         end
%     end
%     tabi = table(n, med, un, ln, uq, lq, uw, lw, ph, e);
%     write(tabi,path+"phase_variation_expert_" + param_names(id) + ".csv",'Delimiter',',');
%     clear n med un ln uq lq uw lw ph e
%     k = 1;
% end
% % h1e1, h1e2, h1e3, h2e1, ...
% 
% % individuals
% k = 1;
% for id = 1:size(impedance_vect,1)
%     for j = 1:length(list_users)        
%         for i = 1:3
%             idx(k,:) = idx_h(2,:) & idx_ph(i,:) & idx_u(j,:); % h 1.75 only
%             q = quantile(impedance_vect(id, idx(k,:)),3);
%             iqe = q(3) - q(1);
%             imp_data_sort = sort(impedance_vect(id, idx(k,:)));
%             n(k,1) = sum(idx(k,:));
%             med(k,1) = q(2);
%             un(k,1) = imp_data_sort(find(q(2) + 1.57*iqe/sqrt(n(i,1)) >= imp_data_sort, 1, 'last'));
%             ln(k,1) = imp_data_sort(find(q(2) - 1.57*iqe/sqrt(n(i,1)) <= imp_data_sort, 1, 'first'));
%             uq(k,1) = q(3);
%             lq(k,1) = q(1);
%             uw(k,1) = imp_data_sort(find(q(3) + 1.5*iqe >= imp_data_sort, 1, 'last'));        
%             lw(k,1) = imp_data_sort(find(q(3) - 1.5*iqe <= imp_data_sort, 1, 'first'));
%             u(k,1) = list_users(j);
%             ph(k,1) = mean(ph_vect_ratio(idx_ph(i,:)));
%             k = k + 1;
%         end
%     end
%     tabi = table(n, med, un, ln, uq, lq, uw, lw, u, ph);
%     write(tabi,path+"phase_variation_users_" + param_names(id) + ".csv",'Delimiter',',');
%     clear n med un ln uq lq uw lw u ph
%     k = 1;
% end
% % h1u1, h2u2, h3u3, h1u2, ...
% 
% %%%%%%%%%%%%%%
% %% R^2 filtering (r^2 > 0.5)
% %%%%%%%%%%%%%%
% %%%%%%%%%%%%%%
% % COND1: same phase (25%), height variation 
% %%%%%%%%%%%%%%
% % all users togethers
% clear idx
% for id = 1:size(impedance_vect,1)
%     for i = 1:3
%         idx(i,:) = idx_ph(1,idx_r2) & idx_h(i,idx_r2); % ph 25% only
%         q = quantile(impedance_vect_R2(id, idx(i,:)),3);
%         iqe = q(3) - q(1);
%         imp_data_sort = sort(impedance_vect_R2(id, idx(i,:)));
%         n(i,1) = sum(idx(i,:));
%         med(i,1) = q(2);
%         un(i,1) = imp_data_sort(find(q(2) + 1.57*iqe/sqrt(n(i,1)) >= imp_data_sort, 1, 'last'));
%         ln(i,1) = imp_data_sort(find(q(2) - 1.57*iqe/sqrt(n(i,1)) <= imp_data_sort, 1, 'first'));
%         uq(i,1) = q(3);
%         lq(i,1) = q(1);
%         uw(i,1) = imp_data_sort(find(q(3) + 1.5*iqe >= imp_data_sort, 1, 'last'));        
%         lw(i,1) = imp_data_sort(find(q(3) - 1.5*iqe <= imp_data_sort, 1, 'first'));  %q(1) - 1.5*iqe;
%         h(i,1) = mean(h_vect(idx_h(i,:)));
%     end
%     tabi = table(n, med, un, ln, uq, lq, uw, lw, h);
%     write(tabi,path+"height_variation_all_users_r2_" + param_names(id) + ".csv",'Delimiter',',');
%     clear n med un ln uq lq uw lw h
% end
% 
% % users per expertise
% k = 1;
% for id = 1:size(impedance_vect,1)
%     for i = 1:3
%         for j = 1:3
%             idx(k,:) = idx_ph(1,idx_r2) & idx_h(i,idx_r2) & idx_e(j,idx_r2); % ph 25% only
%             q = quantile(impedance_vect_R2(id, idx(k,:)),3);
%             iqe = q(3) - q(1);
%             imp_data_sort = sort(impedance_vect_R2(id, idx(k,:)));
%             n(k,1) = sum(idx(k,:));
%             med(k,1) = q(2);
%             un(k,1) = imp_data_sort(find(q(2) + 1.57*iqe/sqrt(n(i,1)) >= imp_data_sort, 1, 'last'));
%             ln(k,1) = imp_data_sort(find(q(2) - 1.57*iqe/sqrt(n(i,1)) <= imp_data_sort, 1, 'first'));
%             uq(k,1) = q(3);
%             lq(k,1) = q(1);
%             uw(k,1) = imp_data_sort(find(q(3) + 1.5*iqe >= imp_data_sort, 1, 'last'));        
%             lw(k,1) = imp_data_sort(find(q(3) - 1.5*iqe <= imp_data_sort, 1, 'first'));
%             h(k,1) = mean(h_vect(idx_h(i,:)));
%             e(k,1) = mean(e_vect(idx_e(j,:)));
%             k = k + 1;
%         end
%     end
%     tabi = table(n, med, un, ln, uq, lq, uw, lw, h, e);
%     write(tabi,path+"height_variation_expert_r2_" + param_names(id) + ".csv",'Delimiter',',');
%     clear n med un ln uq lq uw lw h e
%     k = 1;
% end
% % h1e1, h1e2, h1e3, h2e1, ...
% 
% % individuals
% k = 1;
% for id = 1:size(impedance_vect,1)
%     for j = 1:length(list_users)        
%         for i = 1:3
%             idx(k,:) = idx_ph(1,idx_r2) & idx_h(i,idx_r2) & idx_u(j,idx_r2); % ph 25% only
%             q = quantile(impedance_vect_R2(id, idx(k,:)),3);
%             iqe = q(3) - q(1);
%             imp_data_sort = sort(impedance_vect_R2(id, idx(k,:)));
%             n(k,1) = sum(idx(k,:));
%             med(k,1) = q(2);
%             un(k,1) = imp_data_sort(find(q(2) + 1.57*iqe/sqrt(n(i,1)) >= imp_data_sort, 1, 'last'));
%             ln(k,1) = imp_data_sort(find(q(2) - 1.57*iqe/sqrt(n(i,1)) <= imp_data_sort, 1, 'first'));
%             uq(k,1) = q(3);
%             lq(k,1) = q(1);
%             uw(k,1) = imp_data_sort(find(q(3) + 1.5*iqe >= imp_data_sort, 1, 'last'));        
%             lw(k,1) = imp_data_sort(find(q(3) - 1.5*iqe <= imp_data_sort, 1, 'first'));
%             u(k,1) = list_users(j);
%             h(k,1) = mean(h_vect(idx_h(i,:)));
%             k = k + 1;
%         end
%     end
%     tabi = table(n, med, un, ln, uq, lq, uw, lw, u, h);
%     write(tabi,path+"height_variation_users_r2_" + param_names(id) + ".csv",'Delimiter',',');
%     clear n med un ln uq lq uw lw u h
%     k = 1;
% end
% % h1u1, h2u2, h3u3, h1u2, ...
% %%%%%%%%%%%%%%
% % COND2: same height (1.75), phase variation 
% %%%%%%%%%%%%%%
% % all users togethers
% for id = 1:size(impedance_vect,1)
%     for i = 1:3
%         idx(i,:) = idx_h(2,idx_r2) & idx_ph(i,idx_r2); % h 1.75 only
%         q = quantile(impedance_vect_R2(id, idx(i,:)),3);
%         iqe = q(3) - q(1);
%         imp_data_sort = sort(impedance_vect_R2(id, idx(i,:)));
%         n(i,1) = sum(idx(i,:));
%         med(i,1) = q(2);
%         un(i,1) = imp_data_sort(find(q(2) + 1.57*iqe/sqrt(n(i,1)) >= imp_data_sort, 1, 'last'));
%         ln(i,1) = imp_data_sort(find(q(2) - 1.57*iqe/sqrt(n(i,1)) <= imp_data_sort, 1, 'first'));
%         uq(i,1) = q(3);
%         lq(i,1) = q(1);
%         uw(i,1) = imp_data_sort(find(q(3) + 1.5*iqe >= imp_data_sort, 1, 'last'));        
%         lw(i,1) = imp_data_sort(find(q(3) - 1.5*iqe <= imp_data_sort, 1, 'first'));
%         ph(i,1) = mean(ph_vect_ratio(idx_ph(i,:)));
%     end
%     tabi = table(n, med, un, ln, uq, lq, uw, lw, ph);
%     write(tabi,path+"phase_variation_all_users_r2_" + param_names(id) + ".csv",'Delimiter',',');
%     clear n med un ln uq lq uw lw ph
% end
% 
% % users per expertise
% k = 1;
% for id = 1:size(impedance_vect,1)
%     for i = 1:3
%         for j = 1:3
%             idx(k,:) = idx_h(2,idx_r2) & idx_ph(i,idx_r2) & idx_e(j,idx_r2); % h 1.75 only
%             q = quantile(impedance_vect_R2(id, idx(k,:)),3);
%             iqe = q(3) - q(1);
%             imp_data_sort = sort(impedance_vect_R2(id, idx(k,:)));
%             n(k,1) = sum(idx(k,:));
%             med(k,1) = q(2);
%             un(k,1) = imp_data_sort(find(q(2) + 1.57*iqe/sqrt(n(i,1)) >= imp_data_sort, 1, 'last'));
%             ln(k,1) = imp_data_sort(find(q(2) - 1.57*iqe/sqrt(n(i,1)) <= imp_data_sort, 1, 'first'));
%             uq(k,1) = q(3);
%             lq(k,1) = q(1);
%             uw(k,1) = imp_data_sort(find(q(3) + 1.5*iqe >= imp_data_sort, 1, 'last'));        
%             lw(k,1) = imp_data_sort(find(q(3) - 1.5*iqe <= imp_data_sort, 1, 'first'));
%             ph(k,1) = mean(ph_vect_ratio(idx_ph(i,:)));
%             e(k,1) = mean(e_vect(idx_e(j,:)));
%             k = k + 1;
%         end
%     end
%     tabi = table(n, med, un, ln, uq, lq, uw, lw, ph, e);
%     write(tabi,path+"phase_variation_expert_r2_" + param_names(id) + ".csv",'Delimiter',',');
%     clear n med un ln uq lq uw lw ph e
%     k = 1;
% end
% % h1e1, h1e2, h1e3, h2e1, ...
% 
% % individuals
% k = 1;
% for id = 1:size(impedance_vect,1)
%     for j = 1:length(list_users)        
%         for i = 1:3
%             idx(k,:) = idx_h(2,idx_r2) & idx_ph(i,idx_r2) & idx_u(j,idx_r2); % h 1.75 only
%             q = quantile(impedance_vect_R2(id, idx(k,:)),3);
%             iqe = q(3) - q(1);
%             imp_data_sort = sort(impedance_vect_R2(id, idx(k,:)));
%             n(k,1) = sum(idx(k,:));
%             med(k,1) = q(2);
%             un(k,1) = imp_data_sort(find(q(2) + 1.57*iqe/sqrt(n(i,1)) >= imp_data_sort, 1, 'last'));
%             ln(k,1) = imp_data_sort(find(q(2) - 1.57*iqe/sqrt(n(i,1)) <= imp_data_sort, 1, 'first'));
%             uq(k,1) = q(3);
%             lq(k,1) = q(1);
%             uw(k,1) = imp_data_sort(find(q(3) + 1.5*iqe >= imp_data_sort, 1, 'last'));        
%             lw(k,1) = imp_data_sort(find(q(3) - 1.5*iqe <= imp_data_sort, 1, 'first'));
%             u(k,1) = list_users(j);
%             ph(k,1) = mean(ph_vect_ratio(idx_ph(i,:)));
%             k = k + 1;
%         end
%     end
%     tabi = table(n, med, un, ln, uq, lq, uw, lw, u, ph);
%     write(tabi,path+"phase_variation_users_r2_" + param_names(id) + ".csv",'Delimiter',',');
%     clear n med un ln uq lq uw lw u ph
%     k = 1;
% end
% h1u1, h2u2, h3u3, h1u2, ...
end


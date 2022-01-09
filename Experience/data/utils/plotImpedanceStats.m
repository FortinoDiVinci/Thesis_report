function impedance_analysis = plotImpedanceStats(cycles, exp_list)
%
% colors = hsv(length(cycles));
% users = [];
% 
cycles_dist = cell(1,length(cycles));

for user_i = 1:length(cycles)        
    cycles_dist{user_i} = cycles{user_i}([cycles{user_i}.is_dist]);
end
clear cycles

[~,idx_exp_s] = sort(exp_list);

%%%%%%%%
% Sort perturbations
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
% 
ph1 = mean(ph_vect_ratio(ph_vect == 1));
ph2 = mean(ph_vect_ratio(ph_vect == 2));
ph3 = mean(ph_vect_ratio(ph_vect == 3));
[~,idx_min] = min([ph1,ph2,ph3]);
% 
idx_ph1 = (ph_vect == idx_min);
idx_h2 = (h_vect == 1.75);
disp("Nb novice id (ph1): " + string(sum((e_vect == 1)&idx_ph1)))
disp("Nb intermediate id (ph1): " + string(sum((e_vect == 2)&idx_ph1)))
disp("Nb advanved id (ph1): " + string(sum((e_vect == 3)&idx_ph1)))
%
disp("Nb novice id (h2): " + string(sum((e_vect == 1)&idx_h2)))
disp("Nb intermediate id (h2): " + string(sum((e_vect == 2)&idx_h2)))
disp("Nb advanved id (h2): " + string(sum((e_vect == 3)&idx_h2)))
% %%%%%%%
% PHASE 1 - h variations
% [res_n,~,stats] = anovan(K_vect(idx_ph1),{e_vect(idx_ph1) h_vect(idx_ph1) n_vect(idx_ph1)},...
%     'model','interaction','varnames',{'e','h','n'});
% figure
% multcompare(stats,'CType','bonferroni','Dimension',[1 2]);
% per users
% [res_n,~,stats] = anovan(K_vect(idx_ph1),{users_vect(idx_ph1) h_vect(idx_ph1) n_vect(idx_ph1)},...
%     'model','interaction','varnames',{'u','h','n'});
% figure
% multcompare(stats,'CType','bonferroni','Dimension',[1 2]);
% %%%%%%%
% H2 - phase variation
% [res_n,~,stats] = anovan(K_vect(idx_h2),{e_vect(idx_h2) ph_vect(idx_h2) n_vect(idx_h2)},...
%     'model','interaction','varnames',{'e','h','n'});
% figure
% multcompare(stats,'CType','bonferroni','Dimension',[1 2]);
% per users
% [res_n,~,stats] = anovan(K_vect(idx_h2),{users_vect(idx_h2) ph_vect(idx_h2)},...
%     'model','interaction','varnames',{'u','h'});
% figure
% multcompare(stats,'CType','bonferroni','Dimension',[1 2]);
% %%%%%%
% Without outlier detected with clearOutliers (R<50%, and K outside 3sMAD)
% idx_c = ~out_vect;
% idx = idx_c & idx_ph1;% & (~out_vect);
% [res_n,~,stats] = anovan(K_vect(idx),{e_vect(idx) h_vect(idx) n_vect(idx)},...
%     'model','interaction','varnames',{'e','h','n'});
% figure
% multcompare(stats,'CType','bonferroni','Dimension',[1 2]);
% 
% idx = idx_c & idx_h2;% & (~out_vect);
% [res_n,~,stats] = anovan(K_vect(idx),{e_vect(idx) ph_vect(idx) n_vect(idx)},...
%     'model','interaction','varnames',{'e','ph','n'});
% figure
% multcompare(stats,'CType','bonferroni','Dimension',[1 2]);
% %%%%%%
% Without R<50% only
% idx_r2 = R2_vect < 0.5;
% idx = ~idx_r2 & idx_ph1;% & (~out_vect);
% [res_n,~,stats] = anovan(K_vect(idx),{e_vect(idx) h_vect(idx) n_vect(idx)},...
%     'model','interaction','varnames',{'e','h','n'});
% figure
% multcompare(stats,'CType','bonferroni','Dimension',[1 2]);
% 
% idx = idx_r2 & idx_h2;% & (~out_vect);
% [res_n,~,stats] = anovan(K_vect(idx),{e_vect(idx) ph_vect(idx) n_vect(idx)},...
%     'model','interaction','varnames',{'e','ph','n'});
% figure
% multcompare(stats,'CType','bonferroni','Dimension',[1 2]);



[~,srt_ph] = sort([ph1,ph2,ph3]);
th_list = [1.5,1.75,2.0];

for user_i = 1:length(cycles_dist) 
    
    impedance_analysis(user_i).user = cycles_dist{user_i}(1).user;
    impedance_analysis(user_i).nb_id = length(cycles_dist{user_i});
    for i = 1:3
        idx_ph(i,:) = [cycles_dist{user_i}.type_dist] == srt_ph(i);
        idx_h(i,:) = [cycles_dist{user_i}.target_height] == th_list(i);
    end
    
    K_user_i = [cycles_dist{user_i}.K];
    B_user_i = [cycles_dist{user_i}.B];
    M_user_i = [cycles_dist{user_i}.M];
    R2_user_i = [cycles_dist{user_i}.R2];
    
    impedance_analysis(user_i).nb_ph1 = sum(idx_ph(1,:));
    impedance_analysis(user_i).nb_ph2 = sum(idx_ph(2,:));
    impedance_analysis(user_i).nb_ph3 = sum(idx_ph(3,:));
    impedance_analysis(user_i).nb_h1 = sum(idx_h(1,:));
    impedance_analysis(user_i).nb_h2 = sum(idx_h(2,:));
    impedance_analysis(user_i).nb_h3 = sum(idx_h(3,:));
    
    % high notch : p50 + 1.57*(p75-p25)/sqrt(length(x));
    %%%%%%%
    % K
    %%%%%%%
    %% all
    n = impedance_analysis(user_i).nb_id;
    qiK = quantile(K_user_i,3);
    ieqK = qiK(3) - qiK(1);
    nR2 = sum(R2_user_i > 0.5);
    qiK_R2 = quantile(K_user_i(R2_user_i>0.5),3);
    ieqK_R2 = qiK_R2(3) - qiK_R2(1);
    %
    impedance_analysis(user_i).K_avg = mean(K_user_i);
    impedance_analysis(user_i).K_med = qiK(2);
    impedance_analysis(user_i).K_med_nhi = qiK(2) + 1.57*ieqK/sqrt(n);
    impedance_analysis(user_i).K_med_nlo = qiK(2) - 1.57*ieqK/sqrt(n);     
    impedance_analysis(user_i).K_q3 = qiK(3);
    impedance_analysis(user_i).K_q1 = qiK(1); 
    impedance_analysis(user_i).K_whishi = qiK(3) + 1.5*ieqK;
    impedance_analysis(user_i).K_whislo = qiK(1) - 1.5*ieqK;    
    impedance_analysis(user_i).K_avg_fR2 = mean(K_user_i(R2_user_i>0.5));
    impedance_analysis(user_i).K_med_fR2 = qiK_R2(2);
    impedance_analysis(user_i).K_med_nhi_fR2 = qiK_R2(2) + 1.57*ieqK_R2/sqrt(nR2);
    impedance_analysis(user_i).K_med_nlo_fR2 = qiK_R2(2) - 1.57*ieqK_R2/sqrt(nR2); 
    impedance_analysis(user_i).K_q3_fR2 = qiK_R2(3);
    impedance_analysis(user_i).K_q1_fR2 = qiK_R2(1); 
    impedance_analysis(user_i).K_whishi_fR2 = qiK_R2(3) + 1.5*ieqK_R2;
    impedance_analysis(user_i).K_whislo_fR2 = qiK_R2(1) - 1.5*ieqK_R2;  
    %% ph1
    n = impedance_analysis(user_i).nb_ph1;
    qiK = quantile(K_user_i(idx_ph(1,:)),3);
    ieqK = qiK(3) - qiK(1);
    nR2 = sum(R2_user_i > 0.5 & idx_ph(1,:));
    qiK_R2 = quantile(K_user_i((R2_user_i>0.5) & idx_ph(1,:)),3);
    ieqK_R2 = qiK_R2(3) - qiK_R2(1);
    %
    impedance_analysis(user_i).K_avg_ph1 = mean(K_user_i(idx_ph(1,:)));
    impedance_analysis(user_i).K_med_ph1 = qiK(2);
    impedance_analysis(user_i).K_med_nhi_ph1 = qiK(2) + 1.57*ieqK/sqrt(n);
    impedance_analysis(user_i).K_med_nlo_ph1 = qiK(2) - 1.57*ieqK/sqrt(n);  
    impedance_analysis(user_i).K_q3_ph1 = qiK(3);
    impedance_analysis(user_i).K_q1_ph1 = qiK(1); 
    impedance_analysis(user_i).K_whishi_ph1 = qiK(3) + 1.5*ieqK;
    impedance_analysis(user_i).K_whislo_ph1 = qiK(1) - 1.5*ieqK;  
    impedance_analysis(user_i).K_avg_fR2_ph1 = mean(K_user_i(R2_user_i>0.5 & idx_ph(1,:)));
    impedance_analysis(user_i).K_med_fR2_ph1 = qiK_R2(2);
    impedance_analysis(user_i).K_med_nhi_fR2_ph1 = qiK_R2(2) + 1.57*ieqK_R2/sqrt(nR2);
    impedance_analysis(user_i).K_med_nlo_fR2_ph1 = qiK_R2(2) - 1.57*ieqK_R2/sqrt(nR2);
    impedance_analysis(user_i).K_q3_fR2_ph1 = qiK_R2(3);
    impedance_analysis(user_i).K_q1_fR2_ph1 = qiK_R2(1); 
    impedance_analysis(user_i).K_whishi_fR2_ph1 = qiK_R2(3) + 1.5*ieqK_R2;
    impedance_analysis(user_i).K_whislo_fR2_ph1 = qiK_R2(1) - 1.5*ieqK_R2; 
    %% ph2
    n = impedance_analysis(user_i).nb_ph2;
    qiK = quantile(K_user_i(idx_ph(2,:)),3);
    ieqK = qiK(3) - qiK(1);
    nR2 = sum(R2_user_i > 0.5 & idx_ph(2,:));
    qiK_R2 = quantile(K_user_i((R2_user_i>0.5) & idx_ph(2,:)),3);
    ieqK_R2 = qiK_R2(3) - qiK_R2(1);
    %
    impedance_analysis(user_i).K_avg_ph2 = mean(K_user_i(idx_ph(2,:)));
    impedance_analysis(user_i).K_med_ph2 = qiK(2);
    impedance_analysis(user_i).K_med_nhi_ph2 = qiK(2) + 1.57*ieqK/sqrt(n);
    impedance_analysis(user_i).K_med_nlo_ph2 = qiK(2) - 1.57*ieqK/sqrt(n); 
    impedance_analysis(user_i).K_q3_ph2 = qiK(3);
    impedance_analysis(user_i).K_q1_ph2 = qiK(1); 
    impedance_analysis(user_i).K_whishi_ph2 = qiK(3) + 1.5*ieqK;
    impedance_analysis(user_i).K_whislo_ph2 = qiK(1) - 1.5*ieqK;    
    impedance_analysis(user_i).K_avg_fR2_ph2 = mean(K_user_i(R2_user_i>0.5 & idx_ph(2,:)));
    impedance_analysis(user_i).K_med_fR2_ph2 = qiK_R2(2);
    impedance_analysis(user_i).K_med_nhi_fR2_ph2 = qiK_R2(2) + 1.57*ieqK_R2/sqrt(nR2);
    impedance_analysis(user_i).K_med_nlo_fR2_ph2 = qiK_R2(2) - 1.57*ieqK_R2/sqrt(nR2); 
    impedance_analysis(user_i).K_q3_fR2_ph2 = qiK_R2(3);
    impedance_analysis(user_i).K_q1_fR2_ph2 = qiK_R2(1);  
    impedance_analysis(user_i).K_whishi_fR2_ph2 = qiK_R2(3) + 1.5*ieqK_R2;
    impedance_analysis(user_i).K_whislo_fR2_ph2 = qiK_R2(1) - 1.5*ieqK_R2;  
    %% ph3
    n = impedance_analysis(user_i).nb_ph2;
    qiK = quantile(K_user_i(idx_ph(3,:)),3);
    ieqK = qiK(3) - qiK(1);
    nR2 = sum(R2_user_i > 0.5 & idx_ph(3,:));
    qiK_R2 = quantile(K_user_i((R2_user_i>0.5) & idx_ph(3,:)),3);
    ieqK_R2 = qiK_R2(3) - qiK_R2(1);
    %
    impedance_analysis(user_i).K_avg_ph3 = mean(K_user_i(idx_ph(3,:)));
    impedance_analysis(user_i).K_med_ph3 = qiK(2);
    impedance_analysis(user_i).K_med_nhi_ph3 = qiK(2) + 1.57*ieqK/sqrt(n);
    impedance_analysis(user_i).K_med_nlo_ph3 = qiK(2) - 1.57*ieqK/sqrt(n); 
    impedance_analysis(user_i).K_q3_ph3 = qiK(3);
    impedance_analysis(user_i).K_q1_ph3 = qiK(1); 
    impedance_analysis(user_i).K_whishi_ph3 = qiK(3) + 1.5*ieqK;
    impedance_analysis(user_i).K_whislo_ph3 = qiK(1) - 1.5*ieqK;   
    impedance_analysis(user_i).K_avg_fR2_ph3 = mean(K_user_i(R2_user_i>0.5 & idx_ph(3,:)));
    impedance_analysis(user_i).K_med_fR2_ph3 = qiK_R2(2);
    impedance_analysis(user_i).K_med_nhi_fR2_ph3 = qiK_R2(2) + 1.57*ieqK_R2/sqrt(nR2);
    impedance_analysis(user_i).K_med_nlo_fR2_ph3 = qiK_R2(2) - 1.57*ieqK_R2/sqrt(nR2); 
    impedance_analysis(user_i).K_q3_fR2_ph3 = qiK_R2(3);
    impedance_analysis(user_i).K_q1_fR2_ph3 = qiK_R2(1); 
    impedance_analysis(user_i).K_whishi_fR2_ph3 = qiK_R2(3) + 1.5*ieqK_R2;
    impedance_analysis(user_i).K_whislo_fR2_ph3 = qiK_R2(1) - 1.5*ieqK_R2;    
    %% h1
    n = impedance_analysis(user_i).nb_h1;
    qiK = quantile(K_user_i(idx_h(1,:)),3);
    ieqK = qiK(3) - qiK(1);
    nR2 = sum(R2_user_i > 0.5 & idx_h(1,:));
    qiK_R2 = quantile(K_user_i((R2_user_i>0.5) & idx_h(1,:)),3);
    ieqK_R2 = qiK_R2(3) - qiK_R2(1);
    %
    impedance_analysis(user_i).K_avg_h1 = mean(K_user_i(idx_h(1,:)));
    impedance_analysis(user_i).K_med_h1 = qiK(2);
    impedance_analysis(user_i).K_med_nhi_h1 = qiK(2) + 1.57*ieqK/sqrt(n);
    impedance_analysis(user_i).K_med_nlo_h1 = qiK(2) - 1.57*ieqK/sqrt(n);  
    impedance_analysis(user_i).K_q3_h1 = qiK(3);
    impedance_analysis(user_i).K_q1_h1 = qiK(1); 
    impedance_analysis(user_i).K_whishi_h1 = qiK(3) + 1.5*ieqK;
    impedance_analysis(user_i).K_whislo_h1 = qiK(1) - 1.5*ieqK;  
    impedance_analysis(user_i).K_avg_fR2_h1 = mean(K_user_i(R2_user_i>0.5 & idx_h(1,:)));
    impedance_analysis(user_i).K_med_fR2_h1 = qiK_R2(2);
    impedance_analysis(user_i).K_med_nhi_fR2_h1 = qiK_R2(2) + 1.57*ieqK_R2/sqrt(nR2);
    impedance_analysis(user_i).K_med_nlo_fR2_h1 = qiK_R2(2) - 1.57*ieqK_R2/sqrt(nR2);
    impedance_analysis(user_i).K_q3_fR2_h1 = qiK_R2(3);
    impedance_analysis(user_i).K_q1_fR2_h1 = qiK_R2(1); 
    impedance_analysis(user_i).K_whishi_fR2_h1 = qiK_R2(3) + 1.5*ieqK_R2;
    impedance_analysis(user_i).K_whislo_fR2_h1 = qiK_R2(1) - 1.5*ieqK_R2; 
    %% h2
    n = impedance_analysis(user_i).nb_h2;
    qiK = quantile(K_user_i(idx_h(2,:)),3);
    ieqK = qiK(3) - qiK(1);
    nR2 = sum(R2_user_i > 0.5 & idx_h(2,:));
    qiK_R2 = quantile(K_user_i((R2_user_i>0.5) & idx_h(2,:)),3);
    ieqK_R2 = qiK_R2(3) - qiK_R2(1);
    %
    impedance_analysis(user_i).K_avg_h2 = mean(K_user_i(idx_h(2,:)));
    impedance_analysis(user_i).K_med_h2 = qiK(2);
    impedance_analysis(user_i).K_med_nhi_h2 = qiK(2) + 1.57*ieqK/sqrt(n);
    impedance_analysis(user_i).K_med_nlo_h2 = qiK(2) - 1.57*ieqK/sqrt(n); 
    impedance_analysis(user_i).K_q3_h2 = qiK(3);
    impedance_analysis(user_i).K_q1_h2 = qiK(1); 
    impedance_analysis(user_i).K_whishi_h2 = qiK(3) + 1.5*ieqK;
    impedance_analysis(user_i).K_whislo_h2 = qiK(1) - 1.5*ieqK;   
    impedance_analysis(user_i).K_avg_fR2_h2 = mean(K_user_i(R2_user_i>0.5 & idx_h(2,:)));
    impedance_analysis(user_i).K_med_fR2_h2 = qiK_R2(2);
    impedance_analysis(user_i).K_med_nhi_fR2_h2 = qiK_R2(2) + 1.57*ieqK_R2/sqrt(nR2);
    impedance_analysis(user_i).K_med_nlo_fR2_h2 = qiK_R2(2) - 1.57*ieqK_R2/sqrt(nR2);  
    impedance_analysis(user_i).K_q3_fR2_h2 = qiK_R2(3);
    impedance_analysis(user_i).K_q1_fR2_h2 = qiK_R2(1); 
    impedance_analysis(user_i).K_whishi_fR2_h2 = qiK_R2(3) + 1.5*ieqK_R2;
    impedance_analysis(user_i).K_whislo_fR2_h2 = qiK_R2(1) - 1.5*ieqK_R2;  
    %% h3
    n = impedance_analysis(user_i).nb_h3;
    qiK = quantile(K_user_i(idx_h(3,:)),3);
    ieqK = qiK(3) - qiK(1);
    nR2 = sum(R2_user_i > 0.5 & idx_h(3,:));
    qiK_R2 = quantile(K_user_i((R2_user_i>0.5) & idx_h(3,:)),3);
    ieqK_R2 = qiK_R2(3) - qiK_R2(1);
    %
    impedance_analysis(user_i).K_avg_h3 = mean(K_user_i(idx_h(3,:)));
    impedance_analysis(user_i).K_med_h3 = qiK(2);
    impedance_analysis(user_i).K_med_nhi_h3 = qiK(2) + 1.57*ieqK/sqrt(n);
    impedance_analysis(user_i).K_med_nlo_h3 = qiK(2) - 1.57*ieqK/sqrt(n); 
    impedance_analysis(user_i).K_q3_h3 = qiK(3);
    impedance_analysis(user_i).K_q1_h3 = qiK(1);   
    impedance_analysis(user_i).K_whishi_h3 = qiK(3) + 1.5*ieqK;
    impedance_analysis(user_i).K_whislo_h3 = qiK(1) - 1.5*ieqK;   
    impedance_analysis(user_i).K_avg_fR2_h3 = mean(K_user_i(R2_user_i>0.5 & idx_h(3,:)));
    impedance_analysis(user_i).K_med_fR2_h3 = qiK_R2(2);
    impedance_analysis(user_i).K_med_nhi_fR2_h3 = qiK_R2(2) + 1.57*ieqK_R2/sqrt(nR2);
    impedance_analysis(user_i).K_med_nlo_fR2_h3 = qiK_R2(2) - 1.57*ieqK_R2/sqrt(nR2); 
    impedance_analysis(user_i).K_q3_fR2_h3 = qiK_R2(3);
    impedance_analysis(user_i).K_q1_fR2_h3 = qiK_R2(1);     
    impedance_analysis(user_i).K_whishi_fR2_h3 = qiK_R2(3) + 1.5*ieqK_R2;
    impedance_analysis(user_i).K_whislo_fR2_h3 = qiK_R2(1) - 1.5*ieqK_R2;    
    %%%%%%%
    % B
    %%%%%%%
    %% all
    n = impedance_analysis(user_i).nb_id;
    qiB = quantile(B_user_i,3);
    ieqB = qiB(3) - qiB(1);
    nR2 = sum(R2_user_i > 0.5);
    qiB_R2 = quantile(B_user_i(R2_user_i>0.5),3);
    ieqB_R2 = qiB_R2(3) - qiB_R2(1);
    %
    impedance_analysis(user_i).B_avg = mean(B_user_i);
    impedance_analysis(user_i).B_med = qiB(2);
    impedance_analysis(user_i).B_med_nhi = qiB(2) + 1.57*ieqB/sqrt(n);
    impedance_analysis(user_i).B_med_nlo = qiB(2) - 1.57*ieqB/sqrt(n);    
    impedance_analysis(user_i).B_q3 = qiB(3);
    impedance_analysis(user_i).B_q1 = qiB(1);  
    impedance_analysis(user_i).B_whishi = qiB(3) + 1.5*ieqB;
    impedance_analysis(user_i).B_whislo = qiB(1) - 1.5*ieqB;      
    impedance_analysis(user_i).B_avg_fR2 = mean(B_user_i(R2_user_i>0.5));
    impedance_analysis(user_i).B_med_fR2 = qiB_R2(2);
    impedance_analysis(user_i).B_med_nhi_fR2 = qiB_R2(2) + 1.57*ieqB_R2/sqrt(nR2);
    impedance_analysis(user_i).B_med_nlo_fR2 = qiB_R2(2) - 1.57*ieqB_R2/sqrt(nR2);
    impedance_analysis(user_i).B_q3_fR2 = qiB_R2(3);
    impedance_analysis(user_i).B_q1_fR2 = qiB_R2(1); 
    impedance_analysis(user_i).B_whishi_fR2 = qiB_R2(3) + 1.5*ieqB_R2;
    impedance_analysis(user_i).B_whislo_fR2 = qiB_R2(1) - 1.5*ieqB_R2;  
    %% ph1
    n = impedance_analysis(user_i).nb_ph1;
    qiB = quantile(B_user_i(idx_ph(1,:)),3);
    ieqB = qiB(3) - qiB(1);
    nR2 = sum(R2_user_i > 0.5 & idx_ph(1,:));
    qiB_R2 = quantile(B_user_i((R2_user_i>0.5) & idx_ph(1,:)),3);
    ieqB_R2 = qiB_R2(3) - qiB_R2(1);
    %
    impedance_analysis(user_i).B_avg_ph1 = mean(B_user_i(idx_ph(1,:)));
    impedance_analysis(user_i).B_med_ph1 = qiB(2);
    impedance_analysis(user_i).B_med_nhi_ph1 = qiB(2) + 1.57*ieqB/sqrt(n);
    impedance_analysis(user_i).B_med_nlo_ph1 = qiB(2) - 1.57*ieqB/sqrt(n);   
    impedance_analysis(user_i).B_q3_ph1 = qiB(3);
    impedance_analysis(user_i).B_q1_ph1 = qiB(1);  
    impedance_analysis(user_i).B_whishi_ph1 = qiB(3) + 1.5*ieqB;
    impedance_analysis(user_i).B_whislo_ph1 = qiB(1) - 1.5*ieqB;    
    impedance_analysis(user_i).B_avg_fR2_ph1 = mean(B_user_i(R2_user_i>0.5 & idx_ph(1,:)));
    impedance_analysis(user_i).B_med_fR2_ph1 = qiB_R2(2);
    impedance_analysis(user_i).B_med_nhi_fR2_ph1 = qiB_R2(2) + 1.57*ieqB_R2/sqrt(nR2);
    impedance_analysis(user_i).B_med_nlo_fR2_ph1 = qiB_R2(2) - 1.57*ieqB_R2/sqrt(nR2);
    impedance_analysis(user_i).B_q3_fR2_ph1 = qiB_R2(3);
    impedance_analysis(user_i).B_q1_fR2_ph1 = qiB_R2(1); 
    impedance_analysis(user_i).B_whishi_fR2_ph1 = qiB_R2(3) + 1.5*ieqB_R2;
    impedance_analysis(user_i).B_whislo_fR2_ph1 = qiB_R2(1) - 1.5*ieqB_R2; 
    %% ph2
    n = impedance_analysis(user_i).nb_ph2;
    qiB = quantile(B_user_i(idx_ph(2,:)),3);
    ieqB = qiB(3) - qiB(1);
    nR2 = sum(R2_user_i > 0.5 & idx_ph(2,:));
    qiB_R2 = quantile(B_user_i((R2_user_i>0.5) & idx_ph(2,:)),3);
    ieqB_R2 = qiB_R2(3) - qiB_R2(1);
    %
    impedance_analysis(user_i).B_avg_ph2 = mean(B_user_i(idx_ph(2,:)));
    impedance_analysis(user_i).B_med_ph2 = qiB(2);
    impedance_analysis(user_i).B_med_nhi_ph2 = qiB(2) + 1.57*ieqB/sqrt(n);
    impedance_analysis(user_i).B_med_nlo_ph2 = qiB(2) - 1.57*ieqB/sqrt(n);    
    impedance_analysis(user_i).B_q3_ph2 = qiB(3);
    impedance_analysis(user_i).B_q1_ph2 = qiB(1);  
    impedance_analysis(user_i).B_whishi_ph2 = qiB(3) + 1.5*ieqB;
    impedance_analysis(user_i).B_whislo_ph2 = qiB(1) - 1.5*ieqB;   
    impedance_analysis(user_i).B_avg_fR2_ph2 = mean(B_user_i(R2_user_i>0.5 & idx_ph(2,:)));
    impedance_analysis(user_i).B_med_fR2_ph2 = qiB_R2(2);
    impedance_analysis(user_i).B_med_nhi_fR2_ph2 = qiB_R2(2) + 1.57*ieqB_R2/sqrt(nR2);
    impedance_analysis(user_i).B_med_nlo_fR2_ph2 = qiB_R2(2) - 1.57*ieqB_R2/sqrt(nR2);  
    impedance_analysis(user_i).B_q3_fR2_ph2 = qiB_R2(3);
    impedance_analysis(user_i).B_q1_fR2_ph2 = qiB_R2(1);  
    impedance_analysis(user_i).B_whishi_fR2_ph2 = qiB_R2(3) + 1.5*ieqB_R2;
    impedance_analysis(user_i).B_whislo_fR2_ph2 = qiB_R2(1) - 1.5*ieqB_R2; 
    %% ph3
    n = impedance_analysis(user_i).nb_ph2;
    qiB = quantile(B_user_i(idx_ph(3,:)),3);
    ieqB = qiB(3) - qiB(1);
    nR2 = sum(R2_user_i > 0.5 & idx_ph(3,:));
    qiB_R2 = quantile(B_user_i((R2_user_i>0.5) & idx_ph(3,:)),3);
    ieqB_R2 = qiB_R2(3) - qiB_R2(1);
    %
    impedance_analysis(user_i).B_avg_ph3 = mean(B_user_i(idx_ph(3,:)));
    impedance_analysis(user_i).B_med_ph3 = qiB(2);
    impedance_analysis(user_i).B_med_nhi_ph3 = qiB(2) + 1.57*ieqB/sqrt(n);
    impedance_analysis(user_i).B_med_nlo_ph3 = qiB(2) - 1.57*ieqB/sqrt(n);     
    impedance_analysis(user_i).B_q3_ph3 = qiB(3);
    impedance_analysis(user_i).B_q1_ph3 = qiB(1);  
    impedance_analysis(user_i).B_whishi_ph3 = qiB(3) + 1.5*ieqB;
    impedance_analysis(user_i).B_whislo_ph3 = qiB(1) - 1.5*ieqB;   
    impedance_analysis(user_i).B_avg_fR2_ph3 = mean(B_user_i(R2_user_i>0.5 & idx_ph(3,:)));
    impedance_analysis(user_i).B_med_fR2_ph3 = qiB_R2(2);
    impedance_analysis(user_i).B_med_nhi_fR2_ph3 = qiB_R2(2) + 1.57*ieqB_R2/sqrt(nR2);
    impedance_analysis(user_i).B_med_nlo_fR2_ph3 = qiB_R2(2) - 1.57*ieqB_R2/sqrt(nR2);  
    impedance_analysis(user_i).B_q3_fR2_ph3 = qiB_R2(3);
    impedance_analysis(user_i).B_q1_fR2_ph3 = qiB_R2(1); 
    impedance_analysis(user_i).B_whishi_fR2_ph3 = qiB_R2(3) + 1.5*ieqB_R2;
    impedance_analysis(user_i).B_whislo_fR2_ph3 = qiB_R2(1) - 1.5*ieqB_R2;  
    %% h1
    n = impedance_analysis(user_i).nb_h1;
    qiB = quantile(B_user_i(idx_h(1,:)),3);
    ieqB = qiB(3) - qiB(1);
    nR2 = sum(R2_user_i > 0.5 & idx_h(1,:));
    qiB_R2 = quantile(B_user_i((R2_user_i>0.5) & idx_h(1,:)),3);
    ieqB_R2 = qiB_R2(3) - qiB_R2(1);
    %
    impedance_analysis(user_i).B_avg_h1 = mean(B_user_i(idx_h(1,:)));
    impedance_analysis(user_i).B_med_h1 = qiB(2);
    impedance_analysis(user_i).B_med_nhi_h1 = qiB(2) + 1.57*ieqB/sqrt(n);
    impedance_analysis(user_i).B_med_nlo_h1 = qiB(2) - 1.57*ieqB/sqrt(n);     
    impedance_analysis(user_i).B_q3_h1 = qiB(3);
    impedance_analysis(user_i).B_q1_h1 = qiB(1);  
    impedance_analysis(user_i).B_whishi_h1 = qiB(3) + 1.5*ieqB;
    impedance_analysis(user_i).B_whislo_h1 = qiB(1) - 1.5*ieqB;  
    impedance_analysis(user_i).B_avg_fR2_h1 = mean(B_user_i(R2_user_i>0.5 & idx_h(1,:)));
    impedance_analysis(user_i).B_med_fR2_h1 = qiB_R2(2);
    impedance_analysis(user_i).B_med_nhi_fR2_h1 = qiB_R2(2) + 1.57*ieqB_R2/sqrt(nR2);
    impedance_analysis(user_i).B_med_nlo_fR2_h1 = qiB_R2(2) - 1.57*ieqB_R2/sqrt(nR2);
    impedance_analysis(user_i).B_q3_fR2_h1 = qiB_R2(3);
    impedance_analysis(user_i).B_q1_fR2_h1 = qiB_R2(1); 
    impedance_analysis(user_i).B_whishi_fR2_h1 = qiB_R2(3) + 1.5*ieqB_R2;
    impedance_analysis(user_i).B_whislo_fR2_h1 = qiB_R2(1) - 1.5*ieqB_R2; 
    %% h2
    n = impedance_analysis(user_i).nb_h2;
    qiB = quantile(B_user_i(idx_h(2,:)),3);
    ieqB = qiB(3) - qiB(1);
    nR2 = sum(R2_user_i > 0.5 & idx_h(2,:));
    qiB_R2 = quantile(B_user_i((R2_user_i>0.5) & idx_h(2,:)),3);
    ieqB_R2 = qiB_R2(3) - qiB_R2(1);
    %
    impedance_analysis(user_i).B_avg_h2 = mean(B_user_i(idx_h(2,:)));
    impedance_analysis(user_i).B_med_h2 = qiB(2);
    impedance_analysis(user_i).B_med_nhi_h2 = qiB(2) + 1.57*ieqB/sqrt(n);
    impedance_analysis(user_i).B_med_nlo_h2 = qiB(2) - 1.57*ieqB/sqrt(n);   
    impedance_analysis(user_i).B_q3_h2 = qiB(3);
    impedance_analysis(user_i).B_q1_h2 = qiB(1);  
    impedance_analysis(user_i).B_whishi_h2 = qiB(3) + 1.5*ieqB;
    impedance_analysis(user_i).B_whislo_h2 = qiB(1) - 1.5*ieqB;   
    impedance_analysis(user_i).B_avg_fR2_h2 = mean(B_user_i(R2_user_i>0.5 & idx_h(2,:)));
    impedance_analysis(user_i).B_med_fR2_h2 = qiB_R2(2);
    impedance_analysis(user_i).B_med_nhi_fR2_h2 = qiB_R2(2) + 1.57*ieqB_R2/sqrt(nR2);
    impedance_analysis(user_i).B_med_nlo_fR2_h2 = qiB_R2(2) - 1.57*ieqB_R2/sqrt(nR2); 
    impedance_analysis(user_i).B_q3_fR2_h2 = qiB_R2(3);
    impedance_analysis(user_i).B_q1_fR2_h2 = qiB_R2(1); 
    impedance_analysis(user_i).B_whishi_fR2_h2 = qiB_R2(3) + 1.5*ieqB_R2;
    impedance_analysis(user_i).B_whislo_fR2_h2 = qiB_R2(1) - 1.5*ieqB_R2;   
    %% h3
    n = impedance_analysis(user_i).nb_h3;
    qiB = quantile(B_user_i(idx_h(3,:)),3);
    ieqB = qiB(3) - qiB(1);
    nR2 = sum(R2_user_i > 0.5 & idx_h(3,:));
    qiB_R2 = quantile(B_user_i((R2_user_i>0.5) & idx_h(3,:)),3);
    ieqB_R2 = qiB_R2(3) - qiB_R2(1);
    %
    impedance_analysis(user_i).B_avg_h3 = mean(B_user_i(idx_h(3,:)));
    impedance_analysis(user_i).B_med_h3 = qiB(2);
    impedance_analysis(user_i).B_med_nhi_h3 = qiB(2) + 1.57*ieqB/sqrt(n);
    impedance_analysis(user_i).B_med_nlo_h3 = qiB(2) - 1.57*ieqB/sqrt(n);   
    impedance_analysis(user_i).B_q3_h3 = qiB(3);
    impedance_analysis(user_i).B_q1_h3 = qiB(1);  
    impedance_analysis(user_i).B_whishi_h3 = qiB(3) + 1.5*ieqB;
    impedance_analysis(user_i).B_whislo_h3 = qiB(1) - 1.5*ieqB;   
    impedance_analysis(user_i).B_avg_fR2_h3 = mean(B_user_i(R2_user_i>0.5 & idx_h(3,:)));
    impedance_analysis(user_i).B_med_fR2_h3 = qiB_R2(2);
    impedance_analysis(user_i).B_med_nhi_fR2_h3 = qiB_R2(2) + 1.57*ieqB_R2/sqrt(nR2);
    impedance_analysis(user_i).B_med_nlo_fR2_h3 = qiB_R2(2) - 1.57*ieqB_R2/sqrt(nR2); 
    impedance_analysis(user_i).B_q3_fR2_h3 = qiB_R2(3);
    impedance_analysis(user_i).B_q1_fR2_h3 = qiB_R2(1); 
    impedance_analysis(user_i).B_whishi_fR2_h3 = qiB_R2(3) + 1.5*ieqB_R2;
    impedance_analysis(user_i).B_whislo_fR2_h3 = qiB_R2(1) - 1.5*ieqB_R2;    
    %%%%%%%
    % M
    %%%%%%%
    %% all
    n = impedance_analysis(user_i).nb_id;
    qiM = quantile(M_user_i,3);
    ieqM = qiM(3) - qiM(1);
    nR2 = sum(R2_user_i > 0.5);
    qiM_R2 = quantile(M_user_i(R2_user_i>0.5),3);
    ieqM_R2 = qiM_R2(3) - qiM_R2(1);
    %
    impedance_analysis(user_i).M_avg = mean(M_user_i);
    impedance_analysis(user_i).M_med = qiM(2);
    impedance_analysis(user_i).M_med_nhi = qiM(2) + 1.57*ieqM/sqrt(n);
    impedance_analysis(user_i).M_med_nlo = qiM(2) - 1.57*ieqM/sqrt(n);   
    impedance_analysis(user_i).M_q3 = qiM(3);
    impedance_analysis(user_i).M_q1 = qiM(1);  
    impedance_analysis(user_i).M_whishi = qiM(3) + 1.5*ieqM;
    impedance_analysis(user_i).M_whislo = qiM(1) - 1.5*ieqM;      
    impedance_analysis(user_i).M_avg_fR2 = mean(M_user_i(R2_user_i>0.5));
    impedance_analysis(user_i).M_med_fR2 = qiM_R2(2);
    impedance_analysis(user_i).M_med_nhi_fR2 = qiM_R2(2) + 1.57*ieqM_R2/sqrt(nR2);
    impedance_analysis(user_i).M_med_nlo_fR2 = qiM_R2(2) - 1.57*ieqM_R2/sqrt(nR2);
    impedance_analysis(user_i).M_q3_fR2 = qiM_R2(3);
    impedance_analysis(user_i).M_q1_fR2 = qiM_R2(1); 
    impedance_analysis(user_i).M_whishi_fR2 = qiM_R2(3) + 1.5*ieqM_R2;
    impedance_analysis(user_i).M_whislo_fR2 = qiM_R2(1) - 1.5*ieqM_R2; 
    %% ph1
    n = impedance_analysis(user_i).nb_ph1;
    qiM = quantile(M_user_i(idx_ph(1,:)),3);
    ieqM = qiM(3) - qiM(1);
    nR2 = sum(R2_user_i > 0.5 & idx_ph(1,:));
    qiM_R2 = quantile(M_user_i((R2_user_i>0.5) & idx_ph(1,:)),3);
    ieqM_R2 = qiM_R2(3) - qiM_R2(1);
    %
    impedance_analysis(user_i).M_avg_ph1 = mean(M_user_i(idx_ph(1,:)));
    impedance_analysis(user_i).M_med_ph1 = qiM(2);
    impedance_analysis(user_i).M_med_nhi_ph1 = qiM(2) + 1.57*ieqM/sqrt(n);
    impedance_analysis(user_i).M_med_nlo_ph1 = qiM(2) - 1.57*ieqM/sqrt(n);  
    impedance_analysis(user_i).M_q3_ph1 = qiM(3);
    impedance_analysis(user_i).M_q1_ph1 = qiM(1);     
    impedance_analysis(user_i).M_whishi_ph1 = qiM(3) + 1.5*ieqM;
    impedance_analysis(user_i).M_whislo_ph1 = qiM(1) - 1.5*ieqM; 
    impedance_analysis(user_i).M_avg_fR2_ph1 = mean(M_user_i(R2_user_i>0.5 & idx_ph(1,:)));
    impedance_analysis(user_i).M_med_fR2_ph1 = qiM_R2(2);
    impedance_analysis(user_i).M_med_nhi_fR2_ph1 = qiM_R2(2) + 1.57*ieqM_R2/sqrt(nR2);
    impedance_analysis(user_i).M_med_nlo_fR2_ph1 = qiM_R2(2) - 1.57*ieqM_R2/sqrt(nR2);
    impedance_analysis(user_i).M_q3_fR2 = qiM_R2(3);
    impedance_analysis(user_i).M_q1_fR2 = qiM_R2(1); 
    impedance_analysis(user_i).M_whishi_fR2_ph1 = qiM_R2(3) + 1.5*ieqM_R2;
    impedance_analysis(user_i).M_whislo_fR2_ph1 = qiM_R2(1) - 1.5*ieqM_R2; 
    %% ph2
    n = impedance_analysis(user_i).nb_ph2;
    qiM = quantile(M_user_i(idx_ph(2,:)),3);
    ieqM = qiM(3) - qiM(1);
    nR2 = sum(R2_user_i > 0.5 & idx_ph(2,:));
    qiM_R2 = quantile(M_user_i((R2_user_i>0.5) & idx_ph(2,:)),3);
    ieqM_R2 = qiM_R2(3) - qiM_R2(1);
    %
    impedance_analysis(user_i).M_avg_ph2 = mean(M_user_i(idx_ph(2,:)));
    impedance_analysis(user_i).M_med_ph2 = qiM(2);
    impedance_analysis(user_i).M_med_nhi_ph2 = qiM(2) + 1.57*ieqM/sqrt(n);
    impedance_analysis(user_i).M_med_nlo_ph2 = qiM(2) - 1.57*ieqM/sqrt(n);   
    impedance_analysis(user_i).M_q3_ph2 = qiM(3);
    impedance_analysis(user_i).M_q1_ph2 = qiM(1);    
    impedance_analysis(user_i).M_whishi_ph2 = qiM(3) + 1.5*ieqM;
    impedance_analysis(user_i).M_whislo_ph2 = qiM(1) - 1.5*ieqM; 
    impedance_analysis(user_i).M_avg_fR2_ph2 = mean(M_user_i(R2_user_i>0.5 & idx_ph(2,:)));
    impedance_analysis(user_i).M_med_fR2_ph2 = qiM_R2(2);
    impedance_analysis(user_i).M_med_nhi_fR2_ph2 = qiM_R2(2) + 1.57*ieqM_R2/sqrt(nR2);
    impedance_analysis(user_i).M_med_nlo_fR2_ph2 = qiM_R2(2) - 1.57*ieqM_R2/sqrt(nR2); 
    impedance_analysis(user_i).M_q3_fR2_ph2 = qiM_R2(3);
    impedance_analysis(user_i).M_q1_fR2_ph2 = qiM_R2(1); 
    impedance_analysis(user_i).M_whishi_fR2_ph2 = qiM_R2(3) + 1.5*ieqM_R2;
    impedance_analysis(user_i).M_whislo_fR2_ph2 = qiM_R2(1) - 1.5*ieqM_R2;  
    %% ph3
    n = impedance_analysis(user_i).nb_ph2;
    qiM = quantile(M_user_i(idx_ph(3,:)),3);
    ieqM = qiM(3) - qiM(1);
    nR2 = sum(R2_user_i > 0.5 & idx_ph(3,:));
    qiM_R2 = quantile(M_user_i((R2_user_i>0.5) & idx_ph(3,:)),3);
    ieqM_R2 = qiM_R2(3) - qiM_R2(1);
    %
    impedance_analysis(user_i).M_avg_ph3 = mean(M_user_i(idx_ph(3,:)));
    impedance_analysis(user_i).M_med_ph3 = qiM(2);
    impedance_analysis(user_i).M_med_nhi_ph3 = qiM(2) + 1.57*ieqM/sqrt(n);
    impedance_analysis(user_i).M_med_nlo_ph3 = qiM(2) - 1.57*ieqM/sqrt(n); 
    impedance_analysis(user_i).M_q3_ph3 = qiM(3);
    impedance_analysis(user_i).M_q1_ph3 = qiM(1);    
    impedance_analysis(user_i).M_whishi_ph3 = qiM(3) + 1.5*ieqM;
    impedance_analysis(user_i).M_whislo_ph3 = qiM(1) - 1.5*ieqM;   
    impedance_analysis(user_i).M_avg_fR2_ph3 = mean(M_user_i(R2_user_i>0.5 & idx_ph(3,:)));
    impedance_analysis(user_i).M_med_fR2_ph3 = qiM_R2(2);
    impedance_analysis(user_i).M_med_nhi_fR2_ph3 = qiM_R2(2) + 1.57*ieqM_R2/sqrt(nR2);
    impedance_analysis(user_i).M_med_nlo_fR2_ph3 = qiM_R2(2) - 1.57*ieqM_R2/sqrt(nR2);  
    impedance_analysis(user_i).M_q3_fR2_ph3 = qiM_R2(3);
    impedance_analysis(user_i).M_q1_fR2_ph3 = qiM_R2(1); 
    impedance_analysis(user_i).M_whishi_fR2_ph3 = qiM_R2(3) + 1.5*ieqM_R2;
    impedance_analysis(user_i).M_whislo_fR2_ph3 = qiM_R2(1) - 1.5*ieqM_R2;  
    %% h1
    n = impedance_analysis(user_i).nb_h1;
    qiM = quantile(M_user_i(idx_h(1,:)),3);
    ieqM = qiM(3) - qiM(1);
    nR2 = sum(R2_user_i > 0.5 & idx_h(1,:));
    qiM_R2 = quantile(M_user_i((R2_user_i>0.5) & idx_h(1,:)),3);
    ieqM_R2 = qiM_R2(3) - qiM_R2(1);
    %
    impedance_analysis(user_i).M_avg_h1 = mean(M_user_i(idx_h(1,:)));
    impedance_analysis(user_i).M_med_h1 = qiM(2);
    impedance_analysis(user_i).M_med_nhi_h1 = qiM(2) + 1.57*ieqM/sqrt(n);
    impedance_analysis(user_i).M_med_nlo_h1 = qiM(2) - 1.57*ieqM/sqrt(n); 
    impedance_analysis(user_i).M_q3_h1 = qiM(3);
    impedance_analysis(user_i).M_q1_h1 = qiM(1);   
    impedance_analysis(user_i).M_whishi_h1 = qiM(3) + 1.5*ieqM;
    impedance_analysis(user_i).M_whislo_h1 = qiM(1) - 1.5*ieqM;    
    impedance_analysis(user_i).M_avg_fR2_h1 = mean(M_user_i(R2_user_i>0.5 & idx_h(1,:)));
    impedance_analysis(user_i).M_med_fR2_h1 = qiM_R2(2);
    impedance_analysis(user_i).M_med_nhi_fR2_h1 = qiM_R2(2) + 1.57*ieqM_R2/sqrt(nR2);
    impedance_analysis(user_i).M_med_nlo_fR2_h1 = qiM_R2(2) - 1.57*ieqM_R2/sqrt(nR2);
    impedance_analysis(user_i).M_q3_fR2_h1 = qiM_R2(3);
    impedance_analysis(user_i).M_q1_fR2_h1 = qiM_R2(1); 
    impedance_analysis(user_i).M_whishi_fR2_h1 = qiM_R2(3) + 1.5*ieqM_R2;
    impedance_analysis(user_i).M_whislo_fR2_h1 = qiM_R2(1) - 1.5*ieqM_R2; 
    %% h2
    n = impedance_analysis(user_i).nb_h2;
    qiM = quantile(M_user_i(idx_h(2,:)),3);
    ieqM = qiM(3) - qiM(1);
    nR2 = sum(R2_user_i > 0.5 & idx_h(2,:));
    qiM_R2 = quantile(M_user_i((R2_user_i>0.5) & idx_h(2,:)),3);
    ieqM_R2 = qiM_R2(3) - qiM_R2(1);
    %
    impedance_analysis(user_i).M_avg_h2 = mean(M_user_i(idx_h(2,:)));
    impedance_analysis(user_i).M_med_h2 = qiM(2);
    impedance_analysis(user_i).M_med_nhi_h2 = qiM(2) + 1.57*ieqM/sqrt(n);
    impedance_analysis(user_i).M_med_nlo_h2 = qiM(2) - 1.57*ieqM/sqrt(n);  
    impedance_analysis(user_i).M_q3_h2 = qiM(3);
    impedance_analysis(user_i).M_q1_h2 = qiM(1);    
    impedance_analysis(user_i).M_whishi_h2 = qiM(3) + 1.5*ieqM;
    impedance_analysis(user_i).M_whislo_h2 = qiM(1) - 1.5*ieqM;  
    impedance_analysis(user_i).M_avg_fR2_h2 = mean(M_user_i(R2_user_i>0.5 & idx_h(2,:)));
    impedance_analysis(user_i).M_med_fR2_h2 = qiM_R2(2);
    impedance_analysis(user_i).M_med_nhi_fR2_h2 = qiM_R2(2) + 1.57*ieqM_R2/sqrt(nR2);
    impedance_analysis(user_i).M_med_nlo_fR2_h2 = qiM_R2(2) - 1.57*ieqM_R2/sqrt(nR2); 
    impedance_analysis(user_i).M_q3_fR2_h2 = qiM_R2(3);
    impedance_analysis(user_i).M_q1_fR2_h2 = qiM_R2(1);   
    impedance_analysis(user_i).M_whishi_fR2_h2 = qiM_R2(3) + 1.5*ieqM_R2;
    impedance_analysis(user_i).M_whislo_fR2_h2 = qiM_R2(1) - 1.5*ieqM_R2; 
    %% h3
    n = impedance_analysis(user_i).nb_h3;
    qiM = quantile(M_user_i(idx_h(3,:)),3);
    ieqM = qiM(3) - qiM(1);
    nR2 = sum(R2_user_i > 0.5 & idx_h(3,:));
    qiM_R2 = quantile(M_user_i((R2_user_i>0.5) & idx_h(3,:)),3);
    ieqM_R2 = qiM_R2(3) - qiM_R2(1);
    %
    impedance_analysis(user_i).M_avg_h3 = mean(M_user_i(idx_h(3,:)));
    impedance_analysis(user_i).M_med_h3 = qiM(2);
    impedance_analysis(user_i).M_med_nhi_h3 = qiM(2) + 1.57*ieqM/sqrt(n);
    impedance_analysis(user_i).M_med_nlo_h3 = qiM(2) - 1.57*ieqM/sqrt(n);  
    impedance_analysis(user_i).M_q3_h3 = qiM(3);
    impedance_analysis(user_i).M_q1_h3 = qiM(1);    
    impedance_analysis(user_i).M_whishi_h3 = qiM(3) + 1.5*ieqM;
    impedance_analysis(user_i).M_whislo_h3 = qiM(1) - 1.5*ieqM;  
    impedance_analysis(user_i).M_avg_fR2_h3 = mean(M_user_i(R2_user_i>0.5 & idx_h(3,:)));
    impedance_analysis(user_i).M_med_fR2_h3 = qiM_R2(2);
    impedance_analysis(user_i).M_med_nhi_fR2_h3 = qiM_R2(2) + 1.57*ieqM_R2/sqrt(nR2);
    impedance_analysis(user_i).M_med_nlo_fR2_h3 = qiM_R2(2) - 1.57*ieqM_R2/sqrt(nR2);
    impedance_analysis(user_i).M_q3_fR2_h3 = qiM_R2(3);
    impedance_analysis(user_i).M_q1_fR2_h3 = qiM_R2(1); 
    impedance_analysis(user_i).M_whishi_fR2_h3 = qiM_R2(3) + 1.5*ieqM_R2;
    impedance_analysis(user_i).M_whislo_fR_h3 = qiM_R2(1) - 1.5*ieqM_R2; 
    
    clear idx_ph idx_h
end

% K
figure
tiledlayout('flow', 'TileSpacing', 'compact', 'Padding', 'compact');
nexttile
plot([impedance_analysis(idx_exp_s).K_med], 'bs')
hold on
plot([impedance_analysis(idx_exp_s).K_med_nhi], 'k*')
plot([impedance_analysis(idx_exp_s).K_med_nlo], 'k*')
xticks(1:31)
xticklabels([impedance_analysis(idx_exp_s).user])
xtickangle(60)
xline(13.5)
xline(26.5)
title('K all')
nexttile
plot([impedance_analysis(idx_exp_s).K_med_ph1], 'bs')
hold on
plot([impedance_analysis(idx_exp_s).K_med_nhi_ph1], 'k*')
plot([impedance_analysis(idx_exp_s).K_med_nlo_ph1], 'k*')
xticks(1:31)
xticklabels([impedance_analysis(idx_exp_s).user])
xtickangle(60)
xline(13.5)
xline(26.5)
title('K \phi_1')
nexttile
plot([impedance_analysis(idx_exp_s).K_med_ph2], 'bs')
hold on
plot([impedance_analysis(idx_exp_s).K_med_nhi_ph2], 'k*')
plot([impedance_analysis(idx_exp_s).K_med_nlo_ph2], 'k*')
xticks(1:31)
xticklabels([impedance_analysis(idx_exp_s).user])
xtickangle(60)
xline(13.5)
xline(26.5)
title('K \phi_2')
nexttile
plot([impedance_analysis(idx_exp_s).K_med_ph3], 'bs')
hold on
plot([impedance_analysis(idx_exp_s).K_med_nhi_ph3], 'k*')
plot([impedance_analysis(idx_exp_s).K_med_nlo_ph3], 'k*')
xticks(1:31)
xticklabels([impedance_analysis(idx_exp_s).user])
xtickangle(60)
xline(13.5)
xline(26.5)
title('K \phi_3')
nexttile
plot([impedance_analysis(idx_exp_s).K_med_h1], 'bs')
hold on
plot([impedance_analysis(idx_exp_s).K_med_nhi_h1], 'k*')
plot([impedance_analysis(idx_exp_s).K_med_nlo_h1], 'k*')
xticks(1:31)
xticklabels([impedance_analysis(idx_exp_s).user])
xtickangle(60)
xline(13.5)
xline(26.5)
title('K h_1')
nexttile
plot([impedance_analysis(idx_exp_s).K_med_h2], 'bs')
hold on
plot([impedance_analysis(idx_exp_s).K_med_nhi_h2], 'k*')
plot([impedance_analysis(idx_exp_s).K_med_nlo_h2], 'k*')
xticks(1:31)
xticklabels([impedance_analysis(idx_exp_s).user])
xtickangle(60)
xline(13.5)
xline(26.5)
title('K h_2')
nexttile
plot([impedance_analysis(idx_exp_s).K_med_h3], 'bs')
hold on
plot([impedance_analysis(idx_exp_s).K_med_nhi_h3], 'k*')
plot([impedance_analysis(idx_exp_s).K_med_nlo_h3], 'k*')
xticks(1:31)
xticklabels([impedance_analysis(idx_exp_s).user])
xtickangle(60)
xline(13.5)
xline(26.5)
title('K h_3')
% B
figure
tiledlayout('flow', 'TileSpacing', 'compact', 'Padding', 'compact');
nexttile
plot([impedance_analysis(idx_exp_s).B_med], 'bs')
hold on
plot([impedance_analysis(idx_exp_s).B_med_nhi], 'k*')
plot([impedance_analysis(idx_exp_s).B_med_nlo], 'k*')
xticks(1:31)
xticklabels([impedance_analysis(idx_exp_s).user])
xtickangle(60)
xline(13.5)
xline(26.5)
title('B all')
nexttile
plot([impedance_analysis(idx_exp_s).B_med_ph1], 'bs')
hold on
plot([impedance_analysis(idx_exp_s).B_med_nhi_ph1], 'k*')
plot([impedance_analysis(idx_exp_s).B_med_nlo_ph1], 'k*')
xticks(1:31)
xticklabels([impedance_analysis(idx_exp_s).user])
xtickangle(60)
xline(13.5)
xline(26.5)
title('B \phi_1')
nexttile
plot([impedance_analysis(idx_exp_s).B_med_ph2], 'bs')
hold on
plot([impedance_analysis(idx_exp_s).B_med_nhi_ph2], 'k*')
plot([impedance_analysis(idx_exp_s).B_med_nlo_ph2], 'k*')
xticks(1:31)
xticklabels([impedance_analysis(idx_exp_s).user])
xtickangle(60)
xline(13.5)
xline(26.5)
title('B \phi_2')
nexttile
plot([impedance_analysis(idx_exp_s).B_med_ph3], 'bs')
hold on
plot([impedance_analysis(idx_exp_s).B_med_nhi_ph3], 'k*')
plot([impedance_analysis(idx_exp_s).B_med_nlo_ph3], 'k*')
xticks(1:31)
xticklabels([impedance_analysis(idx_exp_s).user])
xtickangle(60)
xline(13.5)
xline(26.5)
title('B \phi_3')
nexttile
plot([impedance_analysis(idx_exp_s).B_med_h1], 'bs')
hold on
plot([impedance_analysis(idx_exp_s).B_med_nhi_h1], 'k*')
plot([impedance_analysis(idx_exp_s).B_med_nlo_h1], 'k*')
xticks(1:31)
xticklabels([impedance_analysis(idx_exp_s).user])
xtickangle(60)
xline(13.5)
xline(26.5)
title('B h_1')
nexttile
plot([impedance_analysis(idx_exp_s).B_med_h2], 'bs')
hold on
plot([impedance_analysis(idx_exp_s).B_med_nhi_h2], 'k*')
plot([impedance_analysis(idx_exp_s).B_med_nlo_h2], 'k*')
xticks(1:31)
xticklabels([impedance_analysis(idx_exp_s).user])
xtickangle(60)
xline(13.5)
xline(26.5)
title('B h_2')
nexttile
plot([impedance_analysis(idx_exp_s).B_med_h3], 'bs')
hold on
plot([impedance_analysis(idx_exp_s).B_med_nhi_h3], 'k*')
plot([impedance_analysis(idx_exp_s).B_med_nlo_h3], 'k*')
xticks(1:31)
xticklabels([impedance_analysis(idx_exp_s).user])
xtickangle(60)
xline(13.5)
xline(26.5)
title('B h_3')
% M
figure
tiledlayout('flow', 'TileSpacing', 'compact', 'Padding', 'compact');
nexttile
plot([impedance_analysis(idx_exp_s).M_med], 'bs')
hold on
plot([impedance_analysis(idx_exp_s).M_med_nhi], 'k*')
plot([impedance_analysis(idx_exp_s).M_med_nlo], 'k*')
xticks(1:31)
xticklabels([impedance_analysis(idx_exp_s).user])
xtickangle(60)
xline(13.5)
xline(26.5)
title('M all')
nexttile
plot([impedance_analysis(idx_exp_s).M_med_ph1], 'bs')
hold on
plot([impedance_analysis(idx_exp_s).M_med_nhi_ph1], 'k*')
plot([impedance_analysis(idx_exp_s).M_med_nlo_ph1], 'k*')
xticks(1:31)
xticklabels([impedance_analysis(idx_exp_s).user])
xtickangle(60)
xline(13.5)
xline(26.5)
title('M \phi_1')
nexttile
plot([impedance_analysis(idx_exp_s).M_med_ph2], 'bs')
hold on
plot([impedance_analysis(idx_exp_s).M_med_nhi_ph2], 'k*')
plot([impedance_analysis(idx_exp_s).M_med_nlo_ph2], 'k*')
xticks(1:31)
xticklabels([impedance_analysis(idx_exp_s).user])
xtickangle(60)
xline(13.5)
xline(26.5)
title('M \phi_2')
nexttile
plot([impedance_analysis(idx_exp_s).M_med_ph3], 'bs')
hold on
plot([impedance_analysis(idx_exp_s).M_med_nhi_ph3], 'k*')
plot([impedance_analysis(idx_exp_s).M_med_nlo_ph3], 'k*')
xticks(1:31)
xticklabels([impedance_analysis(idx_exp_s).user])
xtickangle(60)
xline(13.5)
xline(26.5)
title('M \phi_3')
nexttile
plot([impedance_analysis(idx_exp_s).M_med_h1], 'bs')
hold on
plot([impedance_analysis(idx_exp_s).M_med_nhi_h1], 'k*')
plot([impedance_analysis(idx_exp_s).M_med_nlo_h1], 'k*')
xticks(1:31)
xticklabels([impedance_analysis(idx_exp_s).user])
xtickangle(60)
xline(13.5)
xline(26.5)
title('M h_1')
nexttile
plot([impedance_analysis(idx_exp_s).M_med_h2], 'bs')
hold on
plot([impedance_analysis(idx_exp_s).M_med_nhi_h2], 'k*')
plot([impedance_analysis(idx_exp_s).M_med_nlo_h2], 'k*')
xticks(1:31)
xticklabels([impedance_analysis(idx_exp_s).user])
xtickangle(60)
xline(13.5)
xline(26.5)
title('M h_2')
nexttile
plot([impedance_analysis(idx_exp_s).M_med_h3], 'bs')
hold on
plot([impedance_analysis(idx_exp_s).M_med_nhi_h3], 'k*')
plot([impedance_analysis(idx_exp_s).M_med_nlo_h3], 'k*')
xticks(1:31)
xticklabels([impedance_analysis(idx_exp_s).user])
xtickangle(60)
xline(13.5)
xline(26.5)
title('M h_3')

%%
med = [impedance_analysis.K_med_ph1]';
ln = [impedance_analysis.K_med_nhi_ph1]';
un = [impedance_analysis.K_med_nlo_ph1]';
lq = [impedance_analysis.K_q1_ph1]';
uq = [impedance_analysis.K_q3_ph1]';
uw = [impedance_analysis.K_whishi_ph1]';
lw = [impedance_analysis.K_whislo_ph1]';

% complete_boxplot_data = table(med,ln,un,lq,uq,uw,lw);
% write(complete_boxplot_data,'K_ph1.csv','Delimiter',',');


end

%%%

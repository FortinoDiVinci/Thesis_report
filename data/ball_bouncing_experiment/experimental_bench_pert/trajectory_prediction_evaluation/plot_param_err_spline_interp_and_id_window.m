%%% display and save the parameters errors according to the spline interp
% window and the indentification window
% data computed from test_splines_interp_window_id
%load('test_spline_interp_window_500ms_k_1')
load('test_spline_interp_window_500ms_m_2')

K_prc = prctile(K_all, [25,50,75], 1);
K_med = squeeze(K_prc(2,:,:));
K_med_err = abs(K_med - Kv)./Kv;
K_q_e = squeeze(K_prc(3,:,:) - K_prc(1,:,:));

B_prc = prctile(B_all, [25,50,75], 1);
B_med = squeeze(B_prc(2,:,:));
B_med_err = abs(B_med - Bv)./Bv;
B_q_e = squeeze(B_prc(3,:,:) - B_prc(1,:,:));

M_prc = prctile(M_all, [25,50,75], 1);
M_med = squeeze(M_prc(2,:,:));
M_med_err = abs(M_med - Mv)./Mv;
M_q_e = squeeze(M_prc(3,:,:) - M_prc(1,:,:));

r2_prc = prctile(r2_all, [25,50,75], 1);
r2_med = squeeze(r2_prc(2,:,:));
r2_q_e = squeeze(r2_prc(3,:,:) - r2_prc(1,:,:));

idx_id_wdw = (idx_wndw_imp_eval_min:STEP:idx_wndw_imp_eval_max);
idx_interp_spl = (idx_wndw_virt_traj_min:STEP:idx_wndw_virt_traj_max);

figure
subplot(2,3,1)
surf(idx_interp_spl, idx_id_wdw, K_med_err)
view(2)
colorbar
title('K median relative error')
ylabel('Indentif. window')
subplot(2,3,4)
surf(idx_interp_spl, idx_id_wdw, K_q_e)
view(2)
colorbar
title('K quartile distribution')
xlabel('Spline interp window')
ylabel('Indentif. window')
subplot(2,3,2)
surf(idx_interp_spl, idx_id_wdw, B_med_err)
view(2)
colorbar
title('B median relative error')
ylabel('Indentif. window')
subplot(2,3,5)
surf(idx_interp_spl, idx_id_wdw, B_q_e)
view(2)
colorbar
title('B quartile distribution')
xlabel('Spline interp window')
ylabel('Indentif. window')
subplot(2,3,3)
surf(idx_interp_spl, idx_id_wdw, M_med_err)
view(2)
colorbar
title('M median relative error')
ylabel('Indentif. window')
subplot(2,3,6)
%surf(idx_interp_spl(1:35), idx_id_wdw, M_q_e(:,1:35))
surf(idx_interp_spl, idx_id_wdw, M_q_e)
view(2)
colorbar
title('M quartile distribution')
xlabel('Spline interp window')
ylabel('Indentif. window')

figure
subplot(2,1,1)
surf(idx_interp_spl, idx_id_wdw, r2_med)
view(2)
colorbar
title('R^2 median')
ylabel('Indentif. window')
subplot(2,1,2)
surf(idx_interp_spl, idx_id_wdw, r2_q_e)
view(2)
colorbar
title('R^2 quartile distribution')
xlabel('Spline interp window')
ylabel('Indentif. window')

%% save data to 3 column csv
red_idx = 41;

K_med_err_csv = reshape(K_med_err(:,1:red_idx),[],1);
B_med_err_csv = reshape(B_med_err(:,1:red_idx),[],1);
M_med_err_csv = reshape(M_med_err(:,1:red_idx),[],1);
idx_id_wdw_csv = repmat(idx_id_wdw,1,red_idx)';
idx_interp_spl_csv = reshape(repmat(idx_interp_spl(1:red_idx),size(K_med_err,1),1),[],1);
table_csv = table(idx_id_wdw_csv, idx_interp_spl_csv, K_med_err_csv, B_med_err_csv, M_med_err_csv);
write(table_csv,'interp_and_id_window_param_2.csv','Delimiter',',');

return

%% get minimum errors

clear min_k min_idx
for i = 1:length(idx_id_wdw)
    [min_k(i), min_idx(i)] = min(K_med_err(i,:));
end
    
figure
subplot(2,2,1)
plot(idx_id_wdw, idx_interp_spl(min_idx))
xlabel('Id window')
ylabel('Spline interp window')
subplot(2,2,2)
plot(idx_id_wdw, min_k.*100)
xlabel('Id window')
ylabel('Relative error (%)')

for i = 1:length(idx_interp_spl)
    [min_k(i), min_idx(i)] = min(K_med_err(:,i));
end
    
subplot(2,2,3)
plot(idx_interp_spl, idx_id_wdw(min_idx))
ylabel('Id window')
xlabel('Spline interp window')
subplot(2,2,4)
plot(idx_interp_spl, min_k.*100)
xlabel('Spline interp window')
ylabel('Relative error (%)')

%%
% del negative stiffness and r^2 < 0.5
K_clc = K_all;
B_clc = B_all;
M_clc = M_all;
r2_clc = r2_all;
idx_Kneg = K_all < 0;
idx_r2_low = r2_all < 0.5;
% K_clc(idx_Kneg | idx_r2_low) = NaN;
% r2_clc(idx_Kneg | idx_r2_low) = NaN;
K_clc(idx_r2_low) = NaN;
Bclc(idx_r2_low) = NaN;
M_clc(idx_r2_low) = NaN;
r2_clc(idx_r2_low) = NaN;

K_prc_clc = prctile(K_clc, [25,50,75], 1);
K_med_clc = squeeze(K_prc_clc(2,:,:));
K_med_err_clc = abs(K_med_clc - Kv)./Kv;
K_q_e_clc = squeeze(K_prc_clc(3,:,:) - K_prc_clc(1,:,:));

B_prc_clc = prctile(B_clc, [25,50,75], 1);
B_med_clc = squeeze(B_prc_clc(2,:,:));
B_med_err_clc = abs(B_med_clc - Bv)./Bv;
B_q_e_clc = squeeze(B_prc_clc(3,:,:) - B_prc_clc(1,:,:));

M_prc_clc = prctile(M_clc, [25,50,75], 1);
M_med_clc = squeeze(M_prc_clc(2,:,:));
M_med_err_clc = abs(M_med_clc - Mv)./Mv;
M_q_e_clc = squeeze(M_prc_clc(3,:,:) - M_prc_clc(1,:,:));

r2_prc_clc = prctile(r2_clc, [25,50,75], 1);
r2_med_clc = squeeze(r2_prc_clc(2,:,:));
r2_q_e_clc = squeeze(r2_prc_clc(3,:,:) - r2_prc_clc(1,:,:));

figure
subplot(2,3,1)
surf(idx_interp_spl, idx_id_wdw, K_med_err_clc)
view(2)
colorbar
title('K median relative error (R^2>0.5)')
ylabel('Indentif. window')
subplot(2,3,4)
surf(idx_interp_spl, idx_id_wdw, K_q_e_clc)
view(2)
colorbar
title('K quartile distribution (R^2>0.5)')
xlabel('Spline interp window')
ylabel('Indentif. window')
subplot(2,3,2)
surf(idx_interp_spl, idx_id_wdw, B_med_err_clc)
view(2)
colorbar
title('B median relative error (R^2>0.5)')
subplot(2,3,5)
surf(idx_interp_spl, idx_id_wdw, B_q_e_clc)
view(2)
colorbar
title('B quartile distribution (R^2>0.5)')
xlabel('Spline interp window')
subplot(2,3,3)
surf(idx_interp_spl, idx_id_wdw, M_med_err_clc)
view(2)
colorbar
title('M median relative error (R^2>0.5)')
subplot(2,3,6)
surf(idx_interp_spl, idx_id_wdw, M_q_e_clc)
view(2)
colorbar
title('M quartile distribution (R^2>0.5)')
xlabel('Spline interp window')

K_med_err_csv = reshape(K_med_err_clc(:,1:35),[],1);
B_med_err_csv = reshape(B_med_err_clc(:,1:35),[],1);
M_med_err_csv = reshape(M_med_err_clc(:,1:35),[],1);
table_csv = table(idx_id_wdw_csv, idx_interp_spl_csv, K_med_err_csv, B_med_err_csv, M_med_err_csv);
%write(table_csv,'interp_and_id_window_param_1_r2.csv','Delimiter',',');

%%
% del negative stiffness and r^2 < 0.5
K_clc_2 = K_clc;
K_clc_2(idx_Kneg) = NaN;
for i = 1:size(K_clc_2,2)
    for j = 1:size(K_clc_2,3)
        [~,idx_nan(:,i,j)] = rmoutliers(K_clc_2(:,i,j));
    end
end
K_clc_2(idx_nan) = NaN;

K_prc_clc_2 = prctile(K_clc_2, [25,50,75], 1);
K_med_clc_2 = squeeze(K_prc_clc_2(2,:,:));
K_med_err_clc_2 = abs(K_prc_clc_2 - Kv)./Kv;
K_q_e_clc_2 = squeeze(K_prc_clc_2(3,:,:) - K_prc_clc_2(1,:,:));

K_med_err_sign_clc_2 = (K_med_clc_2 - Kv)./Kv;
K_med_err_sign_clc = (K_med_clc - Kv)./Kv;
K_med_err_sign = (K_med - Kv)./Kv;

nan_count_K_clc = squeeze(sum(isnan(K_clc),1));
nan_count_K_clc_2 = squeeze(sum(isnan(K_clc_2),1));

figure
subplot(3,1,1)
surf(idx_interp_spl, idx_id_wdw, K_med_err_sign)
view(2)
colorbar
subplot(3,1,2)
surf(idx_interp_spl, idx_id_wdw, K_med_err_sign_clc)
view(2)
colorbar
subplot(3,1,3)
surf(idx_interp_spl, idx_id_wdw, K_med_err_sign_clc_2)
view(2)
colorbar
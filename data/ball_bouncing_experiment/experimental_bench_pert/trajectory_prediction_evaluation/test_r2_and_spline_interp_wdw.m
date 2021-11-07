addpath('../utils') % 
addpath('../../../force_torque_sensor') % for force pre-processing
addpath('../../../utils') % 

load('test_spline_interp_window_500ms_1.mat')

acc = cellfun(@(x) prctile(x.r2_pos, [25,50,75])', impedance, 'UniformOutput', false);
quartiles_r2 = cell2mat(acc);

index = (idx_wndw_virt_traj_min:3:idx_wndw_virt_traj_max)';

%quartiles_r2(quartiles_r2 < 0) = -0;

figure
plot(index, quartiles_r2(2,:))
hold on
plot(index, quartiles_r2(1,:), ':', 'Color', lines(1))
plot(index, quartiles_r2(3,:), ':', 'Color', lines(1))

qrtl_R2_2 = quartiles_r2';
%qrtl_R2_1 = quartiles_r2';
interp_window = index;

%tab_r2 = table(interp_window, qrtl_R2_1, qrtl_R2_2);
%write(tab_r2,'spline_window_interp_id_r2.csv','Delimiter',',');

%%
tmp = cellfun(@(x) [x.r2_pos]', impedance, 'UniformOutput', false);
R2_pos = cell2mat(tmp);
tmp = cellfun(@(x) [x.xi(1,:)]', impedance, 'UniformOutput', false);
K_param = cell2mat(tmp);
tmp = cellfun(@(x) [x.xi(2,:)]', impedance, 'UniformOutput', false);
B_param = cell2mat(tmp);
tmp = cellfun(@(x) [x.xi(3,:)]', impedance, 'UniformOutput', false);
M_param = cell2mat(tmp);

K_param_cln = K_param;
K_param_cln(R2_pos < 0.5) = NaN;
B_param_cln = B_param;
B_param_cln(R2_pos < 0.5) = NaN;
M_param_cln = M_param;
M_param_cln(R2_pos < 0.5) = NaN;
K_cln = median(K_param_cln,'omitnan');
K_all = median(K_param);
B_cln = median(B_param_cln,'omitnan');
B_all = median(B_param);
M_cln = median(M_param_cln,'omitnan');
M_all = median(M_param);

figure
subplot(1,3,1)
plot(index, K_all)
hold on
plot(index, K_cln)
plot([index(1),index(end)], [Kv,Kv])
subplot(1,3,2)
plot(index, B_all)
hold on
plot(index, B_cln)
plot([index(1),index(end)], [Bv,Bv])
subplot(1,3,3)
plot(index, M_all)
hold on
plot(index, M_cln)
plot([index(1),index(end)], [Mv,Mv])

K_prc = prctile(K_param,[25,50,75])';
K_prc_cln = prctile(K_param_cln,[25,50,75])';
B_prc = prctile(B_param,[25,50,75])';
B_prc_cln = prctile(B_param_cln,[25,50,75])';
M_prc = prctile(M_param,[25,50,75])';
M_prc_cln = prctile(M_param_cln,[25,50,75])';

tab_r2 = table(interp_window, K_prc, K_prc_cln, B_prc, B_prc_cln, M_prc, M_prc_cln);
write(tab_r2,'spline_inter_wdw_id_r2_filter_1.csv','Delimiter',',');

%% weighted median

R2_pos_nan = R2_pos;
R2_pos_nan(R2_pos < 0.5) = NaN;

for wdw_idx = 1:size(index)
    K_i = K_param_cln(:,wdw_idx);
    B_i = B_param_cln(:,wdw_idx);
    M_i = M_param_cln(:,wdw_idx);
    r2_i = R2_pos(:,wdw_idx);
    r2_i(isnan(K_i)) = [];
    B_i(isnan(K_i)) = [];
    M_i(isnan(K_i)) = [];
    K_i(isnan(K_i)) = [];
    % K
    K_wm(wdw_idx) = wtmedian(K_i, r2_i);
    K_m(wdw_idx) = median(K_i);
    [tmp, i_rm] = rmoutliers(K_param_cln(:,wdw_idx), 'ThresholdFactor', 5);
    K_a_r(wdw_idx) = nanmean(tmp);
    K_wa_r(wdw_idx) = nansum(tmp.*R2_pos_nan(~i_rm,wdw_idx))./nansum(R2_pos_nan(~i_rm,wdw_idx));
    K_wm_rel(wdw_idx) = abs(K_wm(wdw_idx) - Kv)/Kv;
    K_m_rel(wdw_idx) = abs(K_m(wdw_idx) - Kv)/Kv;
    % B
    B_wm(wdw_idx) = wtmedian(B_i, r2_i);
    B_m(wdw_idx) = median(B_i);
    tmp = B_param_cln(~i_rm,wdw_idx);
    B_a_r(wdw_idx) = nanmean(tmp);
    B_wa_r(wdw_idx) = nansum(tmp.*R2_pos_nan(~i_rm,wdw_idx))./nansum(R2_pos_nan(~i_rm,wdw_idx));
    B_wm_rel(wdw_idx) = abs(B_wm(wdw_idx) - Bv)/Bv;
    B_m_rel(wdw_idx) = abs(B_m(wdw_idx) - Bv)/Bv;
    % M
    M_wm(wdw_idx) = wtmedian(M_i, r2_i);
    M_m(wdw_idx) = median(M_i);
    tmp = M_param_cln(~i_rm,wdw_idx);
    M_a_r(wdw_idx) = nanmean(tmp);
    M_wa_r(wdw_idx) = nansum(tmp.*R2_pos_nan(~i_rm,wdw_idx))./nansum(R2_pos_nan(~i_rm,wdw_idx));
    M_wm_rel(wdw_idx) = abs(M_wm(wdw_idx) - Mv)/Mv;
    M_m_rel(wdw_idx) = abs(M_m(wdw_idx) - Mv)/Mv;
end

% K
K_a = nanmean(K_param_cln,1);
K_wa = nansum(K_param_cln.*R2_pos_nan,1)./nansum(R2_pos_nan,1);
K_a_rel = abs(K_a - Kv)./Kv;
K_wa_rel = abs(K_wa - Kv)./Kv;
K_a_r_rel = abs(K_a_r - Kv)./Kv;
K_wa_r_rel = abs(K_wa_r - Kv)./Kv;
% B
B_a = nanmean(B_param_cln,1);
B_wa = nansum(B_param_cln.*R2_pos_nan,1)./nansum(R2_pos_nan,1);
B_a_rel = abs(B_a - Bv)./Bv;
B_wa_rel = abs(B_wa - Bv)./Bv;
B_a_r_rel = abs(B_a_r - Bv)./Bv;
B_wa_r_rel = abs(B_wa_r - Bv)./Bv;
% M
M_a = nanmean(M_param_cln,1);
M_wa = nansum(M_param_cln.*R2_pos_nan,1)./nansum(R2_pos_nan,1);
M_a_rel = abs(M_a - Mv)./Mv;
M_wa_rel = abs(M_wa - Mv)./Mv;
M_a_r_rel = abs(M_a_r - Mv)./Mv;
M_wa_r_rel = abs(M_wa_r - Mv)./Mv;

figure
subplot(1,3,1)
plot(index, K_wm_rel)
hold on
plot(index, K_m_rel)
plot(index, K_a_rel)
plot(index, K_a_r_rel)
plot(index, K_wa_r_rel)
plot(index, K_wa_rel)
legend('weigh. med', 'med', 'avg.', 'avg. outl', 'weigh. avg. outl', 'weigh. avg.')
title('K errors')
subplot(1,3,2)
plot(index, B_wm_rel)
hold on
plot(index, B_m_rel)
plot(index, B_a_rel)
plot(index, B_a_r_rel)
plot(index, B_wa_r_rel)
plot(index, B_wa_rel)
legend('weigh. med', 'med', 'avg.', 'avg. outl', 'weigh. avg. outl', 'weigh. avg.')
title('B errors')
subplot(1,3,3)
plot(index, M_wm_rel)
hold on
plot(index, M_m_rel)
plot(index, M_a_rel)
plot(index, M_a_r_rel)
plot(index, M_wa_r_rel)
plot(index, M_wa_rel)
legend('weigh. med', 'med', 'avg.', 'avg. outl', 'weigh. avg. outl', 'weigh. avg.')
title('M errors')
% mean spline error profile according to spline interpolation
% run test_splines_interp_window_id up until line 132, just to obtain 
% the virtual trajectory estimation on the position

for i = 1:length(delta_z)
    err_pos(:,:,i) = diff_pos - delta_z{i}.diff_traject;
end

% split between positive and negative perturbations
pert_dir = alt_direction(pert_idx);

neg_idx = (pert_dir == -1);

mean_error_neg_prof = squeeze(mean(err_pos(:,neg_idx,:),2));
mean_error_pos_prof = squeeze(mean(err_pos(:,~neg_idx,:),2));
mean_error_prof = squeeze(mean(err_pos,2));

figure
subplot(1,2,1)
surf(idx_samples,1:3:300,mean_error_neg_prof(1:3:300,:))
view(2)
colorbar
subplot(1,2,2)
surf(idx_samples,1:3:300,mean_error_pos_prof(1:3:300,:))
colorbar
view(2)

mean_error_neg = reshape(mean_error_neg_prof(1:3:300,:),[],1);
mean_error_pos = reshape(mean_error_pos_prof(1:3:300,:),[],1);
time = repmat(1:3:300,1,size(mean_error_pos_prof,2))';
spline_interp_window = reshape(repmat(idx_samples',length(1:3:300),1),[],1);
table_csv = table(time, spline_interp_window, mean_error_neg, mean_error_pos);
write(table_csv,'spline_error_profile_param_2.csv','Delimiter',',');

figure
surf(idx_samples,1:300,mean_error_prof)
view(2)
colorbar
function [idx_off_ramp, idx_on_ramp, idx_first_bounce] = offRampIdx(z_b, dt)

vz_b = zeros(size(z_b));
for i = 2:length(z_b) - 1
    vz_b(i) = ( z_b(i + 1) - z_b(i - 1) ) / (2*dt);
end
vz_b(1) = vz_b(2);
vz_b(end) = vz_b(end-1);

filt_order = 100;
fc = 50; % Hz
fir_filter = fir1(filt_order, (fc/(1/dt/2)), 'low');
vz_b_filt = filtfilt(fir_filter, 1, vz_b);

[~, idx_outl] = rmoutliers(vz_b);
%[~, idx_outl] = rmoutliers(vz_b_filt);

idx_on_ramp = find(idx_outl, 1, 'last') + 1;
idx_first_bounce = find(vz_b(idx_on_ramp:end) > 0, 1, 'first');

TF = ischange(vz_b(idx_on_ramp:idx_on_ramp + idx_first_bounce - 20), 'linear');
idx_off_ramp = find(TF, 1, 'last') + idx_on_ramp;

% figure
% hold on
% plot(z_b)
% plot(idx_on_ramp, z_b(idx_on_ramp), 'p')
% plot(idx_on_ramp+idx_first_bounce, z_b(idx_on_ramp+idx_first_bounce), 'p')
% plot(idx_off_ramp, z_b(idx_off_ramp), 'o')

end


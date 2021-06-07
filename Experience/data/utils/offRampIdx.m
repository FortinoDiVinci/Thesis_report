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

% since the ball is "teleported" when the experiment starts, the velocity
% just before being on the ramp should be huge compare to nominal
% velocities
[~, idx_outl] = rmoutliers(vz_b, 'ThresholdFactor', 5);
%[~, idx_outl] = rmoutliers(vz_b_filt);

idx_on_ramp = find(idx_outl, 1, 'last');
% when the ball finally ends up on the ramp, the velocity should be
% negative
i = 0;
while vz_b(idx_on_ramp + i) > 0
    i = i + 1;
    if i >= 50
        error('Algorithm failed to detect the ball on the ramp.')
    end
end
idx_on_ramp = idx_on_ramp + i;

% the first time the velocity should be positive, is right after the first
% bounce
idx_first_bounce = find(vz_b(idx_on_ramp:end) > 0, 1, 'first');

% The velocity variation (acceleration) should be different from ramp fall
% and free fall
%TF = ischange(vz_b(idx_on_ramp:idx_on_ramp + idx_first_bounce - 20), 'linear');
az_b = zeros(size(z_b));
for i = 2:length(z_b) - 1
    az_b(i) = ( vz_b(i + 1) - vz_b(i - 1) ) / (2*dt);
end
az_b(1) = az_b(2);
az_b(end) = az_b(end-1);
[~,idx_off_ramp] = max(az_b(idx_on_ramp+1:idx_on_ramp + idx_first_bounce - 20));
idx_off_ramp = idx_off_ramp + idx_on_ramp + 1;
%idx_off_ramp = find(TF, 1, 'last') + idx_on_ramp;

%% Debug
% figure
% hold on
% plot(z_b)
% plot(idx_on_ramp, z_b(idx_on_ramp), 'p')
% plot(idx_on_ramp+idx_first_bounce, z_b(idx_on_ramp+idx_first_bounce), 'p')
% plot(idx_off_ramp, z_b(idx_off_ramp), 'o')

end


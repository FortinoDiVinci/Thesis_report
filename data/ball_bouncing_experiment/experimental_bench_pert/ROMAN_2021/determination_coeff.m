% run impedance live script (mlx)

tmp = [data_sine{:}];
rec_tmp = [rec_pos_arx{:}];

r2_sine = 1 - sum((rec_tmp.OutputData - tmp.OutputData).^2)./...
    sum((mean(tmp.OutputData) - tmp.OutputData).^2);

[r2_clc, idx_clc] = rmoutliers(r2_sine, 'ThresholdFactor', 5);

n = 200;
p = 3; % impedance param alone ?
r2_adj = 1 - (1 - r2_clc)*(n-1)/(n-p-1);

mean(r2_sine(~idx_clc))
mean(r2_sine)

mean(K_sine_id(~idx_clc))
mean(B_sine_id(~idx_clc))
mean(M_sine_id(~idx_clc))

mean(rmoutliers(K_sine_id, 'ThresholdFactor', 3))
mean(rmoutliers(B_sine_id, 'ThresholdFactor', 3))
mean(rmoutliers(M_sine_id, 'ThresholdFactor', 3))

figure
plot(r2_sine)

SSE = sum((delta_fz_sine.virt_traject(102:end-2,:)-delta_fz_sim.virt_traject(102:end-2,:)).^2);

[r, p] = corrcoef(SSE,r2_sine);

figure
scatter(SSE,r2_sine);

load('force_traject_comparison_against_delay.mat')

for ii = 1:length(data)
    [~,outliers] = rmoutliers(data(ii).arx.nrmse_p);
    tmp = data(ii).arx.nrmse_p;
    data(ii).arx.nrmse_p = [];
    data(ii).arx.nrmse_p.val = tmp;
    data(ii).arx.nrmse_p.avg = nanmean(data(ii).arx.nrmse_p.val(not(outliers)));
    data(ii).arx.nrmse_p.std = nanstd(data(ii).arx.nrmse_p.val(not(outliers)));   
    %
    [~,outliers] = rmoutliers(data(ii).arxS.nrmse_p);
    tmp = data(ii).arxS.nrmse_p;
    data(ii).arxS.nrmse_p = [];
    data(ii).arxS.nrmse_p.val = tmp;
    data(ii).arxS.nrmse_p.avg = nanmean(data(ii).arxS.nrmse_p.val(not(outliers)));
    data(ii).arxS.nrmse_p.std = nanstd(data(ii).arxS.nrmse_p.val(not(outliers)));   
    %
    [~,outliers] = rmoutliers(data(ii).arxSP.nrmse_p);
    tmp = data(ii).arxSP.nrmse_p;
    data(ii).arxSP.nrmse_p = [];
    data(ii).arxSP.nrmse_p.val = tmp;
    data(ii).arxSP.nrmse_p.avg = nanmean(data(ii).arxSP.nrmse_p.val(not(outliers)));
    data(ii).arxSP.nrmse_p.std = nanstd(data(ii).arxSP.nrmse_p.val(not(outliers)));   
    %
    [~,outliers] = rmoutliers(data(ii).arxSM.nrmse_p);
    tmp = data(ii).arxSM.nrmse_p;
    data(ii).arxSM.nrmse_p = [];
    data(ii).arxSM.nrmse_p.val = tmp;
    data(ii).arxSM.nrmse_p.avg = nanmean(data(ii).arxSM.nrmse_p.val(not(outliers)));
    data(ii).arxSM.nrmse_p.std = nanstd(data(ii).arxSM.nrmse_p.val(not(outliers))); 
end

arx_data = [data(:).arx];
arxS_data = [data(:).arxS];
arxSP_data = [data(:).arxSP];
arxSM_data = [data(:).arxSM];

delays = (0:length(data)-1);

% Display fit against delay for each of the force virtual estimation method
figure('DefaultAxesFontSize',14)
subplot(2,1,1)
title('Position reconstruction fit, according to virtual method and delay')
hold on
tmp_nrmse = [arx_data.nrmse_p];
[max_avg, idx_max] = max([tmp_nrmse.avg]);
plotStdSurface([tmp_nrmse.avg]', [tmp_nrmse.std]', delays, [0, 0.4470, 0.7410])
p0 = plot(delays, [tmp_nrmse.avg], 'Linewidth', 1.5, 'Color', [0, 0.4470, 0.7410]);
plot(idx_max-1, max_avg, 'p', 'Color', [0, 0.4470, 0.7410], 'Markersize', 15) 
tmp_nrmse = [arxS_data.nrmse_p];
[max_avg, idx_max] = max([tmp_nrmse.avg]);
plotStdSurface([tmp_nrmse.avg]', [tmp_nrmse.std]', delays, [0.8500, 0.3250, 0.0980])
p1 = plot(delays, [tmp_nrmse.avg], 'Linewidth', 1.5, 'Color', [0.8500, 0.3250, 0.0980]);
plot(idx_max-1, max_avg, 'p', 'Color', [0.8500, 0.3250, 0.0980], 'Markersize', 15) 
legend([p0, p1], {'Filter+','Sine'})
ylabel('Fit')
xlabel('Delay (ms)')
subplot(2,1,2)
hold on
tmp_nrmse = [arx_data.nrmse_p];
[max_avg, idx_max] = max([tmp_nrmse.avg]);
plotStdSurface([tmp_nrmse.avg]', [tmp_nrmse.std]', delays, [0, 0.4470, 0.7410])
p0 = plot(delays, [tmp_nrmse.avg], 'Linewidth', 1.5, 'Color', [0, 0.4470, 0.7410]);
plot(idx_max-1, max_avg, 'p', 'Color', [0, 0.4470, 0.7410], 'Markersize', 15) 
tmp_nrmse = [arxSP_data.nrmse_p];
[max_avg, idx_max] = max([tmp_nrmse.avg]);
plotStdSurface([tmp_nrmse.avg]', [tmp_nrmse.std]', delays, [0.8500, 0.3250, 0.0980])
p1 = plot(delays, [tmp_nrmse.avg], 'Linewidth', 1.5, 'Color', [0.8500, 0.3250, 0.0980]);
plot(idx_max-1, max_avg, 'p', 'Color', [0.8500, 0.3250, 0.0980], 'Markersize', 15) 
tmp_nrmse = [arxSM_data.nrmse_p];
[max_avg, idx_max] = max([tmp_nrmse.avg]);
plotStdSurface([tmp_nrmse.avg]', [tmp_nrmse.std]', delays, [0.4660, 0.6740, 0.1880])
p2 = plot(delays, [tmp_nrmse.avg], 'Linewidth', 1.5, 'Color', [0.4660, 0.6740, 0.1880]);
plot(idx_max-1, max_avg, 'p', 'Color', [0.4660, 0.6740, 0.1880], 'Markersize', 15) 
legend([p0, p1, p2], {'Filter+','Sine+', 'Sine M'})
ylabel('Fit')
xlabel('Delay (ms)')

% Evolution of the sine parameters for each of the optimisation
idx_d = 12+1;
figure('DefaultAxesFontSize',14)
for i = 1:12
    subplot(5,3,i)
    hold on
    plot(data(idx_d).arxSP.opt_param_s(i,:))
    plot(data(idx_d).arxSM.opt_param_s(i,:))
    if mod(i,3) == 1
        title("A_" + string(floor((i-1)/3)))
    elseif mod(i,3) == 2
        title("f_" + string(floor((i-1)/3)))
    else
        title("\phi_" + string(floor((i-1)/3)))
    end
end
subplot(5,2,9)
hold on
plot(data(idx_d).arxSP.opt_param_s(13,:))
plot(data(idx_d).arxSM.opt_param_s(13,:))
title("a")
subplot(5,2,10)
hold on
plot(data(idx_d).arxSP.opt_param_s(14,:))
plot(data(idx_d).arxSM.opt_param_s(14,:))
title("b")
legend('Sine +','Sine M')
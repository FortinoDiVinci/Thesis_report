% run impedance_identification.mlx before running this script

% sort virtual trajectories according to the RMSE score
[rmse_s, idx_s] = sort(rms(delta_fz_opt.virt_traject-delta_fz_sim.virt_traject));


figure('DefaultAxesFontSize',14)
% best trajectory estimations
for i = 1:3
    subplot(3,3,i)
    hold on
    plot(delta_fz_opt.t_traject(:,idx_s(i)), ...
        delta_fz_opt.traject(:,idx_s(i)))
    plot(delta_fz_opt.t_traject(:,idx_s(i)), ...
        delta_fz_opt.virt_traject(:,idx_s(i)))
    title("Best estimations, id: " + string(idx_s(i)))
end
% worst trajectory estimations
for i = 0:2
    subplot(3,3,4+i)
    hold on
    plot(delta_fz_opt.t_traject(:,idx_s(end-i)), ...
        delta_fz_opt.traject(:,idx_s(end-i)))
    plot(delta_fz_opt.t_traject(:,idx_s(end-i)), ...
        delta_fz_opt.virt_traject(:,idx_s(end-i)))
    title("Worst estimations, id: " + string(idx_s(end-i)))
end
% whole estimations
subplot(3,1,3)
hold on
plot(delta_fz_opt.time, delta_fz_opt.complete_traject)
plot(delta_fz_opt.t_traject, delta_fz_opt.virt_traject, ...
    'Color', [0.8500, 0.3250, 0.0980])
plot(delta_fz_opt.t_traject, delta_fz_opt.traject, '--',...
    'Color', [0.9290, 0.6940, 0.1250])
title('Complete trajectories')
legend('Input force', 'virtual estimation')
ylabel('Force (N)')
xlabel('Time (s)')

%%
err_filterPlus = delta_fz.virt_traject-delta_fz_sim.virt_traject;
err_sineOpt1 = delta_fz_opt.virt_traject(:,flg==1)-delta_fz_sim.virt_traject(:,flg==1);
err_sineOpt0 = delta_fz_opt.virt_traject(:,flg==0)-delta_fz_sim.virt_traject(:,flg==0);
err_sineOpt2 = delta_fz_opt.virt_traject(:,flg==2)-delta_fz_sim.virt_traject(:,flg==2);
err_sineOpt5 = delta_fz_opt.virt_traject(:,flg==5)-delta_fz_sim.virt_traject(:,flg==5);

flg = delta_fz_opt.exit_flag;
figure('DefaultAxesFontSize',14)
subplot(2,1,1)
hold on
p1 = plot(err_filterPlus, 'Color', [0.3010, 0.7450, 0.9330, 0.3]);
p2 = plot(err_sineOpt1, 'Color', [0.4660, 0.6740, 0.1880, 0.8]);
p3 = plot(err_sineOpt0, 'Color', [0.6350, 0.0780, 0.1840, 0.8]);
p4 = plot(err_sineOpt2, 'Color', [0.4940, 0.1840, 0.5560, 0.8]);
p5 = plot(err_sineOpt5, 'Color', [0, 0.4470, 0.7410, 0.8]);
title('Force virtual trajectories errors')
legend('filterPlus','sineOpt 1','sineOpt 0','sineOpt 2','sineOpt 5')
xlabel('Time (s)')
ylabel('Error (N)')
xlim([0, idx_wndw_imp_eval])
legend([p1(1),p2(1),p3(1),p4(1),p5(1)], {'filterPlus', 'sineOpt 1', 'sineOpt 0', 'sineOpt 2', 'sineOpt 5'})
subplot(2,3,4)
hold on
histogram(err_filterPlus,'Normalization','probability', 'FaceColor', ...
    [0.3010, 0.7450, 0.9330],'BinWidth',0.1)
histogram(err_sineOpt1,'Normalization', 'probability', 'FaceColor', ...
    [0.4660, 0.6740, 0.1880],'BinWidth',0.1)
title("F+ vs Sine Opt 1 (" + string(size(err_sineOpt1,2)) + ")")
subplot(2,3,5)
hold on
histogram(err_filterPlus,'Normalization', 'probability', 'FaceColor', ...
    [0.3010, 0.7450, 0.9330],'BinWidth',0.1)
histogram(err_sineOpt0,'Normalization', 'probability', 'FaceColor', ...
    [0.6350, 0.0780, 0.1840],'BinWidth',0.1)
title("F+ vs Sine Opt 0 (" + string(size(err_sineOpt0,2)) + ")")
subplot(2,3,6)
hold on
histogram(err_filterPlus,'Normalization', 'probability', 'FaceColor', ...
    [0.3010, 0.7450, 0.9330],'BinWidth',0.1)
histogram(err_sineOpt5, 'Normalization', 'probability', 'FaceColor', ...
    [0, 0.4470, 0.7410],'BinWidth',0.1)
title("F+ vs Sine Opt 5 (" + string(size(err_sineOpt5,2)) + ")")

disp('filterPlus gaussian properties')
sigfp = std(err_filterPlus(:))
avgfp = mean(err_filterPlus(:))
disp('sineOpt 1')
sigfp = std(err_sineOpt1(:))
avgfp = mean(err_sineOpt1(:))
disp('sineOpt 0')
sigfp = std(err_sineOpt0(:))
avgfp = mean(err_sineOpt0(:))
disp('sineOpt 2')
sigfp = std(err_sineOpt2(:))
avgfp = mean(err_sineOpt2(:))
disp('sineOpt 5')
sigfp = std(err_sineOpt5(:))
avgfp = mean(err_sineOpt5(:))
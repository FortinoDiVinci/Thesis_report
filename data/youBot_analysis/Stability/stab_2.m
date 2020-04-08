stability_mapping_stiff = ...
    [0.015, 0.08, 1;
     0.030, 0.08, 0;
     0.022, 0.08, 0;
     0.026, 0.08, 0;
     0.024, 0.08, 0;
     0.023, 0.08, 0;
     0.019, 0.08, 0;
     0.017, 0.08, 0.5;
     0.016, 0.08, 0.75;
     0.010, 0.08, 1;
     0.013, 0.08, 0.75;
     0.015, 0.05, 1;
     0.030, 0.05, 0;
     0.022, 0.05, 0;
     0.019, 0.05, 0;
     0.017, 0.05, 0.5;
     0.016, 0.05, 0.75;
     0.030, 0.002, 0;
     0.022, 0.002, 0;
     0.019, 0.002, 0.5;
     0.017, 0.002, 0.75;
     0.016, 0.002, 0.75;
     0.015, 0.002, 1;
     0.013, 0.002, 1;
     0.020, 0.005, 0;
     0.017, 0.005, 0.75;
     0.015, 0.005, 1;
     0.013, 0.005, 1;
     0.03, 0.0, 0;
     0.02, 0.0, 0;
     0.017, 0.0, 0.75;
     0.015, 0.0, 1;
     0.019, 0.0, 0.5;
     0.020, 0.1, 0;
     0.019, 0.1, 0;
     0.018, 0.1, 0;
     0.015, 0.1, 0.5;
     0.013, 0.1, 0.75;
     0.01, 0.1, 1;
     0.019, 0.2, 0;
     0.015, 0.2, 0;
     0.010, 0.2, 1;
     0.013, 0.2, 0.75;
     0.011, 0.2, 1;
     0.014, 0.2, 0.5;
     0.015, 0.3, 0;
     0.010, 0.3, 0;
     0.005, 0.3, 0;
     0.001, 0.3, 0.5;
     0.0, 0.3, 0.5;
     0.0, 0.25, 0.75;
     0.0, 0.2, 0.75;
     0.0, 0.1, 1;
     0.0, 0.15, 1;
     0.0, 0.13, 1;
     0.015, 0.13, 0.75;
     0.010, 0.13, 1;
     0.015, 0.12, 0.75;
     0.012, 0.12, 1;
     0.015, 0.15, 0.75;
     0.015, 0.17, 0.5;
     0.012, 0.17, 0.75;
     0.012, 0.15, 1;
     0.015, 0.19, 0.5;
     0.019, 0.15, 0;
     0.018, 0.13, 0.5;
     0.010, 0.25, 0.75;
     0.013, 0.25, 0.75;
     0.014, 0.25, 0.5;
     0.010, 0.27, 0.75;
     0.010, 0.23, 0.75;];
%  figure()
%  scatter(stability_mapping_stiff(:,1), stability_mapping_stiff(:,2), [], stability_mapping_stiff(:,3))
%  grid on;
%  xlabel('P gain')
%  ylabel('I gain')
%  title('Stability mapping')
%  
 
 stability_mapping_loose = ...
     [0.015, 0.08, 1;
      0.020, 0.08, 0.75;
      0.017, 0.08, 1;
      0.015, 0.15, 1;
      0.015, 0.20, 1;
      0.015, 0.25, 0.75;
      0.015, 0.30, 0.75;
      0.015, 0.35, 0.5;
      0.015, 0.40, 0.5;
      0.015, 0.50, 0.5;
      0.015, 0.60, 0;
      0.025, 0.08, 0.75;
      0.040, 0.08, 0.5;
      0.020, 0.15, 0.75;
      0.017, 0.20, 0.75;
      0.000, 0.30, 0.75;
      0.000, 0.25, 0.75;
      0.000, 0.20, 1;
      0.030, 0.00, 0.5;
      0.025, 0.00, 0.75;
      0.020, 0.00, 1;
      0.010, 0.30, 0.75;
      0.005, 0.40, 0.75;
      0.000, 0.50, 0.5;
      0.010, 0.20, 1;
      0.020, 0.05, 1;
      0.025, 0.20, 0.5;
      0.0, 0.0, 1.25;
 
     ];
 
 hold on, grid on
 scatter(stability_mapping_loose(:,1), stability_mapping_loose(:,2), [], stability_mapping_loose(:,3), '^')
 grid on;
 xlabel('P gain')
 ylabel('I gain')
 title('Stability mapping')
 
colorbar
colormap hot

figure()

hold on, grid on,
stable_idx = find(stability_mapping_stiff(:,3) == 1);
unconf_idx = find(stability_mapping_stiff(:,3) == 0.75);
critical_idx = find(stability_mapping_stiff(:,3) == 0.5);
unstable_idx = find(stability_mapping_stiff(:,3) == 0);
plot(stability_mapping_stiff(stable_idx, 1), stability_mapping_stiff(stable_idx, 2), '.', 'MarkerFaceColor', [0/255,153/255,0/255], 'MarkerEdgeColor', [0,0,0], 'MarkerSize', 15)
plot(stability_mapping_stiff(unconf_idx, 1), stability_mapping_stiff(unconf_idx, 2), '.', 'MarkerFaceColor', [204/255,204/255,0/255], 'MarkerEdgeColor', [0,0,0], 'MarkerSize', 15)
plot(stability_mapping_stiff(critical_idx, 1), stability_mapping_stiff(critical_idx, 2), '.', 'MarkerFaceColor', [255/255,128/255,0/255], 'MarkerEdgeColor', [0,0,0], 'MarkerSize', 15)
plot(stability_mapping_stiff(unstable_idx, 1), stability_mapping_stiff(unstable_idx, 2), '.', 'MarkerFaceColor', [204/255,0/255,0/255], 'MarkerEdgeColor', [0,0,0], 'MarkerSize', 15)

xlabel('K_p', 'FontSize', 20);
ylabel('K_i', 'FontSize', 20);
set(get(gca,'ylabel'),'rotation',0)
set(gca,'FontSize', 24)
xlim([0 0.0225])
ylim([0 0.32])


%%

f = figure();
hold on, grid on,
stable_idx = find(stability_mapping_stiff(:,3) == 1);
unconf_idx = find(stability_mapping_stiff(:,3) == 0.75);
critical_idx = find(stability_mapping_stiff(:,3) == 0.5);
unstable_idx = find(stability_mapping_stiff(:,3) == 0);
plot(stability_mapping_stiff(stable_idx, 1), stability_mapping_stiff(stable_idx, 2), 'o', 'MarkerEdgeColor', [0/255,153/255,0/255], 'MarkerSize', 15, 'LineWidth',3)
plot(stability_mapping_stiff(unconf_idx, 1), stability_mapping_stiff(unconf_idx, 2), 'o', 'MarkerEdgeColor', [204/255,204/255,0/255], 'MarkerSize', 15, 'LineWidth',3)
plot(stability_mapping_stiff(critical_idx, 1), stability_mapping_stiff(critical_idx, 2), 'o', 'MarkerEdgeColor', [255/255,128/255,0/255], 'MarkerSize', 15, 'LineWidth',3)
plot(stability_mapping_stiff(unstable_idx, 1), stability_mapping_stiff(unstable_idx, 2), 'o', 'MarkerEdgeColor', [204/255,0/255,0/255], 'MarkerSize', 15, 'LineWidth',3)
stable_idx = find(stability_mapping_loose(:,3) == 1);
unconf_idx = find(stability_mapping_loose(:,3) == 0.75);
critical_idx = find(stability_mapping_loose(:,3) == 0.5);
unstable_idx = find(stability_mapping_loose(:,3) == 0);
plot(stability_mapping_loose(stable_idx, 1), stability_mapping_loose(stable_idx, 2), 'o', 'MarkerFaceColor', [0/255,153/255,0/255], 'MarkerEdgeColor', [0/255,153/255,0/255], 'MarkerSize', 8)
plot(stability_mapping_loose(unconf_idx, 1), stability_mapping_loose(unconf_idx, 2), 'o', 'MarkerFaceColor', [204/255,204/255,0/255], 'MarkerEdgeColor', [204/255,204/255,0/255], 'MarkerSize', 8)
plot(stability_mapping_loose(critical_idx, 1), stability_mapping_loose(critical_idx, 2), 'o', 'MarkerFaceColor', [255/255,128/255,0/255], 'MarkerEdgeColor', [255/255,128/255,0/255], 'MarkerSize', 8)
plot(stability_mapping_loose(unstable_idx, 1), stability_mapping_loose(unstable_idx, 2), 'o', 'MarkerFaceColor', [204/255,0/255,0/255], 'MarkerEdgeColor', [204/255,0/255,0/255], 'MarkerSize', 8)
plot(0.015, 0.08, 'p', 'MarkerFaceColor', [0,0,0], 'MarkerEdgeColor', [0/255,153/255,0/255], 'MarkerSize', 15)
%xlabel('K_p', 'FontSize', 20);
%ylabel('K_i', 'FontSize', 20);
set(get(gca,'ylabel'))
set(gca,'FontSize', 24)
xlim([0 0.025])
ylim([0 0.4])


%%

stability_mapping_stiff_stable = stability_mapping_stiff(stability_mapping_stiff(:,3)==1,1:2);
stability_mapping_stiff_bad_damp = stability_mapping_stiff(stability_mapping_stiff(:,3)==0.75,1:2);
stability_mapping_stiff_critic = stability_mapping_stiff(stability_mapping_stiff(:,3)==0.5,1:2);
stability_mapping_stiff_unstable = stability_mapping_stiff(stability_mapping_stiff(:,3)==0,1:2);

stability_mapping_loose_stable = stability_mapping_loose(stability_mapping_loose(:,3)==1,1:2);

dlmwrite('stability_mapping_stiff_stable.csv', stability_mapping_stiff_stable,'delimiter',',','precision',5);
dlmwrite('stability_mapping_stiff_uncomf.csv', stability_mapping_stiff_bad_damp,'delimiter',',','precision',5);
dlmwrite('stability_mapping_stiff_critic.csv', stability_mapping_stiff_critic,'delimiter',',','precision',5);
dlmwrite('stability_mapping_stiff_unstable.csv', stability_mapping_stiff_unstable,'delimiter',',','precision',5);

dlmwrite('stability_mapping_loose_stable.csv', stability_mapping_loose_stable,'delimiter',',','precision',5);
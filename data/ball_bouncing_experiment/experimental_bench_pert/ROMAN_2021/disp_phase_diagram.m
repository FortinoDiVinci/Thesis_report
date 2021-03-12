function disp_phase_diagram(cycles, traject)


tmp = cellfun(@(x) [x.type_dist], cycles, 'UniformOutput', 0);
tmp2 = cellfun(@(x) x(x ~= 0), tmp, 'UniformOutput', 0);
pert_classification = [tmp2{:}];



for i = 1:length(traject)
    traject(i).d_position = Iu_diffcent(traject(i).position, traject(i).time);
    traject(i).d_force = Iu_diffcent(traject(i).force, traject(i).time);
end

figure('DefaultAxesFontSize',16)
% position phase diagram
subplot(1,2,1)
hold on
plot(vertcat(traject.position), vertcat(traject.d_position), 'Color', [0, 0.4470, 0.7410, 0.1])
for i = 1:length(traject)
    idx_pert1 = [cycles{i}([cycles{i}.type_dist] == 1).glob_ind];
    idx_pert2 = [cycles{i}([cycles{i}.type_dist] == 2).glob_ind];
    idx_pert3 = [cycles{i}([cycles{i}.type_dist] == 3).glob_ind];
    p1 = scatter(traject(i).position(idx_pert1), traject(i).d_position(idx_pert1), ...
        'filled', 'MarkerFaceAlpha', 0.75, 'MarkerFaceColor', [0.8500, 0.3250, 0.0980]); 
    p2 = scatter(traject(i).position(idx_pert2), traject(i).d_position(idx_pert2), ...
        'filled', 'MarkerFaceAlpha', 0.75, 'MarkerFaceColor', [0.9290, 0.6940, 0.1250]); 
    p3 = scatter(traject(i).position(idx_pert3), traject(i).d_position(idx_pert3), ...
        'filled', 'MarkerFaceAlpha', 0.75, 'MarkerFaceColor', [0.4940, 0.1840, 0.5560]);
end
xlabel('Position (m)')
ylabel('Velocity (m/s)')
legend([p1(1), p2(1), p3(1)], {'a)', 'b)', 'c)'})
% force phase diagram
subplot(1,2,2)
hold on
plot(vertcat(traject.force), vertcat(traject.d_force), 'Color', [0, 0.4470, 0.7410, 0.1])
for i = 1:length(traject)
    idx_pert1 = [cycles{i}([cycles{i}.type_dist] == 1).glob_ind];
    idx_pert2 = [cycles{i}([cycles{i}.type_dist] == 2).glob_ind];
    idx_pert3 = [cycles{i}([cycles{i}.type_dist] == 3).glob_ind];
    p1 = scatter(traject(i).force(idx_pert1), traject(i).d_force(idx_pert1), ...
        'filled', 'MarkerFaceAlpha', 0.75, 'MarkerFaceColor', [0.8500, 0.3250, 0.0980]); 
    p2 = scatter(traject(i).force(idx_pert2), traject(i).d_force(idx_pert2), ...
        'filled', 'MarkerFaceAlpha', 0.75, 'MarkerFaceColor', [0.9290, 0.6940, 0.1250]); 
    p3 = scatter(traject(i).force(idx_pert3), traject(i).d_force(idx_pert3), ...
        'filled', 'MarkerFaceAlpha', 0.75, 'MarkerFaceColor', [0.4940, 0.1840, 0.5560]);
end
legend([p1(1), p2(1), p3(1)], {'a)', 'b)', 'c)'})
xlabel('Force (N)')
ylabel('Yank (N/s)')